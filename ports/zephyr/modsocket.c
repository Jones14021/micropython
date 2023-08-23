/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Linaro Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/mpconfig.h"
#ifdef MICROPY_PY_SOCKET

#include "py/runtime.h"
#include "py/stream.h"

#include <stdio.h>
#include <zephyr/zephyr.h>
// Zephyr's generated version header
#include <version.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_pkt.h>
#ifdef CONFIG_NET_SOCKETS
#include <zephyr/net/socket.h>
#endif

#if MICROPY_MODSOCKET_DEBUG_PRINT // print debugging info
#define DEBUG_printf printf
#else // don't print debugging info
#define DEBUG_printf(...) (void)0
#endif

typedef struct _socket_obj_t {
    mp_obj_base_t base;
    int ctx;

    #define STATE_NEW 0
    #define STATE_CONNECTING 1
    #define STATE_CONNECTED 2
    #define STATE_PEER_CLOSED 3
    int8_t state;
} socket_obj_t;

STATIC const mp_obj_type_t socket_type;

// Helper functions

#define RAISE_ERRNO(x) { int _err = x; if (_err < 0) mp_raise_OSError(-_err); }
#define RAISE_SOCK_ERRNO(x) { if ((int)(x) == -1) mp_raise_OSError(errno); }

STATIC void socket_check_closed(socket_obj_t *socket) {
    DEBUG_printf("socket_check_closed\n");
    if (socket->ctx == -1) {
        // already closed
        mp_raise_OSError(EBADF);
    }
}

STATIC void parse_inet_addr(socket_obj_t *socket, mp_obj_t addr_in, struct sockaddr *sockaddr) {
    DEBUG_printf("parse_inet_addr\n");
    // We employ the fact that port and address offsets are the same for IPv4 & IPv6
    struct sockaddr_in *sockaddr_in = (struct sockaddr_in *)sockaddr;

    mp_obj_t *addr_items;
    mp_obj_get_array_fixed_n(addr_in, 2, &addr_items);
    void *context = zsock_get_context_object(socket->ctx);
    sockaddr_in->sin_family = net_context_get_family(context);
    RAISE_ERRNO(net_addr_pton(sockaddr_in->sin_family, mp_obj_str_get_str(addr_items[0]), &sockaddr_in->sin_addr));
    sockaddr_in->sin_port = htons(mp_obj_get_int(addr_items[1]));
}

STATIC mp_obj_t format_inet_addr(struct sockaddr *addr, mp_obj_t port) {
    DEBUG_printf("format_inet_addr\n");
    // We employ the fact that port and address offsets are the same for IPv4 & IPv6
    struct sockaddr_in6 *sockaddr_in6 = (struct sockaddr_in6 *)addr;
    char buf[40];
    net_addr_ntop(addr->sa_family, &sockaddr_in6->sin6_addr, buf, sizeof(buf));
    mp_obj_tuple_t *tuple = mp_obj_new_tuple(addr->sa_family == AF_INET ? 2 : 4, NULL);

    tuple->items[0] = mp_obj_new_str(buf, strlen(buf));
    // We employ the fact that port offset is the same for IPv4 & IPv6
    // not filled in
    // tuple->items[1] = mp_obj_new_int(ntohs(((struct sockaddr_in*)addr)->sin_port));
    tuple->items[1] = port;

    if (addr->sa_family == AF_INET6) {
        tuple->items[2] = MP_OBJ_NEW_SMALL_INT(0); // flow_info
        tuple->items[3] = MP_OBJ_NEW_SMALL_INT(sockaddr_in6->sin6_scope_id);
    }

    return MP_OBJ_FROM_PTR(tuple);
}

socket_obj_t *socket_new(void) {
    DEBUG_printf("socket_new\n");
    socket_obj_t *socket = m_new_obj_with_finaliser(socket_obj_t);
    socket->base.type = (mp_obj_t)&socket_type;
    socket->state = STATE_NEW;
    return socket;
}

// Methods

STATIC void socket_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    DEBUG_printf("socket_print\n");
    socket_obj_t *self = self_in;
    if (self->ctx == -1) {
        mp_printf(print, "<socket NULL>");
    } else {
        void *context = zsock_get_context_object(self->ctx);
        mp_printf(print, "<socket %p type=%d>", self->ctx, net_context_get_type(context));
    }
}

STATIC mp_obj_t socket_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    DEBUG_printf("socket_make_new\n");
    mp_arg_check_num(n_args, n_kw, 0, 4, false);

    socket_obj_t *socket = socket_new();

    int family = AF_INET;
    int socktype = SOCK_STREAM;
    int proto = -1;

    if (n_args >= 1) {
        family = mp_obj_get_int(args[0]);
        if (n_args >= 2) {
            socktype = mp_obj_get_int(args[1]);
            if (n_args >= 3) {
                proto = mp_obj_get_int(args[2]);
            }
        }
    }

    if (proto == -1) {
        proto = IPPROTO_TCP;
        if (socktype != SOCK_STREAM) {
            proto = IPPROTO_UDP;
        }
    }

    socket->ctx = zsock_socket(family, socktype, proto);
    RAISE_SOCK_ERRNO(socket->ctx);

    return MP_OBJ_FROM_PTR(socket);
}

STATIC mp_obj_t socket_bind(mp_obj_t self_in, mp_obj_t addr_in) {
    DEBUG_printf("socket_bind\n");
    socket_obj_t *socket = self_in;
    socket_check_closed(socket);

    struct sockaddr sockaddr;
    parse_inet_addr(socket, addr_in, &sockaddr);

    int res = zsock_bind(socket->ctx, &sockaddr, sizeof(sockaddr));
    RAISE_SOCK_ERRNO(res);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_bind_obj, socket_bind);

STATIC mp_obj_t socket_connect(mp_obj_t self_in, mp_obj_t addr_in) {
    DEBUG_printf("socket_connect\n");
    socket_obj_t *socket = self_in;
    socket_check_closed(socket);

    struct sockaddr sockaddr;
    parse_inet_addr(socket, addr_in, &sockaddr);

    int res = zsock_connect(socket->ctx, &sockaddr, sizeof(sockaddr));
    RAISE_SOCK_ERRNO(res);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_connect_obj, socket_connect);

// method socket.listen([backlog])
STATIC mp_obj_t socket_listen(size_t n_args, const mp_obj_t *args) {
    DEBUG_printf("socket_listen\n");
    socket_obj_t *socket = args[0];
    socket_check_closed(socket);

    mp_int_t backlog = MICROPY_PY_SOCKET_LISTEN_BACKLOG_DEFAULT;
    if (n_args > 1) {
        backlog = mp_obj_get_int(args[1]);
        backlog = (backlog < 0) ? 0 : backlog;
    }

    int res = zsock_listen(socket->ctx, backlog);
    RAISE_SOCK_ERRNO(res);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_listen_obj, 1, 2, socket_listen);

STATIC mp_obj_t socket_accept(mp_obj_t self_in) {
    DEBUG_printf("socket_accept\n");
    socket_obj_t *socket = self_in;
    socket_check_closed(socket);

    struct sockaddr sockaddr;
    socklen_t addrlen = sizeof(sockaddr);
    int ctx = zsock_accept(socket->ctx, &sockaddr, &addrlen);

    socket_obj_t *socket2 = socket_new();
    socket2->ctx = ctx;

    mp_obj_tuple_t *client = mp_obj_new_tuple(2, NULL);
    client->items[0] = MP_OBJ_FROM_PTR(socket2);
    // TODO
    client->items[1] = mp_const_none;

    return MP_OBJ_FROM_PTR(client);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_accept_obj, socket_accept);

STATIC mp_uint_t sock_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode) {
    DEBUG_printf("sock_write\n");
    socket_obj_t *socket = self_in;
    if (socket->ctx == -1) {
        // already closed
        *errcode = EBADF;
        return MP_STREAM_ERROR;
    }

    ssize_t len = zsock_send(socket->ctx, buf, size, 0);
    if (len == -1) {
        *errcode = errno;
        return MP_STREAM_ERROR;
    }

    return len;
}

STATIC mp_obj_t socket_send(mp_obj_t self_in, mp_obj_t buf_in) {
    DEBUG_printf("socket_send\n");
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    int err = 0;
    mp_uint_t len = sock_write(self_in, bufinfo.buf, bufinfo.len, &err);
    if (len == MP_STREAM_ERROR) {
        mp_raise_OSError(err);
    }
    return mp_obj_new_int_from_uint(len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_send_obj, socket_send);

STATIC mp_uint_t sock_read(mp_obj_t self_in, void *buf, mp_uint_t max_len, int *errcode) {
    DEBUG_printf("sock_read\n");
    socket_obj_t *socket = self_in;
    if (socket->ctx == -1) {
        // already closed
        *errcode = EBADF;
        return MP_STREAM_ERROR;
    }

    ssize_t recv_len = zsock_recv(socket->ctx, buf, max_len, 0);
    if (recv_len == -1) {
        *errcode = errno;
        return MP_STREAM_ERROR;
    }

    return recv_len;
}

STATIC mp_obj_t socket_recv(mp_obj_t self_in, mp_obj_t len_in) {
    DEBUG_printf("socket_recv\n");
    mp_int_t max_len = mp_obj_get_int(len_in);
    vstr_t vstr;
    // +1 to accommodate for trailing \0
    vstr_init_len(&vstr, max_len + 1);

    int err;
    mp_uint_t len = sock_read(self_in, vstr.buf, max_len, &err);

    if (len == MP_STREAM_ERROR) {
        vstr_clear(&vstr);
        mp_raise_OSError(err);
    }

    if (len == 0) {
        vstr_clear(&vstr);
        return mp_const_empty_bytes;
    }

    vstr.len = len;
    return mp_obj_new_bytes_from_vstr(&vstr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_recv_obj, socket_recv);

STATIC mp_obj_t socket_setsockopt(size_t n_args, const mp_obj_t *args) {
    DEBUG_printf("socket_setsockopt\n");
    (void)n_args; // always 4
    mp_warning(MP_WARN_CAT(RuntimeWarning), "setsockopt() not implemented");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_setsockopt_obj, 4, 4, socket_setsockopt);

STATIC mp_obj_t socket_makefile(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    return args[0];
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_makefile_obj, 1, 3, socket_makefile);

STATIC mp_uint_t sock_ioctl(mp_obj_t o_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    DEBUG_printf("sock_ioctl\n");
    socket_obj_t *socket = o_in;
    (void)arg;
    switch (request) {
        case MP_STREAM_CLOSE:
            if (socket->ctx != -1) {
                int res = zsock_close(socket->ctx);
                RAISE_SOCK_ERRNO(res);
                if (res == -1) {
                    *errcode = errno;
                    return MP_STREAM_ERROR;
                }
                socket->ctx = -1;
            }
            return 0;

        default:
            *errcode = MP_EINVAL;
            return MP_STREAM_ERROR;
    }
}

STATIC const mp_rom_map_elem_t socket_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_bind), MP_ROM_PTR(&socket_bind_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&socket_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_listen), MP_ROM_PTR(&socket_listen_obj) },
    { MP_ROM_QSTR(MP_QSTR_accept), MP_ROM_PTR(&socket_accept_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&socket_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&socket_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_setsockopt), MP_ROM_PTR(&socket_setsockopt_obj) },

    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_makefile), MP_ROM_PTR(&socket_makefile_obj) },
};
STATIC MP_DEFINE_CONST_DICT(socket_locals_dict, socket_locals_dict_table);

STATIC const mp_stream_p_t socket_stream_p = {
    .read = sock_read,
    .write = sock_write,
    .ioctl = sock_ioctl,
};

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    socket_type,
    MP_QSTR_socket,
    MP_TYPE_FLAG_NONE,
    make_new, socket_make_new,
    print, socket_print,
    protocol, &socket_stream_p,
    locals_dict, &socket_locals_dict
    );

STATIC mp_obj_t pkt_get_info(void) {
    DEBUG_printf("pkt_get_info\n");
    struct k_mem_slab *rx, *tx;
    struct net_buf_pool *rx_data, *tx_data;
    net_pkt_get_info(&rx, &tx, &rx_data, &tx_data);
    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(4, NULL));
    t->items[0] = MP_OBJ_NEW_SMALL_INT(k_mem_slab_num_free_get(rx));
    t->items[1] = MP_OBJ_NEW_SMALL_INT(k_mem_slab_num_free_get(tx));
    t->items[2] = MP_OBJ_NEW_SMALL_INT(rx_data->avail_count);
    t->items[3] = MP_OBJ_NEW_SMALL_INT(tx_data->avail_count);
    return MP_OBJ_FROM_PTR(t);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(pkt_get_info_obj, pkt_get_info);

STATIC const mp_rom_map_elem_t mp_module_socket_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_socket) },
    // objects
    { MP_ROM_QSTR(MP_QSTR_socket), MP_ROM_PTR(&socket_type) },
    // class constants
    { MP_ROM_QSTR(MP_QSTR_AF_INET), MP_ROM_INT(AF_INET) },
    { MP_ROM_QSTR(MP_QSTR_AF_INET6), MP_ROM_INT(AF_INET6) },

    { MP_ROM_QSTR(MP_QSTR_SOCK_STREAM), MP_ROM_INT(SOCK_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_DGRAM), MP_ROM_INT(SOCK_DGRAM) },

    { MP_ROM_QSTR(MP_QSTR_SOL_SOCKET), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_SO_REUSEADDR), MP_ROM_INT(2) },

    { MP_ROM_QSTR(MP_QSTR_pkt_get_info), MP_ROM_PTR(&pkt_get_info_obj) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_socket_globals, mp_module_socket_globals_table);

const mp_obj_module_t mp_module_socket = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_socket_globals,
};

MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_socket, mp_module_socket);

#endif // MICROPY_PY_SOCKET
