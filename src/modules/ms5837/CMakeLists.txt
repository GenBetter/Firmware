set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__ms5837
    MAIN ms5837
    COMPILE_FLAGS
        -Os
    SRCS
        ms5837.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 