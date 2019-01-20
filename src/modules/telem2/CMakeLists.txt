set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__telem2
    MAIN telem2
    COMPILE_FLAGS
        -Os
    SRCS
        telem2.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 