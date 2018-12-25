set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__AHRS
    MAIN AHRS
    COMPILE_FLAGS
        -Os
    SRCS
        AHRS.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 