idf_component_register(
    SRCS "src/bf_lis3dsh.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES bf_pins sk_core
)
