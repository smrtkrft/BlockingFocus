idf_component_register(
    SRCS "src/bf_vibration.c"
    INCLUDE_DIRS "include"
    REQUIRES driver sk_core
    PRIV_REQUIRES bf_pins nvs_flash
)
