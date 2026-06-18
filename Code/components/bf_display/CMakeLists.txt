idf_component_register(
    SRCS "src/bf_display.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES bf_pins sk_core esp_timer
)
