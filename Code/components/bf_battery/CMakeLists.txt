idf_component_register(
    SRCS "src/bf_battery.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc driver
    PRIV_REQUIRES bf_pins sk_core
)
