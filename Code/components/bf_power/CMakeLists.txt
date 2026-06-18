idf_component_register(
    SRCS "src/bf_power.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bf_pins bf_display sk_core esp_timer esp_pm
)
