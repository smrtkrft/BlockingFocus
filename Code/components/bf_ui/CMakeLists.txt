idf_component_register(
    SRCS "src/bf_ui.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bf_display bf_vibration sk_core esp_timer
)
