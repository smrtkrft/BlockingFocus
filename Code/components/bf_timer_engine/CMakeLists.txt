idf_component_register(
    SRCS "src/bf_timer_engine.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bf_pins bf_face_detector bf_vibration sk_core sk_api esp_timer
)
