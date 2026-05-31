idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES sk_core sk_api nvs_flash
                  esp_pm driver
                  bf_pins bf_vibration bf_battery bf_secure_store
                  bf_lis3dsh bf_face_detector bf_timer_engine
                  bf_display bf_ui bf_power
)
