idf_component_register(
    SRCS "src/bf_secure_store.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash spiffs mbedtls json
    PRIV_REQUIRES sk_core
)
