idf_component_register(
    SRCS "src/bf_face_detector.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bf_pins bf_lis3dsh sk_core
)
