# bf_pins — header-only component. pins.h merkezi GPIO ve PCF8574T bit
# atamalarını barındırır. Tüm bf_* component'leri ve main bunu REQUIRES
# eder; bu sayede pin haritası tek noktadan değişir, component'ler kendi
# include path manipülasyonu yapmaz.
idf_component_register(INCLUDE_DIRS "include")
