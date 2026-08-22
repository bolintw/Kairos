idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c LovyanGFX esp_timer lvgl)
