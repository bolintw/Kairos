idf_component_register(SRCS "main.cpp" "attitude_estimator.cpp" "app_controller.cpp"
                    "gui_manager.cpp" "stopwatch_face.cpp" "pomodoro_face.cpp"
                    "reserved_face.cpp"
                    "nvs_calibration.cpp" "calibration_mode.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c LovyanGFX esp_timer lvgl nvs_flash)
