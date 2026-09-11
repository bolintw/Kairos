idf_component_register(SRCS "main.cpp" "attitude_estimator.cpp" "app_controller.cpp"
                    "gui_manager.cpp" "stopwatch_face.cpp" "pomodoro_face.cpp"
                    "breath_face.cpp" "font_space_grotesk_bold_40.c"
                    "nvs_calibration.cpp" "calibration_mode.cpp" "sleep_mode.cpp"
                    "battery_monitor.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c LovyanGFX esp_timer lvgl nvs_flash esp_hw_support esp_adc)
