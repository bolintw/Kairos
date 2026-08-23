idf_component_register(SRCS "main.cpp" "attitude_estimator.cpp" "app_controller.cpp"
                    "gui_manager.cpp" "stopwatch_face.cpp" "pomodoro_face.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c LovyanGFX esp_timer lvgl)
