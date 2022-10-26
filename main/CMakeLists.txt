idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component M5Core2 arduino-esp32 FastLED-idf)
