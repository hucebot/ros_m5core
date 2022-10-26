idf_component_register(INCLUDE_DIRS FastLED FastLED/src
SRCS
FastLED/src/FastLED.cpp
FastLED/src/bitswap.cpp
FastLED/src/colorpalettes.cpp
FastLED/src/colorutils.cpp
FastLED/src/hsv2rgb.cpp
FastLED/src/lib8tion.cpp
FastLED/src/noise.cpp
FastLED/src/platforms.cpp
FastLED/src/power_mgt.cpp
FastLED/src/wiring.cpp
FastLED/src/platforms/esp/32/clockless_rmt_esp32.cpp
REQUIRES arduino-esp32
)