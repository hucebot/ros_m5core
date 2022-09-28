idf_component_register(INCLUDE_DIRS M5Core2 M5Core2/src M5Core2/src/utility M5Core2/src/Fonts
SRCS
M5Core2/src/AXP192.cpp
M5Core2/src/M5Core2.cpp
M5Core2/src/M5Display.cpp
M5Core2/src/M5Touch.cpp
M5Core2/src/RTC.cpp
M5Core2/src/Speaker.cpp
M5Core2/src/utility/CommUtil.cpp
M5Core2/src/utility/In_eSPI.cpp
M5Core2/src/utility/M5Button.cpp
M5Core2/src/utility/M5Timer.cpp
M5Core2/src/utility/MPU6886.cpp
M5Core2/src/utility/MahonyAHRS.cpp
M5Core2/src/utility/PointAndZone.cpp
M5Core2/src/utility/Sprite.cpp
M5Core2/src/utility/quaternionFilters.cpp
REQUIRES arduino-esp32
)
set_source_files_properties(M5Core2/src/AXP192.cpp PROPERTIES COMPILE_FLAGS  -Wno-maybe-uninitialized)