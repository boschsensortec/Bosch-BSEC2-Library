idf_component_register(
    SRCS "src/bsec2.cpp" "src/commMux/commMux.cpp"
    INCLUDE_DIRS "." "src" "src/config" "src/inc"
    REQUIRES BME68x
)

# Link binary library (precompiled) with the ESP32-S3 architecture, adapt for your ESP32 board if necessary

target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/esp32s3/libalgobsec.a")
# target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/esp32/libalgobsec.a")
# target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/esp32s2/libalgobsec.a")
# target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/esp32c3/libalgobsec.a")