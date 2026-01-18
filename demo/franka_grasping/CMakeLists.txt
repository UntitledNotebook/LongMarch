file(GLOB_RECURSE DEMO_SOURCES "src/*.cpp" "src/*.h")

add_executable(${DEMO_NAME} ${DEMO_SOURCES})

target_include_directories(${DEMO_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_link_libraries(${DEMO_NAME} PUBLIC LongMarch ${ASSIMP_LIB_NAME})
