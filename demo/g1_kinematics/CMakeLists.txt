file(GLOB_RECURSE DEMO_SOURCES "*.cpp" "*.h")

add_executable(${DEMO_NAME} ${DEMO_SOURCES})

# Find nlohmann-json locally (not in root CMakeLists.txt)
find_package(nlohmann_json CONFIG REQUIRED)

target_link_libraries(${DEMO_NAME} PUBLIC LongMarch ${ASSIMP_LIB_NAME} nlohmann_json::nlohmann_json)
