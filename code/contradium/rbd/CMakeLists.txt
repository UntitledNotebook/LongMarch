file(GLOB SOURCES "*.cpp" "*.h")

# Add urdf subdirectory sources
file(GLOB URDF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.h")
list(APPEND SOURCES ${URDF_SOURCES})

add_library(${CONTRADIUM_SUBLIB_NAME} ${SOURCES})

# Find Boost for string processing (needed by urdf)
find_package(Boost REQUIRED)

# Find tinyxml2 for URDF XML parsing
find_package(tinyxml2 CONFIG REQUIRED)

target_link_libraries(${CONTRADIUM_SUBLIB_NAME} PUBLIC
    Grassland
    Snowberg
    Boost::boost
    tinyxml2::tinyxml2
)

list(APPEND CONTRADIUM_LIBS ${CONTRADIUM_SUBLIB_NAME})
set(CONTRADIUM_LIBS ${CONTRADIUM_LIBS} PARENT_SCOPE)
