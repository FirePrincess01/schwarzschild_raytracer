cmake_minimum_required(VERSION 3.20)

# Create the project
project("schwarzschild_raytracer" CXX)

# Export compile commands for VSCode C++ Intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    # -fdiagnostics-color
    # -Wall 
    # -Wextra
    # -O0
)

# add exe
add_executable(${PROJECT_NAME})

# include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    "${CMAKE_SOURCE_DIR}/BacOGL"
)

# add source files
file(GLOB SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/BacOGL/*.cpp"
)

target_sources(${PROJECT_NAME} PRIVATE ${SOURCE_FILES})

# link libraries
find_package(GLUT REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE GLUT::GLUT)

find_package(GLEW REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE GLEW::GLEW)

find_package(soil CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE soil)

# copy assets to exe
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
)
