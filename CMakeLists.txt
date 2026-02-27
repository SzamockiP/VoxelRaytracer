cmake_minimum_required(VERSION 3.26)

project(voxel_rt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# LFW
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

# GLAD
FetchContent_Declare(
  glad
  GIT_REPOSITORY https://github.com/Dav1dde/glad.git
  GIT_TAG v0.1.36
)
FetchContent_MakeAvailable(glad)

add_library(vrt STATIC
	"src/io/ppm_writer.cpp"
	"src/core/camera.cpp"
    "src/math/aabb.cpp" 
	"src/voxel/dag_pool_manager.cpp"
 "src/gfx/window.cpp" "src/gfx/presenter.cpp")

target_include_directories(vrt PUBLIC include)

target_link_libraries(vrt PUBLIC glfw glad)

find_package(OpenGL REQUIRED)
target_link_libraries(vrt PUBLIC OpenGL::GL)


if(MSVC)
	target_compile_options(vrt PRIVATE /W4 /permissive-)
else()
	target_compile_options(vrt PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(voxel_app apps/voxel_rt/main.cpp)

# find_package(OpenMP REQUIRED) 
# target_link_libraries(voxel_app PRIVATE vrt OpenMP::OpenMP_CXX)
target_link_libraries(voxel_app PRIVATE vrt)