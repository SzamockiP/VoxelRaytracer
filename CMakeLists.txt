cmake_minimum_required(VERSION 3.26)

project(voxel_rt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_library(voxel_core)
target_sources(voxel_core
	PRIVATE
	"src/io/ppm_writer.cpp"
	"src/core/camera.cpp"
    "src/math/aabb.cpp" )

target_include_directories(voxel_core
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/src
		${CMAKE_CURRENT_SOURCE_DIR}/include
)


if(MSVC)
	target_compile_options(voxel_core PRIVATE /W4 /permissive-)
else()
	target_compile_options(voxel_core PRIVATE -Wall -Wextra -Wpedantic)
endif()


add_executable(voxel_app
	apps/voxel_rt/main.cpp
)
target_link_libraries(voxel_app PRIVATE voxel_core)