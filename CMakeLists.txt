cmake_minimum_required(VERSION 3.5)

project(particle_filter)
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")
set(sources src/particle_filter.cpp src/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
	include_directories(/usr/local/include)
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/lib)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} z ssl uv uWS)

