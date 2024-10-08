cmake_minimum_required(VERSION 3.8)
project(sensr_recorder_sample VERSION 1.0.0 LANGUAGES CXX)

find_package(Protobuf REQUIRED)
find_package(sensr_sdk REQUIRED)

add_executable(bin_output_sample main.cpp)
target_link_libraries(bin_output_sample pthread sensr_sdk sensr_proto) 
target_include_directories(bin_output_sample PRIVATE sensr_sdk::INCLUDE_DIRECTORIES)

if(MSVC)
  target_compile_options(bin_output_sample PRIVATE /W4 /WX)
else()
  target_compile_options(bin_output_sample PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

