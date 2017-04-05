cmake_minimum_required(VERSION 2.8.11)
project(PARTICLE_FILTER)


# Build the particle filter project and solution.
# Use C++11
set(SRCS src/main.cpp src/particle_filter.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(particle_filter ${SRCS})

# Use C++11
#if [ ! -f ./src/particle_filter_sol.cpp]; then
#	echo "No solution file."
#else
#	echo "Solution file exists"
#fi

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/particle_filter_sol.cpp")
	set(SRCS src/main.cpp src/particle_filter_sol.cpp)
	set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

	# Create the executable
	add_executable(particle_filter_solution ${SRCS})
endif()


option(test "Build all tests." OFF) # Makes boolean 'test' available.

################################
# Testing
################################
if (test)
  add_library(PF_lib ./src/particle_filter.cpp)
  target_link_libraries(particle_filter PF_lib)

  # This adds another subdirectory, which has 'project(gtest)'.
  add_subdirectory(test/googletest-release-1.8.0)

  enable_testing()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  ##############
  # Unit Tests
  ##############
  add_executable(RunUnitTests ./test/main.cpp)
  add_executable(PFTest ./test/particle_filter_test.cpp)

  # Standard linking to gtest stuff.
  target_link_libraries(RunUnitTests gtest gtest_main)
  target_link_libraries(PFTest gtest gtest_main)

  # Extra linking for the project.
  target_link_libraries(RunUnitTests PF_lib)
  target_link_libraries(PFTest PF_lib)

  add_test(NAME PFTest COMMAND PFTest)
endif()
