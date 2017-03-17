cmake_minimum_required(VERSION 2.8.11)
project(PARTICLE_FILTER)

# Build the print samples quiz and solution.
# Use C++11
set(SRCS src/print_samples.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(print_samples ${SRCS})

# Use C++11
set(SRCS src/solutions/print_samples_sol.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(print_samples_solution ${SRCS})

# Build the particle filter project and solution.
# Use C++11
set(SRCS src/main.cpp src/particle_filter.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(particle_filter ${SRCS})

# Use C++11
set(SRCS src/solutions/main_sol.cpp src/solutions/particle_filter_sol.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(particle_filter_solution ${SRCS})






