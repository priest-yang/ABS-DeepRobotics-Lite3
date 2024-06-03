cmake_minimum_required(VERSION 3.10)

# project(run_policy)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


set(Torch_DIR "${THIRD_PARTY}/libtorch/${BUILD_PLATFORM}/share/cmake/Torch")
find_package(Torch REQUIRED)
message("Torch_DIR:  ${Torch_DIR}")

include_directories(
    ./
    ${TORCH_INCLUDE_DIRS}
)
file(GLOB_RECURSE RUN_POLICY_SRC "*.cpp")

add_library(policy_runner STATIC ${RUN_POLICY_SRC})
target_link_libraries(policy_runner PUBLIC -lpthread -lm -lrt -ldl -lstdc++ torch)

# add_executable(test_policy ./test.cpp)
# target_link_libraries(test_policy policy_runner)