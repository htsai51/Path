add_library(PathFindAlgorithm OBJECT PathFindAlgorithm.cpp)
add_library(AStarAlgorithm OBJECT AStarAlgorithm.cpp)
add_library(Map OBJECT Map.cpp)
add_executable(shell-app main.cpp PathFindAlgorithm AStarAlgorithm Map)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
