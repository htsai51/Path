add_executable(shell-app main.cpp PathFindAlgorithm.cpp AStarAlgorithm.cpp Map.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
