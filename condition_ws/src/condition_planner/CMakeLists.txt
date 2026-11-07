cmake_minimum_required(VERSION 3.5)
project(condition_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -march=native)

# ---- Dependencies ----
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf
  sandbox_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS .
  CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs visualization_msgs nav_msgs tf sandbox_msgs
)

include_directories(
  .
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/
)

# CasADi
find_library(CASADI_LIB casadi HINTS /usr/local/lib)
if(NOT CASADI_LIB)
  message(WARNING "CasADi not found, trajectory_nlp will not be built")
endif()

# ---- Common libraries ----
add_library(common_math
  common/math/vec2d.cc
  common/math/box2d.cc
  common/math/aabox2d.cc
  common/math/polygon2d.cc
  common/math/line_segment2d.cc
  common/math/math_utils.cc
  common/math/linear_interpolation.cc
  common/math/trajectory1d.cc
)

add_library(common_util
  common/util/point_cloud.cc
  common/util/csv_logger.cc
  common/util/color.cc
)
target_link_libraries(common_util common_math)

# ---- Hybrid A* ----
add_library(hybrid_astar
  hybrid_astar/hybrid_a_star.cc
  hybrid_astar/node3d.cc
  hybrid_astar/grid_search.cc
  hybrid_astar/reeds_shepp_path.cc
)
target_link_libraries(hybrid_astar common_math common_util)

# ---- Trajectory NLP (LIOM) ----
if(CASADI_LIB)
  add_library(trajectory_nlp
    trajectory_nlp/trajectory_nlp.cc
    trajectory_nlp/dp_sv_planner_ecc.cc
    trajectory_nlp/trajectory_optimizer.cc
  )
  target_link_libraries(trajectory_nlp common_math ${catkin_LIBRARIES} ${CASADI_LIB})
endif()

# ---- Environment ----
add_library(my_env
  my_env.cpp
)
target_link_libraries(my_env common_math)

# ---- Main node ----
add_executable(condition_planner_node
  node.cpp
  interface.cc
  visualization_plot.cpp
)
target_link_libraries(condition_planner_node
  ${catkin_LIBRARIES}
  hybrid_astar
  my_env
  common_math
  common_util
)
if(CASADI_LIB)
  target_link_libraries(condition_planner_node trajectory_nlp)
endif()

# ---- Trajectory replay node (读CSV离线发布给pure_pursuit) ----
add_executable(trajectory_replay_node
  src/trajectory_replay_node.cpp
)
target_link_libraries(trajectory_replay_node ${catkin_LIBRARIES})

# ---- Replay visualization node (回放时画场景+起终点) ----
add_executable(replay_viz_node
  src/replay_viz_node.cpp
  visualization_plot.cpp
  my_env.cpp
)
target_link_libraries(replay_viz_node
  ${catkin_LIBRARIES}
  common_math
  common_util
)
