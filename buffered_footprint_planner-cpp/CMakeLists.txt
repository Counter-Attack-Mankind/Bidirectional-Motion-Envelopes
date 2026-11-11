cmake_minimum_required(VERSION 3.5)
project(buffered_footprint_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Hybrid A* performs many small geometry checks. Keep explicit user choices,
# but avoid an unintentionally unoptimized catkin build by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_service_files(FILES SetObjectPose.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common_math common_util hybrid_astar environment visualization_plot
            scene_visualizer trajectory_player footprint_visualizer
            swept_area_visualizer trajectory_playback_visualizer footprint_optimizer scene_core experiment_logging planning_pipeline
            planning_config experiment_runner
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf2
                 tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  include/common/math
  ${catkin_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
)
link_directories(${CASADI_LIBRARY_DIRS})

add_library(common_math
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/vec2d.cc
)

add_library(common_util
  common/util/color.cc
  common/util/csv_logger.cc
)
target_link_libraries(common_util common_math ${catkin_LIBRARIES})

add_library(hybrid_astar
  hybrid_astar/grid_search.cc
  hybrid_astar/hybrid_a_star.cc
  hybrid_astar/node3d.cc
  hybrid_astar/reeds_shepp_path.cc
)
target_link_libraries(hybrid_astar common_math common_util)

add_library(scene_core
  scene/scene_snapshot.cpp
  scene/scene_storage.cpp
  scene/scene_recorder.cpp
)
target_link_libraries(scene_core common_math yaml-cpp)

add_library(environment environment/scenario_manager.cpp)
target_link_libraries(environment scene_core common_math ${catkin_LIBRARIES})

add_library(visualization_plot visualize/visualization_plot.cpp)
target_link_libraries(visualization_plot common_math common_util ${catkin_LIBRARIES})

add_library(scene_visualizer visualize/scene_visualizer.cpp)
target_link_libraries(scene_visualizer visualization_plot hybrid_astar scene_core
                      ${catkin_LIBRARIES})

add_library(trajectory_player visualize/trajectory_player.cpp)
target_link_libraries(trajectory_player common_math ${catkin_LIBRARIES})

add_library(footprint_visualizer visualize/footprint_visualizer.cpp)
target_link_libraries(footprint_visualizer visualization_plot ${catkin_LIBRARIES})

add_library(swept_area_visualizer visualize/swept_area_visualizer.cpp)
target_link_libraries(swept_area_visualizer visualization_plot common_math
                      ${catkin_LIBRARIES})

add_library(trajectory_playback_visualizer
  visualize/trajectory_playback_visualizer.cpp)
target_link_libraries(trajectory_playback_visualizer trajectory_player
                      footprint_visualizer swept_area_visualizer
                      scene_visualizer ${catkin_LIBRARIES})

add_library(footprint_optimizer
  optimizer/embodied_footprint.cpp
  optimizer/initial_guess.cpp
  optimizer/footprint_nlp.cpp
)
target_link_libraries(footprint_optimizer common_math hybrid_astar
                      ${CASADI_LIBRARIES} ${catkin_LIBRARIES})

add_library(experiment_logging experiment/experiment_logger.cpp)
target_link_libraries(experiment_logging scene_core footprint_optimizer)

add_library(planning_config planning/planning_config_loader.cpp)
target_link_libraries(planning_config footprint_optimizer ${catkin_LIBRARIES})

add_library(planning_pipeline planning/planning_pipeline.cpp)
target_link_libraries(planning_pipeline planning_config hybrid_astar footprint_optimizer
                      experiment_logging scene_core ${catkin_LIBRARIES})

add_library(experiment_runner experiment/experiment_runner.cpp)
target_link_libraries(experiment_runner planning_pipeline scene_core)

add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node environment scene_visualizer ${catkin_LIBRARIES})
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node environment hybrid_astar footprint_optimizer
                      planning_pipeline scene_visualizer
                      trajectory_playback_visualizer ${catkin_LIBRARIES})
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scene_recorder_node src/scene_recorder_node.cpp)
target_link_libraries(scene_recorder_node environment scene_core planning_config
                      ${catkin_LIBRARIES})
add_dependencies(scene_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(experiment_node src/experiment_node.cpp)
target_link_libraries(experiment_node experiment_runner planning_pipeline
                      scene_core scene_visualizer trajectory_player
                      footprint_visualizer swept_area_visualizer
                      trajectory_playback_visualizer
                      ${catkin_LIBRARIES})
add_dependencies(experiment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/keyboard_editor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(hybrid_astar_timeout_regression
    tests/hybrid_astar_timeout_regression.cpp)
  target_link_libraries(hybrid_astar_timeout_regression hybrid_astar)
endif()

install(TARGETS common_math common_util hybrid_astar environment visualization_plot
                scene_visualizer trajectory_player footprint_visualizer
                swept_area_visualizer trajectory_playback_visualizer footprint_optimizer scene_core experiment_logging planning_pipeline
                planning_config experiment_runner simulation_node planning_node scene_recorder_node
                experiment_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY environment/tasks
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/environment)
