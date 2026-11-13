#include <cassert>

#include "execution/experiment_manager.h"

namespace {

scene::SceneSnapshot MakeScene() {
  scene::SceneSnapshot scene;
  scene.start = {0.5, 0.5, 0.0};
  scene.obstacles.push_back({"obstacle_1", 1.0, 1.0, 0.0, 0.2, 0.1});
  return scene;
}

void AdvanceToPlanning(execution::ExperimentManager* manager,
                       scene::SceneSnapshot scene) {
  manager->Observe(scene, true, 1, 0.0);
  manager->Observe(scene, true, 1, 3.1);
  assert(manager->state() ==
         execution::ExperimentManager::State::kReadyToPlan);
  scene::SceneSnapshot frozen;
  std::uint64_t generation = 0;
  assert(manager->BeginPlanning(&frozen, &generation));
  assert(frozen.task_id == 1);
}

}  // namespace

int main() {
  execution::ExperimentManager::Configuration config;
  config.stationary_duration = 3.0;

  execution::ExperimentManager planning_manager(config);
  auto scene = MakeScene();
  AdvanceToPlanning(&planning_manager, scene);
  scene.start.x += 0.02;
  planning_manager.Observe(scene, true, 2, 3.2);
  assert(!planning_manager.frozen_scene_valid());

  execution::ExperimentManager tracking_manager(config);
  scene = MakeScene();
  AdvanceToPlanning(&tracking_manager, scene);
  tracking_manager.PlanningFinished(true, true);
  scene.start.x += 0.5;
  tracking_manager.Observe(scene, true, 2, 3.2);
  assert(tracking_manager.frozen_scene_valid());
  scene.obstacles[0].x += 0.02;
  tracking_manager.Observe(scene, true, 3, 3.3);
  assert(!tracking_manager.frozen_scene_valid());
  assert(tracking_manager.state() ==
         execution::ExperimentManager::State::kWaitingForSceneChange);
  return 0;
}
