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

  // Geometrically identical OBBs must not look moved merely because minimum
  // area fitting selected an equivalent heading/corner ordering.
  scene::SceneObstacle square_a{"square", 1.0, 1.0, 0.0, 0.2, 0.2};
  scene::SceneObstacle square_b{"square", 1.0, 1.0, M_PI_2, 0.2, 0.2};
  assert(execution::ExperimentManager::ObstacleBoundaryDistance(
             square_a, square_b) < 1e-12);
  scene::SceneObstacle rectangle_a{"rect", 1.0, 1.0, 0.0, 0.3, 0.1};
  scene::SceneObstacle rectangle_b{"rect", 1.0, 1.0, M_PI, 0.3, 0.1};
  assert(execution::ExperimentManager::ObstacleBoundaryDistance(
             rectangle_a, rectangle_b) < 1e-12);

  execution::ExperimentManager marker_failure(config);
  scene = MakeScene();
  AdvanceToPlanning(&marker_failure, scene);
  marker_failure.Observe(scene, false, 1, 3.2);
  assert(!marker_failure.frozen_scene_valid());
  assert(marker_failure.state() ==
         execution::ExperimentManager::State::kWaitingForData);

  execution::ExperimentManager revoked(config);
  scene = MakeScene();
  AdvanceToPlanning(&revoked, scene);
  revoked.AbortTask();
  assert(!revoked.frozen_scene_valid());
  assert(revoked.state() ==
         execution::ExperimentManager::State::kWaitingForData);

  execution::ExperimentManager rounds(config);
  scene = MakeScene();
  rounds.Observe(scene, true, 1, 0.0);
  rounds.Observe(scene, true, 1, 3.1);
  scene::SceneSnapshot frozen;
  std::uint64_t generation = 0;
  assert(rounds.BeginPlanning(&frozen, &generation));
  assert(generation == 1);
  rounds.PlanningFinished(true, false);
  assert(rounds.state() == execution::ExperimentManager::State::kCompleted);
  scene.start.x += config.rearm_distance + 0.01;
  rounds.Observe(scene, true, 2, 3.2);
  rounds.Observe(scene, true, 2, 6.3);
  assert(rounds.BeginPlanning(&frozen, &generation));
  assert(generation == 2 && frozen.task_id == 2);

  execution::ExperimentManager obstacle_round(config);
  scene = MakeScene();
  obstacle_round.Observe(scene, true, 1, 0.0);
  obstacle_round.Observe(scene, true, 1, 3.1);
  assert(obstacle_round.BeginPlanning(&frozen, &generation));
  obstacle_round.PlanningFinished(true, false);
  scene.obstacles[0].x += 0.02;
  obstacle_round.Observe(scene, true, 2, 3.2);
  obstacle_round.Observe(scene, true, 2, 6.3);
  assert(obstacle_round.state() ==
         execution::ExperimentManager::State::kReadyToPlan);
  return 0;
}
