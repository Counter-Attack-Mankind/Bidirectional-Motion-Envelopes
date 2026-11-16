#pragma once

#include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>

namespace buffered_footprint_planner {

class LockedTopDownOrthoViewController
    : public rviz::FixedOrientationOrthoViewController {
 public:
  void onInitialize() override;
  void handleMouseEvent(rviz::ViewportMouseEvent& event) override;
  void reset() override;
  void update(float dt, float ros_dt) override;

 private:
  void LockAngle();
};

}  // namespace buffered_footprint_planner
