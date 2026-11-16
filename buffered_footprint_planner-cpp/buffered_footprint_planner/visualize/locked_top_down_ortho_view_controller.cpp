#include "visualize/locked_top_down_ortho_view_controller.h"

#include <QEvent>

#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>

#include <pluginlib/class_list_macros.hpp>

namespace buffered_footprint_planner {

void LockedTopDownOrthoViewController::LockAngle() {
  angle_property_->setFloat(0.0f);
  orientCamera();
}

void LockedTopDownOrthoViewController::onInitialize() {
  rviz::FixedOrientationOrthoViewController::onInitialize();
  LockAngle();
  angle_property_->hide();
}

void LockedTopDownOrthoViewController::reset() {
  rviz::FixedOrientationOrthoViewController::reset();
  LockAngle();
}

void LockedTopDownOrthoViewController::update(float dt, float ros_dt) {
  LockAngle();
  rviz::FixedOrientationOrthoViewController::update(dt, ros_dt);
}

void LockedTopDownOrthoViewController::handleMouseEvent(
    rviz::ViewportMouseEvent& event) {
  setStatus("<b>Middle-Click</b> or <b>Shift+Left-Click:</b> Move X/Y.  "
            "<b>Right-Click:</b> Zoom. Rotation is locked.");
  int32_t diff_x = 0;
  int32_t diff_y = 0;
  if (event.type == QEvent::MouseButtonPress) {
    dragging_ = true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    dragging_ = false;
  } else if (dragging_ && event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
  }

  bool moved = false;
  if (event.middle() || (event.shift() && event.left())) {
    setCursor(MoveXY);
    const float scale = scale_property_->getFloat();
    move(-diff_x / scale, diff_y / scale);
    moved = diff_x != 0 || diff_y != 0;
  } else if (event.right()) {
    setCursor(Zoom);
    scale_property_->multiply(1.0f - diff_y * 0.01f);
    moved = diff_y != 0;
  }
  if (event.wheel_delta != 0) {
    scale_property_->multiply(1.0f + event.wheel_delta * 0.001f);
    moved = true;
  }
  LockAngle();
  if (moved) {
    context_->queueRender();
    emitConfigChanged();
  }
}

}  // namespace buffered_footprint_planner

PLUGINLIB_EXPORT_CLASS(
    buffered_footprint_planner::LockedTopDownOrthoViewController,
    rviz::ViewController)
