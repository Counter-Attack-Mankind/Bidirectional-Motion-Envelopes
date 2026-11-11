#!/usr/bin/env python3

"""Interactive terminal editor for the running simulation scene."""

import math
import select
import sys
import termios
import threading
import tty

import rospy
from std_msgs.msg import Bool

from buffered_footprint_planner.srv import SetObjectPose
from buffered_footprint_planner.srv import SetObjectPoseRequest


SIMULATION_NAMESPACE = "/buffered_footprint_simulation"
SCENE_SERVICE = "/simulation/set_object_pose"
LOCK_TOPIC = "/simulation/start_locked"


class TerminalMode:
    """Restores the original terminal settings on every exit path."""

    def __init__(self):
        self._fd = None
        self._original = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "keyboard_editor.py requires an interactive terminal; "
                "run it with rosrun in a terminal, not as a roslaunch node"
            )
        self._fd = sys.stdin.fileno()
        self._original = termios.tcgetattr(self._fd)
        # cbreak keeps ISIG enabled, so Ctrl+C continues to reach rospy.
        tty.setcbreak(self._fd)
        return self

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if readable else None

    def __exit__(self, _exception_type, _exception, _traceback):
        if self._fd is not None and self._original is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original)


class KeyboardSceneEditor:
    def __init__(self):
        self._print_lock = threading.RLock()
        self._last_locked = False
        self._obstacle_ids = []
        self._vehicle_id = ""
        self._selected_id = ""
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._translation_step = 0.0
        self._rotation_step = 0.0
        self._service = None

    @staticmethod
    def _parameter_path(relative_name):
        return SIMULATION_NAMESPACE + "/" + relative_name

    def initialize(self):
        names = {
            "obstacles": self._parameter_path("obstacles"),
            "translation_step": self._parameter_path("editor/translation_step"),
            "rotation_step": self._parameter_path("editor/rotation_step"),
            "vehicle_id": self._parameter_path("vehicle/id"),
        }
        missing = [path for path in names.values() if not rospy.has_param(path)]
        if missing:
            rospy.logerr(
                "Simulation parameters are unavailable: %s. Start "
                "simulation.launch before running keyboard_editor.py.",
                ", ".join(missing),
            )
            return False

        try:
            obstacles = rospy.get_param(names["obstacles"])
            self._vehicle_id = rospy.get_param(names["vehicle_id"])
            self._translation_step = rospy.get_param(names["translation_step"])
            self._rotation_step = rospy.get_param(names["rotation_step"])
        except (KeyError, rospy.ROSException) as exception:
            rospy.logerr("Cannot read simulation parameters: %s", exception)
            return False
        if not isinstance(obstacles, list):
            rospy.logerr(
                "Parameter %s must be a YAML array, but the running simulation "
                "provides %s.",
                names["obstacles"], type(obstacles).__name__,
            )
            return False
        if not 1 <= len(obstacles) <= 9:
            rospy.logerr(
                "Parameter %s must contain 1 to 9 obstacles; got %d.",
                names["obstacles"], len(obstacles),
            )
            return False

        seen_ids = set()
        for index, obstacle in enumerate(obstacles):
            if not isinstance(obstacle, dict):
                rospy.logerr(
                    "Obstacle %d in %s must be a YAML mapping.",
                    index + 1, names["obstacles"],
                )
                return False
            obstacle_id = obstacle.get("id")
            if (not isinstance(obstacle_id, str) or not obstacle_id or
                    obstacle_id in seen_ids):
                rospy.logerr(
                    "Obstacle %d has an empty, non-string, or duplicate ID.",
                    index + 1,
                )
                return False
            seen_ids.add(obstacle_id)
            self._obstacle_ids.append(obstacle_id)

        if not isinstance(self._vehicle_id, str) or not self._vehicle_id:
            rospy.logerr("Parameter %s must be a non-empty string.",
                         names["vehicle_id"])
            return False
        if not self._valid_step(self._translation_step):
            rospy.logerr("Parameter %s must be a finite positive number.",
                         names["translation_step"])
            return False
        if not self._valid_step(self._rotation_step):
            rospy.logerr("Parameter %s must be a finite positive number.",
                         names["rotation_step"])
            return False
        self._translation_step = float(self._translation_step)
        self._rotation_step = float(self._rotation_step)

        try:
            rospy.wait_for_service(SCENE_SERVICE, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr(
                "Scene service %s is unavailable. Ensure simulation.launch "
                "is running.", SCENE_SERVICE,
            )
            return False
        self._service = rospy.ServiceProxy(SCENE_SERVICE, SetObjectPose)
        rospy.Subscriber(LOCK_TOPIC, Bool, self._locked_callback, queue_size=1)
        return self._select_object(self._obstacle_ids[0])

    @staticmethod
    def _valid_step(value):
        return (isinstance(value, (int, float)) and
                math.isfinite(float(value)) and float(value) > 0.0)

    def _call(self, action, object_id, x=0.0, y=0.0, yaw=0.0):
        request = SetObjectPoseRequest()
        request.action = action
        request.object_id = object_id
        request.x = x
        request.y = y
        request.yaw = yaw
        try:
            return self._service(request)
        except rospy.ServiceException as exception:
            with self._print_lock:
                print("\nService call failed: {}".format(exception), flush=True)
            return None

    def _select_object(self, object_id):
        response = self._call(SetObjectPoseRequest.QUERY_POSE, object_id)
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            with self._print_lock:
                print("\nCannot select {}: {}".format(object_id, message),
                      flush=True)
            return False
        self._selected_id = object_id
        self._x = response.x
        self._y = response.y
        self._yaw = response.yaw
        self._print_current()
        return True

    def _move(self, key):
        x, y, yaw = self._x, self._y, self._yaw
        if key == "W":
            y += self._translation_step
        elif key == "S":
            y -= self._translation_step
        elif key == "A":
            x -= self._translation_step
        elif key == "D":
            x += self._translation_step
        elif key == "R":
            yaw += self._rotation_step
        elif key == "F":
            yaw -= self._rotation_step
        response = self._call(SetObjectPoseRequest.SET_POSE,
                              self._selected_id, x, y, yaw)
        if response is None:
            return
        if not response.success:
            with self._print_lock:
                print("\nRejected: {}".format(response.message), flush=True)
            self._print_current()
            return
        self._x, self._y, self._yaw = response.x, response.y, response.yaw
        self._print_current()

    def _finish_editing(self):
        response = self._call(SetObjectPoseRequest.FINISH_EDITING,
                              self._selected_id,
                              self._x, self._y, self._yaw)
        if response is not None:
            prefix = "OK" if response.success else "Rejected"
            with self._print_lock:
                print("\n{}: {}".format(prefix, response.message), flush=True)

    def _print_scene(self):
        with self._print_lock:
            print("\nCurrent scene:")
        for index, object_id in enumerate(self._obstacle_ids, start=1):
            response = self._call(SetObjectPoseRequest.QUERY_POSE, object_id)
            if response is not None and response.success:
                with self._print_lock:
                    print("  {} ({}) : x={:.4f} y={:.4f} yaw={:.2f} deg".format(
                        index, object_id, response.x, response.y,
                        math.degrees(response.yaw)))
        response = self._call(SetObjectPoseRequest.QUERY_POSE, self._vehicle_id)
        if response is not None and response.success:
            with self._print_lock:
                print("  {}: x={:.4f} y={:.4f} yaw={:.2f} deg".format(
                    self._vehicle_id, response.x, response.y,
                    math.degrees(response.yaw)))
        self._print_current()

    def _print_help(self):
        with self._print_lock:
            print("\nW/S: +Y/-Y  A/D: -X/+X  R/F: CCW/CW")
            print("1-{}: select obstacle  V: Vehicle 1  P: print scene".format(
                len(self._obstacle_ids)))
            print("Enter: finish placement/start 3 s lock  Q: quit editor")
            print("Obstacle keys: " + "  ".join(
                "{}={}".format(index, object_id)
                for index, object_id in enumerate(self._obstacle_ids, start=1)))

    def _print_current(self):
        role = "VEHICLE" if self._selected_id == self._vehicle_id else "OBSTACLE"
        with self._print_lock:
            sys.stdout.write(
                "\r[{}] {}  x={:.4f}  y={:.4f}  yaw={:.2f} deg          ".format(
                    role, self._selected_id, self._x, self._y,
                    math.degrees(self._yaw)))
            sys.stdout.flush()

    def _locked_callback(self, message):
        if message.data and not self._last_locked:
            with self._print_lock:
                print("\nVehicle 1 start locked; the active mode will "
                      "continue automatically.", flush=True)
            self._print_current()
        self._last_locked = message.data

    def run(self):
        self._print_help()
        self._print_current()
        try:
            with TerminalMode() as terminal:
                while not rospy.is_shutdown():
                    key = terminal.read_key()
                    if key is None:
                        continue
                    if key in ("\n", "\r"):
                        self._finish_editing()
                        continue
                    key = key.upper()
                    if key == "Q":
                        break
                    if "1" <= key <= "9":
                        index = ord(key) - ord("1")
                        if index < len(self._obstacle_ids):
                            self._select_object(self._obstacle_ids[index])
                    elif key == "V":
                        self._select_object(self._vehicle_id)
                    elif key == "P":
                        self._print_scene()
                    elif key in ("W", "A", "S", "D", "R", "F"):
                        self._move(key)
        except RuntimeError as exception:
            rospy.logerr(str(exception))
            return False
        except KeyboardInterrupt:
            pass
        finally:
            print("\nKeyboard editor exited; ROS simulation remains running.",
                  flush=True)
        return True


def main():
    rospy.init_node("keyboard_scene_editor")
    editor = KeyboardSceneEditor()
    if not editor.initialize():
        return 1
    return 0 if editor.run() else 1


if __name__ == "__main__":
    sys.exit(main())
