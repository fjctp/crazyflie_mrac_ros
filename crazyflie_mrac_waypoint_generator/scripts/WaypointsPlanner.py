from BasePlanner import * 

from rospy import loginfo
from math import pi as M_PI
import tf

from jutils.jmath import * 

def wait_for_sec(tick, last_tick, hz, sec):
    if (tick-last_tick)/hz > sec:
        return True
    return False

class WaypointsPlanner(BasePlanner):
    """
    This class implements the trajectory planner API using pre-defined waypoints.
    
    Joystick
    Button 0:   Autonomous, Landing Mode (Depends on current mode)
    Button 1:   Emergency Mode (Shutdown all motors)
    """
    def __init__(self, hz, waypoints=[], z_takeoff=0.5, wait_for_sec=10.0):
        super(WaypointsPlanner, self).__init__(z_takeoff)
        self._hz = hz
        self._wps = waypoints
        self._wait_sec = wait_for_sec
        self.reset()
    
    def reset(self):
        self._tick = 0
        self._iwps = -1
        self._last_tick = 0

    def next_wp(self):
        self._iwps = self._iwps + 1
        try:
            xyz = self._wps[self._iwps]
        except IndexError as e:
            return 0 # end of waypoints
        else:
            rpy = (0.0, 0.0 ,0.0)
            self._update_target_pose(xyz[0], xyz[1], xyz[2],
                                     rpy[0], rpy[1], rpy[2])
            loginfo('Next WP (%.4f, %.4f, %.4f)', xyz[0], xyz[1], xyz[2])
            return 1

    def create_loop_callback(self):
        def loop(self_timer):
            if self.planner_states == PlannerStates.TAKEOFF:
                if wait_for_sec(self._tick, self._last_tick, self._hz, self._wait_sec):
                    self.tracking()
            elif self.planner_states == PlannerStates.TRACKING:
                if wait_for_sec(self._tick, self._last_tick, self._hz, self._wait_sec):
                    if not self.next_wp():
                        self.landing()
                    self._last_tick = self._tick
            self._tick = self._tick + 1
            
        return loop

    def sub_joy_callback(self, joy_msg):
        
        # Controller state
        if joy_msg.buttons[0] == 1:
            self.reset()
            if self.planner_states in (PlannerStates.TRACKING, PlannerStates.TAKEOFF):
                self.landing()
            else:
                self.takeoff()

        elif joy_msg.buttons[1] == 1:
            if self.planner_states != PlannerStates.EMERGENCY:
                self.reset()
                self.emergency()
        
    def sub_vrpn_callback(self, pose_stamped_msg):
        (pitch, roll, yaw) = tf.transformations.euler_from_quaternion(
                                (pose_stamped_msg.pose.orientation.x, 
                                 pose_stamped_msg.pose.orientation.y,
                                 pose_stamped_msg.pose.orientation.z,
                                 pose_stamped_msg.pose.orientation.w))
        
        self._update_current_pose(
                    pose_stamped_msg.pose.position.x,
                    pose_stamped_msg.pose.position.y,
                    pose_stamped_msg.pose.position.z,
                    roll,
                    pitch,
                    yaw)
        
