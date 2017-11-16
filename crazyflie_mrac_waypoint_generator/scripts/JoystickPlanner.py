from BasePlanner import * 

from math import pi as M_PI
import tf

from jutils.jmath import * 

class JoystickPlanner(BasePlanner):
    """
    This class implements the trajectory planner API using a joystick.
    
    Joystick
    Axis 0:     X
    Axis 1:     Y
    Axis 3:     Z
    Button 0:   Standby, Takeoff, Landing (depends on current mode)
    Button 3:   Tracking Mode
    Button 1:   Emergency Mode (Shutdown all motors)
    """
    def __init__(self, x_center=0.0, y_center=0.0,
                 x_offset=0.5, y_offset=0.5,
                 z_max=1.0, z_takeoff=0.5):
        super(JoystickPlanner, self).__init__(z_takeoff)
        self._CENTER = (x_center, y_center)
        self._LIMIT = {
            "min": (x_center - x_offset, y_center - y_offset, 0.0),
            "max": (x_center + x_offset, y_center + y_offset, z_max)
        } 
        
    def sub_joy_callback(self, joy_msg):
        
        # Controller state
        if joy_msg.buttons[0] == 1:
            if self.planner_states == PlannerStates.EMERGENCY:
                self.standby()
            elif (self.planner_states in [PlannerStates.STANDBY, PlannerStates.LANDING, PlannerStates.EMERGENCY]):
                self.takeoff()
            elif self.planner_states in [PlannerStates.TAKEOFF, PlannerStates.TRACKING]:
                self.landing()
            
        elif joy_msg.buttons[3] == 1:
            if self.planner_states == PlannerStates.TAKEOFF:
                self.tracking()
                
        elif joy_msg.buttons[1] == 1:
            if self.planner_states != PlannerStates.EMERGENCY:
                self.emergency()
                
        
        # Target pose
        if self.planner_states == PlannerStates.TRACKING:
            # Position
            xyz = (
                mapping(-joy_msg.axes[0],-1, 1, self._LIMIT["min"][0], self._LIMIT["max"][0]), 
                mapping( joy_msg.axes[1],-1, 1, self._LIMIT["min"][1], self._LIMIT["max"][1]),
                mapping( joy_msg.axes[3],-1, 1, self._LIMIT["min"][2], self._LIMIT["max"][2]))

            # Orientation
            yaw = -mapping(joy_msg.axes[2],-1, 1, -120/180.0*M_PI, 120/180.0*M_PI)
            rpy = (0, 0, yaw)
            
            self._update_target_pose(xyz[0], xyz[1], xyz[2],
                                     rpy[0], rpy[1], rpy[2])
        
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
        
