from enum import IntEnum

from rospy import loginfo
import tf
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

class HoverControllerStates(IntEnum):
	STANDBY = 0
	TRACKING = 1
	EMERGENCY = 3
    
class PlannerStates(IntEnum):
	STANDBY = 0
	TAKEOFF = 1
	TRACKING = 2
	LANDING = 3
	EMERGENCY = 4
    
class BasePlanner(object):
    """
    This class provides a skeleton to implement a trajectory planner.
    
    User needs to implement their own functions to update class variables:
        _current_pose
        _target_pose
    by using private class functions:
        _update_current_pose()
        _update_target_pose()

    Note: DO NOT TOUCH self._XXXX pirvate class variables directly
    """
    def __init__(self, z_takeoff=0.5):
        self._Z_GROUND  = 0.03  # Sadly, ground is not at ZERO...
        self._Z_MAX     = 1.50  # Optitrack cannot see that high, 
                                # and I really don't want to hit the ceil
        self._Z_TAKEOFF = z_takeoff

        # pose format: (x, y, z) (roll, pitch, yaw) in radians
        self._current_pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._target_pose = ((0.0, 0.0, self._Z_GROUND), (0.0, 0.0, 0.0))

        self.standby()
        
    # update pose
    def _update_current_pose(self, x, y, z, roll, pitch, yaw):
        self._current_pose = ((x, y, z), (roll, pitch, yaw))
    def _update_target_pose(self, x, y, z, roll, pitch, yaw):
        # Make sure Z is in range
        z = min(max(z, self._Z_GROUND), self._Z_MAX)
        # to-do: may want to add X and Y limit?


        # to-do: move Roll, Pitch, and Yaw limit to here from hover_controller, 
        #        since limiting the commands should be planner's job, not controller's?
        self._target_pose = ((x, y, z), (roll, pitch, yaw))
        
    @property
    def planner_states(self):
        """
        Read-only property
        return planner_state
        """
        return self._planner_state
        
    @property
    def controller_states(self):
        """
        Read-only property
        return controller_state
        """
        return self._controller_state
    
    # change states
    def standby(self):
        loginfo("Standby")
        self._planner_state = PlannerStates.STANDBY
        self._controller_state = HoverControllerStates.STANDBY

    def takeoff(self):
        """
        Target position = self.current XY, self._Z_TAKEOFF
        """
        loginfo("Takeoff")
        self._update_target_pose(self._current_pose[0][0],
                                 self._current_pose[0][1],
                                 self._Z_TAKEOFF,
                                 0.0,
                                 0.0,
                                 0.0)
        self._planner_state = PlannerStates.TAKEOFF
        self._controller_state = HoverControllerStates.TRACKING

    def tracking(self):
        """
        Target position = self._target_pose
        """
        loginfo("Tracking")
        self._planner_state = PlannerStates.TRACKING
        self._controller_state = HoverControllerStates.TRACKING
        
    def landing(self):
        """
        Target position = self.current XY, 0.0
        """
        loginfo("Landing")
        self._update_target_pose(self._current_pose[0][0],
                                 self._current_pose[0][1],
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0)
        self._planner_state = PlannerStates.LANDING
        self._controller_state = HoverControllerStates.TRACKING

    def emergency(self):
        loginfo("Emergency")
        self._update_target_pose(self._current_pose[0][0], self._current_pose[0][1], 0.0,
                                 0.0, 0.0, 0.0)
        self._planner_state = PlannerStates.EMERGENCY
        self._controller_state = HoverControllerStates.EMERGENCY
        
    def create_pub_target_timer_callback(self, publisher):
        """
        Standard API
        invoked by a ROS timer to send target pose to hover controller
        
        Type: geometry_msgs.Pose
        """
        planner_instance = self
        def pub_target_timer_callback(self):
            (xyz, rpy) = planner_instance._target_pose
            q = tf.transformations.quaternion_from_euler(*rpy)
            
            msg = geometry_msgs.Pose(
                geometry_msgs.Point(*xyz), 
                geometry_msgs.Quaternion(*q))
            publisher.publish(msg)
            
        return pub_target_timer_callback
        
    def create_pub_states_timer_callback(self, publisher):
        """
        Standard API
        invoked by a ROS timer to send controller_state to hover controller
        
        Type: std_msgs.UInt8
        """
        planner_instance = self
        def pub_states_timer_callback(self):
            msg = std_msgs.UInt8(planner_instance.controller_states)
            publisher.publish(msg)
           
        return pub_states_timer_callback