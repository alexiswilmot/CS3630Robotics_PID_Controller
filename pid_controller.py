import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

class PidController():
    def __init__(self):
        #TODO: tune the gains as needed
        self.linear_kp = 2
        self.linear_ki = 1
        self.linear_kd = .5

        self.angular_kp = 2
        self.angular_ki = 1
        self.angular_kd = 1

        self.old_linE = 0.0
        self.integral_linE = 0.0

        self.old_angE = 0.0
        self.integral_angE = 0.0

    def linear_controller(self, pose, goal_point):
        """
        Set the linear velocity based on the robot's current pose and goal_point.
        
        Parameters:
            pose (np.array): Current pose (x, y, theta)
            goal_point (np.array): Goal pose at the end of the trajectory (x, y)

        Returns: linear_velocity (float) 
        """
        #TODO: return the linear velocity based on the robot's current pose and goal_point. 
        # proportional control: y = K_p(e)
        # linear_error
        linE = np.linalg.norm(pose[:2] - goal_point)
        print(linE)
        prop = self.linear_kp * linE # gain * current linear error

        # integral part
        self.integral_linE += linE # add current error to the integral 
        integ = self.linear_ki * self.integral_linE

        # derivative
        deriv = self.linear_kd * (linE - self.old_linE)
        self.old_linE = linE 
        print(prop + integ + deriv)
        if (linE < .35) & (linE > .01):
            #if (prop + integ + deriv)
            if (-1.1 * (prop + integ + deriv) < .007):
                return .02
            return -1.3 * (prop+ integ + deriv)
        if linE <= .01:
            #return -1.5 * (prop + integ + deriv)
            return -.2
            #return 0
        return prop + integ + deriv

    def angular_controller(self, pose, waypoint):
        """
        Set the angular velocity based on the robot's current pose and next waypoint.
        
        Parameters:
            pose (np.array): Current pose (x, y, theta)
            waypoint (np.array): Next waypoint pose to navigate to (x, y)

        Returns: angular_velocity (float) 
        """
        #TODO: return the angular velocity based on the robot's current pose and next waypoint.
        angE = -1*(pose[2] - np.arctan2(waypoint[1] - pose[1], waypoint[0] - pose[0]))
        #print(np.arctan2(waypoint[1] - pose[1], waypoint[0] - pose[0]))
        #print(angE)
        prop = self.angular_kp * (angE)

        # integral
        self.integral_angE += angE
        integ = self.angular_ki * self.integral_angE

        # derivative part
        deriv = self.angular_kd * (angE - self.old_angE)

        # update
        self.old_angE = angE

        tot = prop + integ + deriv
        return tot

    def set_velocity(self, pose, waypoint, goal_point):
        """
        Set the linear and angular velocities based on the robot's current pose, next waypoint, 
        and goal pose.
        
        Parameters:
            pose (np.array): Current pose (x, y, theta)
            waypoint (np.array): Next waypoint pose to navigate to (x, y)
            goal_point (np.array): Goal pose at the end of the trajectory (x, y)

        Returns: np.array([linear_velocity, angular_velocity])
        """

        linear_velocity = self.linear_controller(pose, goal_point)
        angular_velocity = self.angular_controller(pose, waypoint)
        
        return np.array([linear_velocity, angular_velocity])
    
    
    