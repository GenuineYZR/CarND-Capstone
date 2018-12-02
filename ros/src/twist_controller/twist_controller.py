from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, vehicle_mass, decel_limit, wheel_radius):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
                
        kp = 0.3
        ki = 0.1
        kd = 0.
        MIN_NUM = 0.
        MAX_NUM = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM)
        
        tau = 0.5
        ts = 0.02
        self.velocity_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        pass

    def control(self, dbw_enable, linear_vel, angular_vel, current_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enable:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.velocity_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        velocity_error = linear_vel - current_vel
        self.last_velocity = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)
        brake = 0
        
        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 700
        elif throttle < 0.1 and velocity_error < 0:
            throttle = 0
            decel = max(velocity_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        # Return throttle, brake, steer
        return throttle, brake, steering
