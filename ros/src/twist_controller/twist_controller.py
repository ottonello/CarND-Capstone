from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, params):
        self.params = params
        min_speed = 0.0

        self.linear_pid = PID(kp=0.8, ki=0, kd=0.05, mn=self.params.decel_limit, mx=self.params.accel_limit)
        self.yaw_controller = YawController(self.params.wheel_base, self.params.steer_ratio, min_speed, self.params.max_lat_accel, self.params.max_steer_angle)

        self.throttle_lpf = LowPassFilter(tau = 3, ts = 1)
        self.steer_lpf = LowPassFilter(tau = 3, ts = 1)

    def reset(self):
        self.linear_pid.reset()

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, duration_in_seconds):
        linear_velocity_error = proposed_linear_velocity - current_linear_velocity

        throttle = self.linear_pid.step(linear_velocity_error, duration_in_seconds)
        throttle = self.throttle_lpf.filt(throttle)

        steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        steering = self.steer_lpf.filt(steering)

        brake = 0
        if(throttle <= 0):
            total_mass = self.params.vehicle_mass + self.params.fuel_capacity * GAS_DENSITY
            brake = total_mass * self.params.wheel_radius * abs(throttle)
            brake = brake if brake > self.params.brake_deadband else 0.
            throttle = 0
            
        return throttle, brake, steering