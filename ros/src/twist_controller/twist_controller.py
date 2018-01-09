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

        self.linear_pid = PID(kp=0.8, ki=0, kd=0.05, mn=self.params.decel_limit, mx=0.5 * self.params.accel_limit)
        self.yaw_controller = YawController(self.params.wheel_base, self.params.steer_ratio, min_speed, self.params.max_lat_accel, self.params.max_steer_angle)
        self.steering_pid = PID(kp=0.25, ki=0.001, kd=0.5, mn=-self.params.max_steer_angle, mx=self.params.max_steer_angle)

        self.throttle_lpf = LowPassFilter(tau = 3, ts = 1)
        self.steer_lpf = LowPassFilter(tau = 3, ts = 1)

    def reset(self):
        self.linear_pid.reset()
        self.steering_pid.reset()

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, cross_track_error, duration_in_seconds):
        linear_velocity_error = proposed_linear_velocity - current_linear_velocity

        brake = 0
        throttle = self.linear_pid.step(linear_velocity_error, duration_in_seconds)
        throttle = self.throttle_lpf.filt(throttle)

        if(throttle <= 0):
            deceleration = abs(throttle)
            total_mass = self.params.vehicle_mass + self.params.fuel_capacity * GAS_DENSITY
            brake = total_mass * self.params.wheel_radius * deceleration if deceleration > self.params.brake_deadband else 0.
            throttle = 0

        predictive_steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        corrective_steering = self.steering_pid.step(cross_track_error, duration_in_seconds)
        steering = predictive_steering + corrective_steering
        steering = self.steer_lpf.filt(steering)

        return throttle, brake, steering