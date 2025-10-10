"""PID Controller for UAV Navigation"""

import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PIDController:
    def __init__(self, config_file=None):
        """
        Initialize PID Controller with parameters from config file
        """
        # Load configuration
        self._load_config(config_file)
        
        # Initialize error terms from config
        self.prev_error_x = self.config['initial_errors']['prev_error_x']
        self.prev_error_y = self.config['initial_errors']['prev_error_y']
        self.prev_error_z = self.config['initial_errors']['prev_error_z']
        self.prev_error_yaw = self.config['initial_errors']['prev_error_yaw']
        
        self.integral_x = self.config['initial_errors']['integral_x']
        self.integral_y = self.config['initial_errors']['integral_y']
        self.integral_z = self.config['initial_errors']['integral_z']
        self.integral_yaw = self.config['initial_errors']['integral_yaw']
        
        # Time tracking for derivative and integral calculations
        self.prev_time = None
        
        print(f"PID Controller initialized with config:")
        print(f"  X gains: Kp={self.kp_x}, Ki={self.ki_x}, Kd={self.kd_x}")
        print(f"  Y gains: Kp={self.kp_y}, Ki={self.ki_y}, Kd={self.kd_y}")
        print(f"  Z gains: Kp={self.kp_z}, Ki={self.ki_z}, Kd={self.kd_z}")
        print(f"  Output limits: [{self.min_output}, {self.max_output}]")

    def _load_config(self, config_file=None):
        """Load configuration from YAML file"""
        try:
            if config_file is None:
                # Default to package config file
                package_share = get_package_share_directory('uav_navigation')
                config_file = os.path.join(package_share, 'config', 'pid_params.yaml')
            
            with open(config_file, 'r') as file:
                full_config = yaml.safe_load(file)
                self.config = full_config['pid_controller']
            
            # Extract PID gains
            self.kp_x = self.config['gains_x']['kp']
            self.ki_x = self.config['gains_x']['ki']
            self.kd_x = self.config['gains_x']['kd']
            
            self.kp_y = self.config['gains_y']['kp']
            self.ki_y = self.config['gains_y']['ki']
            self.kd_y = self.config['gains_y']['kd']
            
            self.kp_z = self.config['gains_z']['kp']
            self.ki_z = self.config['gains_z']['ki']
            self.kd_z = self.config['gains_z']['kd']
            
            self.kp_yaw = self.config['gains_yaw']['kp']
            self.ki_yaw = self.config['gains_yaw']['ki']
            self.kd_yaw = self.config['gains_yaw']['kd']
            
            # Extract output limits
            self.max_output = self.config['limits']['max_output']
            self.min_output = self.config['limits']['min_output']
            
            print(f"Successfully loaded PID config from: {config_file}")
            
        except FileNotFoundError:
            print(f"Config file not found: {config_file}")
            print("Using default PID parameters")
            self._load_default_config()
        except KeyError as e:
            print(f"Missing key in config file: {e}")
            print("Using default PID parameters")
            self._load_default_config()
        except Exception as e:
            print(f"Error loading config: {e}")
            print("Using default PID parameters")
            self._load_default_config()

    def _load_default_config(self):
        """Load default configuration if config file is not available"""
        self.kp_x = 1.0
        self.ki_x = 0.1
        self.kd_x = 0.2
        
        self.kp_y = 1.0
        self.ki_y = 0.1
        self.kd_y = 0.2
        
        self.kp_z = 1.5
        self.ki_z = 0.2
        self.kd_z = 0.3
        
        self.kp_yaw = 2.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.5
        
        self.max_output = 3.0
        self.min_output = -3.0
        
        self.config = {
            'initial_errors': {
                'prev_error_x': 0.0, 'prev_error_y': 0.0, 'prev_error_z': 0.0, 'prev_error_yaw': 0.0,
                'integral_x': 0.0, 'integral_y': 0.0, 'integral_z': 0.0, 'integral_yaw': 0.0
            }
        }

    def update(self, desired_velocity, current_position=None, dt=0.1):
        """
        Update PID controller with desired velocity
        
        Args:
            desired_velocity: Target velocity (Twist message)
            current_position: Current position (optional, for future enhancements)
            dt: Time step for integration/differentiation
        """
        if hasattr(desired_velocity, 'linear'):
            # For now, pass through the desired velocity with limits
            # Future enhancement: implement full PID control with position feedback
            
            linear_x = self._limit_output(desired_velocity.linear.x)
            linear_y = self._limit_output(desired_velocity.linear.y)  
            linear_z = self._limit_output(desired_velocity.linear.z)
            angular_z = self._limit_output(getattr(desired_velocity.angular, 'z', 0.0))
            
            return {
                'linear': {
                    'x': linear_x,
                    'y': linear_y,
                    'z': linear_z
                },
                'angular': {
                    'z': angular_z
                }
            }
        else:
            return {
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'z': 0.0}
            }

    def compute_pid(self, error, prev_error, integral, kp, ki, kd, dt=0.1):
        """
        Compute PID output for a single axis
        
        Args:
            error: Current error
            prev_error: Previous error
            integral: Accumulated integral
            kp, ki, kd: PID gains
            dt: Time step
            
        Returns:
            tuple: (output, new_integral, new_prev_error)
        """
        # Proportional term
        proportional = kp * error
        
        # Integral term (with windup protection)
        integral += error * dt
        integral = max(min(integral, self.max_output/ki if ki != 0 else 1000), 
                      self.min_output/ki if ki != 0 else -1000)
        integral_term = ki * integral
        
        # Derivative term
        derivative = kd * (error - prev_error) / dt if dt > 0 else 0.0
        
        # Combine terms
        output = proportional + integral_term + derivative
        
        return self._limit_output(output), integral, error

    def reset(self):
        """Reset all error terms and integrals"""
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0
        
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0
        
        print("PID Controller reset")

    def update_gains(self, axis, kp=None, ki=None, kd=None):
        """
        Update PID gains for specific axis during runtime
        
        Args:
            axis: 'x', 'y', 'z', or 'yaw'
            kp, ki, kd: New gain values (None to keep current)
        """
        if axis == 'x':
            if kp is not None: self.kp_x = kp
            if ki is not None: self.ki_x = ki
            if kd is not None: self.kd_x = kd
        elif axis == 'y':
            if kp is not None: self.kp_y = kp
            if ki is not None: self.ki_y = ki
            if kd is not None: self.kd_y = kd
        elif axis == 'z':
            if kp is not None: self.kp_z = kp
            if ki is not None: self.ki_z = ki
            if kd is not None: self.kd_z = kd
        elif axis == 'yaw':
            if kp is not None: self.kp_yaw = kp
            if ki is not None: self.ki_yaw = ki
            if kd is not None: self.kd_yaw = kd
        
        print(f"Updated {axis} gains: Kp={getattr(self, f'kp_{axis}')}, "
              f"Ki={getattr(self, f'ki_{axis}')}, Kd={getattr(self, f'kd_{axis}')}")

    def _limit_output(self, value):
        """Limit output to min/max bounds"""
        return max(self.min_output, min(self.max_output, value))

    def get_status(self):
        """Get current PID controller status"""
        return {
            'gains': {
                'x': {'kp': self.kp_x, 'ki': self.ki_x, 'kd': self.kd_x},
                'y': {'kp': self.kp_y, 'ki': self.ki_y, 'kd': self.kd_y},
                'z': {'kp': self.kp_z, 'ki': self.ki_z, 'kd': self.kd_z},
                'yaw': {'kp': self.kp_yaw, 'ki': self.ki_yaw, 'kd': self.kd_yaw}
            },
            'limits': {'min': self.min_output, 'max': self.max_output},
            'errors': {
                'prev': {'x': self.prev_error_x, 'y': self.prev_error_y, 'z': self.prev_error_z, 'yaw': self.prev_error_yaw},
                'integral': {'x': self.integral_x, 'y': self.integral_y, 'z': self.integral_z, 'yaw': self.integral_yaw}
            }
        }