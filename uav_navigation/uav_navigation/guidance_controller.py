"""Guidance Controller for UAV Navigation"""

import math
from geometry_msgs.msg import Twist

class GuidanceController:
    def __init__(self):
        self.max_velocity = 2.0
        self.lookahead_distance = 1.0
        
    def update_current_position(self, position):
        """Update current position for guidance calculations"""
        self.current_position = position
        
    def compute_target_velocity(self, target, current_position=None, current_orientation=None):
        """Compute target velocity using line-of-sight guidance"""
        if current_position is None:
            current_position = getattr(self, 'current_position', None)
            
        if current_position is None:
            return Twist()
            
        # Calculate direction vector to target
        dx = target.x - current_position.x
        dy = target.y - current_position.y
        dz = target.z - current_position.z
        
        # Calculate distance
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance < 0.1:  # Very close to target
            return Twist()
            
        # Normalize direction and scale by max velocity
        velocity_scale = min(self.max_velocity, distance)
        
        cmd = Twist()
        cmd.linear.x = (dx / distance) * velocity_scale
        cmd.linear.y = (dy / distance) * velocity_scale
        cmd.linear.z = (dz / distance) * velocity_scale
        cmd.angular.z = 0.0  # No yaw control for now
        
        return cmd