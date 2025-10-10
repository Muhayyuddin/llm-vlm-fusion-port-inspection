#!/usr/bin/env python3
"""
Test script for the Unified Mission Planning System
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MissionTestNode(Node):
    def __init__(self):
        super().__init__('mission_test_node')
        
        # Publishers
        self.mission_request_pub = self.create_publisher(String, '/mission_request', 10)
        self.mission_plan_pub = self.create_publisher(String, '/unified_mission_plan', 10)
        
        # Subscribers for monitoring
        self.create_subscription(String, '/mission_coordinator/status', self.status_callback, 10)
        
        # Wait a bit for connections
        time.sleep(2.0)
        
        self.get_logger().info("Mission Test Node initialized")
        
        # Test with predefined mission
        self.test_predefined_mission()
    
    def test_predefined_mission(self):
        """Test with a predefined mission plan"""
        test_mission = {
            "objective": "Test USV-UAV coordination",
            "area": {"test_zone": {"x": 100, "y": 50}},
            "plan": [
                {
                    "action": "Navigate",
                    "robot": "USV",
                    "theta": {
                        "position": {"x": 100.0, "y": 50.0},
                        "orientation": 1.57,
                        "speed": 2.0
                    },
                    "sigma": {}
                },
                {
                    "action": "Takeoff",
                    "robot": "UAV",
                    "theta": {
                        "location": "USVDeck",
                        "altitude": 15.0,
                        "duration": 30
                    },
                    "sigma": {}
                },
                {
                    "action": "FlyTo",
                    "robot": "UAV",
                    "theta": {
                        "position": {"x": 120.0, "y": 50.0},
                        "altitude": 15.0,
                        "speed": 10.0
                    },
                    "sigma": {}
                },
                {
                    "action": "Survey",
                    "robot": "UAV",
                    "theta": {
                        "pattern": "Orbit360",
                        "center": {"x": 120.0, "y": 50.0},
                        "radius": 20.0,
                        "altitude": 15.0,
                        "duration": 180
                    },
                    "sigma": {
                        "sensors": ["camera"],
                        "detection_mode": "anomaly"
                    }
                },
                {
                    "action": "LandOnUSV",
                    "robot": "UAV",
                    "theta": {
                        "location": "USVDeck",
                        "approach_speed": 2.0
                    },
                    "sigma": {
                        "landing_sensors": ["camera"],
                        "safety_checks": True
                    }
                },
                {
                    "action": "GoHome",
                    "robot": "USV",
                    "theta": {
                        "position": {"x": 0.0, "y": 0.0},
                        "speed": 3.0,
                        "final_orientation": 0.0
                    },
                    "sigma": {}
                }
            ]
        }
        
        self.get_logger().info("🧪 Publishing test mission plan...")
        
        message = String()
        message.data = json.dumps(test_mission)
        self.mission_plan_pub.publish(message)
        
        self.get_logger().info("✅ Test mission published to /unified_mission_plan")
    
    def test_llm_mission_request(self):
        """Test LLM-generated mission (requires OpenAI API key)"""
        mission_request = {
            "objective": "Survey the port crane area for anomalies and return safely",
            "area": {
                "port_crane_zone": {"x": 1400, "y": 20, "radius": 50},
                "home_base": {"x": 0, "y": 0}
            },
            "constraints": {
                "max_altitude": 20,
                "survey_duration": 300,
                "weather": "clear"
            }
        }
        
        self.get_logger().info("🤖 Requesting LLM-generated mission...")
        
        message = String()
        message.data = json.dumps(mission_request)
        self.mission_request_pub.publish(message)
        
        self.get_logger().info("✅ Mission request sent to LLM planner")
    
    def status_callback(self, msg):
        """Monitor mission execution status"""
        try:
            status = json.loads(msg.data)
            if status['status'] == 'in_progress':
                progress = status.get('progress_percent', 0)
                self.get_logger().info(f"📊 Mission Progress: {progress:.1f}%")
            elif status['status'] == 'completed':
                self.get_logger().info("🎉 Mission completed successfully!")
        except:
            self.get_logger().info(f"Status: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    test_node = MissionTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
