#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import json
import openai
import os
import time
import re
from typing import Dict, List, Any, Optional
from ast import literal_eval
from ament_index_python.packages import get_package_share_directory

class HeterogeneousMissionPlanner(Node):
    """
    Heterogeneous Mission Planner for USV and UAV Coordinated Operations
    
    Generates coordinated mission plans using GPT-4 and manages execution with
    proper dependency handling and precondition-based coordination.
    
    Mathematical Framework:
    - Action Set: A = {Takeoff, FlyTo, Survey, Record, Hover, Navigate, Dock, LandOnUSV, Inspect, Report, GoHome}
    - Symbolic Action: τᵢ = aᵢ(robot, {θᵢ}, {σᵢ})
    - Dependency Graph: P(τⱼ) = P_spatial ∪ P_seq ∪ P_causal
    """
    
    def __init__(self):
        super().__init__('heterogeneous_mission_planner')
        
        # Debug mode - set to True to use debug mission file instead of GPT
        self.debug_mode = True  # Changed to True to use debug survey mission plan
        
        # Initialize OpenAI client
        self.openai_client = None
        self.load_openai_config()
        
        # Mission state
        self.current_mission = None
        self.mission_steps = []
        self.usv_steps = []
        self.uav_steps = []
        self.completed_steps = set()
        self.executing_steps = set()
        self.dependency_graph = {}
        self.step_preconditions = {}
        
        # Robot states
        self.usv_status = "idle"  # idle, navigating, recording, reached_goal
        self.uav_status = "idle"  # idle, takeoff, flying, hovering, recording, landing
        self.usv_reached_subgoal = False
        
        # Publishers for unified mission coordination
        self.unified_mission_pub = self.create_publisher(String, '/unified_mission_plan', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)
        self.dependency_graph_pub = self.create_publisher(String, '/dependency_graph', 10)
        
        # Publishers for USV (compatible with navigator.py)
        self.usv_target_pub = self.create_publisher(String, '/target_vessel_pose', 10)
        
        # Publishers for UAV (compatible with uav_mission_executor.py)
        self.uav_command_pub = self.create_publisher(String, '/uav_mission_command', 10)
        self.uav_waypoint_pub = self.create_publisher(Point, '/uav_target_waypoint', 10)
        
        # Subscribers for coordination
        self.usv_reach_sub = self.create_subscription(String, '/reach_subgoal', self.usv_reach_callback, 10)
        self.uav_complete_sub = self.create_subscription(Bool, '/uav_mission_step_complete', self.uav_complete_callback, 10)
        self.mission_request_sub = self.create_subscription(String, '/mission_request', self.mission_request_callback, 10)
        
        # Control timers
        self.coordination_timer = self.create_timer(1.0, self.coordination_loop)
        self.auto_demo_timer = self.create_timer(3.0, self.auto_demo_mission)
        
        self.get_logger().info("🤖 Heterogeneous Mission Planner initialized")
        self.get_logger().info("📡 Ready for coordinated USV-UAV missions")
        
    def load_openai_config(self):
        """Load OpenAI API configuration"""
        try:
            # Try package config first
            package_share = get_package_share_directory('unified_mission_planner')
            key_path = os.path.join(package_share, 'config', 'openai_key.json')
            
            if not os.path.exists(key_path):
                # Fallback to source directory
                key_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'openai_key.json')
            
            with open(key_path, 'r') as f:
                config = json.load(f)
                openai.api_key = config['key']
                self.get_logger().info("✅ OpenAI API key loaded successfully")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load OpenAI key: {e}")
            openai.api_key = "your-api-key-here"
    
    def load_system_prompt(self) -> str:
        """Load system prompt from config file"""
        try:
            # Try package config first
            try:
                package_share = get_package_share_directory('unified_mission_planner')
                prompt_path = os.path.join(package_share, 'config', 'heterogeneous_prompt.txt')
                self.get_logger().info(f"🔍 Trying package path: {prompt_path}")
            except Exception:
                prompt_path = None
            
            # If package path doesn't exist, try source directory with absolute path
            if not prompt_path or not os.path.exists(prompt_path):
                # Use absolute path to the source directory
                source_config_path = '/home/muhayy/mbzirc_ws/src/unified_mission_planner/config/heterogeneous_prompt.txt'
                if os.path.exists(source_config_path):
                    prompt_path = source_config_path
                    self.get_logger().info(f"🔍 Using source path: {prompt_path}")
                else:
                    # Try relative path from current file location
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    relative_path = os.path.join(current_dir, '..', '..', 'config', 'heterogeneous_prompt.txt')
                    prompt_path = os.path.normpath(relative_path)
                    self.get_logger().info(f"🔍 Trying relative path: {prompt_path}")
            
            if os.path.exists(prompt_path):
                with open(prompt_path, 'r') as f:
                    prompt_content = f.read()
                    self.get_logger().info("✅ System prompt loaded from config")
                    return prompt_content
            else:
                raise FileNotFoundError(f"Prompt file not found at {prompt_path}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load system prompt: {e}")
            # Return a basic fallback prompt
            return """You are an expert mission planner for heterogeneous USV-UAV systems. 
            Generate coordinated mission plans in JSON format with proper dependency management."""
    
    def load_debug_mission(self) -> Optional[Dict]:
        """Load debug mission plan from JSON file"""
        try:
            # Get package share directory
            package_share_directory = get_package_share_directory('unified_mission_planner')
            debug_mission_path = os.path.join(package_share_directory, '..', '..', 'src', 'unified_mission_planner', 'config', 'debug_survey_mission_plan.json')
            
            # Fallback paths for survey mission plan
            fallback_paths = [
                '/home/muhayy/mbzirc_ws/src/unified_mission_planner/config/debug_survey_mission_plan.json',
                os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'debug_survey_mission_plan.json')
            ]
            
            # Try main path first
            if not os.path.exists(debug_mission_path):
                for fallback_path in fallback_paths:
                    if os.path.exists(fallback_path):
                        debug_mission_path = fallback_path
                        break
            
            if os.path.exists(debug_mission_path):
                self.get_logger().info(f"🐛 Loading debug survey mission from: {debug_mission_path}")
                with open(debug_mission_path, 'r') as f:
                    mission_data = json.load(f)
                self.get_logger().info("✅ Debug survey mission loaded successfully")
                return mission_data
            else:
                self.get_logger().error(f"❌ Debug survey mission file not found at: {debug_mission_path}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load debug survey mission: {e}")
            return None
    
    def auto_demo_mission(self):
        """Auto-start demonstration mission"""
        if self.current_mission is None:
            self.get_logger().info("🚀 Auto-starting heterogeneous demonstration mission...")
            
            # demo_request = {
            #     "objective": "Your task is to inspect the Crane perform areal inspection via UAV.",
            #     "area": {
            #         "surveillance_zone": {"center": [-1385, -78], "radius": 70},
            #         "usv_reacheach": {"x": -1385, "y": -78, "z": 20},
            #         "crane_location": {"x": -1385, "y": -118, "z": 20},
            #         "home_base": {"x": -1450, "y": -16, "z": 0}
            #     },
            #     "robots": ["USV", "UAV"],
            #     "mission_type": "surveillance_inspection",
            #     "constraints": {
            #         "uav_takeoff_after_usv_navigation": True,
            #         "coordinate_data_collection": True,
            #         "safe_return_sequence": True
            #     }
            # }
            demo_request = {
                "objective": "Your task is to survey the container stack, perform areal survey via UAV.", #Your task is to inspect the Crane perform areal inspection via UAV.
                "area": {
                    "usv_reach": {"x": -1460, "y": -75, "z": 20},
                    "container_stack": {"x1": -1490.0, "y1": -115.0, "x2": -1435.0, "y2": -106.0, "altitude": 20.0},
                    "home_base": {"x": -1450, "y": -16, "z": 0}
                },
                "robots": ["USV", "UAV"],
                "mission_type": "perimeter_survey",
                "constraints": {
                    "uav_takeoff_after_usv_navigation": True,
                    "coordinate_data_collection": True,
                    "safe_return_sequence": True
                }
            }
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            print(demo_request)
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            mission_plan = self.generate_heterogeneous_mission(demo_request)
            if mission_plan:
                self.execute_coordinated_mission(mission_plan)
            
            # Disable auto-demo after first run
            self.auto_demo_timer.cancel()
    
    def generate_heterogeneous_mission(self, mission_request: Dict) -> Optional[Dict]:
        """Generate coordinated mission plan using GPT-4 or debug file"""
        try:
            # Check if debug mode is enabled
            if self.debug_mode:
                self.get_logger().info("🐛 Debug mode enabled - loading debug survey mission instead of calling GPT")
                debug_mission = self.load_debug_mission()
                if debug_mission:
                    self.get_logger().info("✅ Debug survey mission loaded successfully")
                    self.log_mission_plan(debug_mission)
                    return debug_mission
                else:
                    self.get_logger().warn("⚠️  Debug survey mission failed to load, falling back to GPT")
            
            # Original GPT-4 generation code
            objective = mission_request.get('objective', '')
            area = mission_request.get('area', {})
            constraints = mission_request.get('constraints', {})
            
            # Load system prompt from config file
            system_prompt = self.load_system_prompt()
            
            user_prompt = f"""
Mission Objective: {objective}

Area Configuration:
{json.dumps(area, indent=2)}

Coordination Constraints:
{json.dumps(constraints, indent=2)}
"""

            self.get_logger().info("🤖 Querying GPT-4 for heterogeneous mission plan...")
            
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=3000,
                temperature=0.7
            )
            
            response_text = response.choices[0].message.content.strip()
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            print(response_text)
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            self.get_logger().info("📋 Received GPT-4 response, parsing...")
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            print(response_text)
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            # Extract JSON from response
            try:
                json_start = response_text.find('{')
                json_end = response_text.rfind('}') + 1
                json_str = response_text[json_start:json_end]
                mission_data = json.loads(json_str)
                
                self.get_logger().info("✅ Mission plan generated successfully")
               
                self.log_mission_plan(mission_data)
                

                return mission_data
                
            except json.JSONDecodeError as e:
                self.get_logger().error(f"❌ Failed to parse JSON response: {e}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"❌ Error generating mission plan: {e}")
            return None
    
    def log_mission_plan(self, mission_data: Dict):
        """Log the generated mission plan"""
        mission_plan = mission_data.get('mission_plan', {})
        
        self.get_logger().info("🎯 === HETEROGENEOUS MISSION PLAN ===")
        self.get_logger().info(f"📋 Mission: {mission_plan.get('objective', 'Unknown')}")
        self.get_logger().info(f"🕐 Duration: {mission_plan.get('estimated_duration', 'Unknown')}")
        self.get_logger().info(f"📊 Total Steps: {mission_plan.get('total_steps', 0)}")
        
        steps = mission_plan.get('steps', [])
        self.get_logger().info("🛣️  Execution Plan:")
        
        for step in steps:
            robot = step.get('robot', 'Unknown')
            action = step.get('action', 'Unknown')
            dependencies = step.get('dependencies', [])
            description = step.get('description', '')
            
            dep_str = f" (depends on: {dependencies})" if dependencies else ""
            self.get_logger().info(f"   {step.get('step_id', 0)}: {robot} - {action}{dep_str}")
            self.get_logger().info(f"      → {description}")
        
        self.get_logger().info("=====================================")
    
    def execute_coordinated_mission(self, mission_data: Dict):
        """Execute the coordinated mission with dependency management"""
        mission_plan = mission_data.get('mission_plan', {})
        self.current_mission = mission_plan
        self.mission_steps = mission_plan.get('steps', [])
        self.dependency_graph = mission_data.get('dependency_graph', {})
        
        # Separate USV and UAV steps
        self.usv_steps = [step for step in self.mission_steps if step.get('robot') == 'USV']
        self.uav_steps = [step for step in self.mission_steps if step.get('robot') == 'UAV']
        
        # Build precondition map
        self.build_precondition_map()
        
        # Publish unified mission plan
        self.publish_unified_mission(mission_data)
        
        # Start execution with dependency-aware coordination
        self.get_logger().info("🚀 Starting coordinated mission execution...")
        self.execute_ready_steps()
    
    def build_precondition_map(self):
        """Build map of step preconditions for dependency management"""
        self.step_preconditions = {}
        
        for step in self.mission_steps:
            step_id = step.get('step_id')
            dependencies = step.get('dependencies', [])
            self.step_preconditions[step_id] = {
                'required_steps': set(dependencies),
                'robot': step.get('robot'),
                'ready': len(dependencies) == 0
            }
    
    def execute_ready_steps(self):
        """Execute all steps that have their preconditions satisfied"""
        for step in self.mission_steps:
            step_id = step.get('step_id')
            
            # Skip if already completed or executing
            if step_id in self.completed_steps or step_id in self.executing_steps:
                continue
            
            # Check if preconditions are satisfied
            preconditions = self.step_preconditions.get(step_id, {})
            required_steps = preconditions.get('required_steps', set())
            
            if required_steps.issubset(self.completed_steps):
                self.execute_step(step)
                self.executing_steps.add(step_id)
    
    def execute_step(self, step: Dict):
        """Execute a single mission step"""
        robot = step.get('robot')
        action = step.get('action')
        parameters = step.get('parameters', {})
        step_id = step.get('step_id')
        
        self.get_logger().info(f"▶️ Executing Step {step_id}: {robot} - {action}")
        
        if robot == 'USV':
            self.execute_usv_step(step)
        elif robot == 'UAV':
            self.execute_uav_step(step)
        else:
            self.get_logger().warn(f"⚠️ Unknown robot type: {robot}")
    
    def execute_usv_step(self, step: Dict):
        """Execute USV-specific mission step"""
        action = step.get('action')
        parameters = step.get('parameters', {})
        theta = parameters.get('theta', {})
        sigma = parameters.get('sigma', {})
        
        if action == 'move_to':
            # Extract coordinates from theta parameters
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 0.0)
            
            # Format message compatible with navigator.py
            location_dict = {'x': x, 'y': y, 'z': z}
            message = f"Action: move_to, Location: {location_dict}"
            
            self.usv_target_pub.publish(String(data=message))
            self.get_logger().info(f"🚢 USV move_to: ({x}, {y}, {z})")
            
        elif action == 'record_data':
            # Send record_data command
            message = "Action: record_data"
            self.usv_target_pub.publish(String(data=message))
            self.get_logger().info("📹 USV record_data command sent")
            
        elif action == 'inspect_target':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            radius = theta.get('radius', 10.0)
            
            # Send inspection coordinates
            location_dict = {'x': x, 'y': y, 'z': 0.0}
            message = f"Action: move_to, Location: {location_dict}"
            self.usv_target_pub.publish(String(data=message))
            self.get_logger().info(f"🔍 USV inspect_target: ({x}, {y}) radius: {radius}m")
            
        elif action == 'dock_at':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 0.0)
            
            location_dict = {'x': x, 'y': y, 'z': z}
            message = f"Action: move_to, Location: {location_dict}"
            self.usv_target_pub.publish(String(data=message))
            self.get_logger().info(f"⚓ USV dock_at: ({x}, {y}, {z})")
        
        # Update USV status
        self.usv_status = "executing"
    
    def execute_uav_step(self, step: Dict):
        """Execute UAV-specific mission step"""
        action = step.get('action')
        parameters = step.get('parameters', {})
        theta = parameters.get('theta', {})
        sigma = parameters.get('sigma', {})
        
        if action == 'takeoff_to':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 10.0)
            
            command = f"takeoff_to:{x},{y},{z}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"🛫 UAV takeoff_to: ({x}, {y}, {z})")
            
        elif action == 'fly_to':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 10.0)
            
            command = f"fly_to:{x},{y},{z}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"✈️ UAV fly_to: ({x}, {y}, {z})")
            
        elif action == 'hover_at':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 10.0)
            duration = theta.get('duration', 10.0)
            
            command = f"hover_at:{x},{y},{z},{duration}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"🚁 UAV hover_at: ({x}, {y}, {z}) for {duration}s")
            
        elif action == 'survey_area':
            x1 = theta.get('x1', 0.0)
            y1 = theta.get('y1', 0.0)
            x2 = theta.get('x2', 100.0)
            y2 = theta.get('y2', 100.0)
            altitude = theta.get('altitude', 15.0)
            method = sigma.get('method', 'rectangular')  # rectangular or zigzag
            
            command = f"survey_area:{x1},{y1},{x2},{y2},{altitude},{method}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"📊 UAV survey_area: ({x1},{y1}) to ({x2},{y2}) at {altitude}m using {method} pattern")
            
        elif action == 'record_data':
            duration = theta.get('duration', 5.0)
            command = f"record_data:{duration}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"📹 UAV record_data for {duration}s")
            
        elif action == 'inspect_target':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 15.0)
            radius = theta.get('radius', 20.0)
            
            command = f"inspect_target:{x},{y},{z},{radius}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"🔍 UAV inspect_target: ({x}, {y}, {z}) radius: {radius}m")
            
        elif action == 'land_at':
            x = theta.get('x', 0.0)
            y = theta.get('y', 0.0)
            z = theta.get('z', 0.0)
            
            command = f"land_at:{x},{y},{z}"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info(f"🛬 UAV land_at: ({x}, {y}, {z})")
            
        elif action == 'land_on_usv':
            # Land on USV - no parameters needed as it uses USV odometry
            command = "land_on_usv"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info("🚁🚢 UAV land_on_usv: Landing on USV using odometry tracking")
            
        elif action == 'return_home':
            command = "return_home"
            self.uav_command_pub.publish(String(data=command))
            self.get_logger().info("🏠 UAV return_home")
        
        # Update UAV status
        self.uav_status = "executing"
    
    def usv_reach_callback(self, msg):
        """Handle USV subgoal reached signal"""
        self.get_logger().info("✅ USV reached subgoal")
        self.usv_reached_subgoal = True
        self.usv_status = "reached_goal"
        
        # Mark USV steps as completed and check for next steps
        self.mark_usv_step_completed()
        self.execute_ready_steps()
    
    def uav_complete_callback(self, msg):
        """Handle UAV mission step completion"""
        if msg.data:
            self.get_logger().info("✅ UAV step completed")
            self.uav_status = "completed"
            
            # Mark UAV steps as completed and check for next steps
            self.mark_uav_step_completed()
            self.execute_ready_steps()
    
    def mark_usv_step_completed(self):
        """Mark the current executing USV step as completed"""
        for step_id in list(self.executing_steps):
            step = next((s for s in self.mission_steps if s.get('step_id') == step_id), None)
            if step and step.get('robot') == 'USV':
                self.completed_steps.add(step_id)
                self.executing_steps.remove(step_id)
                self.get_logger().info(f"✅ Marked USV step {step_id} as completed")
                break
    
    def mark_uav_step_completed(self):
        """Mark the current executing UAV step as completed"""
        for step_id in list(self.executing_steps):
            step = next((s for s in self.mission_steps if s.get('step_id') == step_id), None)
            if step and step.get('robot') == 'UAV':
                self.completed_steps.add(step_id)
                self.executing_steps.remove(step_id)
                self.get_logger().info(f"✅ Marked UAV step {step_id} as completed")
                break
    
    def coordination_loop(self):
        """Main coordination loop for mission management"""
        if not self.current_mission:
            return
        
        # Check mission completion
        total_steps = len(self.mission_steps)
        completed_count = len(self.completed_steps)
        
        if completed_count >= total_steps:
            self.get_logger().info("🎉 Heterogeneous mission completed successfully!")
            self.publish_mission_status("mission_complete")
            self.current_mission = None
            return
        
        # Publish status
        progress = f"Progress: {completed_count}/{total_steps} steps completed"
        self.publish_mission_status(progress)
        
        # Log current status
        if hasattr(self, '_last_status_log'):
            if time.time() - self._last_status_log > 10.0:  # Log every 10 seconds
                self.log_mission_status()
                self._last_status_log = time.time()
        else:
            self._last_status_log = time.time()
    
    def log_mission_status(self):
        """Log current mission execution status"""
        self.get_logger().info("📊 === MISSION STATUS ===")
        self.get_logger().info(f"🚢 USV Status: {self.usv_status}")
        self.get_logger().info(f"🚁 UAV Status: {self.uav_status}")
        self.get_logger().info(f"✅ Completed Steps: {sorted(list(self.completed_steps))}")
        self.get_logger().info(f"⏳ Executing Steps: {sorted(list(self.executing_steps))}")
        
        # Show next ready steps
        ready_steps = []
        for step in self.mission_steps:
            step_id = step.get('step_id')
            if step_id not in self.completed_steps and step_id not in self.executing_steps:
                preconditions = self.step_preconditions.get(step_id, {})
                required_steps = preconditions.get('required_steps', set())
                if required_steps.issubset(self.completed_steps):
                    ready_steps.append(step_id)
        
        self.get_logger().info(f"🎯 Ready Steps: {ready_steps}")
        self.get_logger().info("========================")
    
    def mission_request_callback(self, msg):
        """Handle external mission requests"""
        try:
            request_data = json.loads(msg.data)
            self.get_logger().info(f"📥 Received mission request: {request_data.get('objective', 'Unknown')}")
            
            mission_plan = self.generate_heterogeneous_mission(request_data)
            if mission_plan:
                self.execute_coordinated_mission(mission_plan)
            else:
                self.get_logger().error("❌ Failed to generate mission plan from request")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Invalid JSON in mission request: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing mission request: {e}")
    
    def publish_unified_mission(self, mission_data: Dict):
        """Publish the unified mission plan"""
        try:
            mission_json = json.dumps(mission_data, indent=2)
            self.unified_mission_pub.publish(String(data=mission_json))
            self.get_logger().info("📡 Published unified mission plan")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish mission plan: {e}")
    
    def publish_mission_status(self, status: str):
        """Publish mission execution status"""
        self.mission_status_pub.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = HeterogeneousMissionPlanner()
        planner.get_logger().info("🤖 Heterogeneous Mission Planner ready!")
        planner.get_logger().info("🔗 Coordinating USV-UAV operations with dependency management")
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info("🛑 Heterogeneous Mission Planner shutting down")
    except Exception as e:
        print(f"❌ Failed to start planner: {e}")
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
