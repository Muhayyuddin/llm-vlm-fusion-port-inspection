#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai
import os
from typing import Dict, List, Any
from ament_index_python.packages import get_package_share_directory

class UnifiedMissionPlanner(Node):
    """
    Unified Mission Planner for USV and UAV Heterogeneous System
    
    Generates coordinated mission plans using a single LLM call and distributes
    tasks to individual USV and UAV executors with precondition-based execution.
    """
    
    def __init__(self):
        super().__init__('unified_mission_planner')
        
        # Load OpenAI API key
        self.load_openai_key()
        
        # Publishers for unified mission plan
        self.unified_mission_pub = self.create_publisher(
            String, '/unified_mission_plan', 10)
        
        # Publishers compatible with existing systems
        self.usv_mission_pub = self.create_publisher(
            String, '/target_vessel_pose', 10)
        self.uav_mission_pub = self.create_publisher(
            String, '/uav_mission_command', 10)
        self.mission_status_pub = self.create_publisher(
            String, '/mission_status', 10)
        
        # Subscribers for mission requests and status
        self.mission_request_sub = self.create_subscription(
            String, '/unified_mission_request', 
            self.mission_request_callback, 10)
        self.usv_status_sub = self.create_subscription(
            String, '/reach_subgoal', 
            self.usv_status_callback, 10)
        self.uav_status_sub = self.create_subscription(
            String, '/uav_mission_status', 
            self.uav_status_callback, 10)
        
        # Mission state
        self.current_mission = None
        self.completed_steps = set()
        self.waiting_for_preconditions = {}
        self.step_execution_status = {}
        
        self.get_logger().info("Unified Mission Planner initialized")
        
        # Auto-start demonstration mission
        self.create_timer(2.0, self.auto_start_mission)
    
    def auto_start_mission(self):
        """Auto-start a demonstration mission"""
        self.get_logger().info("🚀 Auto-starting demonstration mission...")
        
        auto_mission_request = {
            "objective": "Demonstrate coordinated USV-UAV port surveillance mission with crane inspection and safe return",
            "area": {
                "port_zone": {"x": -1400, "y": 40, "radius": 100},
                "crane_location": {"x": -1350, "y": -50},
                "home_base": {"x": -1450, "y": -16}
            },
            "robots": ["USV", "UAV"],
            "mission_type": "demonstration"
        }
        
        # Generate mission plan using LLM
        mission_plan = self.generate_mission_plan(auto_mission_request)
        
        if mission_plan:
            self.get_logger().info("✅ Mission plan generated successfully")
            print('222222222222222222222222222222222222222222222')
            self.execute_mission_plan(mission_plan)
        else:
            self.get_logger().error("❌ Failed to generate mission plan")

    def generate_mission_plan(self, mission_request):
        """Generate mission plan from request using the unified mission generator"""
        try:
            objective = mission_request.get('objective', '')
            area = mission_request.get('area', {})
            
            # Use the existing generate_unified_mission method
            mission_steps = self.generate_unified_mission(objective, area)
            
            if mission_steps:
                mission_plan = {
                    "objective": objective,
                    "area": area,
                    "plan": mission_steps,
                    "timestamp": self.get_clock().now().nanoseconds
                }
                
                # Print symbolic mission plan
                self.print_symbolic_plan(mission_plan)
                return mission_plan
        except Exception as e:
            self.get_logger().error(f"Failed to generate mission plan: {e}")
        
        return None

    def print_symbolic_plan(self, mission_plan):
        """Print the symbolic mission plan for analysis"""
        self.get_logger().info("============================================================")
        self.get_logger().info("🎯 SYMBOLIC MISSION PLAN (GPT Generated)")
        self.get_logger().info("============================================================")
        self.get_logger().info(f"📋 Objective: {mission_plan['objective']}")
        self.get_logger().info(f"🗺️  Area: {mission_plan['area']}")
        self.get_logger().info(f"📊 Total Steps: {len(mission_plan['plan'])}")
        self.get_logger().info("------------------------------------------------------------")
        
        for i, step in enumerate(mission_plan['plan']):
            robot = step.get('robot', 'Unknown')
            action = step.get('action', 'Unknown')
            theta = step.get('theta', {})
            sigma = step.get('sigma', {})
            
            # Create symbolic representation
            theta_str = self.format_params(theta)
            self.get_logger().info(f"Step {i+1:2d}: τ{i} = {action}({robot}, {{{theta_str}}}, {sigma})")
            self.get_logger().info(f"         θ{i}: {theta}")
            if sigma:
                self.get_logger().info(f"         σ{i}: {sigma}")
        
        self.get_logger().info("============================================================")

    def format_params(self, params):
        """Format parameters for symbolic display"""
        if not params:
            return ""
        
        key_params = []
        for key, value in list(params.items())[:3]:  # Limit to first 3 for readability
            if isinstance(value, dict):
                key_params.append(f"{key}={value}")
            elif isinstance(value, (int, float)):
                key_params.append(f"{key}={value}")
            else:
                key_params.append(f"{key}='{value}'")
        
        return ", ".join(key_params[:3])  # Limit to first 3 for readability

    def load_openai_key(self):
        """Load OpenAI API key from config file"""
        try:
            # Try to find the key file in the package config directory
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('unified_mission_planner')
            key_path = os.path.join(package_share, 'config', 'openai_key.json')
            
            if not os.path.exists(key_path):
                # Fallback to source directory path
                key_path = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'config', 'openai_key.json')
            
            with open(key_path, 'r') as f:
                config = json.load(f)
                openai.api_key = config['key']
                self.get_logger().info("OpenAI API key loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load OpenAI key: {e}")
            openai.api_key = "your-api-key-here"
    
    def mission_request_callback(self, msg):
        """Handle incoming mission requests"""
        try:
            request_data = json.loads(msg.data)
            mission_objective = request_data.get('objective', '')
            mission_area = request_data.get('area', {})
            
            self.get_logger().info(f"Received mission request: {mission_objective}")
            
            # Generate unified mission plan
            mission_plan = self.generate_unified_mission(mission_objective, mission_area)
            
            if mission_plan:
                self.current_mission = mission_plan
                self.execute_mission_plan(mission_plan)
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission request: {e}")
    
    def generate_unified_mission(self, objective: str, area: Dict) -> List[Dict]:
        """Generate unified mission plan using GPT-4"""
        try:
            # Create detailed system prompt for unified mission planning
            system_prompt = """You are an expert mission planner for heterogeneous autonomous systems consisting of USV (Unmanned Surface Vehicle) and UAV (Unmanned Aerial Vehicle).

            Generate a coordinated mission plan where:
            1. USV provides mobile platform and navigation
            2. UAV performs aerial reconnaissance and inspection
            3. Actions are coordinated with proper dependencies

            Available Actions:
            USV: Navigate, GoHome, Dock
            UAV: Takeoff, FlyTo, Survey, Inspect, Report, LandOnUSV

            Return ONLY a JSON array of steps with this exact format:
            [
              {
                "robot": "USV",
                "action": "Navigate", 
                "theta": {"position": {"x": -1400, "y": 40}, "speed": 2.0},
                "sigma": {}
              },
              {
                "robot": "UAV",
                "action": "Takeoff",
                "theta": {"location": "USVDeck", "altitude": 20},
                "sigma": {}
              }
            ]

            Rules:
            - θ (theta) contains control parameters
            - σ (sigma) contains sensor/data collection parameters  
            - Ensure proper sequencing (USV navigation before UAV takeoff)
            - Include realistic coordinates and parameters
            - Plan should be executable and safe"""

            user_prompt = f"""
            Mission Objective: {objective}
            
            Area Details: {area}
            
            Generate a coordinated mission plan with 8-12 steps that accomplishes this objective using both USV and UAV systems."""

            # Call OpenAI API
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=2000,
                temperature=0.7
            )
            
            # Parse response
            content = response.choices[0].message.content.strip()
            
            # Extract JSON from response
            if "```json" in content:
                json_start = content.find("```json") + 7
                json_end = content.find("```", json_start)
                json_content = content[json_start:json_end].strip()
            elif "[" in content and "]" in content:
                json_start = content.find("[")
                json_end = content.rfind("]") + 1
                json_content = content[json_start:json_end]
            else:
                json_content = content
            
            mission_steps = json.loads(json_content)
            
            # Validate and log the generated plan
            self.get_logger().info("Generated unified mission plan:")
            for i, step in enumerate(mission_steps, 1):
                robot = step.get('robot', 'Unknown')
                action = step.get('action', 'Unknown')
                theta = step.get('theta', {})
                sigma = step.get('sigma', {})
                
                # Format theta parameters for display
                theta_str = self.format_theta_for_display(theta)
                
                self.get_logger().info(f"  {i}. {action}({robot}, {theta_str}, {sigma})")
            
            return mission_steps
            
        except Exception as e:
            self.get_logger().error(f"Error generating mission plan: {e}")
            return None
    
    def format_theta_for_display(self, theta):
        """Format theta parameters for readable display"""
        if not theta:
            return "{}"
            
        formatted_parts = []
        for key, value in theta.items():
            if isinstance(value, dict):
                if 'x' in value and 'y' in value:
                    formatted_parts.append(f"'{key}': {{'x': {value['x']}, 'y': {value['y']}}}")
                else:
                    formatted_parts.append(f"'{key}': {value}")
            else:
                formatted_parts.append(f"'{key}': {value}")
        
        return "{" + ", ".join(formatted_parts) + "}"

    def execute_mission_plan(self, mission_plan):
        """Execute the generated mission plan with precondition-based coordination"""
        if not mission_plan or 'plan' not in mission_plan:
            self.get_logger().error("Invalid mission plan structure")
            return
        
        self.current_mission = mission_plan
        
        # Initialize tracking structures
        self.completed_steps = set()
        self.waiting_for_preconditions = {}
        self.step_execution_status = {}
        
        # Set up dependency tracking based on the plan
        self.setup_precondition_tracking()
        
        self.get_logger().info(f"🎯 Starting mission execution with {len(mission_plan['plan'])} steps")
        
        # Publish the complete mission plan to the coordinator
        plan_msg = String()
        plan_msg.data = json.dumps(mission_plan)
        self.unified_mission_pub.publish(plan_msg)
        
        # Execute steps that have no preconditions immediately
        self.execute_ready_steps()
    
    def setup_precondition_tracking(self):
        """Set up precondition tracking based on dependency relationships"""
        plan = self.current_mission['plan']
        
        for i, step in enumerate(plan):
            robot = step.get('robot', '')
            action = step.get('action', '')
            self.step_execution_status[i] = 'pending'
            
            # Determine dependencies based on robot coordination logic
            dependencies = []
            
            # UAV actions depend on USV positioning (except Report actions)
            if robot == "UAV" and action in ["Takeoff", "FlyTo", "Survey", "Inspect", "LandOnUSV"]:
                # Find the most recent USV Navigate action
                for j in range(i-1, -1, -1):
                    if plan[j].get('robot') == 'USV' and plan[j].get('action') == 'Navigate':
                        dependencies.append(j)
                        break
            
            # Sequential UAV actions (each UAV step depends on previous UAV step)
            if robot == "UAV" and action != "Report":
                for j in range(i-1, -1, -1):
                    if plan[j].get('robot') == 'UAV' and plan[j].get('action') != "Report":
                        dependencies.append(j)
                        break
            
            # USV GoHome depends on UAV completion
            if robot == "USV" and action == "GoHome":
                # Find the last UAV action
                for j in range(len(plan)-1, -1, -1):
                    if plan[j].get('robot') == 'UAV':
                        dependencies.append(j)
                        break
            
            if dependencies:
                self.waiting_for_preconditions[i] = dependencies
                self.get_logger().info(f"📋 Step {i}: {robot}-{action} waits for steps {dependencies}")
            else:
                self.get_logger().info(f"🟢 Step {i}: {robot}-{action} ready to execute (no dependencies)")
    
    def execute_ready_steps(self):
        """Execute all steps that have their preconditions satisfied"""
        if not self.current_mission:
            return
            
        plan = self.current_mission['plan']
        executed_any = False
        
        for i, step in enumerate(plan):
            if self.step_execution_status[i] == 'pending':
                # Check if all preconditions are met
                dependencies = self.waiting_for_preconditions.get(i, [])
                if all(dep in self.completed_steps for dep in dependencies):
                    # Execute this step
                    robot = step.get('robot', 'Unknown')
                    action = step.get('action', 'Unknown')
                    
                    self.get_logger().info(f"✅ Executing Step {i + 1}: {robot} - {action}")
                    self.step_execution_status[i] = 'executing'
                    
                    if robot == "USV":
                        self.send_usv_command(step)
                    elif robot == "UAV":
                        self.send_uav_command(step)
                    
                    executed_any = True
        
        if not executed_any:
            # Check if mission is complete
            if all(status == 'completed' for status in self.step_execution_status.values()):
                self.get_logger().info("🎉 Mission execution completed!")
            else:
                # Log what we're waiting for
                waiting_steps = [i for i, status in self.step_execution_status.items() if status == 'pending']
                if waiting_steps:
                    for step_idx in waiting_steps[:3]:  # Show first 3 waiting steps
                        deps = self.waiting_for_preconditions.get(step_idx, [])
                        pending_deps = [d for d in deps if d not in self.completed_steps]
                        step = plan[step_idx]
                        self.get_logger().info(f"⏳ Step {step_idx + 1} ({step.get('robot')}-{step.get('action')}) waiting for steps {pending_deps}")
    
    def mark_step_completed(self, step_index):
        """Mark a step as completed and execute any ready dependent steps"""
        if step_index in self.step_execution_status:
            self.step_execution_status[step_index] = 'completed'
            self.completed_steps.add(step_index)
            
            step = self.current_mission['plan'][step_index]
            robot = step.get('robot', 'Unknown')
            action = step.get('action', 'Unknown')
            
            self.get_logger().info(f"✅ Step {step_index + 1} completed: {robot} - {action}")
            
            # Execute any steps that are now ready
            self.execute_ready_steps()
    
    def get_executing_step_index(self, robot, action=None):
        """Find the index of the currently executing step for a robot"""
        if not self.current_mission:
            return None
            
        plan = self.current_mission['plan']
        for i, step in enumerate(plan):
            if (self.step_execution_status.get(i) == 'executing' and 
                step.get('robot') == robot):
                if action is None or step.get('action') == action:
                    return i
        return None
    
    def send_usv_command(self, step):
        """Send command to USV navigator in compatible format"""
        try:
            action = step.get('action', '')
            params = step.get('theta', {})
            
            if action in ['Navigate', 'GoHome', 'Dock']:
                position = params.get('position', {'x': 0, 'y': 0})
                speed = params.get('speed', 2.0)
                
                # Send multiple copies to ensure delivery (as per original implementation)
                for i in range(5):
                    message = String()
                    message.data = f'move_to Location: {{"x": {position["x"]}, "y": {position["y"]}, "z": 1.57}}'
                    self.usv_mission_pub.publish(message)
                    self.get_logger().info(f"Published USV message {i+1}/5: Action: move_to, Location: ({position['x']}, {position['y']})")
                
                self.get_logger().info(f"🚢 USV {action}: x={position['x']}, y={position['y']}, speed={speed}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending USV command: {e}")
    
    def send_uav_command(self, step):
        """Send command to UAV mission executor"""
        try:
            action = step.get('action', '')
            theta = step.get('theta', {})
            sigma = step.get('sigma', {})
            
            if action == 'Takeoff':
                altitude = theta.get('altitude', 20)
                command_str = f"takeoff_to(0, 0, {altitude})"
                
            elif action == 'FlyTo':
                position = theta.get('position', {'x': 0, 'y': 0})
                altitude = theta.get('altitude', 20)
                command_str = f"fly_to({position['x']}, {position['y']}, {altitude})"
                
            elif action == 'Survey':
                pattern = theta.get('pattern', 'Orbit360')
                altitude = theta.get('altitude', 20)
                duration = theta.get('duration', 600)
                command_str = f"survey_area({pattern}, {altitude}, {duration})"
                
            elif action == 'Inspect':
                position = theta.get('position', {'x': 0, 'y': 0})
                altitude = theta.get('altitude', 10)
                duration = theta.get('duration', 300)
                command_str = f"inspect_target({position['x']}, {position['y']}, {altitude}, {duration})"
                
            elif action == 'Report':
                duration = sigma.get('duration', 30)
                command_str = f"record_data({duration})"
                
            elif action == 'LandOnUSV':
                command_str = "land_on_usv()"
                
            else:
                command_str = f"{action.lower()}()"
            
            self.get_logger().info(f"🚁 UAV Command: {command_str}")
            
            # Publish to UAV mission executor
            message = String()
            message.data = json.dumps({
                'action': action,
                'theta': theta,
                'sigma': sigma,
                'command': command_str,
                'timestamp': self.get_clock().now().nanoseconds
            })
            self.uav_mission_pub.publish(message)
            
        except Exception as e:
            self.get_logger().error(f"Error sending UAV command: {e}")

    def usv_status_callback(self, msg):
        """Handle USV status updates and mark steps as completed"""
        if "Trigger next goal" in msg.data or "reached" in msg.data.lower():
            # Check if we have an active mission
            if not self.current_mission or not hasattr(self, 'step_execution_status'):
                self.get_logger().debug(f"🚢 USV status received but no active mission: {msg.data}")
                return
                
            # Find the currently executing USV step
            step_index = self.get_executing_step_index("USV")
            if step_index is not None:
                self.get_logger().info(f"🚢 USV step {step_index + 1} completed: {msg.data}")
                self.mark_step_completed(step_index)
            else:
                # Check if there are any USV steps in executing state
                executing_steps = [i for i, status in self.step_execution_status.items() 
                                 if status == 'executing']
                usv_executing = [i for i in executing_steps 
                               if self.current_mission['plan'][i].get('robot') == 'USV']
                
                if usv_executing:
                    self.get_logger().info(f"🚢 USV status received for steps {usv_executing}: {msg.data}")
                else:
                    self.get_logger().debug(f"🚢 USV status received but no USV step executing: {msg.data}")
    
    def uav_status_callback(self, msg):
        """Handle UAV status updates and mark steps as completed"""
        try:
            # Try to parse as JSON first
            status = json.loads(msg.data)
            if status.get('status') == 'completed' or status.get('status') == 'mission_complete':
                step_index = self.get_executing_step_index("UAV")
                if step_index is not None:
                    self.get_logger().info(f"🚁 UAV step {step_index + 1} completed: {status}")
                    self.mark_step_completed(step_index)
        except json.JSONDecodeError:
            # Handle string-based status messages
            if any(keyword in msg.data.lower() for keyword in ["completed", "finished", "reached", "landed"]):
                step_index = self.get_executing_step_index("UAV")
                if step_index is not None:
                    self.get_logger().info(f"🚁 UAV step {step_index + 1} completed: {msg.data}")
                    self.mark_step_completed(step_index)
        except Exception as e:
            self.get_logger().warn(f"Error processing UAV status: {e}")

def main(args=None):
    rclpy.init(args=args)
    planner = UnifiedMissionPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
