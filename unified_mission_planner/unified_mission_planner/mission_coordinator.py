#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, List, Set

class MissionCoordinator(Node):
    """
    Mission Coordinator for managing dependencies and execution order
    with precondition-based execution instead of timer-based delays
    """
    
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Publishers for robot commands
        self.usv_cmd_pub = self.create_publisher(String, '/target_vessel_pose', 10)
        self.uav_cmd_pub = self.create_publisher(String, '/uav_mission_command', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)
        
        # Subscribers for mission plans and status
        self.unified_mission_sub = self.create_subscription(
            String, '/unified_mission_plan', 
            self.unified_mission_callback, 10)
        self.usv_status_sub = self.create_subscription(
            String, '/reach_subgoal', 
            self.usv_status_callback, 10)
        self.uav_status_sub = self.create_subscription(
            String, '/uav_mission_status', 
            self.uav_status_callback, 10)
        
        # Mission execution state with precondition tracking
        self.current_mission = None
        self.dependency_graph = {}
        self.completed_steps: Set[int] = set()
        self.executing_steps: Set[int] = set()
        self.current_step = 0
        self.usv_status = "idle"
        self.uav_status = "idle"
        
        # Timer for progress monitoring
        self.progress_timer = self.create_timer(5.0, self.publish_progress)
        
        self.get_logger().info("Mission Coordinator initialized")
    
    def unified_mission_callback(self, msg):
        """Handle incoming unified mission plans"""
        try:
            mission_data = json.loads(msg.data)
            self.current_mission = mission_data
            self.completed_steps = set()
            self.executing_steps = set()
            self.current_step = 0
            
            # Build dependency graph
            self.build_dependency_graph()
            self.print_dependency_analysis()
            
            # Start mission execution
            self.get_logger().info(f"Received mission plan with {len(mission_data['plan'])} steps")
            self.get_logger().info("Starting mission execution")
            self.execute_ready_steps()
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission plan: {e}")
    
    def build_dependency_graph(self):
        """Build dependency graph based on robot types and action sequences"""
        if not self.current_mission:
            return
        
        plan = self.current_mission['plan']
        self.dependency_graph = {}
        
        # Initialize dependencies
        for i in range(len(plan)):
            self.dependency_graph[i] = []
        
        # Build dependencies based on robot coordination rules
        usv_steps = []
        uav_steps = []
        
        for i, step in enumerate(plan):
            robot = step.get('robot', '')
            if robot == 'USV':
                usv_steps.append(i)
            elif robot == 'UAV':
                uav_steps.append(i)
        
        # UAV steps depend on previous UAV steps (sequential execution)
        for i in range(1, len(uav_steps)):
            self.dependency_graph[uav_steps[i]].append(uav_steps[i-1])
        
        # UAV takeoff depends on USV navigation (spatial coordination)
        if uav_steps and usv_steps:
            for uav_step in uav_steps:
                action = plan[uav_step].get('action', '')
                if action == 'Takeoff':
                    # Takeoff depends on first USV navigation
                    self.dependency_graph[uav_step].append(usv_steps[0])
                    break
        
        # USV return home depends on UAV landing (causal dependency)
        for usv_step in usv_steps[1:]:  # Skip first USV step
            action = plan[usv_step].get('action', '')
            if action == 'GoHome':
                # GoHome depends on UAV completion
                if uav_steps:
                    self.dependency_graph[usv_step].append(uav_steps[-1])
        
        self.get_logger().info(f"Built dependency graph: {self.dependency_graph}")
    
    def print_dependency_analysis(self):
        """Print detailed dependency analysis"""
        if not self.current_mission:
            return
        
        plan = self.current_mission['plan']
        
        self.get_logger().info("============================================================")
        self.get_logger().info("🔗 DEPENDENCY GRAPH ANALYSIS")
        self.get_logger().info("============================================================")
        
        for i, step in enumerate(plan):
            robot = step.get('robot', 'Unknown')
            action = step.get('action', 'Unknown')
            dependencies = self.dependency_graph.get(i, [])
            
            if not dependencies:
                self.get_logger().info(f"Step {i:2d}: τ{i} = {action}({robot}) (No dependencies)")
            else:
                self.get_logger().info(f"Step {i:2d}: τ{i} = {action}({robot})")
                self.get_logger().info(f"         Preconditions: P(τ{i}) = {{τ{', τ'.join(map(str, dependencies))}}}")
                
                # Classify dependency types
                dep_types = []
                for dep in dependencies:
                    dep_step = plan[dep]
                    if dep_step.get('robot') != robot:
                        dep_types.append(f"P_spatial(τ{dep})")
                    else:
                        dep_types.append(f"P_seq(τ{dep})")
                
                self.get_logger().info(f"         Types: {dep_types}")
        
        self.get_logger().info("----------------------------------------")
        self.get_logger().info("📊 EXECUTION ORDER ANALYSIS:")
        
        # Find steps with no dependencies
        ready_steps = [i for i, deps in self.dependency_graph.items() if not deps]
        if ready_steps:
            ready_actions = [plan[i].get('action', '') for i in ready_steps]
            self.get_logger().info(f"🟢 Ready to execute: {', '.join([f'τ{i}({action})' for i, action in zip(ready_steps, ready_actions)])}")
        
        # Calculate maximum dependency depth
        max_depth = 0
        for i in range(len(plan)):
            depth = self.calculate_dependency_depth(i, set())
            max_depth = max(max_depth, depth)
        
        self.get_logger().info(f"📏 Maximum dependency depth: {max_depth}")
        self.get_logger().info("============================================================")
    
    def calculate_dependency_depth(self, step_index: int, visited: Set[int]) -> int:
        """Calculate the dependency depth for a step"""
        if step_index in visited:
            return 0  # Avoid cycles
        
        visited.add(step_index)
        dependencies = self.dependency_graph.get(step_index, [])
        
        if not dependencies:
            return 0
        
        max_depth = 0
        for dep in dependencies:
            depth = self.calculate_dependency_depth(dep, visited.copy())
            max_depth = max(max_depth, depth)
        
        return max_depth + 1
    
    def execute_ready_steps(self):
        """Execute all steps that have their preconditions satisfied"""
        if not self.current_mission:
            return
        
        plan = self.current_mission['plan']
        executed_any = False
        
        for i, step in enumerate(plan):
            if i not in self.completed_steps and i not in self.executing_steps:
                # Check if all dependencies are satisfied
                dependencies = self.dependency_graph.get(i, [])
                if all(dep in self.completed_steps for dep in dependencies):
                    # Execute this step
                    self.execute_step(i)
                    executed_any = True
                    break  # Execute one step at a time for proper coordination
        
        if not executed_any:
            # Check if mission is complete
            if len(self.completed_steps) == len(plan):
                self.get_logger().info("🎉 Mission execution completed!")
                status_msg = String()
                status_msg.data = json.dumps({
                    "status": "mission_complete", 
                    "timestamp": time.time(),
                    "completed_steps": len(self.completed_steps),
                    "total_steps": len(plan)
                })
                self.mission_status_pub.publish(status_msg)
            else:
                # Log what we're waiting for
                waiting_steps = [i for i in range(len(plan)) 
                               if i not in self.completed_steps and i not in self.executing_steps]
                if waiting_steps:
                    for step_idx in waiting_steps[:2]:  # Show first 2 waiting steps
                        deps = self.dependency_graph.get(step_idx, [])
                        pending_deps = [d for d in deps if d not in self.completed_steps]
                        if pending_deps:
                            step = plan[step_idx]
                            self.get_logger().info(f"⏳ Step {step_idx + 1} ({step.get('robot')}-{step.get('action')}) waiting for steps {pending_deps}")
    
    def execute_step(self, step_index: int):
        """Execute a specific step"""
        if not self.current_mission:
            return
        
        plan = self.current_mission['plan']
        step = plan[step_index]
        
        robot = step.get('robot', '')
        action = step.get('action', '')
        theta = step.get('theta', {})
        sigma = step.get('sigma', {})
        
        self.get_logger().info(f"Executing step {step_index}: {action}({robot})")
        self.executing_steps.add(step_index)
        
        if robot == 'USV':
            self.execute_usv_step(step_index, action, theta, sigma)
        elif robot == 'UAV':
            self.execute_uav_step(step_index, action, theta, sigma)
    
    def execute_usv_step(self, step_index: int, action: str, theta: Dict, sigma: Dict):
        """Execute USV mission step"""
        if action == 'Navigate':
            position = theta.get('position', {'x': 0, 'y': 0})
            message = String()
            message.data = f'move_to Location: {{"x": {position["x"]}, "y": {position["y"]}, "z": 0.0}}'
            self.usv_cmd_pub.publish(message)
            
            self.get_logger().info(f"USV Navigate to ({position['x']}, {position['y']}, 0.0)")
            
        elif action == 'GoHome':
            position = theta.get('position', {'x': 0, 'y': 0})
            message = String()
            message.data = f'move_to Location: {{"x": {position["x"]}, "y": {position["y"]}, "z": 0.0}}'
            self.usv_cmd_pub.publish(message)
            
            self.get_logger().info(f"USV Go Home to ({position['x']}, {position['y']}, 0.0)")
            
        elif action == 'Dock':
            # Handle docking procedure
            dock_position = theta.get('position', {'x': 0, 'y': 0})
            message = String()
            message.data = f'move_to Location: {{"x": {dock_position["x"]}, "y": {dock_position["y"]}, "z": 0.0}}'
            self.usv_cmd_pub.publish(message)
            
            self.get_logger().info(f"USV Docking at ({dock_position['x']}, {dock_position['y']})")
        
        # Mark as in progress
        self.usv_status = f"executing_step_{step_index}"
    
    def execute_uav_step(self, step_index: int, action: str, theta: Dict, sigma: Dict):
        """Execute UAV mission step"""
        command = {
            'step_index': step_index,
            'action': action,
            'theta': theta,
            'sigma': sigma,
            'timestamp': self.get_clock().now().nanoseconds  # Fixed: use nanoseconds for JSON serialization
        }
        
        message = String()
        message.data = json.dumps(command)
        self.uav_cmd_pub.publish(message)
        
        self.get_logger().info(f"UAV {action} command sent: {theta}")
        
        # Mark as in progress
        self.uav_status = f"executing_step_{step_index}"
    
    def usv_status_callback(self, msg):
        """Handle USV status updates - wait for completion before next step"""
        if "Trigger next goal" in msg.data or "reached" in msg.data.lower():
            # Extract step index from status if available
            step_index = self.extract_step_index_from_status(self.usv_status)
            if step_index is not None and step_index in self.executing_steps:
                self.completed_steps.add(step_index)
                self.executing_steps.remove(step_index)
                self.get_logger().info(f"✅ USV completed step {step_index}: {msg.data}")
                self.usv_status = "idle"
                
                # Execute any steps that are now ready
                self.execute_ready_steps()
            else:
                self.get_logger().debug(f"🚢 USV status: {msg.data} (no matching executing step)")
    
    def uav_status_callback(self, msg):
        """Handle UAV status updates - wait for completion before next step"""
        try:
            status = json.loads(msg.data)
            if status.get('status') == 'completed':
                step_index = status.get('step_index')
                if step_index is not None and step_index in self.executing_steps:
                    self.completed_steps.add(step_index)
                    self.executing_steps.remove(step_index)
                    self.get_logger().info(f"✅ UAV completed step {step_index}: {status}")
                    self.uav_status = "idle"
                    
                    # Execute any steps that are now ready
                    self.execute_ready_steps()
        except json.JSONDecodeError:
            # Handle simple status messages
            if "completed" in msg.data.lower() or "finished" in msg.data.lower():
                step_index = self.extract_step_index_from_status(self.uav_status)
                if step_index is not None and step_index in self.executing_steps:
                    self.completed_steps.add(step_index)
                    self.executing_steps.remove(step_index)
                    self.get_logger().info(f"✅ UAV completed step {step_index}: {msg.data}")
                    self.uav_status = "idle"
                    
                    # Execute any steps that are now ready
                    self.execute_ready_steps()
    
    def extract_step_index_from_status(self, status: str) -> int:
        """Extract step index from status string"""
        try:
            if "executing_step_" in status:
                return int(status.split("executing_step_")[1])
        except (ValueError, IndexError):
            pass
        return None
    
    def publish_progress(self):
        """Publish mission progress"""
        if not self.current_mission:
            return
        
        total_steps = len(self.current_mission['plan'])
        completed = len(self.completed_steps)
        executing = len(self.executing_steps)
        progress = (completed / total_steps) * 100 if total_steps > 0 else 0
        
        status_msg = String()
        status_msg.data = json.dumps({
            "progress": progress,
            "completed_steps": completed,
            "executing_steps": executing,
            "total_steps": total_steps,
            "current_usv_status": self.usv_status,
            "current_uav_status": self.uav_status,
            "timestamp": time.time()
        })
        self.mission_status_pub.publish(status_msg)
        
        if total_steps > 0:
            self.get_logger().info(f"📊 Mission Progress: {progress:.1f}% ({completed}/{total_steps} completed, {executing} executing)")


def main(args=None):
    rclpy.init(args=args)
    coordinator = MissionCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
