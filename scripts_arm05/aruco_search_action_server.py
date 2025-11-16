#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from arm05_sim.action import SearchAruco
from nav2_msgs.action import NavigateToPose
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import numpy as np
import time


class ArucoSearchActionServer(Node):
    def __init__(self):
        super().__init__('aruco_search_action_server')
        
        # Parameters for waypoint generation
        # Divide map into grids 
        self.declare_parameter('grid_spacing', 3.0)
        self.grid_spacing = self.get_parameter('grid_spacing').value
        
        # State variables
        self.occupancy_grid = None
        self.map_received = False
        self.coverage_path = []
        self.detected_markers = {}  # Dictionary bcs better
        
        # Action server
        self._action_server = ActionServer(
            self,
            SearchAruco,
            'search_aruco_markers',
            execute_callback=self.execute_callback
        )
        
        # Navigation client - sends goals to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to map with changed QOS (latched topic)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # Subscribe to Aruco detections
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10
        )
        
        self.get_logger().info('Aruco Search Action Server ready NOWAY!')
    
    def map_callback(self, msg):
        """Receive the map once"""
        if not self.map_received:
            self.occupancy_grid = msg
            self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')
            self.map_received = True
    
    def aruco_callback(self, msg):
        """Save detected Aruco marker IDs with their positions"""
        # msg.marker_ids is a list of ints
        # msg.poses is a list of Pose objects (same length as marker_ids)
        
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id not in self.detected_markers:
                pose = msg.poses[i]
                self.detected_markers[marker_id] = {
                    'x': pose.position.x,
                    'y': pose.position.y
                }
                self.get_logger().info(
                    f'NEW marker {marker_id} found at '
                    f'({pose.position.x:.2f}, {pose.position.y:.2f})'
                )
    
    async def execute_callback(self, goal_handle):
        """Main action execution - this runs when you send a goal"""
        self.get_logger().info('=== Starting Aruco Search ===')
        start_time = time.time()
        
        # Step 1: Wait for map
        self.get_logger().info('Step 1: Waiting for map...')
        timeout = 30.0
        wait_start = time.time()
        while not self.map_received and (time.time() - wait_start) < timeout:
            time.sleep(0.1)
        
        if not self.map_received:
            self.get_logger().error('No map received!')
            result = SearchAruco.Result()
            result.found_marker_ids = []
            result.total_search_time = 0.0
            goal_handle.abort()
            return result
        
        # Step 2: Generate waypoints from free space in map
        self.get_logger().info('Step 2: Generating waypoints...')
        self.coverage_path = self.generate_waypoints()
        self.get_logger().info(f'Generated {len(self.coverage_path)} waypoints')
        
        # Step 3: Wait for Nav2
        self.get_logger().info('Step 3: Waiting for Nav2...')
        nav2_ready = self.nav_client.wait_for_server(timeout_sec=10.0)
        if not nav2_ready:
            self.get_logger().error('Nav2 action server not available!')
            result = SearchAruco.Result()
            result.found_marker_ids = []
            result.total_search_time = 0.0
            goal_handle.abort()
            return result
        self.get_logger().info('Nav2 is ready!')
        
        # Step 4: Clear previous detections and visit each waypoint
        self.detected_markers.clear()
        self.get_logger().info('Step 4: Visiting waypoints...')
        
        for i, waypoint in enumerate(self.coverage_path):
            # Navigate to waypoint
            self.get_logger().info(f'Going to waypoint {i+1}/{len(self.coverage_path)}: ({waypoint[0]:.1f}, {waypoint[1]:.1f})')
            success = await self.go_to_point(waypoint[0], waypoint[1])
            
            if not success:
                self.get_logger().warn(f'Failed to reach waypoint {i+1}')
            
            # Publish feedback after each waypoint (markers are detected continuously by callback)
            feedback = SearchAruco.Feedback()
            feedback.waypoint_index = i + 1
            feedback.total_waypoints = len(self.coverage_path)
            feedback.currently_found_ids = list(self.detected_markers.keys())
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Markers found so far: {feedback.currently_found_ids}')
        
        # Step 5: Return results
        total_time = time.time() - start_time
        result = SearchAruco.Result()
        result.found_marker_ids = list(self.detected_markers.keys())
        result.total_search_time = total_time
        
        self.get_logger().info(f'=== Search Complete! ===')
        self.get_logger().info(f'Found {len(result.found_marker_ids)} markers: {result.found_marker_ids}')
        for marker_id, pos in self.detected_markers.items():
            self.get_logger().info(f"  Marker {marker_id}: ({pos['x']:.2f}, {pos['y']:.2f})")
        self.get_logger().info(f'Time: {total_time:.1f} seconds')
        if not self.detected_markers.items():
            self.get_logger().info('No markers found during search.')
            result = SearchAruco.Result()
            result.succeeded = False
            return result
        goal_handle.succeed()
        return result
    
    async def go_to_point(self, x, y):
        """Send navigation goal to Nav2 and wait for result"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        try:
            # Send goal and wait
            goal_handle = await self.nav_client.send_goal_async(goal_msg)
            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal to ({x:.1f}, {y:.1f}) was REJECTED by Nav2')
                return False
            
            self.get_logger().info(f'Goal accepted, navigating...')
            result = await goal_handle.get_result_async()
            
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Reached waypoint!')
                return True
            else:
                self.get_logger().warn(f'Navigation ended with status {result.status}')
                return False
        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            return False
    
    def is_free_space(self, x, y):
        """Check if point is in free space on the map"""
        if self.occupancy_grid is None:
            return False
        
        # Convert world coords to grid coords
        grid_x = int((x - self.occupancy_grid.info.origin.position.x) / self.occupancy_grid.info.resolution)
        grid_y = int((y - self.occupancy_grid.info.origin.position.y) / self.occupancy_grid.info.resolution)
        
        # Check bounds
        if grid_x < 0 or grid_x >= self.occupancy_grid.info.width or \
           grid_y < 0 or grid_y >= self.occupancy_grid.info.height:
            return False
        
        # Check if free (0 = free, 100 = occupied)
        index = grid_y * self.occupancy_grid.info.width + grid_x
        return self.occupancy_grid.data[index] == 0
    
    def generate_waypoints(self):
        """Generate grid of waypoints in free space"""
        points = []
        
        min_x = self.occupancy_grid.info.origin.position.x
        min_y = self.occupancy_grid.info.origin.position.y
        max_x = min_x + self.occupancy_grid.info.width * self.occupancy_grid.info.resolution
        max_y = min_y + self.occupancy_grid.info.height * self.occupancy_grid.info.resolution
        
        # Create grid of points
        x = min_x + self.grid_spacing
        while x < max_x:
            y = min_y + self.grid_spacing
            while y < max_y:
                if self.is_free_space(x, y):
                    points.append([x, y])
                y += self.grid_spacing
            x += self.grid_spacing
        
        # Order points to minimize travel (nearest neighbor)
        if len(points) <= 1:
            return points
        
        ordered = [points[0]]
        remaining = points[1:]
        
        while remaining:
            last = ordered[-1]
            distances = [np.hypot(p[0] - last[0], p[1] - last[1]) for p in remaining]
            nearest_idx = np.argmin(distances)
            ordered.append(remaining[nearest_idx])
            remaining.pop(nearest_idx)
        
        return ordered
    


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSearchActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
