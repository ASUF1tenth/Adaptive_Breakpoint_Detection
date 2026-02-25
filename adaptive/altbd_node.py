#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class ALTBDNode(Node):
    def __init__(self):
        super().__init__('altbd_node')
        
        # ROS Parameters
        self.declare_parameter('sigma_offset', 0.04) # Average error percentage
        self.declare_parameter('dth_corner', 0.23)   # Tolerance value for corner limit (meters)
        
        self.sigma_offset = self.get_parameter('sigma_offset').value
        self.dth_corner = self.get_parameter('dth_corner').value

        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.corner_pub = self.create_publisher(MarkerArray, '/corners', 10)
        
        self.get_logger().info("ALTBD Perception Node Initialized.")

    def scan_callback(self, msg):
        """
        Process the raw LiDAR scan and extract corner breakpoints.
        """
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Filter out infinite/invalid ranges
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        # Convert polar to Cartesian coordinates (x, y)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.vstack((x, y)).T
        
        # Run the ALTBD algorithm
        corners = self.detect_breakpoints(points, ranges)
        
        # Visualize the results
        self.publish_markers(corners, msg.header.frame_id)

    def detect_breakpoints(self, points, ranges):
        """
        Adaptive Line Tracking Breakpoint Detector (ALTBD) core logic.
        """
        corners = []
        n = len(points)
        
        if n < 4:
            return corners

        i = 2 # Start from the third point
        
        while i < n - 1:
            # TODO 1: Determine the linear regression equation from three starting points (p_{n-2}, p_{n-1}, p_n)
            # Transform it into a predictive line F'_n(x)
            
            # TODO 2: Determine the difference between the starting points and the predicted line
            
            # TODO 3: Calculate the percentage error relative to the distance to the sensor (r_n)
            # e_n = |j_n| / r_n
            
            # TODO 4: Determine the minor axis limit (hmax)
            # hmax = average_error_percentage + sigma_offset
            
            # TODO 5: Calculate the prediction point (p'_{n+1}) and major axis limit (qmax)
            
            # Get the next data point
            p_next = points[i+1]
            
            # TODO 6: Transform p_next to new coordinates with the predicted point as the origin (0,0)
            # Calculate k_{n+1} and j_{n+1} using sine and cosine of the angle
            
            # TODO 7: Evaluate the ellipse equation
            # e_result = (k_next**2 / qmax) + (j_next**2 / hmax)
            
            e_result = 0.0 # Replace with your calculated e_result
            
            if e_result > 1.0:
                # Breakpoint detected!
                # TODO 8: Determine whether the position is within the tolerance range of the corner (Dth_corner)
                is_corner = False # Replace with your logic
                
                if is_corner:
                    # Transform back to original coordinates and append to corners list
                    # corners.append(corner_point)
                    
                    # Reset indices to start a new line segment
                    i += 3 
                    continue
                else:
                    # Not a corner, just a regular breakpoint (e.g., edge of an obstacle)
                    # Reset to use the next three points as a new starting line
                    i += 1
            else:
                # Point is within the ellipse (belongs to the same line)
                # TODO 9: Recalculate error percentage and minor axis limit for the next iteration
                i += 1
                
        return corners

    def publish_markers(self, corners, frame_id):
        """
        Publishes detected corners as spherical markers in RViz.
        """
        marker_array = MarkerArray()
        
        for idx, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "altbd_corners"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = corner[0]
            marker.pose.position.y = corner[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            
        self.corner_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ALTBDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
