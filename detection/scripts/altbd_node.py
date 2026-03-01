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

        i = 2 # Start from the third point; window = [i-2, i-1, i]
        
        while i < n - 1:

            # ------------------------------------------------------------------
            # TODO 1: Determine the linear regression equation from three
            # starting points (p_{n-2}, p_{n-1}, p_n) and transform it into a
            # predictive line F'_n(x).
            #
            # We fit  y = m*x + b  through the window using np.polyfit.
            # Near-vertical walls (x-span < 1 cm) make polyfit singular, so we
            # swap axes and fit  x = m*y + b  instead, then compute theta from
            # the recovered wall direction.
            # ------------------------------------------------------------------
            
            p0, p1, p2 = points[i-2], points[i-1], points[i]
            r0, r1, r2 = ranges[i-2], ranges[i-1], ranges[i]

            xs = np.array([p0[0], p1[0], p2[0]])
            ys = np.array([p0[1], p1[1], p2[1]])

            near_vertical = np.ptp(xs) < 0.01   # x-span < 1 cm → near-vertical wall

            if near_vertical:
                # Fit  x = m*y + b  →  wall angle  theta = atan2(1, m)
                m, b = np.polyfit(ys, xs, 1)
                theta = math.atan2(1.0, m)
                # Perpendicular distance for a near-vertical line:
                # j = (x - m*y - b) / sqrt(m^2 + 1)
                norm = math.sqrt(m * m + 1.0)
                def line_residual(px, py): return (px - m * py - b) / norm
            else:
                # Fit  y = m*x + b  →  wall angle  theta = atan(m)
                m, b = np.polyfit(xs, ys, 1)
                theta = math.atan(m)
                # Perpendicular distance for y = m*x + b:
                # j = (y - m*x - b) / sqrt(1 + m^2)
                norm = math.sqrt(1.0 + m * m)
                def line_residual(px, py): return (py - m * px - b) / norm

            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            # ------------------------------------------------------------------
            # TODO 2: Determine the difference between the starting points and
            # the predicted line F'_n(x).
            #
            # j_n is the signed orthogonal (perpendicular) distance from each
            # window point to the regression line.  A point sitting perfectly
            # on the line has j_n = 0; sensor noise pushes it slightly off.
            # ------------------------------------------------------------------
            
            j0 = line_residual(p0[0], p0[1])
            j1 = line_residual(p1[0], p1[1])
            j2 = line_residual(p2[0], p2[1])

            # ------------------------------------------------------------------
            # TODO 3: Calculate the percentage error relative to the distance
            # to the sensor (r_n):
            #   e_n = |j_n| / r_n
            #
            # Dividing by range makes the error dimensionless and adaptive:
            # a 1 cm residual at 0.5 m (2%) is noisier than the same residual
            # at 3 m (0.3%), so hmax naturally tightens when the wall is close.
            # ------------------------------------------------------------------
            
            e0 = abs(j0) / r0 if r0 > 1e-9 else 0.0
            e1 = abs(j1) / r1 if r1 > 1e-9 else 0.0
            e2 = abs(j2) / r2 if r2 > 1e-9 else 0.0

            # ------------------------------------------------------------------
            # TODO 4: Determine the minor axis limit (hmax).
            #   hmax = average_error_percentage + sigma_offset
            #
            # hmax is the ellipse's semi-minor axis — how far a point is allowed
            # to deviate laterally from the prediction line before being flagged
            # as a breakpoint.  The rolling average captures current sensor noise;
            # sigma_offset provides a fixed safety margin.
            # ------------------------------------------------------------------
            
            avg_error = (e0 + e1 + e2) / 3.0
            hmax = max(avg_error + self.sigma_offset, 1e-6)

            # ------------------------------------------------------------------
            # TODO 5: Calculate the prediction point (p'_{n+1}) and the major
            # axis limit (qmax).
            #
            # p'_{n+1} is where the prediction line F'_n(x) meets the next scan
            # ray.  The scan ray from the sensor origin at angle α is the line
            # y = tan(α)*x.  Setting equal to y = m*x + b:
            #   tan(α)*x = m*x + b  →  x_pred = b / (tan(α) - m)
            #
            # For near-vertical walls we use the x = m*y + b form instead:
            #   cot(α)*y = m*y + b  →  y_pred = b / (cot(α) - m)
            #
            # qmax is the distance from p_n to p'_{n+1} projected along the wall
            # direction — the ellipse's semi-major axis.
            # ------------------------------------------------------------------
            
            angle_next = math.atan2(points[i+1][1], points[i+1][0])

            if near_vertical:
                # Scan ray in x = cot(α)*y form
                cot_a = math.cos(angle_next) / math.sin(angle_next) \
                        if abs(math.sin(angle_next)) > 1e-9 else 1e9
                denom_pred = cot_a - m
                if abs(denom_pred) < 1e-9:   # ray parallel to wall — skip
                    i += 1
                    continue
                y_pred = b / denom_pred
                x_pred = cot_a * y_pred
            else:
                # Scan ray in y = tan(α)*x form
                tan_a = math.tan(angle_next)
                denom_pred = tan_a - m
                if abs(denom_pred) < 1e-9:   # ray parallel to wall — skip
                    i += 1
                    continue
                x_pred = b / denom_pred
                y_pred = tan_a * x_pred

            p_pred = np.array([x_pred, y_pred])

            # qmax: projection of vector (p2 → p_pred) onto the wall direction
            delta = p2 - p_pred
            qmax  = max(abs(cos_t * delta[0] + sin_t * delta[1]), 1e-6)

            # Get the next data point
            p_next = points[i+1]

            # ------------------------------------------------------------------
            # TODO 6: Transform p_next to new coordinates with the predicted
            # point as the origin (0, 0) and the prediction line as the x-axis.
            #
            # This is a standard 2D rigid rotation:
            #   k_{n+1} =  cos(θ)*Δx + sin(θ)*Δy   [along the wall]
            #   j_{n+1} = -sin(θ)*Δx + cos(θ)*Δy   [across the wall]
            #
            # After the transform, a point sitting on the prediction line has
            # j ≈ 0.  A corner or wall-end causes a large |j|.
            # ------------------------------------------------------------------
            
            dp     = p_next - p_pred
            k_next =  cos_t * dp[0] + sin_t * dp[1]
            j_next = -sin_t * dp[0] + cos_t * dp[1]

            # ------------------------------------------------------------------
            # TODO 7: Evaluate the ellipse equation.
            #   e_result = (k_{n+1}^2 / qmax^2) + (j_{n+1}^2 / hmax^2)
            #
            # This is the canonical ellipse equation (x/a)^2 + (y/b)^2 = 1
            # with semi-major axis a = qmax and semi-minor axis b = hmax.
            #   e_result <= 1 → inside  → same wall segment, keep sliding
            #   e_result >  1 → outside → BREAKPOINT detected
            #
            # Note: the denominators are squared (qmax^2, hmax^2), not linear.
            # ------------------------------------------------------------------
            
            e_result = (k_next**2 / qmax**2) + (j_next**2 / hmax**2)
            
            if e_result > 1.0:
                # Breakpoint detected!

                # --------------------------------------------------------------
                # TODO 8: Determine whether the position is within the tolerance
                # range of the corner (Dth_corner).
                #   |j_{n+1}| <= Dth_corner  →  true room corner
                #
                # A real corner means the wall turned; p_next overshot the
                # prediction line by a small lateral amount.  The actual corner
                # tip lies ON the prediction line (j = 0).  We recover it by
                # projecting back: set j = 0, keep k, rotate to original frame.
                #   corner = p_pred + k_{n+1} * (cos θ, sin θ)
                # --------------------------------------------------------------
                
                is_corner = abs(j_next) <= self.dth_corner
                
                if is_corner:
                    # Recover the corner position on the prediction line (j = 0)
                    cx = p_pred[0] + k_next * cos_t
                    cy = p_pred[1] + k_next * sin_t
                    corners.append(np.array([cx, cy]))
                    
                    # Reset indices to start a new line segment
                    i += 3 
                    continue
                else:
                    # Not a corner — wall edge, open door, or free space.
                    # Reset to use the next three points as a new starting line.
                    i += 1
            else:
                # Point is within the ellipse (belongs to the same line).

                # --------------------------------------------------------------
                # TODO 9: Recalculate error percentage and minor axis limit for
                # the next iteration.
                #
                # No explicit update is needed here — the window automatically
                # slides forward when i increments.  On the next iteration the
                # regression (TODO 1) will be recomputed from [i-1, i, i+1],
                # which includes the new point, so sigma and hmax update
                # naturally.
                # --------------------------------------------------------------
                
                i += 1
                
        return corners

    def publish_markers(self, corners, frame_id):
        """
        Publishes detected corners as spherical markers in RViz.
        """
        marker_array = MarkerArray()

        # Clear previous markers so stale detections don't persist in RViz
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for idx, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "altbd_corners"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(corner[0])
            marker.pose.position.y = float(corner[1])
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
