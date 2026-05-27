import cv2
import cv2.aruco as aruco
import numpy as np
import time

import rclpy
from rclpy.node import Node
from my_arm_interfaces.msg import ArucoDetection

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- 1. CAMERA INIT ---
        camera_index = 4  # Change this to match your setup!
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Could not open camera.")
            raise RuntimeError('Could not open camera')

        # --- 2. DETECTOR SETUP ---
        dict_configs = [aruco.DICT_4X4_250, aruco.DICT_5X5_250,
                        aruco.DICT_6X6_250, aruco.DICT_ARUCO_ORIGINAL]
        self.dictionaries = [aruco.Dictionary_get(d) for d in dict_configs]
        self.parameters = aruco.DetectorParameters_create()

        # --- 3. WORKSPACE & ROBOT OFFSETS ---
        self.PHYSICAL_WIDTH_CM = 15.0
        self.PHYSICAL_HEIGHT_CM = 10.0
        
        # MEASURE THESE WITH A RULER (from camera origin to robot base):
        self.ROBOT_OFFSET_X_CM = 0.0  
        self.ROBOT_OFFSET_Y_CM = 25.0 
        
        # Configurable Z Target (Matches your ESP32 safe zone)
        self.TARGET_Z_CM = 0.0 
        
        self.HALF_W_CM = self.PHYSICAL_WIDTH_CM / 2.0

        self.pts_dst = np.float32([
            [-self.HALF_W_CM, 0],                                 
            [self.HALF_W_CM, 0],                                  
            [self.HALF_W_CM, self.PHYSICAL_HEIGHT_CM],            
            [-self.HALF_W_CM, self.PHYSICAL_HEIGHT_CM]            
        ])

        self.last_valid_matrix = None
        self.last_workspace_corners = None
        self.payload_memory = {}
        self.MEMORY_TIMEOUT = 0.5

        # ROS2 publisher
        self.pub = self.create_publisher(ArucoDetection, '/aruco_detection', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info("--- MATRIX PROJECTION TRACKER ACTIVE ---")
        self.get_logger().info(f"Targeting: Offset X:{self.ROBOT_OFFSET_X_CM}cm, Y:{self.ROBOT_OFFSET_Y_CM}cm, Z:{self.TARGET_Z_CM}cm")

    def detect_across_dicts(self, image):
        all_corners, all_ids = [], []
        for dictionary in self.dictionaries:
            corners, ids, _ = aruco.detectMarkers(image, dictionary, parameters=self.parameters)
            if ids is not None:
                for idx in range(len(ids)):
                    all_corners.append(corners[idx])
                    all_ids.append(ids[idx])
        return (tuple(all_corners), np.array(all_ids)) if len(all_ids) > 0 else (None, None)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame')
            return

        current_time = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_across_dicts(gray)
        workspace_bounds = {}

        if ids is not None:
            # 1. FIND BOUNDARIES AND DRAW EDGES
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                pts = corners[i][0]

                cv2.polylines(frame, [np.int32(pts)], isClosed=True, color=(255, 150, 0), thickness=2)

                cx = int(sum([p[0] for p in pts]) / 4)
                cy = int(sum([p[1] for p in pts]) / 4)

                cv2.putText(frame, f"ID:{m_id}", (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                if m_id == 1: workspace_bounds[1] = [cx, cy]
                elif m_id == 2: workspace_bounds[2] = [cx, cy]
                elif m_id == 3: workspace_bounds[3] = [cx, cy]
                elif m_id == 4: workspace_bounds[4] = [cx, cy]

            # UPDATE PERSPECTIVE MATRIX IF ALL 4 VISIBLE
            if len(workspace_bounds) == 4:
                self.last_workspace_corners = [workspace_bounds[1], workspace_bounds[2], workspace_bounds[3], workspace_bounds[4]]
                pts_src = np.float32(self.last_workspace_corners)
                self.last_valid_matrix = cv2.getPerspectiveTransform(pts_src, self.pts_dst)

            # 2. PROCESS PAYLOADS USING THE MATRIX
            if self.last_valid_matrix is not None:
                for i in range(len(ids)):
                    m_id = int(ids[i][0])
                    if m_id in [1, 2, 3, 4]:
                        continue

                    pts = corners[i][0]
                    raw_cx = int(sum([p[0] for p in pts]) / 4)
                    raw_cy = int(sum([p[1] for p in pts]) / 4)

                    raw_pt = np.array([[[raw_cx, raw_cy]]], dtype=np.float32)
                    warped_pt = cv2.perspectiveTransform(raw_pt, self.last_valid_matrix)

                    grid_x = float(warped_pt[0][0][0])
                    grid_y = float(warped_pt[0][0][1])

                    # Calculate Robot Coordinates
                    robot_x = round(grid_x + self.ROBOT_OFFSET_X_CM, 1)
                    robot_y = round(grid_y + self.ROBOT_OFFSET_Y_CM, 1)

                    box_x = grid_x + self.HALF_W_CM
                    col = int(min(max(box_x // (self.PHYSICAL_WIDTH_CM / 3), 0), 2))
                    row = int(min(max(grid_y // (self.PHYSICAL_HEIGHT_CM / 2), 0), 1))

                    self.payload_memory[m_id] = {
                        'raw_x': raw_cx, 'raw_y': raw_cy,
                        'robot_x': robot_x, 'robot_y': robot_y,
                        'cam_grid_x': grid_x, 'cam_grid_y': grid_y,
                        'row': row, 'col': col,
                        'last_seen': current_time
                    }

                    # ---- PUBLISH ROS2 MESSAGE CONSTANTLY ----
                    msg = ArucoDetection()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera'
                    msg.marker_id = m_id
                    msg.x = float(robot_x)
                    msg.y = float(robot_y)
                    msg.z = float(self.TARGET_Z_CM) 
                    self.pub.publish(msg)

        # 3. DRAW FROM MEMORY
        ids_to_remove = []
        for m_id, data in self.payload_memory.items():
            if current_time - data['last_seen'] > self.MEMORY_TIMEOUT:
                ids_to_remove.append(m_id)
                continue

            text_box = f"Box({data['row']},{data['col']})"
            text_xy = f"R_X:{data['robot_x']}cm R_Y:{data['robot_y']}cm"

            cv2.putText(frame, text_box, (data['raw_x'] + 30, data['raw_y'] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, text_xy, (data['raw_x'] + 30, data['raw_y'] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.circle(frame, (data['raw_x'], data['raw_y']), 6, (0, 255, 0), -1)

        for m_id in ids_to_remove:
            del self.payload_memory[m_id]

        # 4. DRAW WORKSPACE RECTANGLE & CENTER ORIGIN
        if self.last_workspace_corners is not None:
            cv2.polylines(frame, [np.int32(self.last_workspace_corners)], isClosed=True, color=(255, 0, 0), thickness=2)
            pt1 = np.array(self.last_workspace_corners[0])
            pt2 = np.array(self.last_workspace_corners[1])
            origin_pt = tuple(np.int32((pt1 + pt2) / 2))
            cv2.circle(frame, origin_pt, 8, (0, 0, 255), -1)
            cv2.putText(frame, "CAM ORIGIN", (origin_pt[0] - 50, origin_pt[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Show System Status Text
        if self.last_valid_matrix is not None:
            cv2.putText(frame, "Tracking Active (Publishing)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Show all 4 corners to set Origin", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Raw Feed + Grid Coordinates", frame)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
