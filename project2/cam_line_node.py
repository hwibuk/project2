#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.state_sub  = self.create_subscription(String, '/control_state', self.state_callback, 10)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_pub = self.create_publisher(
            CompressedImage, '/image_line/compressed', best_effort_qos)

        self.bridge = CvBridge()
        self.lane_width            = 350
        self.last_valid_target_x   = None
        self.crosswalk_detected    = False
        self.current_control_state = "CRUISE"
        self.last_sent_cmd         = ""

        # 디버그 이미지 스레드
        self.debug_frame = None
        self.frame_lock  = threading.Lock()
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()

    def _display_loop(self):
        while rclpy.ok():
            with self.frame_lock:
                frame = self.debug_frame
            if frame is not None:
                cv2.imshow("Lane Follower Debug", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        cv2.destroyAllWindows()

    def state_callback(self, msg):
        try:
            self.current_control_state, self.last_sent_cmd = msg.data.split('|')
        except ValueError:
            pass

    def remove_crosswalk(self, mask_black):
        result = mask_black.copy()
        removed_rows = 0
        for y in range(mask_black.shape[0]):
            row = mask_black[y, :]
            transitions = np.diff(row.astype(int))
            runs = int(np.sum(transitions > 0))
            if runs >= 3:
                result[y, :] = 0
                removed_rows += 1
        return result, removed_rows

    def estimate_lanes_by_fitting(self, mask, min_pts=40):
        """
        검출된 차선 픽셀들의 기하학적 기울기(각도)를 분석하여 
        '/' 형태는 좌측 차선, '\' 형태는 우측 차선으로 자동 분류한 뒤 직선 피팅합니다.
        """
        h, w = mask.shape
        
        left_lane_pts = []
        right_lane_pts = []

        # 1. 픽셀 조각(컨투어)들을 찾아 각 조각의 고유 기울기를 개별 분석
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 30:  # 미세 노이즈 제거
                continue
                
            if len(cnt) >= 5:
                # 컨투어에 가장 잘 들어맞는 직선 벡터 추출 [vx, vy, x0, y0]
                [vx, vy, _, _] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
                if float(vy) == 0:
                    continue
                slope = float(vx) / float(vy)  # dx/dy 구하기 (x = m*y + c 꼴 분석용)
            else:
                # 점의 개수가 부족할 경우 양 끝점의 좌표 차이로 단순 기울기 계산
                pt_start = cnt[0][0]
                pt_end = cnt[-1][0]
                dx = pt_end[0] - pt_start[0]
                dy = pt_end[1] - pt_start[1]
                if dy == 0:
                    continue
                slope = dx / dy

            # 2. 이미지 좌표계 기준 기울기 부호 필터링
            # '/' 형태: y가 작아질 때(위로 갈 때) x가 커지므로 기울기(dx/dy)는 '음수'
            # '\' 형태: y가 작아질 때(위로 갈 때) x도 작아지므로 기울기(dx/dy)는 '양수'
            # (수평에 가까운 과도한 기울기는 노이즈 예방을 위해 상한선 제한 처리)
            if -5 < slope < -0.1:
                for pt in cnt:
                    left_lane_pts.append([pt[0][1], pt[0][0]])  # [y, x] 형태로 매핑
            elif 0.1 < slope < 3:
                for pt in cnt:
                    right_lane_pts.append([pt[0][1], pt[0][0]])  # [y, x] 형태로 매핑

        left_pts = np.array(left_lane_pts)
        right_pts = np.array(right_lane_pts)

        left_lane_x = None
        right_lane_x = None
        target_y = int(h * 0.5)

        # 3. 분류된 좌측 차선 포인트 피팅
        if len(left_pts) >= min_pts:
            poly_left = np.polyfit(left_pts[:, 0], left_pts[:, 1], 1)
            left_lane_x = int(np.polyval(poly_left, target_y))
            left_func = lambda y: int(np.polyval(poly_left, y))
        else:
            left_func = None

        # 4. 분류된 우측 차선 포인트 피팅
        if len(right_pts) >= min_pts:
            poly_right = np.polyfit(right_pts[:, 0], right_pts[:, 1], 1)
            right_lane_x = int(np.polyval(poly_right, target_y))
            right_func = lambda y: int(np.polyval(poly_right, y))
        else:
            right_func = None

        # 5. 한쪽 차선 소실 시 상보적 복원 로직 (기존 유지)
        if left_lane_x is not None and right_lane_x is None:
            right_lane_x = left_lane_x + self.lane_width
            if left_func is not None:
                right_func = lambda y: left_func(y) + self.lane_width
        elif right_lane_x is not None and left_lane_x is None:
            left_lane_x = right_lane_x - self.lane_width
            if right_func is not None:
                left_func = lambda y: right_func(y) - self.lane_width

        if left_lane_x is not None and right_lane_x is not None:
            measured = right_lane_x - left_lane_x
            if 100 < measured < w:
                self.lane_width = measured

        return left_lane_x, right_lane_x, left_func, right_func

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape
            
          
            roi = cv_image.copy()
            roi_h, roi_w = height, width
            display_roi = roi.copy()
            # 사다리꼴 마스크 영역 좌표 정의
            trap_pts = np.array([
                [int(roi_w * 0.25), int(roi_h * 0.5)], 
                [int(roi_w * 0.75), int(roi_h * 0.5)], 
                [int(roi_w * 1.00), int(roi_h * 0.88)], 
                [int(roi_w * 0.00), int(roi_h * 0.88)]  
            ], dtype=np.int32)
            
            # 사다리꼴 마스크 내부(255) 생성
            trap_mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
            cv2.fillPoly(trap_mask, [trap_pts], 255)
            
            # 사다리꼴 마스크 바깥 영역(반전 마스크) 생성
            bg_mask = cv2.bitwise_not(trap_mask)
            
            # 관심영역 내부의 원본 이미지 추출
            fg = cv2.bitwise_and(roi, roi, mask=trap_mask)
            
            # 관심영역 바깥을 완전한 흰색 배경으로 전처리
            bg = np.zeros_like(roi)
            bg[:] = [255, 255, 255]
            bg = cv2.bitwise_and(bg, bg, mask=bg_mask)
            
            # 두 영역을 더해 사다리꼴 바깥이 '흰색'인 정제된 이미지 생성
            roi_cleaned = cv2.add(fg, bg)
            
            # 정제된 전처리 이미지에서 HSV 추출
            hsv = cv2.cvtColor(roi_cleaned, cv2.COLOR_BGR2HSV)

            if self.last_valid_target_x is None:
                self.last_valid_target_x = roi_w // 2

            # 검은 차선 임계값 범위 지정
            mask_yellow    = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40,  255, 255]))
            mask_black_raw = cv2.inRange(hsv, np.array([0,  0,   0  ]), np.array([180, 255, 50]))
            mask_gray      = cv2.inRange(hsv, np.array([0,  0,   50 ]), np.array([180, 50,  180]))
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])),
                cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            )

            # ── 횡단보도 제거 ──
            mask_black, removed_rows = self.remove_crosswalk(mask_black_raw)
            self.crosswalk_detected  = (removed_rows > roi_h * 0.3)
            combined_mask = cv2.bitwise_or(mask_yellow, mask_black)

            # ── 기하학적 각도 피팅 기반 차선 추정 ──
            cx_left, cx_right, left_line_func, right_line_func = self.estimate_lanes_by_fitting(combined_mask)

            # ── 장애물 분석 ──
            contours, _ = cv2.findContours(mask_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours    = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
            obstacle_detected = len(valid_contours) > 0
            obstacle_in_front = False
            largest_cnt       = None

            # 노란선 기준 회피 방향
            yellow_left  = cv2.countNonZero(mask_yellow[:, :roi_w//2])
            yellow_right = cv2.countNonZero(mask_yellow[:, roi_w//2:])
            avoid_direction = 1 if yellow_left > yellow_right else -1

            if obstacle_detected:
                largest_cnt = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(largest_cnt)
                if M['m00'] > 0:
                    obj_cx = int(M['m10'] / M['m00'])
                    if int(roi_w * 0.3) <= obj_cx <= int(roi_w * 0.7):
                        obstacle_in_front = True

            red_line_detected    = cv2.countNonZero(mask_red)    > 5000
            yellow_line_detected = cv2.countNonZero(mask_yellow) > 5000

            # ── 조향 목표 계산 ──
            if self.crosswalk_detected:
                target_x = self.last_valid_target_x
            else:
                if cx_left is not None and cx_right is not None:
                    target_x = (cx_left + cx_right) // 2
                else:
                    target_x = self.last_valid_target_x
                self.last_valid_target_x = target_x

            error = (roi_w / 2) - target_x

            # ── 제어 노드로 전송 ──
            status_msg = String()
            status_msg.data = (
                f"{error}|{1 if red_line_detected else 0}|"
                f"{1 if self.crosswalk_detected else 0}|"
                f"{1 if yellow_line_detected else 0}|"
                f"{avoid_direction}"
            )
            self.vision_pub.publish(status_msg)

            # ── 디버그 시각화 ──
            overlay = display_roi.copy()
            
            # 사다리꼴 ROI 가이드라인 주황색 선 그리기
            cv2.polylines(overlay, [trap_pts], True, (0, 165, 255), 2)
            
            # 검출된 마스크 오버레이 시각화
            overlay[mask_yellow > 0] = (0, 220, 220)
            overlay[mask_black  > 0] = (0, 255, 0)
            overlay[mask_gray   > 0] = (255, 200, 0)
            overlay[mask_red    > 0] = (0, 0, 255)

            crosswalk_removed = cv2.bitwise_and(mask_black_raw, cv2.bitwise_not(mask_black))
            overlay[crosswalk_removed > 0] = (180, 80, 255)

            # 피팅 추세선 그리기
            for y_pos in range(0, roi_h, 4):
                if left_line_func is not None:
                    xl = left_line_func(y_pos)
                    if 0 <= xl < roi_w:
                        cv2.circle(overlay, (xl, y_pos), 2, (255, 0, 255), -1)
                if right_line_func is not None:
                    xr = right_line_func(y_pos)
                    if 0 <= xr < roi_w:
                        cv2.circle(overlay, (xr, y_pos), 2, (0, 255, 255), -1)

            if largest_cnt is not None:
                x, y, w, h = cv2.boundingRect(largest_cnt)
                color = (0, 0, 255) if obstacle_in_front else (0, 200, 255)
                cv2.rectangle(overlay, (x, y), (x+w, y+h), color, 2)
                cv2.putText(overlay, "OBS" + (" FRONT" if obstacle_in_front else ""),
                            (x, max(y-5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            draw_y = roi_h - 20
            if cx_left  is not None: cv2.circle(overlay, (cx_left,  draw_y), 8, (255, 0, 255), -1)
            if cx_right is not None: cv2.circle(overlay, (cx_right, draw_y), 8, (0, 255, 255), -1)
            cv2.circle(overlay, (target_x, draw_y), 12, (0, 0, 255), -1)
            cv2.line(overlay, (roi_w//2, 0), (roi_w//2, roi_h), (180, 180, 180), 1)

            state_color = {"CRUISE": (255,255,255), "AVOID": (0,165,255), "STOP_TIMER": (0,0,255)}
            status_text = f"State:{self.current_control_state}"
            if self.crosswalk_detected:
                status_text += " [CROSSWALK]"

            cv2.putText(overlay, status_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        state_color.get(self.current_control_state, (255,255,255)), 2)
            cv2.putText(overlay, f"Cmd:{self.last_sent_cmd}", (10, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(overlay, f"RED:{'ON' if red_line_detected else 'off'}", (10, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255) if red_line_detected else (100, 100, 100), 1)
            cv2.putText(overlay, f"LW:{self.lane_width}", (10, 84),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
            avoid_text  = "AVOID: RIGHT(+1)" if avoid_direction == 1 else "AVOID: LEFT(-1)"
            avoid_color = (0, 220, 220) if avoid_direction == 1 else (255, 100, 100)
            cv2.putText(overlay, avoid_text, (10, 104),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, avoid_color, 1)

            def make_mask_vis(m, color, label):
                vis = np.zeros((m.shape[0], m.shape[1], 3), dtype=np.uint8)
                vis[m > 0] = color
                cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                return vis

            cell_w, cell_h = roi_w // 4, roi_h // 2
            m_y = cv2.resize(make_mask_vis(mask_yellow, (0, 220, 220), "YELLOW"),   (cell_w, cell_h))
            m_b = cv2.resize(make_mask_vis(mask_black,  (0, 255, 0),   "BLACK"),    (cell_w, cell_h))
            m_o = cv2.resize(make_mask_vis(mask_gray,   (255, 200, 0), "OBSTACLE"), (cell_w, cell_h))
            m_r = cv2.resize(make_mask_vis(mask_red,    (0, 0, 255),   "RED"),      (cell_w, cell_h))

            mask_row    = cv2.resize(np.hstack([m_y, m_b, m_o, m_r]), (roi_w, cell_h))
            debug_final = np.vstack([overlay, mask_row])

            # CompressedImage 퍼블리시
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp    = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_frame"
            compressed_msg.format          = "jpeg"
            compressed_msg.data            = cv2.imencode('.jpg', debug_final)[1].tobytes()
            self.image_pub.publish(compressed_msg)

            # 디버그 창 업데이트
            with self.frame_lock:
                self.debug_frame = debug_final.copy()

        except Exception as e:
            self.get_logger().error(f'image_callback 오류: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
