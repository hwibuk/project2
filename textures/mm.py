import cv2

import numpy as np

# 치수 설정 (1cm = 2픽셀 스케일)

scale = 10

width_px = int(360 * scale)    # 720 px

height_px = int(726.5 * scale) # 1453 px



# 1. 배경 생성 (회색 바닥)

img = np.ones((height_px, width_px, 3), dtype=np.uint8) * 160



# 색상 정의 (BGR 순서)

YELLOW = (0, 242, 255)  # 노란색 차선

RED = (0, 0, 255)       # 빨간색 구획/정지선

BLACK = (0, 0, 0)       # 검은색 횡단보도



# 선 두께 (2.5cm = 5픽셀)

thickness = int(2.5 * scale)



# 2. 도로 위치 계산 (중앙 정렬 및 도면 치수 반영)

start_x = int(130 * scale)

road_w_px = int(126.6 * scale)

end_x = start_x + road_w_px



lane_w_px = int(42.2 * scale)

lane1_x = start_x + lane_w_px

lane2_x = start_x + 2 * lane_w_px



top_road_bottom = int(87 * scale)

lane_h_top = int(45 * scale)



# 3. 노란색 차선 그리기

cv2.line(img, (0, 0), (width_px, 0), YELLOW, thickness)

cv2.line(img, (0, lane_h_top), (width_px, lane_h_top), YELLOW, thickness)

cv2.line(img, (0, top_road_bottom), (start_x, top_road_bottom), YELLOW, thickness)

cv2.line(img, (end_x, top_road_bottom), (width_px, top_road_bottom), YELLOW, thickness)



cv2.line(img, (start_x, top_road_bottom), (start_x, height_px), YELLOW, thickness)

cv2.line(img, (lane1_x, top_road_bottom), (lane1_x, height_px), YELLOW, thickness)

cv2.line(img, (lane2_x, top_road_bottom), (lane2_x, height_px), YELLOW, thickness)

cv2.line(img, (end_x, top_road_bottom), (end_x, height_px), YELLOW, thickness)



# 4. 횡단보도 영역 계산 및 검은색 칸 그리기

crosswalk_bottom = height_px - int(316 * scale)

crosswalk_top = crosswalk_bottom - int(36.5 * scale)



# 횡단보도 내부 차선 지우기

img[crosswalk_top:crosswalk_bottom, start_x+thickness:end_x-thickness] = 160



# 검은색 침목 그리기

stripe_w = int(6 * scale)

for x in range(start_x + int(4*scale), end_x - int(4*scale), stripe_w * 2):

    cv2.rectangle(img, (x, crosswalk_top), (x + stripe_w, crosswalk_bottom), BLACK, -1)



# 5. 빨간색 라인들 추가

cv2.line(img, (thickness, 0), (thickness, top_road_bottom), RED, thickness)

cv2.line(img, (int(85*scale), 0), (int(85*scale), top_road_bottom), RED, thickness)

cv2.line(img, (width_px - int(45*scale), 0), (width_px - int(45*scale), top_road_bottom), RED, thickness)

cv2.line(img, (width_px - int(90*scale), 0), (width_px - int(90*scale), top_road_bottom), RED, thickness)



cv2.line(img, (start_x, top_road_bottom + int(20*scale)), (end_x, top_road_bottom + int(20*scale)), RED, thickness)

cv2.line(img, (start_x, crosswalk_bottom + int(50*scale)), (end_x, crosswalk_bottom + int(50*scale)), RED, thickness)

cv2.line(img, (start_x, height_px - thickness), (end_x, height_px - thickness), RED, thickness)



# 6. 우측 분기 도로 레이아웃

right_branch_top = top_road_bottom + int(150 * scale)

right_branch_bottom = right_branch_top + int(42.2 * scale)

cv2.line(img, (end_x, right_branch_top), (width_px, right_branch_top), YELLOW, thickness)

cv2.line(img, (end_x, right_branch_bottom), (width_px, right_branch_bottom), YELLOW, thickness)

img[right_branch_top+thickness:right_branch_bottom-thickness, end_x-thickness:end_x+thickness] = 160



# 7. 이미지 파일로 최종 저장

output_name = 't_junction_colored_map.png'

cv2.imwrite(output_name, img)

print(f"[{output_name}] 파일이 성공적으로 생성되었습니다!")
