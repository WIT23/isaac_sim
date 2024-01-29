import cv2 as cv

img = cv.imread("/home/gr-agv-lx91/isaac_sim_ws/src/isaac_sim/map/isaac_map.png", cv.IMREAD_GRAYSCALE)
cv.imwrite("/home/gr-agv-lx91/isaac_sim_ws/src/isaac_sim/map/isaac_map.pgm", img)
