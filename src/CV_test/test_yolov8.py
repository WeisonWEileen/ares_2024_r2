from ultralytics import YOLO

# model = YOLO('hand_detection/best_cups_2024_1_26.pt')
model = YOLO('/home/ws/yolov8m.pt')

results = model(source=0, show=True, conf = 0.4)