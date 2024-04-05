from ultralytics import YOLO

# model = YOLO('hand_detection/best_cups_2024_1_26.pt')
model = YOLO('/home/ws/src/CV_test/2024_400_pict.pt')

results = model(source=0, show=True, conf = 0.4)