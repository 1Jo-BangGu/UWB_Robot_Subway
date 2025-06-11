from ultralytics import YOLO
import torch

model_path = 'fire_fall_YOLOv5_best.pt'  # 파일명 그대로 유지

try:
    model = YOLO(model_path)
    print("✅ This is a YOLOv8 model or compatible with Ultralytics YOLOv8 format.")
except Exception as e:
    print("❌ Not a YOLOv8 model. Error:")
    print(e)
