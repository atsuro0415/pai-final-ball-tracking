"""CPU fine-tuning of YOLOv8n on the auto-labelled Gazebo dataset.
usage: python3 train.py ~/ball_dataset/data.yaml"""
import sys
from ultralytics import YOLO
m = YOLO('yolov8n.pt')
m.train(data=sys.argv[1], imgsz=320, epochs=30, batch=16, device='cpu', workers=2, project='runs', name='ball')
print('best weights: runs/ball/weights/best.pt')
