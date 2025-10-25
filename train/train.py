from ultralytics import YOLO

EPOCHS = 60 #30
IMG_SIZE = 640 #640
WORKERS = 2 #2
BATCH = 4 #4
YOLO_MODEL = 'yolov8n.pt' #yolo11n.pt


def main():
    print(f"Training with:\nepochs:{EPOCHS}\nimgsz:{IMG_SIZE}\nworkers:{WORKERS}\nbatch:{BATCH}\nmodel:{YOLO_MODEL}")
    model = YOLO(YOLO_MODEL)
    result = model.train(data='dataset/data.yaml', epochs=EPOCHS, imgsz=IMG_SIZE, workers=WORKERS, batch=BATCH)
if __name__=='__main__':
    main()
