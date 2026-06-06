from ultralytics import YOLO

model = YOLO("yolo11n.pt") #change to your model file name
model.export(format="onnx", opset=12, simplify=True, dynamic=False, end2end=False)
