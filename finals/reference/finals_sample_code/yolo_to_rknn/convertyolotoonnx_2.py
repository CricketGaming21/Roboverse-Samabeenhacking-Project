from ultralytics import YOLO

# 1. Load your trained model weights (works for YOLOv8, YOLOv9, and YOLOv11)
# Replace with 'yolov8n.pt' or your custom 'best.pt' weight file path
model = YOLO("yolov8n.pt")

# 2. Export to ONNX with specific optimization arguments for RKNN compatibility
success = model.export(
    format="onnx",
    imgsz=640,          # Match the resolution used in your RKNN test scripts
    keras=False,        # Disable Keras format formatting
    optimize=False,     # Disable TorchScript graph optimizations to prevent broken nodes
    half=False,         # Keep FP32 precision (RKNN compiler handles FP16/INT8 conversion)
    int8=False,         # Do not use PyTorch quantization
    dynamic=False,      # CRITICAL: RKNN requires FIXED static input sizes (no dynamic batch/shapes)
    simplify=True       # CRITICAL: Blends redundant math nodes together so the NPU parses it cleanly
)

if success:
    print("--> ONNX conversion successful! Your file is ready for the RKNN Toolkit.")
else:
    print("--> Export failed.")
