import cv2
import numpy as np
from rknnlite.api import RKNNLite

# ==================== CONFIGURATION ====================
RKNN_MODEL = "your_model.rknn"  # Use the newly converted model from Step 1
IMAGE_PATH = "test.jpg"
OUTPUT_IMAGE = "output_fixed.jpg"

INPUT_SIZE = (640, 640)      
CONF_THRESH = 0.25           
NMS_THRESH = 0.45            

# Update with your custom classes if not using standard COCO 80
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat'] 
# ========================================================

def post_process_yolov8(outputs, ori_w, ori_h):
    output = outputs[0]
    
    if output.ndim == 4:
        output = np.squeeze(output, axis=0)
    elif output.ndim == 3 and output.shape[0] == 1:
        output = output[0]

    # Ensure shape is (8400, 84) instead of (84, 8400)
    if output.shape[0] < output.shape[1]:
        output = output.T

    boxes, classes, scores = [], [], []
    scale_x, scale_y = ori_w / INPUT_SIZE[0], ori_h / INPUT_SIZE[1]

    for row in output:
        class_scores = row[4:]
        class_id = np.argmax(class_scores)
        score = class_scores[class_id]

        if score > CONF_THRESH:
            cx, cy, w, h = row[0:4]

            # Convert center format to top-left corner coordinates
            x1 = int((cx - w / 2) * scale_x)
            y1 = int((cy - h / 2) * scale_y)
            w_scaled = int(w * scale_x)
            h_scaled = int(h * scale_y)

            # Prevent values from scaling outside image frame boundaries
            x1 = max(0, min(x1, ori_w - 1))
            y1 = max(0, min(y1, ori_h - 1))

            boxes.append([x1, y1, w_scaled, h_scaled])
            classes.append(class_id)
            scores.append(float(score))

    indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESH, NMS_THRESH)
    
    final_detections = []
    if len(indices) > 0:
        for idx in indices.flatten():
            x, y, w, h = boxes[idx]
            # Clip secondary coordinates safely
            x2 = max(0, min(x + w, ori_w - 1))
            y2 = max(0, min(y + h, ori_h - 1))
            final_detections.append(([x, y, x2, y2], classes[idx], scores[idx]))
            
    return final_detections

# 1. Initialize NPU
rknn = RKNNLite()
if rknn.load_rknn(RKNN_MODEL) != 0 or rknn.init_runtime() != 0:
    print("NPU Initialization Failed!"); exit(-1)

# 2. Image Pipeline
img_ori = cv2.imread(IMAGE_PATH)
if img_ori is None:
    print("Image not found."); exit(-1)
    
orig_h, orig_w = img_ori.shape[:2]

# YOLOv8 expects RGB colorspace internally
img_rgb = cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB)
img_resized = cv2.resize(img_rgb, INPUT_SIZE)
img_input = np.expand_dims(img_resized, axis=0)

# 3. Inference & Decode
outputs = rknn.inference(inputs=[img_input])
detections = post_process_yolov8(outputs, orig_w, orig_h)

# 4. Annotate Screen Output
for box, class_id, score in detections:
    x1, y1, x2, y2 = box
    class_text = CLASSES[class_id] if class_id < len(CLASSES) else f"ID_{class_id}"
    label = f"{class_text}: {score:.2f}"
    
    color = (0, 255, 0) # Clear Green Box
    cv2.rectangle(img_ori, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img_ori, label, (x1, max(y1 - 10, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

cv2.imwrite(OUTPUT_IMAGE, img_ori)
print(f"Process complete. Image written to {OUTPUT_IMAGE}")
rknn.release()
