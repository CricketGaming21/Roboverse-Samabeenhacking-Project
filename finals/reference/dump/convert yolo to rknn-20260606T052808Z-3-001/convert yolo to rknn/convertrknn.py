from rknn.api import RKNN

# Initialize RKNN framework
rknn = RKNN()

# 1. Configure the model settings
# Adjust mean/std values based on your training normalization (0 to 255 shown here)
rknn.config(
    mean_values=[[0, 0, 0]], 
    std_values=[[255.0, 255.0, 255.0]], 
    target_platform='rk3588'  # Alternatives: 'rk3566', 'rk3568', 'rk3576'
)

# 2. Load the stripped ONNX model
print("--> Loading ONNX model")
ret = rknn.load_onnx(model='./yolo11n.onnx')
if ret != 0:
    print("Failed to load ONNX model!")
    exit(ret)

# 3. Build the RKNN graph
print("--> Building RKNN model")
# Set do_quantization=True and provide a dataset text file if you want INT8 acceleration
ret = rknn.build(do_quantization=False) 
if ret != 0:
    print("Failed to build RKNN model!")
    exit(ret)

# 4. Save the compiled binary file
print("--> Exporting RKNN model")
ret = rknn.export_rknn('./yolo11n.rknn')
if ret != 0:
    print("Failed to export RKNN file!")
    exit(ret)

print("Success! Your model is ready for the NPU.")
rknn.release()
