from rknn.api import RKNN

rknn = RKNN(verbose=True)

# CRITICAL: YOLO expects 0-1 range. 
# We divide by 255 by setting mean_values to 0 and std_values to 255.
rknn.config(
    target_platform='rk3588', # or rk3568
    optimization_level=3,
    mean_values=[[0, 0, 0]],
    std_values=[[255, 255, 255]],
    quantized_dtype='w8a8' # optional for INT8
)

rknn.load_onnx(model='./yolo11n.onnx')
rknn.build(do_quantization=False) 
rknn.export_rknn('./your_model.rknn')
rknn.release()