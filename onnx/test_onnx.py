import os
import cv2
import numpy as np

def test_onnx_model():
    base_dir = os.path.dirname(__file__)
    onnx_path = os.path.join(base_dir, "classifier_2class.onnx")

    if not os.path.exists(onnx_path):
        print(f"❌ 找不到导出的模型 `{onnx_path}`，请先运行 `python3 export_onnx.py` 导出模型！")
        return

    print(f"🔍 正在加载 ONNX 模型进行独立推理测试: {onnx_path}")

    # 使用 OpenCV DNN 引擎加载导出的 ONNX 模型 (无需任何外部依赖)
    net = cv::dnn = cv2.dnn.readNetFromONNX(onnx_path)

    # 构造 32x32 假图像做单次前向推理测试
    dummy_img = np.random.randint(0, 255, (32, 32), dtype=np.uint8)

    # 预处理：1.0 / 255.0 归一化
    blob = cv2.dnn.blobFromImage(dummy_img, 1.0 / 255.0, (32, 32), (0, 0, 0))
    net.setInput(blob)

    # 前向推理
    outputs = net.forward()

    # 计算 Softmax 概率
    exp_scores = np.exp(outputs - np.max(outputs))
    probs = exp_scores / np.sum(exp_scores)

    pred_class = np.argmax(probs)
    confidence = probs[0][pred_class]

    print(f"✅ 独立推理测试成功！")
    print(f"  - 模型输出 Raw Scores: {outputs}")
    print(f"  - 预测类别 (Class ID): {pred_class}")
    print(f"  - 预测置信度 (Confidence): {confidence * 100:.2f}%")

if __name__ == "__main__":
    test_onnx_model()
