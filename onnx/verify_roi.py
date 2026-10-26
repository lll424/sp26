import os
import cv2
import numpy as np

def main():
    base_dir = os.path.dirname(__file__)
    onnx_path = os.path.join(base_dir, "classifier_2class.onnx")

    print("=" * 60)
    print("      独立版 2-Class ONNX 分类器实时 ROI 验证工具")
    print("=" * 60)

    # 1. 检查导出的 ONNX 模型是否存在
    if not os.path.exists(onnx_path):
        print(f"❌ 找不到导出的 ONNX 模型文件: {onnx_path}")
        print("💡 请先运行 `python3 export_onnx.py` 导出模型！")
        return

    print(f"🔍 正在加载 ONNX 模型: {onnx_path}")

    # 优先尝试 onnxruntime，若无则使用 OpenCV DNN 引擎
    ort_session = None
    cv_net = None

    try:
        import onnxruntime as ort
        ort_session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        input_name = ort_session.get_inputs()[0].name
        print("⚡ 成功加载 onnxruntime 推理引擎！")
    except Exception:
        cv_net = cv2.dnn.readNetFromONNX(onnx_path)
        print("⚡ 成功加载 OpenCV DNN 推理引擎！")

    # 2. 初始化相机：优先使用迈德威视 MindVision 工业相机，失败则降级使用普通 USB 相机
    cam_type = "uvc"
    mv_cam = None
    cv_cap = None

    try:
        from mvsdk import MindVisionCamera
        mv_cam = MindVisionCamera(exposure_ms=2.5, gamma=0.5)
        mv_cam.open()
        cam_type = "mvsdk"
        print("🎥 [成功] 启用了 迈德威视 (MindVision) 工业相机！")
    except Exception as e:
        print(f"ℹ️ 未使用 MindVision 驱动 ({e})，正在寻找标准 USB 摄像头...")
        cam_type = "uvc"
        for cam_idx in [0, 1, 4, 2]:
            temp_cap = cv2.VideoCapture(cam_idx)
            if temp_cap.isOpened():
                cv_cap = temp_cap
                print(f"🎥 成功打开标准 USB 相机 (Index: {cam_idx})")
                break

    if cam_type == "uvc" and (cv_cap is None or not cv_cap.isOpened()):
        print("\n❌ 无法找到或打开任何相机设备！")
        return

    window_name = "Realtime ONNX ROI Verifier (Press Q or ESC to Quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    # 类别名称定义
    class_names = ["class0", "class1"]
    
    # 75% 置信度过滤阈值
    CONFIDENCE_THRESHOLD = 0.75

    print("\n运行控制:")
    print(f"  - 实时针对中心 ROI 区域进行神经网络推理解算 (置信度阈值: {CONFIDENCE_THRESHOLD*100:.0f}%)")
    print("  - 【Q 键】 或 【ESC 键】：退出程序\n")

    try:
        while True:
            if cam_type == "mvsdk":
                ret, frame = mv_cam.read()
            else:
                ret, frame = cv_cap.read()

            if not ret or frame is None or frame.shape[0] <= 0 or frame.shape[1] <= 0:
                continue

            h, w = frame.shape[:2]

            # 保持与 capture_roi.py 完全一致的中心 ROI 尺寸 (480x480 像素)
            roi_size = 480
            cx, cy = w // 2, h // 2
            x1 = max(cx - roi_size // 2, 0)
            y1 = max(cy - roi_size // 2, 0)
            x2 = min(cx + roi_size // 2, w)
            y2 = min(cy + roi_size // 2, h)

            # 截取 ROI 区域
            roi_bgr = frame[y1:y2, x1:x2]
            display_frame = frame.copy()

            pred_class = 0
            confidence = 0.0

            if roi_bgr.shape[0] > 0 and roi_bgr.shape[1] > 0:
                # 预处理: 转换为 32x32 灰度图并归一化到 [0.0, 1.0]
                gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                resized_gray = cv2.resize(gray_roi, (32, 32))
                
                input_tensor = resized_gray.astype(np.float32) / 255.0
                input_tensor = np.expand_dims(input_tensor, axis=(0, 1))  # -> [1, 1, 32, 32]

                if ort_session is not None:
                    outputs = ort_session.run(None, {input_name: input_tensor})[0]
                else:
                    cv_net.setInput(input_tensor)
                    outputs = cv_net.forward()

                # 计算 Softmax 概率
                exp_scores = np.exp(outputs - np.max(outputs))
                probs = exp_scores / np.sum(exp_scores)

                pred_class = int(np.argmax(probs))
                confidence = float(probs[0][pred_class])

            # 结果可视化渲染与 75% 置信度过滤
            if confidence >= CONFIDENCE_THRESHOLD:
                # 只有置信度大于等于 75% 才认可识别结果
                box_color = (0, 255, 0) if pred_class == 0 else (0, 0, 255)
                pred_text = f"Pred: {class_names[pred_class]} | Conf: {confidence*100:.1f}%"
            else:
                # 置信度低于 75% 判定为无效/无目标，显示灰色框
                box_color = (128, 128, 128)
                pred_text = f"No Target (Conf < {CONFIDENCE_THRESHOLD*100:.0f}%, Raw: {confidence*100:.1f}%)"

            cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(display_frame, (cx, cy), 3, (0, 0, 255), -1)

            # 显示推理预测文本
            cv2.putText(display_frame, pred_text, (x1, max(y1 - 10, 25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, box_color, 2)

            info_text = f"Mode: {cam_type.upper()} | Threshold: {CONFIDENCE_THRESHOLD*100:.0f}%"
            cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(display_frame, "Press Q or ESC to Quit", (10, h - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 右下角画出当前输入的 32x32 灰度 ROI 预览图
            if roi_bgr.shape[0] > 0 and roi_bgr.shape[1] > 0:
                preview_size = min(96, h // 4)
                resized_preview = cv2.resize(gray_roi, (preview_size, preview_size))
                preview_bgr = cv2.cvtColor(resized_preview, cv2.COLOR_GRAY2BGR)
                
                py1, py2 = h - preview_size - 10, h - 10
                px1, px2 = w - preview_size - 10, w - 10
                if py1 >= 0 and px1 >= 0:
                    display_frame[py1:py2, px1:px2] = preview_bgr
                    cv2.rectangle(display_frame, (px1-1, py1-1), (px2, py2), box_color, 1)

            cv2.imshow(window_name, display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27:
                print("\n👋 退出实时验证程序。")
                break

    finally:
        if mv_cam is not None:
            try:
                mv_cam.release()
            except Exception:
                pass
        if cv_cap is not None:
            cv_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
