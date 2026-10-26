import os
import cv2
import numpy as np
from mvsdk import MindVisionCamera

def main():
    base_dir = os.path.dirname(__file__)
    # 真实自瞄系统使用的 9 分类官方权重 assets/tiny_resnet.onnx
    real_onnx_path = os.path.join(base_dir, "..", "assets", "tiny_resnet.onnx")

    print("=" * 60)
    print("      真实自瞄官方 ONNX 分类器 (assets/tiny_resnet.onnx) 实时测试")
    print("=" * 60)

    if not os.path.exists(real_onnx_path):
        print(f"❌ 找不到官方模型文件: {real_onnx_path}")
        return

    print(f"🔍 成功加载真实自瞄 ONNX 模型: {real_onnx_path}")
    net = cv2.dnn.readNetFromONNX(real_onnx_path)

    # 官方自瞄支持的 9 个类别映射表 (定义于 armor.hpp)
    class_names = [
        "1 Hero (1号英雄)",
        "2 Engineer (2号工程)",
        "3 Infantry (3号步兵)",
        "4 Infantry (4号步兵)",
        "5 Infantry (5号步兵)",
        "Sentry (哨兵)",
        "Outpost (前哨站)",
        "Base (基地)",
        "Not Armor (非装甲板)"
    ]

    CONFIDENCE_THRESHOLD = 0.75  # 75% 置信度过滤阈值

    # 初始化相机
    cam_type = "mvsdk"
    mv_cam = None
    cv_cap = None

    try:
        mv_cam = MindVisionCamera(exposure_ms=2.5, gamma=0.5)
        mv_cam.open()
        print("🎥 [成功] 启用了 迈德威视 (MindVision) 工业相机！")
    except Exception as e:
        print(f"ℹ️ 未启动 MindVision 驱动 ({e})，尝试切换标准 USB 相机...")
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

    window_name = "Real Auto-Aim ONNX Verifier (assets/tiny_resnet.onnx)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print("\n运行控制:")
    print(f"  - 实时使用官方 9 分类模型进行推理测试 (置信度阈值: {CONFIDENCE_THRESHOLD*100:.0f}%)")
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

            # 保持 ROI 框尺寸 (480x480 像素)
            roi_size = 480
            cx, cy = w // 2, h // 2
            x1 = max(cx - roi_size // 2, 0)
            y1 = max(cy - roi_size // 2, 0)
            x2 = min(cx + roi_size // 2, w)
            y2 = min(cy + roi_size // 2, h)

            roi_bgr = frame[y1:y2, x1:x2]
            display_frame = frame.copy()

            pred_class = 8  # 默认 not_armor
            confidence = 0.0

            if roi_bgr.shape[0] > 0 and roi_bgr.shape[1] > 0:
                # 与真实 C++ Classifier 保持完全相同的预处理 (32x32 灰度图 + 1/255.0 归一化)
                gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                resized_gray = cv2.resize(gray_roi, (32, 32))

                blob = cv2.dnn.blobFromImage(resized_gray, 1.0 / 255.0, (32, 32), (0, 0, 0))
                net.setInput(blob)
                outputs = net.forward()  # 输出 9 维 Vector

                # Softmax 处理
                exp_scores = np.exp(outputs - np.max(outputs))
                probs = exp_scores / np.sum(exp_scores)

                pred_class = int(np.argmax(probs))
                confidence = float(probs[0][pred_class])

            # 结果过滤与绘制 (置信度过滤: < 75% 或者识别为 not_armor 则显示灰色框)
            if confidence >= CONFIDENCE_THRESHOLD and pred_class != 8:
                box_color = (0, 255, 0)  # 绿色合格
                pred_text = f"Class: {class_names[pred_class]} | Conf: {confidence*100:.1f}%"
            else:
                box_color = (128, 128, 128)  # 灰色过滤
                pred_text = f"No Armor (Conf < {CONFIDENCE_THRESHOLD*100:.0f}%, Raw: {class_names[pred_class]} {confidence*100:.1f}%)"

            cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(display_frame, (cx, cy), 3, (0, 0, 255), -1)

            # 渲染预测结果文本
            cv2.putText(display_frame, pred_text, (x1, max(y1 - 10, 25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

            info_text = f"Real Model: assets/tiny_resnet.onnx | Thresh: {CONFIDENCE_THRESHOLD*100:.0f}%"
            cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(display_frame, "Press Q or ESC to Quit", (10, h - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 右下角画出 32x32 灰度 ROI 预览
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
                print("\n👋 退出真实自瞄 ONNX 测试程序。")
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
