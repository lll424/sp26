import os
import cv2
import datetime

def main():
    base_dir = os.path.dirname(__file__)
    dataset_dir = os.path.join(base_dir, "dataset")

    print("=" * 60)
    print("      独立版装甲板 ROI 自动截图与数据集采集工具")
    print("=" * 60)

    # 1. 询问保存到 train 还是 val
    while True:
        split_choice = input("\n[1/2] 请选择保存到训练集还是验证集? (1: train / 2: val) [默认: 1]: ").strip()
        if split_choice == "" or split_choice == "1":
            split = "train"
            break
        elif split_choice == "2":
            split = "val"
            break
        else:
            print("❌ 输入无效，请输入 1 或 2")

    # 2. 询问选择 class0 还是 class1
    while True:
        class_choice = input("\n[2/2] 请选择拍摄的类别? (0: class0 / 1: class1) [默认: 0]: ").strip()
        if class_choice == "" or class_choice == "0":
            class_name = "class0"
            break
        elif class_choice == "1":
            class_name = "class1"
            break
        else:
            print("❌ 输入无效，请输入 0 或 1")

    # 目标保存文件夹路径
    save_dir = os.path.join(dataset_dir, split, class_name)
    os.makedirs(save_dir, exist_ok=True)

    print("\n" + "-" * 60)
    print(f"✅ 配置完成！照片将自动切取 32x32 灰度 ROI 保存至:")
    print(f"   👉 {save_dir}")
    print("-" * 60)

    # 3. 初始化相机：安全尝试 迈德威视 SDK 相机，若不可用自动降级至标准 USB 相机
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
        print(f"ℹ️ 未使用 MindVision Python 驱动 ({e})，正在寻找标准 USB 摄像头...")
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

    saved_count = 0
    window_name = "Independent ROI Capture Tool (Space/S: Save | Q/ESC: Quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print("\n操作按键说明:")
    print("  - 【空格键 Space】 或 【S 键】：裁切中心 ROI 并保存为 32x32 灰度图")
    print("  - 【Q 键】 或 【ESC 键】：退出程序\n")

    try:
        while True:
            if cam_type == "mvsdk":
                ret, frame = mv_cam.read()
            else:
                ret, frame = cv_cap.read()

            if not ret or frame is None:
                continue

            h, w = frame.shape[:2]
            
            # 中心 ROI 框大小 (调整为 480x480 像素，即 4 倍)
            roi_size = 480
            cx, cy = w // 2, h // 2
            x1 = max(cx - roi_size // 2, 0)
            y1 = max(cy - roi_size // 2, 0)
            x2 = min(cx + roi_size // 2, w)
            y2 = min(cy + roi_size // 2, h)

            # 截取 ROI 图像
            roi_bgr = frame[y1:y2, x1:x2]
            display_frame = frame.copy()
            
            # 绘制中心 ROI 绿色框与中心原点
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(display_frame, (cx, cy), 3, (0, 0, 255), -1)

            # 状态提示信息
            info_text = f"Saving: {split}/{class_name} | Saved: {saved_count} | ROI Size: {x2-x1}x{y2-y1}"
            cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(display_frame, "Press SPACE to Save ROI | Press Q to Exit", (10, h - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 右下角实时画出 32x32 灰度 ROI 预览
            if roi_bgr.shape[0] > 0 and roi_bgr.shape[1] > 0:
                gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                preview_size = min(96, h // 4)
                resized_roi = cv2.resize(gray_roi, (preview_size, preview_size))
                resized_bgr = cv2.cvtColor(resized_roi, cv2.COLOR_GRAY2BGR)
                
                py1, py2 = h - preview_size - 10, h - 10
                px1, px2 = w - preview_size - 10, w - 10
                if py1 >= 0 and px1 >= 0:
                    display_frame[py1:py2, px1:px2] = resized_bgr
                    cv2.rectangle(display_frame, (px1-1, py1-1), (px2, py2), (255, 255, 0), 1)

            cv2.imshow(window_name, display_frame)

            key = cv2.waitKey(1) & 0xFF
            # 按空格键 (32) 或 's'/'S' 保存
            if key == 32 or key == ord('s') or key == ord('S'):
                if roi_bgr.shape[0] > 0 and roi_bgr.shape[1] > 0:
                    final_gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                    final_gray_roi = cv2.resize(final_gray_roi, (32, 32))

                    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
                    file_name = f"{class_name}_{timestamp_str}.jpg"
                    full_save_path = os.path.join(save_dir, file_name)

                    cv2.imwrite(full_save_path, final_gray_roi)
                    saved_count += 1
                    print(f"📸 [{saved_count}] 已保存 480x480->32x32 灰度 ROI 图像: {full_save_path}")

            # 按 'q'/'Q' 或 ESC 退出
            elif key == ord('q') or key == ord('Q') or key == 27:
                print("\n👋 退出程序。")
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
        print(f"\n🎉 本次采集结束！共计成功保存了 {saved_count} 张图像到 {save_dir}")

if __name__ == "__main__":
    main()
