import os
import torch
from model import ArmorNet2Class

def main():
    base_dir = os.path.dirname(__file__)
    pth_path = os.path.join(base_dir, "best_model_2class.pth")
    onnx_output_path = os.path.join(base_dir, "classifier_2class.onnx")

    if not os.path.exists(pth_path):
        print(f"❌ 未找到模型权重文件 `{pth_path}`，请先运行 `python3 train.py` 进行训练！")
        return

    # 1. 实例化 2-Class 模型并加载权重
    model = ArmorNet2Class(num_classes=2)
    model.load_state_dict(torch.load(pth_path, map_location="cpu"))
    model.eval()

    # 2. 构造规范输入虚拟张量 [1, 1, 32, 32]
    dummy_input = torch.randn(1, 1, 32, 32, dtype=torch.float32)

    # 3. 导出为 OpenCV DNN 完美兼容的标准 ONNX 模型 (使用经典模式 + Opset 11)
    try:
        # 兼容 PyTorch 2.x: 关闭 dynamo 强制使用经典 ONNX 导出器
        torch.onnx.export(
            model,
            dummy_input,
            onnx_output_path,
            export_params=True,
            opset_version=11,               # OpenCV DNN 推荐的最稳版本
            do_constant_folding=True,       # 开启常量折叠
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={
                'input': {0: 'batch_size'},
                'output': {0: 'batch_size'}
            },
            dynamo=False                    # 关键：禁用 Dynamo 新机制，避免生成 OpenCV 无法解析的动态 Conv 节点
        )
    except TypeError:
        # 兼容旧版本 PyTorch (无 dynamo 参数)
        torch.onnx.export(
            model,
            dummy_input,
            onnx_output_path,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={
                'input': {0: 'batch_size'},
                'output': {0: 'batch_size'}
            }
        )

    print(f"✅ ONNX 模型导出成功 (OpenCV DNN 完美兼容)！")
    print(f"路径: {onnx_output_path}")
    print(f"输入尺寸: [1, 1, 32, 32] (Float32)")
    print(f"输出尺寸: [1, 2] (Float32, 2 类别)")

    # 自动拷贝一份到 assets/ 目录以便 C++ 配置文件直接引用
    assets_dir = os.path.join(base_dir, "..", "assets")
    if os.path.exists(assets_dir):
        target_assets_path = os.path.join(assets_dir, "classifier_2class.onnx")
        import shutil
        shutil.copy(onnx_output_path, target_assets_path)
        print(f"🚀 已自动复制到项目 assets 目录: {target_assets_path}")

if __name__ == "__main__":
    main()
