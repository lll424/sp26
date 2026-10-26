# 100% 独立版 双类别与官方自瞄 ONNX 训练、采集、导出与实时验证工具包

本目录下的所有程序均为 **100% 独立解耦的 Python 程序**，不依赖主项目的任何 C++ 文件或外部编译工程。

能够直接独立完成：**迈德威视工业相机驱动采集图像 $\rightarrow$ ROI 自动切图保存 $\rightarrow$ PyTorch 训练模型 $\rightarrow$ 导出 ONNX 模型 $\rightarrow$ 实时 ROI 识别验证**。

---

## 文件夹解耦结构说明

* `mvsdk.py`：独立 Python 版迈德威视 (MindVision) 工业相机 C-Types 原生驱动接口。
* `capture_roi.py`：**独立拍照切图工具**。询问 `train/val` 和 `class0/class1`，自动连迈德威视相机并切图 $32 \times 32$ 灰度图保存。
* `verify_real_auto_aim.py`：**真实自瞄系统官方 9 分类模型 (assets/tiny_resnet.onnx) 实时测试工具**。直接加载官方发布的 ResNet 装甲板分类权重，针对中心 ROI 实时识别（1~5号车、哨兵、前哨站、基地、非装甲板）。
* `verify_roi.py`：**独立 2-Class ONNX 实时 ROI 识别验证工具**。
* `setup_dataset.py`：数据集目录结构一键创建脚本。
* `model.py`：双类别轻量级卷积神经网络模型定义。
* `train.py`：模型训练与验证脚本。
* `export_onnx.py`：导出 ONNX 模型。
* `test_onnx.py`：单张图片假数据 Python 独立测试脚本。

---

## 独立运行全部命令清单（纯 Python）

在终端中只需使用 `python3` 即可独立完成全部流程：

### 1. 实时测试“真实自瞄项目官方 9 分类 ONNX 模型” (assets/tiny_resnet.onnx)
```bash
cd /home/lida/桌面/sp25/sp_vision_25/onnx
python3 verify_real_auto_aim.py
```
* **效果**：对准装甲板贴纸，实时在镜头目标框上方显示官方模型的判定结果（如 `Class: 3 Infantry (3号步兵) | Conf: 98.4%`）。按 **【Q 键】** 退出。

### 2. 实时测试“你自己训练的双类别 ONNX 模型”
```bash
python3 verify_roi.py
```
