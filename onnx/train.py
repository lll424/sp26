import os
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import transforms, datasets
from model import ArmorNet2Class

def main():
    # 数据集根目录路径
    dataset_dir = os.path.join(os.path.dirname(__file__), "dataset")
    train_dir = os.path.join(dataset_dir, "train")
    val_dir = os.path.join(dataset_dir, "val")

    if not os.path.exists(train_dir) or not os.path.exists(val_dir):
        print(f"❌ 找不到数据集目录，请先运行 `python3 setup_dataset.py` 创建目录并放入图片！")
        return

    # 1. 定义数据增强与预处理 (保持 32x32 单通道)
    train_transform = transforms.Compose([
        transforms.Grayscale(num_output_channels=1),
        transforms.Resize((32, 32)),
        transforms.RandomRotation(15),           # 旋转 +-15 度
        transforms.ColorJitter(brightness=0.2),  # 亮度扰动
        transforms.ToTensor(),                   # 归一化到 [0, 1]
    ])

    val_transform = transforms.Compose([
        transforms.Grayscale(num_output_channels=1),
        transforms.Resize((32, 32)),
        transforms.ToTensor(),
    ])

    # 2. 加载 2 类别数据集
    train_dataset = datasets.ImageFolder(train_dir, transform=train_transform)
    val_dataset = datasets.ImageFolder(val_dir, transform=val_transform)

    print(f"训练集样本总数: {len(train_dataset)}, 类别映射: {train_dataset.class_to_idx}")
    print(f"验证集样本总数: {len(val_dataset)}")

    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

    # 3. 初始化设备与双类别模型
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"使用计算设备: {device}")

    model = ArmorNet2Class(num_classes=2).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)

    # 4. 训练循环 (训练 30 个 Epoch)
    epochs = 30
    best_acc = 0.0
    model_save_path = os.path.join(os.path.dirname(__file__), "best_model_2class.pth")

    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()

        # 验证集评估
        model.eval()
        correct, total = 0, 0
        with torch.no_grad():
            for images, labels in val_loader:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                _, predicted = torch.max(outputs, 1)
                total += labels.size(0)
                correct += (predicted == labels).sum().item()

        val_acc = correct / total if total > 0 else 0.0
        avg_loss = running_loss / len(train_loader) if len(train_loader) > 0 else 0.0
        
        print(f"Epoch [{epoch+1:02d}/{epochs:02d}] Loss: {avg_loss:.4f} | Val Accuracy: {val_acc*100:.2f}%")

        if val_acc >= best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), model_save_path)
            print(f"--> Saved best model to {model_save_path}")

    print(f"\n🎉 训练完成！最佳验证准确率: {best_acc*100:.2f}%")

if __name__ == "__main__":
    main()
