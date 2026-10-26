import torch
import torch.nn as nn

class ArmorNet2Class(nn.Module):
    """
    轻量级双类别 (2-Class) 图像分类网络
    输入大小: [BatchSize, 1, 32, 32] (单通道灰度图)
    输出大小: [BatchSize, 2] (对应 2 个类别)
    """
    def __init__(self, num_classes=2):
        super(ArmorNet2Class, self).__init__()
        
        # 卷积特征提取网络
        self.features = nn.Sequential(
            # Layer 1: 32x32 -> 16x16
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),

            # Layer 2: 16x16 -> 8x8
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),

            # Layer 3: 8x8 -> 4x4
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
        )
        
        # 全连接分类头
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 4 * 4, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(128, num_classes)  # 双类别输出
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

if __name__ == "__main__":
    model = ArmorNet2Class(num_classes=2)
    dummy_input = torch.randn(1, 1, 32, 32)
    output = model(dummy_input)
    print("✅ Model created successfully. Output shape:", output.shape)
