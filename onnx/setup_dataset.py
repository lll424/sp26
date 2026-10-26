import os

def setup_directories():
    base_dir = os.path.dirname(__file__)
    dataset_dir = os.path.join(base_dir, "dataset")
    
    # 2 个类别的文件夹名称定义 (例如 class0 和 class1，可自行重命名)
    categories = ["class0", "class1"]
    
    splits = ["train", "val"]
    
    for split in splits:
        for cat in categories:
            path = os.path.join(dataset_dir, split, cat)
            os.makedirs(path, exist_ok=True)
            print(f"📁 已创建目录: {path}")

    print("\n✅ 数据集目录创建成功！")
    print(f"请将你的 2 种类别的图片放置在以下路径中:")
    print(f"  - 类别 0 (如: 正样本/装甲板): dataset/train/class0/ 与 dataset/val/class0/")
    print(f"  - 类别 1 (如: 负样本/非装甲板): dataset/train/class1/ 与 dataset/val/class1/")

if __name__ == "__main__":
    setup_directories()
