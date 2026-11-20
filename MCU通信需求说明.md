# 视觉 ↔ 电控 CAN 通信协议

> 工程：sp26（视觉）　｜　版本：2026-11-20
> 物理层：SocketCAN，接口 `can0`，标准数据帧（11 位 ID），DLC=8，**大端序（高字节先发）**
> 本文档与代码 `io/cboard.cpp` 逐字节对应，已核对。

---

## 一页速查

| 方向 | CAN ID | 名称 | 频率 | 谁发 |
|---|---|---|---|---|
| 电控 → 视觉 | **0x01** | 四元数 | ≥ 500 Hz（建议 1 kHz） | 电控 |
| 电控 → 视觉 | **0x110** | 弹速/模式 | 1~10 Hz 周期重发 | 电控 |
| 视觉 → 电控 | **0xFF** | 瞄准指令 | 30~120 Hz | 视觉 |

**三个都要改的话，改 `configs/*.yaml` 里的 `quaternion_canid` / `bullet_speed_canid` / `send_canid`，代码里没有写死。**

---

## 帧 1：四元数（电控 → 视觉，0x01）

**这是视觉启动的前置条件。视觉程序会阻塞等第一帧，起不来就是没发这个。**

| 字节 | 内容 | 类型 | 定标 |
|---|---|---|---|
| data[0..1] | **x** | int16 大端 | × 10000 |
| data[2..3] | **y** | int16 大端 | × 10000 |
| data[4..5] | **z** | int16 大端 | × 10000 |
| data[6..7] | **w** | int16 大端 | × 10000 |

### ⚠️ 顺序是 x,y,z,w，不是 w,x,y,z

绝大多数 IMU 库返回的成员顺序是 `(w, x, y, z)`，**直接按结构体顺序打包会错**。

错过之后的后果特别隐蔽：视觉侧有归一化校验 `x²+y²+z²+w² ≈ 1`，
**重排分量不改变模长，校验照样通过**，不会报任何错。但解出来的姿态是乱的，
表现为"云台乱指、完全对不上方向"，而且很难查到原因。

### 要求

1. **必须归一化**，视觉侧校验 `|‖q‖ − 1| ≤ 0.01`，超差丢帧并打 `Invalid q` 警告
2. **周期必须恒定**。视觉按接收时刻做线性插值（slerp），发送不均匀直接变成姿态误差
3. **上电就发，不要等视觉上线**，也不要在视觉没响应时停发
4. 数值范围：分量 ±1，×1e4 后 ±10000，int16 装得下

### 坐标系必须书面约定

这是手眼标定的前提。需要写清：

- 四元数描述的是**哪个系到哪个系的旋转**（通常是 IMU 体轴系 → 世界系）
- **yaw = 0 时车头朝哪个方向**
- 旋转顺序约定（ZYX 还是 XYZ）

**这一条不写清，后面所有角度都对不上，而且双方都以为是对方的问题。**

---

## 帧 2：弹速 / 模式（电控 → 视觉，0x110）

| 字节 | 内容 | 类型 | 定标 / 取值 |
|---|---|---|---|
| data[0..1] | 弹速 bullet_speed | int16 大端 | × 100，单位 m/s |
| data[2] | 模式 mode | uint8 | 见下表 |
| data[3] | 射击模式 shoot_mode | uint8 | 见下表 |
| data[4..5] | ft_angle | int16 大端 | × 10000，rad，**仅无人机用，其他车填 0** |
| data[6..7] | 保留 | — | 填 0 |

### mode 枚举（必须一致）

| 值 | 含义 |
|---|---|
| 0 | idle（待机，视觉不工作） |
| 1 | auto_aim（自瞄） |
| 2 | small_buff（小符） |
| 3 | big_buff（大符） |
| 4 | outpost（前哨站） |

### shoot_mode 枚举（哨兵专用）

| 值 | 含义 |
|---|---|
| 0 | left_shoot |
| 1 | right_shoot |
| 2 | both_shoot |

### 两个必须遵守的点

1. **周期重发**（1~10 Hz），不要只在变化时发一次。视觉比模式切换晚上电时，
   收不到这一帧就永远停在 idle，表现为"程序在跑但什么都不做"。
2. **弹速必须用实测值**，不要填档位设定值。弹道解算直接吃这个数，
   差 1 m/s 在 5 m 处约差 2 cm 落点。

---

## 帧 3：瞄准指令（视觉 → 电控，0xFF）

| 字节 | 内容 | 类型 | 说明 |
|---|---|---|---|
| data[0] | control | uint8 | 1 = 视觉请求接管云台；0 = 交还 |
| data[1] | shoot | uint8 | 1 = 请求开火；0 = 停止 |
| data[2..3] | yaw | int16 大端 | × 10000，rad，**世界系绝对角** |
| data[4..5] | pitch | int16 大端 | × 10000，rad，**含弹道重力补偿** |
| data[6..7] | horizon_distance | int16 大端 | × 10000，m |

### 控制分工：视觉只给设定值，伺服由电控做

视觉发出的是**云台应该指向的世界系绝对角**，不是增量、不是速度。

电控要做的是：

```
误差 = 指令角 − 当前 IMU 解算角
驱动电机把误差收敛到 0
```

角度基准来自帧 1 的四元数世界系，双方天然一致。

### 超时保护（电控必须实现）

**超过 500 ms 没收到新帧，自动切回手动控制或保持当前位置。**

没有这条，视觉一掉线（程序崩溃、USB 掉、断电），云台会保持最后一帧的指令角
一直冲下去，可能直接打限位。

### ⚠️ horizon_distance 有溢出风险

int16 上限 32767，÷10000 后最大 **3.2767 m**。

这个是无人机用的高度距离，**地面车可以不用（填 0）**。
如果确实要用，超过 3.2 m 就会溢出成负数，必须改编码（×1000 或拆两帧）。
当前代码还没改，属于已知问题。

---

## 电控侧代码模板

### 发送四元数（1 kHz）

```c
// 注意参数顺序按 x,y,z,w 传，别按 IMU 库的 (w,x,y,z) 直接铺
void CAN_SendQuaternion(float qx, float qy, float qz, float qw)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = 0x01;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = 8;

    int16_t ix = (int16_t)(qx * 10000.0f);
    int16_t iy = (int16_t)(qy * 10000.0f);
    int16_t iz = (int16_t)(qz * 10000.0f);
    int16_t iw = (int16_t)(qw * 10000.0f);

    uint8_t d[8];
    d[0] = (uint8_t)(ix >> 8);  d[1] = (uint8_t)(ix & 0xFF);
    d[2] = (uint8_t)(iy >> 8);  d[3] = (uint8_t)(iy & 0xFF);
    d[4] = (uint8_t)(iz >> 8);  d[5] = (uint8_t)(iz & 0xFF);
    d[6] = (uint8_t)(iw >> 8);  d[7] = (uint8_t)(iw & 0xFF);

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &mailbox);
}
```

### 发送弹速/模式

```c
void CAN_SendStatus(float bullet_speed, uint8_t mode, uint8_t shoot_mode)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = 0x110;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = 8;

    int16_t spd = (int16_t)(bullet_speed * 100.0f);

    uint8_t d[8] = {0};
    d[0] = (uint8_t)(spd >> 8);
    d[1] = (uint8_t)(spd & 0xFF);
    d[2] = mode;
    d[3] = shoot_mode;
    // d[4..7] 留 0（ft_angle 仅无人机用）

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &mailbox);
}
```

### 接收瞄准指令

```c
typedef struct {
    uint8_t  control;
    uint8_t  shoot;
    float    yaw;       // rad, 世界系绝对角
    float    pitch;     // rad, 含弹道补偿
    uint32_t last_tick; // 用于超时判断
} VisionCmd_t;

VisionCmd_t g_vision;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t d[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d);

    if (hdr.StdId != 0xFF) return;

    g_vision.control = d[0];
    g_vision.shoot   = d[1];

    int16_t yaw_i   = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    int16_t pitch_i = (int16_t)(((uint16_t)d[4] << 8) | d[5]);

    g_vision.yaw   = yaw_i   / 10000.0f;
    g_vision.pitch = pitch_i / 10000.0f;
    g_vision.last_tick = HAL_GetTick();
}

// 主循环里调用：超时检查
void Vision_Watchdog(void)
{
    if (HAL_GetTick() - g_vision.last_tick > 500) {
        g_vision.control = 0;
        g_vision.shoot   = 0;
    }
}
```

**注意 `((uint16_t)d[2] << 8)` 里的强制转换**。如果 `d[2]` 是 `uint8_t` 且值 ≥ 0x80，
直接移位在某些编译器上会先做整型提升再移位，结果正确；但负角度（int16 高位为 1）
必须靠这个转换保证不被当成负数扩展。**这是最容易出错的一行。**

---

## 联调检查清单

按顺序做，每步对不上就别往下走。

### 第 1 步：确认总线通了

```bash
sudo ip link set can0 up type can bitrate 1000000
candump can0
```

电控上电后应该能看到 0x01 和 0x110 在刷。

**看不到 → 查终端电阻（两端各 120Ω）、波特率、接线 H/L 是否反了。**

### 第 2 步：确认视觉能起来

```bash
./build/mt_auto_aim_debug configs/mindvision_vm.yaml
```

日志出现 `[Cboard] Opened.` 说明收到四元数了。

**卡在 `Waiting for q...` → 帧 1 没发或发错 ID。**

### 第 3 步：确认模式对

日志应该打印：

```
[CBoard] Bullet speed: 15.00 m/s, Mode: auto_aim, Shoot mode: left_shoot, FT angle: 0.00 rad
```

**一直显示 idle → 帧 2 没发，或 mode 字节位置错了。**

### 第 4 步：确认视觉在发指令

```bash
candump can0,0xFF:0x7FF
```

应该能看到 0xFF 的帧在刷，帧率跟视觉帧率一致。

**看不到 → 视觉没进 tracking 状态，先查检测有没有框到装甲板。**

### 第 5 步：确认云台跟着动

让视觉对准静靶，看云台是否指向目标。这里才涉及手眼标定准不准。

---

## 三个最容易踩的坑

### 坑 1：四元数顺序按 w,x,y,z 打包

参考帧 1 那一节。**不报错，但姿态全错。**

### 坑 2：视觉程序启动时卡住

`[Cboard] Waiting for q...` 一直不动。原因永远是帧 1 没到。
先 `candump can0` 确认总线有没有数据，再看 ID 和字节顺序。

### 坑 3：云台一通电就冲限位

说明收到了 `control=1` 但角度是错的。可能原因：

- 四元数顺序错（姿态基准错）
- 视觉侧手眼标定没做（相机装的位置和配置里写的不一致）
- 电控没做超时保护，视觉掉线后一直用最后一帧

**先在 `control=0` 下把角度读出来对一遍**，确认 yaw/pitch 数值合理（比如正前方约 0、
正右方约 +1.57），再开 `control=1`。

---

## 待双方书面确认的事项

| # | 事项 | 状态 |
|---|---|---|
| 1 | 三个 CAN ID 最终取值（0x01 / 0x110 / 0xFF 是否沿用） | ☐ |
| 2 | 四元数坐标系定义：哪个系到哪个系、yaw=0 朝向、旋转顺序 | ☐ |
| 3 | 四元数发送频率（目标 1 kHz） | ☐ |
| 4 | 电控接收端超时保护时间（建议 500 ms） | ☐ |
| 5 | 弹速来源（**必须实测**，不能用档位设定值） | ☐ |
| 6 | 波特率（当前按 1 Mbps 假设） | ☐ |

---

## 附：字节序示例

**四元数**，设 x=0.5, y=0.5, z=0.5, w=0.5：

```
0.5 × 10000 = 5000 = 0x1388

data[0]=0x13  data[1]=0x88     (x)
data[2]=0x13  data[3]=0x88     (y)
data[4]=0x13  data[5]=0x88     (z)
data[6]=0x13  data[7]=0x88     (w)
```

**弹速**，设 15.30 m/s：

```
15.30 × 100 = 1530 = 0x05FA

data[0]=0x05  data[1]=0xFA
```

**负角度**，设 yaw = −0.5 rad：

```
−0.5 × 10000 = −5000 = int16 的 0xEC78

data[2]=0xEC  data[3]=0x78
```

接收端必须按 int16 解析，按 uint16 会得到 60536。
