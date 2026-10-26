import ctypes
import numpy as np

class MindVisionCamera:
    """
    独立 Python 版迈德威视 (MindVision) 工业相机 CTypes 安全封装
    使用 CameraGetImageBufferEx API，零内存结构风险，彻底解决 SegFault
    """
    def __init__(self, exposure_ms=2.5, gamma=0.5):
        self.exposure_ms = exposure_ms
        self.gamma = gamma
        self.handle = ctypes.c_int(-1)
        self.width = 640
        self.height = 480
        self.is_opened = False
        self.lib = None

        # 加载系统的 libMVSDK.so 动态链接库
        try:
            self.lib = ctypes.CDLL("libMVSDK.so")
        except Exception:
            try:
                self.lib = ctypes.CDLL("libMVSDK.so.2")
            except Exception as e:
                raise RuntimeError("无法加载 libMVSDK.so，请确保系统已安装 MindVision SDK 驱动库！") from e

        # 显式设置所有 64 位 C 函数签名
        self.lib.CameraSdkInit.argtypes = [ctypes.c_int]
        self.lib.CameraSdkInit.restype = ctypes.c_int

        self.lib.CameraEnumerateDevice.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.CameraEnumerateDevice.restype = ctypes.c_int

        self.lib.CameraInit.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        self.lib.CameraInit.restype = ctypes.c_int

        self.lib.CameraSetAeState.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.CameraSetAeState.restype = ctypes.c_int

        self.lib.CameraSetExposureTime.argtypes = [ctypes.c_int, ctypes.c_double]
        self.lib.CameraSetExposureTime.restype = ctypes.c_int

        self.lib.CameraSetGamma.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.CameraSetGamma.restype = ctypes.c_int

        self.lib.CameraSetIspOutFormat.argtypes = [ctypes.c_int, ctypes.c_uint]
        self.lib.CameraSetIspOutFormat.restype = ctypes.c_int

        self.lib.CameraSetTriggerMode.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.CameraSetTriggerMode.restype = ctypes.c_int

        self.lib.CameraPlay.argtypes = [ctypes.c_int]
        self.lib.CameraPlay.restype = ctypes.c_int

        # CameraGetImageBufferEx: 直接获取 RGB/BGR 数据缓冲区首地址
        self.lib.CameraGetImageBufferEx.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]
        self.lib.CameraGetImageBufferEx.restype = ctypes.c_void_p

        self.lib.CameraUnInit.argtypes = [ctypes.c_int]
        self.lib.CameraUnInit.restype = ctypes.c_int

    def open(self):
        # 1. 初始化 C SDK
        self.lib.CameraSdkInit(1)
        
        dev_info_buffer = (ctypes.c_ubyte * 512)()
        camera_num = ctypes.c_int(1)
        
        status = self.lib.CameraEnumerateDevice(ctypes.byref(dev_info_buffer), ctypes.byref(camera_num))
        if camera_num.value == 0:
            raise RuntimeError("未找到迈德威视 MindVision 相机设备！")

        # 2. 初始化相机句柄
        status = self.lib.CameraInit(ctypes.byref(dev_info_buffer), -1, -1, ctypes.byref(self.handle))
        if status != 0 or self.handle.value <= 0:
            raise RuntimeError(f"CameraInit 失败，错误码: {status} (如 -18 表示设备正被占用，请等待或重启相机)")

        # 3. 参数配置
        CAMERA_MEDIA_TYPE_BGR8 = 0x01080008
        self.lib.CameraSetAeState(self.handle.value, 0)
        self.lib.CameraSetExposureTime(self.handle.value, ctypes.c_double(self.exposure_ms * 1000.0))
        self.lib.CameraSetGamma(self.handle.value, ctypes.c_int(int(self.gamma * 100)))
        self.lib.CameraSetIspOutFormat(self.handle.value, CAMERA_MEDIA_TYPE_BGR8)
        self.lib.CameraSetTriggerMode(self.handle.value, 0)

        # 4. 开启取图
        play_status = self.lib.CameraPlay(self.handle.value)
        if play_status != 0:
            raise RuntimeError(f"CameraPlay 启动采集失败，错误码: {play_status}")

        self.is_opened = True
        print(f"✅ [MindVision Python] 相机初始化成功 (Handle: {self.handle.value})")

    def read(self):
        if not self.is_opened or self.handle.value <= 0:
            return False, None

        w_val = ctypes.c_int(0)
        h_val = ctypes.c_int(0)

        # 超时时间设为 300ms，获取输出数据首地址
        p_buf = self.lib.CameraGetImageBufferEx(self.handle.value, ctypes.byref(w_val), ctypes.byref(h_val), 300)
        if not p_buf or w_val.value <= 0 or h_val.value <= 0:
            return False, None

        self.width = w_val.value
        self.height = h_val.value

        # 从内存首地址零拷贝映射 NumPy 图像句柄
        buf_size = self.width * self.height * 3
        frame_data = (ctypes.c_ubyte * buf_size).from_address(p_buf)
        frame = np.ctypeslib.as_array(frame_data).reshape((self.height, self.width, 3))

        return True, frame.copy()

    def release(self):
        if self.is_opened and self.handle.value > 0:
            try:
                self.lib.CameraUnInit(self.handle.value)
            except Exception:
                pass
            self.is_opened = False
            self.handle = ctypes.c_int(-1)
            print("👋 [MindVision Python] 相机连接已关闭。")

    def __del__(self):
        self.release()
