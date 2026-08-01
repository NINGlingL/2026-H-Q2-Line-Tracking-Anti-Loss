

"""
钢球位置检测系统 — 针对 1/1.8" M12 2.8-12mm 工业镜头优化版

核心优化:
  1. 摄像头参数调优 — 曝光/增益针对工业大光圈镜头精细配置 + 自适应曝光
  2. 滤波器优化 — 速度钳制 + 丢帧预测 + 工业镜头低噪声参数

硬件适配:
  - 传感器: 1/1.8" (7.18×5.32mm), 感光面积大, 信噪比优秀
  - 镜头: M12 2.8-12mm 变焦, 推荐长焦端 (~12mm) 让目标像素更多
  - 光圈: 工业级大光圈, 可降低曝光时间换取更高帧率
  - IR: 支持红外, 建议配合红外补光灯提升钢球与背景对比度
"""

from maix import camera, display, image, nn, app, time, sys, uart, pinmap, err
import math


# =============================================================================
# ball_position 模块 — 硬件无关的几何计算与位置滤波
# =============================================================================

def project_onto_axis(point, axis_start, axis_end):
    """投影点到标定轴，返回归一化比例和垂距。"""
    px, py = point
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")

    relative_x = px - x0 
    relative_y = py - y0
    ratio = (relative_x * dx + relative_y * dy) / float(length_squared)
    projected_x = x0 + ratio * dx
    projected_y = y0 + ratio * dy
    distance = math.hypot(px - projected_x, py - projected_y)
    return ratio, distance


def axis_point(ratio, axis_start, axis_end):
    """返回标定轴上 ratio 处的图像坐标。"""
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


def position_from_pixel(
    point,
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
):
    """图像像素 → 物理位置(cm)。"""
    if start_position_cm == end_position_cm:
        raise ValueError("physical calibration positions must be different")
    ratio, distance = project_onto_axis(point, axis_start, axis_end)
    position_cm = start_position_cm + ratio * (
        end_position_cm - start_position_cm
    )
    return position_cm, ratio, distance


class AdaptiveAlphaBetaFilter:
    """
    速度自适应 Alpha-Beta 位置滤波器。

    核心思想: 测量误差大(球快速移动)时, 加大 alpha/beta 以快速跟踪;
              测量误差小(球静止)时, 减小 alpha/beta 以平滑噪声。

    针对工业镜头优化:
      - 大光圈 + 1/1.8" sensor → 短曝光 → 低像素噪声
      - alpha_slow 可设得更低, 稳态输出更平滑
      - 增加速度钳制防野值
      - 增加 predict() 用于丢帧时的位置估算
    """

    def __init__(
        self,
        alpha_slow=0.18,         # 稳态平滑增益 (稍高以应对微振动)
        alpha_fast=0.95,         # 快速跟踪增益 (快速运动时几乎直接跟随测量值)
        beta_slow=0.015,         # 稳态速度更新 (更快感知速度变化)
        beta_fast=0.18,          # 快速速度更新 (高动态下速度估计更激进)
        fast_error_px=8.0,       # 更低阈值 → 更早触发快速跟踪模式
        reset_ms=200,            # 丢帧更快重置 (快速运动时不能等太久)
        max_velocity_px_s=1200,  # 速度钳制放宽 (快速运动容忍度提高)
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_px = fast_error_px
        self.reset_ms = reset_ms
        self.max_velocity = max_velocity_px_s
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        # 限制 dt 范围防止数值不稳定
        dt = max(0.005, min(0.100, dt))

        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)

        # 平方映射让运动响应更敏锐
        motion = min(1.0, error / self.fast_error_px) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt

        # 速度钳制
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_velocity:
            scale = self.max_velocity / speed
            self.vx *= scale
            self.vy *= scale

        self.last_ms = now_ms
        return self.x, self.y

    def predict(self, now_ms):
        """用当前速度模型预测位置 (丢帧时使用)。"""
        if self.x is None:
            return None, None
        dt = (now_ms - self.last_ms) * 0.001
        dt = min(0.200, dt)  # 最多预测 200ms
        return self.x + self.vx * dt, self.y + self.vy * dt

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()


def validate_calibration(
    axis_start, axis_end, start_position_cm, end_position_cm, width, height
):
    """校验标定参数合法性。"""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    for name, point in (("start", axis_start), ("end", axis_end)):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError(f"{name} calibration point is outside the image")
    position_from_pixel(
        axis_start, axis_start, axis_end,
        start_position_cm, end_position_cm,
    )


# =============================================================================
# ball_position 模块结束
# =============================================================================


# =============================================================================
# 运行模式配置
# =============================================================================

# --- 调试开关 ---
# 生产环境设为 False: 关闭串口打印 + 关闭额外绘制, 帧率显著提升
DEBUG_LOG = False  # 关闭调试，保持高FPS

def print_debug(s):
    if DEBUG_LOG:
        print(s)

# --- 低延迟模式 ---
# dual_buff=False: 每帧检测结果对应当前帧, 延迟最低
LOW_LATENCY_MODE = True

# --- 视频流配置 ---
# 推荐只开其中一个, 默认 WebRTC (延迟最低)
USE_RTSP = False
USE_JPEG = False
USE_WEBRTC = True

# 推流分辨率 — 独立于检测分辨率, 降低带宽和编码延迟
STREAM_WIDTH = 320
STREAM_HEIGHT = 180

# --- 串口通信配置 ---
# 启用串口通信: 与天猛星MSPM0G3507通信
USE_UART = True

# 串口设备和参数
# 根据设备ID选择不同的UART引脚和设备
# MaixCAM (非Pro): A16=UART0_TX, A17=UART0_RX -> /dev/ttyS0
#                  A19=UART1_TX, A18=UART1_RX -> /dev/ttyS1
# MaixCAM2 (Pro):  A21=UART4_TX, A22=UART4_RX -> /dev/ttyS4
#
# MSPM0配置: UART0 (PA10/PA11), 115200bps, 8N1
# 连接: MaixCam_TX -> MSPM0_RX(PA11), MaixCam_RX -> MSPM0_TX(PA10), GND共地

# 根据设备自动选择UART配置
device_id = sys.device_id()
if device_id == "maixcam2":
    # MaixCAM Pro (MaixCAM2)
    UART_PIN_TX = "A21"
    UART_PIN_RX = "A22"
    UART_FUNC_TX = "UART4_TX"
    UART_FUNC_RX = "UART4_RX"
    UART_DEVICE = "/dev/ttyS4"
else:
    # 标准 MaixCAM - 使用UART1避免与MaixComm冲突
    UART_PIN_TX = "A19"
    UART_PIN_RX = "A18"
    UART_FUNC_TX = "UART1_TX"
    UART_FUNC_RX = "UART1_RX"
    UART_DEVICE = "/dev/ttyS1"

UART_BAUDRATE = 115200         # 波特率: 115200 bps (与MSPM0一致)

# 串口发送频率控制 (避免OLED刷新过快，同时保证FPS)
UART_SEND_INTERVAL_MS = 50     # 每50ms发送一次 (20Hz更新率，不影响检测FPS)

# --- 镜头校准 ---
# 工业镜头畸变通常比手机镜头小, 变焦镜头可能有轻微枕形畸变
LENS_CORR_ENABLE = False
LENS_CORR_STRENGTH = 0.5


# =============================================================================
# 摄像头参数 — 针对 1/1.8" M12 2.8-12mm 工业镜头深度优化
# =============================================================================

class CameraConfig:
    """
    工业镜头摄像头参数配置 + 自适应曝光逻辑。

    传感器 1/1.8" (7.18×5.32mm):
      - 感光面积是典型手机 sensor 的 2-3 倍
      - 每个像素接收的光子更多 → 信噪比更高
      - 大光圈工业镜头 → 可用更低曝光时间换取高帧率

    M12 2.8-12mm 变焦镜头建议:
      - 安装高度 30-50cm 时, 推荐 8-12mm 焦段, 让钢球(约 2-3cm)占 15-30 像素
      - 对焦在钢球运动平面上, 充分利用工业镜头大光圈浅景深
      - IR 支持: 环境光不足时加装 850nm 红外补光灯

    自适应曝光:
      - 检测到球: 保持当前曝光, 确保目标亮度稳定
      - 丢球超过阈值: 尝试逐步调整曝光, 帮助重新捕获
    """

    # ---- 基础曝光参数 ----
    # 曝光时间 (微秒): 物体快速运动时需要极短曝光来冻结运动模糊
    # 工业大光圈 + 1/1.8" sensor → 800-1500μs 即可获得足够亮度
    # 原则: 曝光越短 → 运动模糊越小 → 快速运动时检测越准 → 帧率越高
    EXPOSURE_US = 1200

    # 曝光调整范围
    EXPOSURE_MIN_US = 400     # 最短曝光 (强光/快速运动, 极限压缩模糊)
    EXPOSURE_MAX_US = 6000    # 最长曝光 (避免帧率过低, 最慢也要 >30fps)

    # 模拟增益 (dB): 工业镜头信号强, 增益设低以减少噪声
    ANALOG_GAIN_DB = 3
    GAIN_MIN_DB = 0
    GAIN_MAX_DB = 12

    # ---- 自适应曝光 ----
    # 启用自适应曝光微调
    AUTO_EXPOSURE_ADAPT = True

    # 丢球多少帧后开始调整曝光 (快速运动时尽快响应)
    ADAPT_TRIGGER_FRAMES = 8

    # 每次调整的步长 (μs): 快速运动时用更大步长
    ADAPT_STEP_US = 400

    # 调整方向: 快速运动场景优先降曝光减少模糊
    ADAPT_DIRECTION = -1  # -1=先减曝光(降低运动模糊), +1=先增曝光

    # ---- 自动控制锁定 ----
    # 锁定自动曝光: 画面亮度稳定, 检测结果更一致
    AUTO_EXPOSURE_LOCK = True

    # 锁定自动白平衡: 颜色稳定
    AUTO_AWB_LOCK = True

    # 亮度目标: 工业镜头进光量大, 降低目标减少曝光时间
    BRIGHTNESS_TARGET = 50  # 0-100


CAM_CFG = CameraConfig()


class ExposureAdapter:
    """
    自适应曝光调节器。

    策略:
      - 正常跟踪时不动曝光, 保持画面稳定
      - 连续丢球超过 ADAPT_TRIGGER_FRAMES 帧后, 逐步调整曝光
      - 先在当前方向上调整几帧, 无效则反向
      - 恢复跟踪时锁定当前曝光值
    """

    def __init__(self, cam, cfg):
        self._cam = cam
        self._cfg = cfg
        self._current_us = cfg.EXPOSURE_US
        self._current_gain = cfg.ANALOG_GAIN_DB
        self._lost_frames = 0
        self._adapting = False
        self._direction = cfg.ADAPT_DIRECTION
        self._steps_tried = 0

    def on_ball_found(self):
        """检测到球: 重置丢失计数, 锁定当前曝光。"""
        self._lost_frames = 0
        if self._adapting:
            # 刚恢复, 锁定当前值
            self._cfg.EXPOSURE_US = self._current_us
            self._cfg.ANALOG_GAIN_DB = self._current_gain
            self._adapting = False
            self._steps_tried = 0
            print_debug(f"[曝光] 恢复跟踪, 锁定曝光={self._current_us}μs")

    def on_ball_lost(self):
        """丢球: 累计丢帧数, 超过阈值后开始自适应调整。"""
        if not self._cfg.AUTO_EXPOSURE_ADAPT:
            return
        self._lost_frames += 1

        if self._lost_frames >= self._cfg.ADAPT_TRIGGER_FRAMES:
            self._adapting = True

        if self._adapting and self._lost_frames % 2 == 0:  # 每 2 帧调一次 (快速响应)
            self._step_exposure()

    def _step_exposure(self):
        """单步调整曝光。"""
        self._steps_tried += 1

        # 超过 10 步未恢复, 换方向
        if self._steps_tried > 10:
            self._direction *= -1
            self._steps_tried = 0
            print_debug(f"[曝光] 切换调整方向: {'+' if self._direction > 0 else '-'}")

        self._current_us += self._direction * self._cfg.ADAPT_STEP_US
        self._current_us = max(self._cfg.EXPOSURE_MIN_US,
                               min(self._cfg.EXPOSURE_MAX_US, self._current_us))

        try:
            self._cam.set_exposure_time(self._current_us)
        except Exception:
            pass  # 部分设备可能不支持运行时调整

        print_debug(f"[曝光] 调整至 {self._current_us}μs (丢{self._lost_frames}帧)")

    def reset(self):
        """重置到初始值。"""
        self._current_us = self._cfg.EXPOSURE_US
        self._current_gain = self._cfg.ANALOG_GAIN_DB
        self._lost_frames = 0
        self._adapting = False
        self._steps_tried = 0

    @property
    def current_exposure_us(self):
        return self._current_us


# =============================================================================
# 模型与设备选择
# =============================================================================

# 钢珠检测置信度阈值 — 动态调整
# 静态/慢速时用较高阈值抑制误检, 快速运动时自动降低补偿运动模糊
DETECTION_CONFIDENCE_STATIC = 0.40   # 球静止/慢速时的阈值 (工业镜头干净, 可设高)
DETECTION_CONFIDENCE_FAST = 0.18     # 球快速运动时的最低阈值 (补偿运动模糊)
# 当前使用的动态阈值 (每帧根据球速自动在两者之间插值)
DETECTION_CONFIDENCE = DETECTION_CONFIDENCE_STATIC

# NMS IoU 阈值
DETECTION_IOU_TH = 0.45

# 模型路径
MAIXCAM_MODEL_PATH = "/root/models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud"



def model_path_for_device(device_name):
    """根据设备名称返回匹配的 YOLO26 模型路径。"""
    normalized_name = device_name.strip().lower()
    if normalized_name in ("maixcam", "maixcam-pro", "maixcam_pro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError(f"unsupported device: {device_name}")


# =============================================================================
# 标定参数
# =============================================================================

AXIS_START_PX = (40, 112)
AXIS_END_PX = (280, 112)
AXIS_START_CM = -12.5  # 左侧为负12.5cm
AXIS_END_CM = 12.5     # 右侧为正12.5cm（中心为0）


# =============================================================================
# 初始化
# =============================================================================

model = model_path_for_device(sys.device_name())

detector = nn.YOLO26(
    model=model,
    dual_buff=not LOW_LATENCY_MODE,
)

input_w = detector.input_width()
input_h = detector.input_height()
AXIS_START_PX = (5, input_h // 2)
AXIS_END_PX = (input_w - 5, input_h // 2)

print(f"[INFO] 设备: {sys.device_name()}")
print(f"[INFO] 模型输入: {input_w}x{input_h}")
print(f"[INFO] 低延迟模式: {LOW_LATENCY_MODE}")
print(f"[INFO] 动态置信度: {DETECTION_CONFIDENCE_FAST}~{DETECTION_CONFIDENCE_STATIC} (高速~静止)")
print(f"[INFO] 曝光时间: {CAM_CFG.EXPOSURE_US}μs (工业镜头快速运动优化)")
print(f"[INFO] 滤波器: alpha_fast={0.95} beta_fast={0.18} max_v={1200}px/s")

# 初始化摄像头
cam = camera.Camera(input_w, input_h, detector.input_format())

# --- 应用工业镜头参数 ---
try:
    cam.set_exposure_time(CAM_CFG.EXPOSURE_US)
    print(f"[INFO] 曝光时间: {CAM_CFG.EXPOSURE_US}μs")
except Exception as e:
    print(f"[WARN] 设置曝光时间失败: {e}")

try:
    if hasattr(cam, 'set_analog_gain'):
        cam.set_analog_gain(CAM_CFG.ANALOG_GAIN_DB)
        print(f"[INFO] 模拟增益: {CAM_CFG.ANALOG_GAIN_DB}dB")
except Exception as e:
    print(f"[WARN] 设置模拟增益失败: {e}")

try:
    if CAM_CFG.AUTO_EXPOSURE_LOCK and hasattr(cam, 'set_auto_exposure'):
        cam.set_auto_exposure(False)
        print("[INFO] 自动曝光: 锁定")
except Exception as e:
    print(f"[WARN] 锁定自动曝光失败: {e}")

try:
    if CAM_CFG.AUTO_AWB_LOCK and hasattr(cam, 'set_auto_white_balance'):
        cam.set_auto_white_balance(False)
        print("[INFO] 自动白平衡: 锁定")
except Exception as e:
    print(f"[WARN] 锁定白平衡失败: {e}")

# 显示
disp = display.Display()

# 位置滤波器 (快速运动优化参数)
position_filter = AdaptiveAlphaBetaFilter(
    alpha_slow=0.18,     # 稳态: 稍高以应对微振动
    alpha_fast=0.95,     # 快速: 几乎直接跟随测量值
    beta_slow=0.015,     # 稳态速度感知更灵敏
    beta_fast=0.18,      # 高动态速度估计激进
    fast_error_px=8.0,   # 更低阈值 → 更早进入快跟踪
    reset_ms=200,        # 更快重置
    max_velocity_px_s=1200,  # 高速容忍
)

# 自适应曝光调节器
exposure_adapter = ExposureAdapter(cam, CAM_CFG)

# --- 视频流 (推流使用低分辨率通道, 与检测分离) ---
jpeg_server = None

if USE_WEBRTC:
    from maix import webrtc
    cam2 = cam.add_channel(STREAM_WIDTH, STREAM_HEIGHT, image.Format.FMT_YVU420SP)
    webrtc_server = webrtc.WebRTC()
    webrtc_server.bind_camera(cam2)
    webrtc_server.start()
    webrtc_url = webrtc_server.get_url().replace("0.0.0.0", "192.168.0.1")
    print(f"[INFO] WebRTC: {webrtc_url}")

# --- 串口初始化 (与天猛星MSPM0G3507通信) ---
uart_device = None
if USE_UART:
    try:
        # 步骤1: 设置引脚功能映射 (pinmap)
        pinmap.set_pin_function(UART_PIN_TX, UART_FUNC_TX)
        pinmap.set_pin_function(UART_PIN_RX, UART_FUNC_RX)

        # 步骤2: 初始化UART设备
        uart_device = uart.UART(UART_DEVICE, UART_BAUDRATE)

        print(f"[INFO] 串口已打开: {UART_DEVICE} @ {UART_BAUDRATE} bps")
        print(f"[INFO] 物理引脚: TX={UART_PIN_TX}, RX={UART_PIN_RX}")
        print(f"[INFO] 数据格式: ASCII (位置cm + 换行)")

    except Exception as e:
        print(f"[ERROR] 串口初始化失败: {e}")
        uart_device = None

# 验证标定
validate_calibration(
    AXIS_START_PX, AXIS_END_PX,
    AXIS_START_CM, AXIS_END_CM,
    input_w, input_h,
)
print(f"[INFO] 标定轴: {AXIS_START_PX} → {AXIS_END_PX}  ({AXIS_START_CM}cm → {AXIS_END_CM}cm)")


# =============================================================================
# 绘制函数
# =============================================================================

# 预定义颜色 (只创建一次, 减少内存分配)
AXIS_COLOR = image.Color.from_rgb(0, 220, 255)
ZERO_COLOR = image.Color.from_rgb(255, 220, 0)
PANEL_COLOR = image.Color.from_rgb(0, 0, 0)
PREDICT_COLOR = image.Color.from_rgb(255, 128, 0)


def draw_calibration_axis(img):
    """绘制标定轴、端点、零点标记。"""
    img.draw_line(
        AXIS_START_PX[0], AXIS_START_PX[1],
        AXIS_END_PX[0], AXIS_END_PX[1],
        AXIS_COLOR, 2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], AXIS_COLOR, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], AXIS_COLOR, 7, 2)
    zero_ratio = (0.0 - AXIS_START_CM) / (AXIS_END_CM - AXIS_START_CM)
    zero_x, zero_y = axis_point(zero_ratio, AXIS_START_PX, AXIS_END_PX)
    img.draw_cross(int(zero_x), int(zero_y), ZERO_COLOR, 9, 2)


# =============================================================================
# 主循环
# =============================================================================

last_ms = time.ticks_ms()
frame_count = 0
fps_update_ms = last_ms
fps_display = 0
predicted_x = None
predicted_y = None
last_uart_send_ms = 0  # 上次串口发送时间
uart_send_count = 0    # 串口发送计数
velocity_cm_s_signed = 0.0  # 带方向的速度（全局变量）

print("[INFO] 主循环开始")

while not app.need_exit():
    now_ms = time.ticks_ms()
    loop_ms = now_ms - last_ms
    last_ms = now_ms

    # --- 1. 读取图像 ---
    img = cam.read()

    # --- 2. 镜头畸变校正 (按需) ---
    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)

    # --- 3. YOLO 检测 (动态置信度: 球速越快阈值越低, 补偿运动模糊) ---
    speed_px_s = math.hypot(position_filter.vx, position_filter.vy) if position_filter.x is not None else 0
    # 速度 → 置信度线性插值: 0 px/s → STATIC, 600+ px/s → FAST
    speed_ratio = min(1.0, speed_px_s / 600.0)
    dynamic_conf = (DETECTION_CONFIDENCE_STATIC
                    + speed_ratio * (DETECTION_CONFIDENCE_FAST - DETECTION_CONFIDENCE_STATIC))
    objs = detector.detect(img, conf_th=dynamic_conf, iou_th=DETECTION_IOU_TH)
    ball = max(objs, key=lambda obj: obj.score) if objs else None

    # --- 4. 状态分析与滤波 ---
    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        filtered_x, filtered_y = position_filter.update(raw_x, raw_y, now_ms)
        predicted_x, predicted_y = filtered_x, filtered_y

        position_cm, axis_ratio, _axis_distance = position_from_pixel(
            (filtered_x, filtered_y),
            AXIS_START_PX, AXIS_END_PX,
            AXIS_START_CM, AXIS_END_CM,
        )
        projected_x, projected_y = axis_point(axis_ratio, AXIS_START_PX, AXIS_END_PX)

        # 自适应曝光: 通知追踪成功
        exposure_adapter.on_ball_found()

        # --- 计算速度 (像素/秒 转换为 cm/秒) ---
        # 滤波器已经提供了像素速度 (vx, vy)
        velocity_px_s = math.hypot(position_filter.vx, position_filter.vy)

        # 将像素速度转换为物理速度 (cm/s)
        # 计算标定轴的像素长度和物理长度
        axis_length_px = math.hypot(
            AXIS_END_PX[0] - AXIS_START_PX[0],
            AXIS_END_PX[1] - AXIS_START_PX[1]
        )
        axis_length_cm = abs(AXIS_END_CM - AXIS_START_CM)

        # 像素到cm的转换比例
        px_to_cm_ratio = axis_length_cm / axis_length_px if axis_length_px > 0 else 0

        # 速度转换: px/s -> cm/s
        # 速度校正系数: 根据实际测试调整
        VELOCITY_SCALE = 0.3  # 提升到30%
        velocity_cm_s = velocity_px_s * px_to_cm_ratio * VELOCITY_SCALE

        # 计算速度方向（正负）
        # 标定轴从左到右: AXIS_START_PX -> AXIS_END_PX
        # vx > 0 表示向右移动（正方向）, vx < 0 表示向左移动（负方向）
        axis_direction_x = AXIS_END_PX[0] - AXIS_START_PX[0]
        axis_direction_y = AXIS_END_PX[1] - AXIS_START_PX[1]
        axis_length_sq = axis_direction_x * axis_direction_x + axis_direction_y * axis_direction_y

        if axis_length_sq > 0:
            # 将速度投影到标定轴上，得到带方向的速度
            velocity_on_axis = (position_filter.vx * axis_direction_x +
                               position_filter.vy * axis_direction_y) / math.sqrt(axis_length_sq)
            velocity_cm_s_signed = velocity_on_axis * px_to_cm_ratio * VELOCITY_SCALE
        else:
            velocity_cm_s_signed = 0.0

        # 速度平滑: 过滤掉小幅度抖动，显示更稳定
        if abs(velocity_cm_s_signed) < 0.5:
            velocity_cm_s_signed = 0.0  # 小于0.5cm/s视为静止

        # --- 串口数据发送 (向天猛星MSPM0发送球位置和速度) ---
        # 频率控制: 避免OLED刷新过快，同时不影响检测FPS
        if uart_device is not None and (now_ms - last_uart_send_ms >= UART_SEND_INTERVAL_MS):
            try:
                # ASCII格式: 位置cm + 速度cm/s (带正负号) + 换行符
                # 格式: "P:12.34,V:+5.67\n" 或 "P:-8.50,V:-12.30\n"
                # P = Position (位置cm), V = Velocity (速度cm/s，正=向右，负=向左)
                uart_device.write_str(f"P:{position_cm:.2f},V:{velocity_cm_s_signed:+.2f}\n")
                last_uart_send_ms = now_ms
                uart_send_count += 1
            except Exception:
                pass  # 静默处理错误，不影响主循环性能

    else:
        # 未检测到球: 尝试用速度模型预测位置
        position_filter.mark_missing(now_ms)
        pred_x, pred_y = position_filter.predict(now_ms)
        predicted_x, predicted_y = pred_x, pred_y
        velocity_cm_s_signed = 0.0  # 丢球时速度为0

        # 自适应曝光: 通知丢球, 可能触发曝光调整
        exposure_adapter.on_ball_lost()

        # --- 串口数据发送 (丢球状态) ---
        # 丢球时也可以发送状态通知MCU (可选, 根据协议需求决定是否发送)
        # 如果MSPM0需要知道丢球状态, 取消下面注释:
        # tx_data = f"$BALL,0,0.00,0.0,0.00,{now_ms}*\n"
        # if uart_device is not None:
        #     try:
        #         uart_device.write(tx_data.encode('utf-8'))
        #         print_debug(f"[UART TX] {tx_data.strip()}")
        #     except Exception as e:
        #         print_debug(f"[UART ERROR] {e}")

        ball = None

    # --- 5. 绘制 ---
    draw_calibration_axis(img)

    if ball is not None:
        # 检测框 (始终显示)
        img.draw_rect(ball.x, ball.y, ball.w, ball.h, color=image.COLOR_GREEN, thickness=2)

        if DEBUG_LOG:
            # 调试模式: 绘制滤波十字、投影十字、标签、面板
            img.draw_cross(int(filtered_x), int(filtered_y), image.COLOR_GREEN, 12, 2)
            img.draw_cross(int(projected_x), int(projected_y), ZERO_COLOR, 7, 2)
            msg = (f"{detector.labels[ball.class_id]}: {ball.score:.2f} "
                   f"px=({filtered_x:.1f},{filtered_y:.1f})")
            img.draw_string(ball.x, ball.y, msg, color=image.COLOR_RED)
            img.draw_rect(0, 0, input_w, 35, PANEL_COLOR, -1)

        # 位置和速度信息 (始终显示，速度带正负号)
        # 正速度=向右移动，负速度=向左移动
        img.draw_string(8, 5, f"BALL {position_cm:+.2f}cm {velocity_cm_s_signed:+.1f}cm/s",
                        color=image.COLOR_WHITE, scale=1.2, thickness=2)
    else:
        # 丢球显示
        img.draw_rect(0, 0, input_w, 35, PANEL_COLOR, -1)
        status_text = "BALL LOST"
        if predicted_x is not None:
            status_text += f" (pred: {predicted_x:.0f},{predicted_y:.0f})"
        img.draw_string(8, 5, status_text,
                        color=image.COLOR_RED, scale=1.4, thickness=2)

        if DEBUG_LOG and predicted_x is not None:
            img.draw_cross(int(predicted_x), int(predicted_y), PREDICT_COLOR, 10, 1)

    # --- FPS 显示 ---
    frame_count += 1
    if now_ms - fps_update_ms >= 1000:
        fps_display = frame_count * 1000 // (now_ms - fps_update_ms)
        frame_count = 0
        fps_update_ms = now_ms

    fps_str = f"FPS:{fps_display}"
    fps_w = image.string_size(fps_str, scale=1.4, thickness=2).width()
    img.draw_string(input_w - fps_w - 5, 5, fps_str,
                    color=image.COLOR_GREEN, scale=1.4, thickness=2)

    # 调试打印
    print_debug(f"loop={loop_ms}ms  exp={exposure_adapter.current_exposure_us}μs  "
                f"conf={dynamic_conf:.2f}  speed={speed_px_s:.0f}px/s  fps={fps_display}")

    # --- 推流 ---
    if USE_JPEG and jpeg_server is not None:
        jpeg_server.write(img)

    # --- 显示 ---
    disp.show(img)

# --- 程序退出清理 ---
print("[INFO] 程序退出")

# 关闭串口
if uart_device is not None:
    try:
        uart_device.close()
        print("[INFO] 串口已关闭")
    except Exception as e:
        print(f"[WARN] 关闭串口时出错: {e}")
