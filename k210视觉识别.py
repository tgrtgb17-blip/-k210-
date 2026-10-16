import sensor, image, time, lcd
from fpioa_manager import fm
import ustruct
import machine
import collections
import math

MIN_PIXELS = 100
SEND_THRESHOLD = 1

PIXEL_TO_MM_X = 10/6.5
PIXEL_TO_MM_y = 10/6.7
center_x = 160
center_y = 50

last_send_x = -999
last_send_y = -999
fps_count = 0
lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
lcd.rotation(2)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)

fm.register(6, fm.fpioa.UART2_TX, force=True)
fm.register(7, fm.fpioa.UART2_RX, force=True)

uart = machine.UART(1,115200,8,0,1, timeout=1000, read_buf_len=4096)
clock = time.clock()

#green_thresholds= (11, 62, -128, -10, -45, 70)
blue_thresholds = (65, 0, -128, 34, -128, -19)
red_thresholds = (26, 41, 39, 11, 28, 7) #(28, 52, 18, 106, 107, 8)

# =========================
# 串口发送函数
# 将识别到的颜色、中心坐标和角度发送给STM32
# 数据格式：
# 帧头(0x31,0x51) + color + cx + cy + angle + 帧尾(0x5B)
# =========================
def sending_data(color,cx,cy,angle):
    data = ustruct.pack(
       "<BBBhhhB",
       0x31,0x51,color,int(cx),int(cy),int(angle),0x5B)
    uart.write(data)
# =========================
# 坐标平滑滤波
#
# 功能：
# 本程序采用：
# 限幅滤波,滑动平均滤波使输出坐标平滑
# =========================
class Smooth:
    def __init__(self, max_error, step):
        self.max_error = max_error
        self.step = step

        self.x_queue = []
        self.y_queue = []

        self.last_x = None
        self.last_y = None

    def get_smooth(self, raw_x, raw_y):

        if self.last_x is not None:
            if abs(raw_x - self.last_x) > self.max_error:
                raw_x = self.last_x

        if self.last_y is not None:
            if abs(raw_y - self.last_y) > self.max_error:
                raw_y = self.last_y

        self.last_x = raw_x
        self.last_y = raw_y

        self.x_queue.append(raw_x)
        self.y_queue.append(raw_y)

        if len(self.x_queue) > self.step:
            self.x_queue.pop(0)

        if len(self.y_queue) > self.step:
            self.y_queue.pop(0)

        avg_x = sum(self.x_queue) / len(self.x_queue)
        avg_y = sum(self.y_queue) / len(self.y_queue)
        return int(avg_x), int(avg_y)

    def reset(self):
        self.last_x = None
        self.last_y = None
        self.x_queue = []
        self.y_queue = []
# =========================
# 角度指数滤波器
# 功能：
#使用指数滤波降低角度抖动
# =========================
class AngleFilter:

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, raw):

        if self.value is None:
            self.value = raw
            return raw

        diff = raw - self.value

        if diff > 90:
            raw -= 180

        elif diff < -90:
            raw += 180

        self.value = (
            self.alpha * raw +
            (1-self.alpha) * self.value
        )

        while self.value > 90:
            self.value -= 180

        while self.value < -90:
            self.value += 180

        return self.value
last_send = 0
last_color = None
angle = 0
center_smooth = Smooth(max_error=20, step=5)
rect_smooth = Smooth(max_error=20, step=5)
angle_smooth = AngleFilter(alpha=0.25)
while(True):
    clock.tick()
    img = sensor.snapshot()
    img.lens_corr(strength=1.8, zoom=1.0)
    red_blobs = img.find_blobs([red_thresholds], x_stride=10, y_stride=10, pixels_threshold=225, merge=True)
    blue_blobs = img.find_blobs([blue_thresholds], x_stride=10, y_stride=10, pixels_threshold=225, merge=True)
    rects = img.find_rects(
        roi=(80,60,160,120),
        threshold=10000
    )

    current_color = None
    if red_blobs:
        current_color = 'R'
    elif blue_blobs:
        current_color = 'B'
    if current_color != last_color:
        center_smooth.reset()
        rect_smooth.reset()
        angle_smooth.value = None
        last_color = current_color

    img.draw_cross(160, 50,size=200,color=(0,255,0))

    if red_blobs:
        blob = max(red_blobs, key=lambda b:b.pixels())
        #blob = red_blobs[0]

        if blob.pixels() < MIN_PIXELS:
            continue
        color_status = ord('R')
        print("找到", len(red_blobs), "个红色色块")
        raw_left_x = blob[0]
        raw_left_y = blob[1]
        raw_x = blob[5]
        raw_y = blob[6]
        smooth_cx, smooth_cy = center_smooth.get_smooth(raw_x,raw_y)

        x_mm = (smooth_cx - 160) * PIXEL_TO_MM_X
        y_mm = (smooth_cy - 50) * PIXEL_TO_MM_y


        smooth_left_x, smooth_left_y = rect_smooth.get_smooth(raw_left_x,raw_left_y)
        img.draw_rectangle(int(smooth_left_x),int(smooth_left_y),blob[2],blob[3],color=(255,255,255))
        img.draw_cross(int(smooth_cx),int(smooth_cy),color=(255,255,255))
        img.draw_string(smooth_left_x,smooth_left_y-10,"red",color=(0,0,255))
        try:
            angle = math.degrees(blob.rotation())
            while angle > 90:
                angle -= 180

            while angle < -90:
                angle += 180
            angle = angle_smooth.update(angle)
        except Exception as e:
            print("rotation error:", e)
        #for i, blob in enumerate(blob):
        print("红色块x=%d, y=%d, w=%d, h=%d, 旋转角=%d" % (x_mm,y_mm, blob[2], blob[3] , angle))
        if time.ticks_diff(time.ticks_ms(), last_send) > 80:

            if abs(smooth_cx - last_send_x) > SEND_THRESHOLD or \
               abs(smooth_cy - last_send_y) > SEND_THRESHOLD:

                sending_data(
                    color_status,
                    x_mm,
                    y_mm,
                    angle
                )

                last_send_x = smooth_cx
                last_send_y = smooth_cy

            last_send = time.ticks_ms()
    elif blue_blobs:
        blob = max(blue_blobs, key=lambda b:b.pixels())
        if blob.pixels() < MIN_PIXELS:
            continue
        color_status = ord('B')
        print("找到", len(blue_blobs), "个蓝色色块")
        raw_left_x = blob[0]
        raw_left_y = blob[1]
        raw_x = blob[5]
        raw_y = blob[6]
        smooth_cx, smooth_cy = center_smooth.get_smooth(raw_x,raw_y)

        x_mm = (smooth_cx - 160) * PIXEL_TO_MM_X
        y_mm = (smooth_cy - 50) * PIXEL_TO_MM_y

        smooth_left_x, smooth_left_y = rect_smooth.get_smooth(raw_left_x,raw_left_y)
        img.draw_rectangle(int(smooth_left_x),int(smooth_left_y),blob[2],blob[3],color=(255,255,255))
        img.draw_cross(int(smooth_cx),int(smooth_cy),color=(255,255,255))
        img.draw_string(smooth_left_x,smooth_left_y-10,"blue",color=(0,0,255))
        #for i, blob in enumerate(blue_blobs):
        try:
            angle = math.degrees(blob.rotation())
            while angle > 90:
                angle -= 180

            while angle < -90:
                angle += 180
            angle = angle_smooth.update(angle)
        except Exception as e:
            print("rotation error:", e)
        print("蓝色块: x=%d, y=%d, w=%d, h=%d, 旋转角=%d" % (x_mm,y_mm , blob[2], blob[3] , angle))
        if time.ticks_diff(time.ticks_ms(), last_send) > 80:

            if abs(smooth_cx - last_send_x) > SEND_THRESHOLD or \
               abs(smooth_cy - last_send_y) > SEND_THRESHOLD:

                sending_data(
                    color_status,
                    x_mm,
                    y_mm,
                    angle
                )

                last_send_x = smooth_cx
                last_send_y = smooth_cy

            last_send = time.ticks_ms()

    lcd.display(img)
    fps_count += 1

    if fps_count >= 300:
        print("FPS:", clock.fps())
        fps_count = 0
