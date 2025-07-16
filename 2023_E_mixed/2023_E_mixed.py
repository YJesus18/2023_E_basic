import sensor, image, time, math, ustruct, lcd
from machine import UART
from fpioa_manager import fm
from Maix import GPIO

# 初始化串口
fm.register(24, fm.fpioa.UART1_TX, force=True)
fm.register(25, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(False)

lcd.init()
clock = time.clock()

thresholds = (0, 34, -128, 127, -128, 127)
roi = (80,35,160,170)
flage = 100

# 发送激光目标点
def outuart(x,y,flage):
    global uart;
    data = ustruct.pack("<bbhhhb",
                   0x2C,
                   0x12, #命令码
                   int(x*100),
                   int(y*100),
                   int(flage),
                   0x5B)
    uart.write(data);
    time.sleep_ms(20)


def move_to_center():
    print(">> 正在复位...")
    while True:
        dx, dy = 0, 0
        img = sensor.snapshot()
        blobs = img.find_blobs([thresholds], roi=roi)
        if blobs:
            b = max(blobs, key=lambda b: b.area())
            cx = b.cx() - 160
            cy = b.cy() - 120
            dx, dy = -cx, -cy
            outuart(dx, dy, 250)
            img.draw_cross(b.cx(), b.cy(), color=(255, 0, 0))
            lcd.display(img)
            print("纠正：", dx, dy)
            if abs(cx) < 5 and abs(cy) < 5:
                outuart(0, 0, 100)  # 表示复位完成
                print(">> 复位完成")
                break
        else:
            print("未找到色块")
        time.sleep_ms(200)

def move_screen_frame():
    print(">> 沿屏幕边线顺时针运动...")
    points = [(20,20),(300,20),(300,220),(20,220)]
    start = time.ticks_ms()
    for i, (x, y) in enumerate(points):
        outuart(x-160, y-120, 240+i)
        print("> 点 {}: ({},{})".format(i, x, y))
        time.sleep_ms(7000)  # 每个点最多7.5秒
    outuart(0,0,100)
    print(">> 边线运动完成")

def move_a4_tape():
    print(">> 检测A4黑边框...")
    start = time.ticks_ms()
    while True:
        clock.tick()
        fps =clock.fps()#获取当前显示帧率
        img = sensor.snapshot()
        img.draw_rectangle((roi),color = (255,0,0), thickness=1,fill=False)
        #img.draw_circle(160+22, 120+2, 3, color = (0, 0, 0), thickness = 1, fill = True)

        img.draw_cross(160,120,color = (255,0,0),thickness=1) #画红色十字位置与激光在一个位置上

        for blob in img.find_blobs([thresholds],roi=roi,x_stride = 1, y_stride=1,pixels_threshold=300, area_threshold=300, merge=True):
            print("返回色块面积")
            print(blob.area()) #"色块面积"指的是像素点的数量，并非简单的计算面积，该函数是用来判断黑色边框是否真实存在
            if blob:
                b = max(blob, key=lambda b: b.area())
                x, y, w, h = b.rect()
                A, B, C, D = (x, y), (x+w, y), (x+w, y+h), (x, y+h)
                corners = [A, B, C, D]
                #沿着四个角点画一个边框
                img.draw_line(A[0], A[1], B[0], B[1], color=(0, 255, 0), thickness=2)
                img.draw_line(B[0], B[1], C[0], C[1], color=(0, 255, 0), thickness=2)
                img.draw_line(C[0], C[1], D[0], D[1], color=(0, 255, 0), thickness=2)
                img.draw_line(D[0], D[1], A[0], A[1], color=(0, 255, 0), thickness=2)

                #在4个角点处标注A、B、C、D
                img.draw_string((corners[0])[0],(corners[0])[1],"A",color = (0,255,255),scale=4, mono_space=False)
                img.draw_string((corners[1])[0],(corners[1])[1],"B",color = (0,255,255),scale=4, mono_space=False)
                img.draw_string((corners[2])[0],(corners[2])[1],"C",color = (0,255,255),scale=4, mono_space=False)
                img.draw_string((corners[3])[0],(corners[3])[1],"D",color = (0,255,255),scale=4, mono_space=False)

                for i, (px, py) in enumerate(corners):
                    dx = px - 160
                    dy = py - 120
                    outuart(dx, dy, 230+i)
                    time.sleep_ms(7000)
                      # 完成信号
                    print(">> 胶带打点完成")


            else:
                print("未识别到黑边框")


            if time.ticks_diff(time.ticks_ms(), start) > 30000:
                print(">> 超时未完成")
                outuart(0, 0, 255)  # 失败信号
                break

        lcd.display(img)


# 主循环
while True:
    clock.tick()
    fps =clock.fps()#获取当前显示帧率
    if flage == 100:

        img = sensor.snapshot()
        img.draw_string(0,0, "Waiting...", color = (255,0,0),scale=2)
        img.draw_rectangle((roi),color = (255,0,0), thickness=1,fill=False)
        img.draw_cross(160,120,color = (255,0,0),thickness=1) #画红色十字位置与激光在一个位置上
        print("等待电控给我值")
        data = uart.read(4)  # 一次读取4个字节
        if data and len(data) == 4:
            flage_val = ustruct.unpack("<i", data)[0]  # 按小端格式解包为整数
            print("接收到值:", flage_val)
            if flage_val == 150:
                flage = 0
            else:
                flage = 100
        else:
            print("接收数据失败或超时")
            flage = 100

        lcd.display(img)

    if flage != 100:  # 当串口接收到下位机发来的信号（150），开始工作
        img = sensor.snapshot()

        img.draw_rectangle((roi),color = (0,255,0), thickness=1,fill=False)
        #img.draw_circle(160, 120, 3, color = (0, 0, 0), thickness = 1, fill = True)

        img.draw_cross(160,120,color = (0,255,0),thickness=1) #画红色十字位置与激光在一个位置上

        blobs = img.find_blobs([thresholds], roi=roi)
        if blobs:
            b = max(blobs, key=lambda b: b.area())
            x, y, w, h = b.rect()
            A, B, C, D = (x, y), (x+w, y), (x+w, y+h), (x, y+h)
            corners = [A, B, C, D]
            #沿着四个角点画一个边框
            img.draw_line(A[0], A[1], B[0], B[1], color=(0, 255, 0), thickness=2)
            img.draw_line(B[0], B[1], C[0], C[1], color=(0, 255, 0), thickness=2)
            img.draw_line(C[0], C[1], D[0], D[1], color=(0, 255, 0), thickness=2)
            img.draw_line(D[0], D[1], A[0], A[1], color=(0, 255, 0), thickness=2)

            #在4个角点处标注A、B、C、D
            img.draw_string((corners[0])[0],(corners[0])[1],"A",color = (0,255,255),scale=4, mono_space=False)
            img.draw_string((corners[1])[0],(corners[1])[1],"B",color = (0,255,255),scale=4, mono_space=False)
            img.draw_string((corners[2])[0],(corners[2])[1],"C",color = (0,255,255),scale=4, mono_space=False)
            img.draw_string((corners[3])[0],(corners[3])[1],"D",color = (0,255,255),scale=4, mono_space=False)

            for i, (px, py) in enumerate(corners):
                dx = px - 160
                dy = py - 120
                outuart(dx, dy, i)
                time.sleep_ms(50)

            outuart(0, 0, 100)  # 循环结束后再发完成信号
            img.draw_string(0,0, "Finish", color = (0,255,0),scale=2)

            flage = 100
            print(">> 胶带打点完成")
            lcd.display(img)
            time.sleep_ms(5000)

        else:
            print("未识别到黑边框")
            lcd.display(img)
