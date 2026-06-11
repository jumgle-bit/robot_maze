# MazeRobot_Keil5_STM32F103C8T6_PA11_UART_03s_LeftHand

本工程为 STM32F103C8T6 + L298N + HC-SR04 + 双红外避障模块的小车迷宫寻迹 Keil5 工程。工程使用寄存器级最小库，不依赖 HAL 库。

## 1. 当前版本要点

1. 右侧红外传感器使用 PA11，左侧红外传感器使用 PA12。
2. 红外传感器检测到障碍物默认为低电平：

```c
#define IR_BLOCKED_LEVEL 0
```

3. 前向 HC-SR04 使用 PB14/PB15：TRIG 接 PB14，ECHO 接 PB15。
4. USART1 使用 PA9/PA10，默认 115200-8-N-1，每 0.3 s 输出一次超声波、红外和电机 PWM 状态。
5. 迷宫算法采用左手原则。
6. 左右 90°转弯使用差速转弯，避免原地甩头导致两侧红外丢失侧墙参考。
7. 死路掉头使用一侧正转、一侧反转的原地左旋，更容易完成 180°。
8. PWM、距离阈值和动作时间都集中在 `USER/config.h` 中调整。

## 2. 管脚分配

| 模块 | STM32 管脚 | 说明 |
|---|---|---|
| HC-SR04 TRIG | PB14 | 前方超声波触发 |
| HC-SR04 ECHO | PB15 | 前方超声波回波；PB15 为 FT 数字输入，可选分压保护 |
| 左红外 OUT | PA12 | 低电平表示检测到障碍物 |
| 右红外 OUT | PA11 | 低电平表示检测到障碍物 |
| USART1 TX | PA9 | 接 USB-TTL 的 RX |
| USART1 RX | PA10 | 接 USB-TTL 的 TX，当前预留接收 |
| L298N IN1 | PB6 | 左电机方向 |
| L298N IN2 | PB7 | 左电机方向 |
| L298N IN3 | PB8 | 右电机方向 |
| L298N IN4 | PB9 | 右电机方向 |
| L298N ENB | PA1 / TIM2_CH2 | 右电机 PWM |
| L298N ENA | PA2 / TIM2_CH3 | 左电机 PWM |

## 3. 主要调参位置

所有关键参数都在：

```text
USER/config.h
```

当前常用参数如下：

```c
#define IR_BLOCKED_LEVEL          0

#define MOTOR_FORWARD_PWM         750
#define MOTOR_SLOW_PWM            620
#define MOTOR_TURN_DELAY_PWM      620
#define MOTOR_TURN_INNER_PWM      520
#define MOTOR_TURN_OUTER_PWM      780
#define MOTOR_TURN_BACK_SPIN_PWM  820

#define UART_STATUS_ENABLE        1
#define UART1_BAUDRATE            115200U
#define UART_STATUS_INTERVAL_MS   300U

#define FRONT_SAFE_DISTANCE_CM    18
#define FRONT_SLOW_DISTANCE_CM    28

#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U

#define MAZE_LOOP_DELAY_MS        35
#define MAZE_FORWARD_CHECK_STEP_MS 40
#define MAZE_FORWARD_CHECK_PAUSE_MS 20
#define MAZE_STOP_BEFORE_TURN_MS  120
#define MAZE_TURN_DELAY_FORWARD_MS 220
#define MAZE_POST_TURN_FORWARD_MS 220
#define MAZE_TURN_STEP_MS         120
#define MAZE_TURN_STEP_PAUSE_MS   20
#define MAZE_TURN_90_MS           720
#define MAZE_TURN_BACK_MS         1500
```

90°转弯采用低速脉冲式差速转弯，由 `MOTOR_TURN_INNER_PWM`、`MOTOR_TURN_OUTER_PWM`、`MAZE_TURN_STEP_MS`、`MAZE_TURN_STEP_PAUSE_MS` 和 `MAZE_TURN_90_MS` 调整。转角不足就增大 `MAZE_TURN_90_MS` 或略微提高外侧轮 PWM，转过头就减小。死路掉头使用一侧正转、一侧反转的原地掉头，由 `MOTOR_TURN_BACK_SPIN_PWM` 和 `MAZE_TURN_BACK_MS` 调整。

检测到需要转弯后，如果前方距离安全，小车会先用 `MOTOR_TURN_DELAY_PWM` 低速前进 `MAZE_TURN_DELAY_FORWARD_MS`，让车身更深入路口后再开始转弯。

转弯结束后会锁定触发转弯的一侧。例如右侧空旷触发右转后，会锁定右侧；锁定期间不响应另一侧红外，也不会恢复普通直行决策，而是继续朝锁定侧做小段差速修正，直到右侧红外重新检测到墙/障碍后才解锁。

## 4. 串口状态输出

USART1 使用 `PA9/PA10`，串口参数为：

```text
115200 bit/s，8 位数据位，无校验，1 位停止位
```

输出格式示例：

```text
US=18cm,US_OK=1,IR_L=BLOCK,IR_R=CLEAR,PWM_L=+600,PWM_R=+800
```

字段说明：

- `US`：前方超声波距离。
- `US_OK`：`1` 表示本次测距有效，`0` 表示本次测距失败。
- `IR_L` / `IR_R`：左右红外状态，`BLOCK` 为检测到障碍物，`CLEAR` 为无障碍。
- `PWM_L` / `PWM_R`：左右电机实际 PWM，正数为前进，负数为后退，`0` 为停止。

输出周期约为 0.3 s。接线时注意共地：`PA9` 接 USB-TTL 的 `RX`，`PA10` 接 USB-TTL 的 `TX`，STM32 `GND` 接 USB-TTL `GND`。

## 5. 左手原则迷宫决策逻辑

正式运行在 `APP/maze.c` 的 `Maze_Task()` 中：

```text
1. 左侧无障碍：优先左转；
2. 左侧有障碍但前方安全：继续前进；
3. 左侧和前方不可通行但右侧无障碍：右转；
4. 左、前、右均不可通行：原地左旋掉头。
```

转弯前如果前方距离大于 `FRONT_SAFE_DISTANCE_CM`，小车会先短距离前进，让车身中心进入路口后再差速转弯。转弯后如果前方仍安全，也会短距离前进，减少在路口边缘反复触发传感器。短距离前进期间会按 `MAZE_FORWARD_CHECK_STEP_MS` 分段重新测距，发现前方不安全会立即停车。

## 6. Keil5 打开方式

打开：

```text
MDK-ARM/MazeRobot.uvprojx
```

当前工程已整理为正式运行版，上电后直接进入迷宫寻迹主循环。

## 7. HC-SR04 电平保护

HC-SR04 通常使用 5V 供电，其 ECHO 输出约为 5V。STM32F103 的 PB15 属于 FT 数字输入，因此可以直接连接。若使用来源不明的兼容板或希望增加保护，也可以增加分压电阻或电平转换模块。
