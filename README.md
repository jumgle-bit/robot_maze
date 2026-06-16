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
7. 死路掉头使用一侧正转、一侧反转的分段原地左旋，前方超声波恢复到安全距离后提前退出。
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

#define MOTOR_FORWARD_PWM         650
#define MOTOR_SLOW_PWM            620
#define MOTOR_TURN_LOST_PWM       560
#define MOTOR_TURN_INNER_PWM      550
#define MOTOR_TURN_OUTER_PWM      980
#define MOTOR_TURN_BACK_SPIN_PWM  650

#define UART_STATUS_ENABLE        1
#define UART1_BAUDRATE            115200U
#define UART_STATUS_INTERVAL_MS   300U

#define FRONT_SAFE_DISTANCE_CM    10
#define FRONT_SLOW_DISTANCE_CM    15
#define FRONT_TURN_BACK_CLEAR_CM  15
#define MAZE_IR_CONFIRM_COUNT     2

#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U

#define MAZE_LOOP_DELAY_MS        35
#define MAZE_FORWARD_CHECK_STEP_MS 40
#define MAZE_FORWARD_CHECK_PAUSE_MS 20
#define MAZE_STOP_BEFORE_TURN_MS  120
#define MAZE_POST_TURN_FORWARD_MS 220
#define MAZE_TURN_STEP_MS         45
#define MAZE_TURN_STEP_PAUSE_MS   20
#define MAZE_TURN_LOST_FORWARD_MS 200
#define MAZE_TURN_MIN_MS          260
#define MAZE_TURN_MAX_MS          1200
#define MAZE_TURN_BACK_MIN_MS     260
#define MAZE_TURN_BACK_STEP_MS    80
#define MAZE_TURN_BACK_CHECK_PAUSE_MS 20
#define MAZE_TURN_BACK_MAX_MS     1800
```

90°转弯采用状态机分段差速转弯，由 `MOTOR_TURN_INNER_PWM`、`MOTOR_TURN_OUTER_PWM`、`MAZE_TURN_STEP_MS`、`MAZE_TURN_STEP_PAUSE_MS`、`MAZE_TURN_MIN_MS` 和 `MAZE_TURN_MAX_MS` 调整。达到最小转弯时间后，对应侧红外连续 `MAZE_IR_CONFIRM_COUNT` 次检测到墙就退出；如果一直没有确认，则达到 `MAZE_TURN_MAX_MS` 后兜底退出。转弯期间若左右红外都检测不到墙且前方安全，每次转弯状态最多会用 `MOTOR_TURN_LOST_PWM` 前进 `MAZE_TURN_LOST_FORWARD_MS` 做一次补位，然后继续原转弯状态。死路掉头使用一侧正转、一侧反转的分段原地左旋，由 `MOTOR_TURN_BACK_SPIN_PWM`、`MAZE_TURN_BACK_STEP_MS` 和 `MAZE_TURN_BACK_CHECK_PAUSE_MS` 调整动作细腻程度；达到 `MAZE_TURN_BACK_MIN_MS` 后，当前方超声波距离连续大于 `FRONT_TURN_BACK_CLEAR_CM` 时提前退出，`MAZE_TURN_BACK_MAX_MS` 作为超声波异常或距离未恢复时的最大兜底时间。

检测到需要转弯后，小车只会先停止 `MAZE_STOP_BEFORE_TURN_MS`，让车身稳定后直接进入转弯状态，不再执行转弯前前进动作。

转弯期间进入独立状态，不再执行新的路口决策。例如右侧空旷触发右转后，会持续执行右转状态；达到最小转弯时间后，只有右侧红外连续检测到墙，或达到最大转弯时间，才会回到普通左手原则决策。

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
4. 左、前、右均不可通行：分段原地左旋掉头，直到前方超声波恢复安全距离或达到最大兜底时间。
```

转弯后如果前方仍安全，小车会短距离前进，减少在路口边缘反复触发传感器。短距离前进期间会按 `MAZE_FORWARD_CHECK_STEP_MS` 分段重新测距，发现前方不安全会立即停车。

## 6. Keil5 打开方式

打开：

```text
MDK-ARM/MazeRobot.uvprojx
```

当前工程已整理为正式运行版，上电后直接进入迷宫寻迹主循环。

## 7. HC-SR04 电平保护

HC-SR04 通常使用 5V 供电，其 ECHO 输出约为 5V。STM32F103 的 PB15 属于 FT 数字输入，因此可以直接连接。若使用来源不明的兼容板或希望增加保护，也可以增加分压电阻或电平转换模块。
