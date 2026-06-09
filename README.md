# MazeRobot_Keil5_STM32F103C8T6_PA11_UART_03s_LeftHand

本工程为 STM32F103C8T6 + L298N + HC-SR04 + 双红外避障模块的小车迷宫寻迹 Keil5 工程。

## 1. 当前版本修改点

1. 右侧红外传感器管脚保持 PA11。
2. 红外传感器检测到障碍物默认为低电平：

```c
#define IR_BLOCKED_LEVEL 0
```

3. USART1 串口实时输出前方超声波距离，PA9 为 TX，PA10 为 RX，默认 115200 波特率。
4. 串口距离输出周期已改为每 0.3 s 输出一次：

```c
#define UART_DISTANCE_INTERVAL_MS 300U
```

5. 迷宫算法已由右手原则改为左手原则。
6. 转弯方式仍为差速转弯：左右轮都向前转，内侧轮 PWM 小，外侧轮 PWM 大，不使用正反转原地转弯。
7. 差速驱动 PWM 已整体调低，降低小车速度和转弯冲击。

## 2. 管脚分配

| 模块 | STM32 管脚 | 说明 |
|---|---|---|
| HC-SR04 TRIG | PA6 | 输出 10 us 触发脉冲 |
| HC-SR04 ECHO | PA7 | 输入回波脉宽 |
| 左红外 OUT | PA12 | 低电平表示检测到障碍物 |
| 右红外 OUT | PA11 | 低电平表示检测到障碍物 |
| USART1 TX | PA9 | 串口实时发送超声波距离，接 USB-TTL RX |
| USART1 RX | PA10 | 串口接收预留，接 USB-TTL TX |
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

常用参数如下：

```c
#define UART_DISTANCE_ENABLE      1
#define UART1_BAUDRATE            115200U
#define UART_DISTANCE_INTERVAL_MS 300U

#define MOTOR_FORWARD_PWM         500
#define MOTOR_SLOW_PWM            350
#define MOTOR_TURN_INNER_PWM      180
#define MOTOR_TURN_OUTER_PWM      560
#define MOTOR_TURN_BACK_INNER_PWM 160
#define MOTOR_TURN_BACK_OUTER_PWM 580

#define FRONT_SAFE_DISTANCE_CM    18
#define FRONT_SLOW_DISTANCE_CM    28

#define MAZE_TURN_90_MS           650
#define MAZE_TURN_BACK_MS         1350
```

若小车转弯角度不足，增大 `MAZE_TURN_90_MS`；若转弯过头，减小该值。由于本版使用差速转弯，转弯半径比原地正反转更大，实际参数要根据迷宫宽度和小车轮距调整。

## 4. 串口距离输出

USART1 使用 `PA9/PA10`，串口参数为：

```text
115200 bit/s，8 位数据位，无校验，1 位停止位
```

输出格式为：

```text
US=25cm
```

输出周期约为 0.3 s。接线时注意共地：`PA9` 接 USB-TTL 的 `RX`，`PA10` 接 USB-TTL 的 `TX`，STM32 `GND` 接 USB-TTL `GND`。

## 5. 左手原则迷宫决策逻辑

正式运行在 `APP/maze.c` 的 `Maze_Task()` 中：

```text
1. 左侧无障碍：优先左转；
2. 左侧有障碍但前方安全：继续前进；
3. 左侧和前方不可通行但右侧无障碍：右转；
4. 左、前、右均不可通行：差速左转掉头。
```

## 6. Keil5 打开方式

打开：

```text
MDK-ARM/MazeRobot.uvprojx
```

工程使用寄存器级最小库，不依赖 HAL 库。首次上车建议先把 `APP_RUN_MODE` 改为 `APP_MODE_MOTOR_TEST`，确认左右电机方向和 PWM 有效，再切换回 `APP_MODE_MAZE`。
