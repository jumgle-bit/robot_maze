# MazeRobot STM32F103C8T6

本工程是基于 STM32F103C8T6、L298N、HC-SR04 和双红外避障模块的迷宫小车 Keil5 工程。代码使用寄存器级最小库，不依赖 HAL；上电后直接进入正式迷宫寻迹逻辑。

## 当前版本

- 迷宫策略：左手原则。
- 运动控制：显式状态机，普通前进、左转、右转、掉头分状态执行。
- 左右 90 度转弯：差速转弯，内侧轮慢、外侧轮快，不做原地甩头。
- 死路掉头：一侧正转、一侧反转，前方超声波恢复安全距离后退出。
- 调头保护：掉头完成后进入左转抑制锁，直到左侧红外重新检测到墙才允许再次左转。
- 丢墙补位：转弯中若左右红外都检测不到墙且前方安全，每次转弯状态最多前进补位一次。
- 串口状态：USART1 每 0.3 s 输出超声波、红外和电机 PWM 状态。

## 项目路径

```text
APP/        迷宫状态机与寻迹算法
CORE/       STM32F103 启动文件
DRIVER/     GPIO、延时、电机、红外、超声波、USART1 驱动
SYSTEM/     STM32F103 最小寄存器定义和系统时钟
USER/       main.c 和集中调参文件 config.h
Doc/        管脚、调参说明和文件清单
MDK-ARM/    Keil5 工程文件
Objects/    Keil 编译输出
Listings/   Keil map/listing 输出
```

主要入口：

```text
MDK-ARM/MazeRobot.uvprojx   Keil5 工程入口
USER/config.h               所有主要调参项
USER/main.c                 初始化和主循环
APP/maze.c                  迷宫运动状态机
Doc/管脚与调参说明.md       接线和调参细节
```

## 管脚分配

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

## 当前关键参数

所有关键参数都集中在 `USER/config.h`。

```c
#define IR_BLOCKED_LEVEL       0

#define MOTOR_FORWARD_PWM         600
#define MOTOR_SLOW_PWM            580
#define MOTOR_TURN_LOST_PWM       560
#define MOTOR_TURN_INNER_PWM      550
#define MOTOR_TURN_OUTER_PWM      950
#define MOTOR_TURN_BACK_SPIN_PWM  650

#define UART_STATUS_ENABLE        1
#define UART1_BAUDRATE            115200U
#define UART_STATUS_INTERVAL_MS   300U

#define FRONT_SAFE_DISTANCE_CM   10
#define FRONT_SLOW_DISTANCE_CM   15
#define FRONT_RIGHT_TURN_DISTANCE_CM 25
#define FRONT_TURN_BACK_CLEAR_CM 15
#define MAZE_IR_CONFIRM_COUNT    2

#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U

#define MAZE_LOOP_DELAY_MS        35
#define MAZE_FORWARD_CHECK_STEP_MS 40
#define MAZE_FORWARD_CHECK_PAUSE_MS 20
#define MAZE_STOP_BEFORE_TURN_MS  120
#define MAZE_POST_TURN_FORWARD_MS 220
#define MAZE_TURN_STEP_MS         45
#define MAZE_TURN_STEP_PAUSE_MS   20
#define MAZE_TURN_LOST_FORWARD_MS 90
#define MAZE_TURN_MIN_MS          260
#define MAZE_TURN_MAX_MS          1200
#define MAZE_TURN_BACK_MIN_MS     260
#define MAZE_TURN_BACK_STEP_MS    80
#define MAZE_TURN_BACK_CHECK_PAUSE_MS 20
#define MAZE_TURN_BACK_MAX_MS     1800
```

调参建议：

- 直行太慢或容易停转：优先调大 `MOTOR_FORWARD_PWM`。
- 前方接近墙时速度过快：调小 `MOTOR_SLOW_PWM` 或调大 `FRONT_SLOW_DISTANCE_CM`。
- 左右转弯不足：调大 `MAZE_TURN_MAX_MS` 或略微提高 `MOTOR_TURN_OUTER_PWM`。
- 左右转弯过头：调小 `MAZE_TURN_MAX_MS` 或略微降低 `MOTOR_TURN_OUTER_PWM`。
- 右转触发太晚：调大 `FRONT_RIGHT_TURN_DISTANCE_CM`。
- 掉头退出太早：调大 `FRONT_TURN_BACK_CLEAR_CM` 或 `MAZE_TURN_BACK_MIN_MS`。

## 运动逻辑

正式逻辑位于 `APP/maze.c` 的 `Maze_Task()`。

普通前进状态按左手原则选择动作：

```text
1. 左侧可通且未被左转抑制：左转
2. 右侧可通，且前方距离不大于 FRONT_RIGHT_TURN_DISTANCE_CM：右转
3. 前方安全：直行
4. 左、前、右均不可通：掉头
```

转弯状态不再执行新的路口决策。左转和右转达到 `MAZE_TURN_MIN_MS` 后，等待对应侧红外连续 `MAZE_IR_CONFIRM_COUNT` 次检测到墙再退出；若一直没有确认，则达到 `MAZE_TURN_MAX_MS` 后兜底退出。

掉头状态达到 `MAZE_TURN_BACK_MIN_MS` 后，当前方超声波连续恢复到 `FRONT_TURN_BACK_CLEAR_CM` 以上时退出；若没有恢复，则达到 `MAZE_TURN_BACK_MAX_MS` 后兜底退出。

## 串口状态输出

USART1 使用 `PA9/PA10`，参数为 `115200-8-N-1`。

输出示例：

```text
US=18cm,US_OK=1,US_ERR=NONE,IR_L=BLOCK,IR_R=CLEAR,PWM_L=+600,PWM_R=+800
```

字段说明：

- `US`：前方超声波距离。
- `US_OK`：本次测距是否有效。
- `US_ERR`：超声波错误原因。
- `IR_L` / `IR_R`：左右红外状态，`BLOCK` 表示检测到墙/障碍，`CLEAR` 表示未检测到。
- `PWM_L` / `PWM_R`：左右电机实际 PWM，正数前进，负数后退，0 停止。

## Keil5 使用

打开：

```text
MDK-ARM/MazeRobot.uvprojx
```

常用输出：

```text
Objects/MazeRobot.hex   烧录用 hex
Listings/MazeRobot.map  链接 map
```

`Objects/` 和 `Listings/` 是 Keil 生成目录，当前保留在工程根目录，避免破坏 Keil 工程配置。

## HC-SR04 电平保护

HC-SR04 通常使用 5V 供电，ECHO 输出约为 5V。STM32F103 的 PB15 属于 FT 数字输入，因此可以直接连接。若使用来源不明的兼容板或希望增加保护，可以增加分压电阻或电平转换模块。
