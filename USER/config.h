#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * ==============================
 *  红外传感器逻辑配置
 * ==============================
 * 已按要求设置：红外检测到障碍物为低电平。
 * 若后续更换模块，实测为高电平触发，再把 IR_BLOCKED_LEVEL 改成 1。
 */
#define IR_BLOCKED_LEVEL       0

/*
 * ==============================
 *  电机方向修正
 * ==============================
 * 如果执行 Motor_Forward() 时某一侧电机反转，
 * 只修改下面对应宏，不建议直接改算法。
 */
#define MOTOR_LEFT_REVERSE     0
#define MOTOR_RIGHT_REVERSE    0

/*
 * ==============================
 *  PWM 与速度参数：全部在这里改
 * ==============================
 * TIM2 PWM 周期默认为 1000，因此下面 PWM 值范围建议 0~1000。
 * 本版转弯采用“差速转弯”：左右轮均向前转，只是内侧轮慢、外侧轮快，
 * 不再使用左轮反转/右轮正转那种正反转原地转弯方式。
 * 本次已把差速驱动 PWM 整体调低，若实车动力不足可只调大下面几个宏。
 */
#define MOTOR_PWM_MAX             1000

/* 正式迷宫运行用 PWM */
#define MOTOR_FORWARD_PWM         700    /* 正常直行 PWM */
#define MOTOR_SLOW_PWM            620    /* 前方较近时减速 PWM */
#define MOTOR_TURN_DELAY_PWM      620    /* 检测到转弯后，延时前进用低速 PWM */

/* 差速 90°转弯 PWM：内侧轮慢，外侧轮快，避免原地甩头 */
#define MOTOR_TURN_INNER_PWM      420
#define MOTOR_TURN_OUTER_PWM      780

/* 原地掉头 PWM：一侧正转、一侧反转 */
#define MOTOR_TURN_BACK_SPIN_PWM  650

/* 左右轮 PWM 微调量：正数表示略微增加该侧，负数表示略微减小该侧 */
#define MOTOR_LEFT_PWM_TRIM       0
#define MOTOR_RIGHT_PWM_TRIM      0

/*
 * ==============================
 *  串口实时状态输出配置
 * ==============================
 * USART1：PA9=TX，PA10=RX。默认 115200-8-N-1。
 * 用 USB-TTL 模块接线时：PA9 接 USB-TTL RX，PA10 接 USB-TTL TX，GND 共地。
 * 每 0.3 s 输出超声波、左右红外状态和左右电机实际 PWM。
 */
#define UART_STATUS_ENABLE        1
#define UART1_BAUDRATE            115200U
#define UART_STATUS_INTERVAL_MS   300U

/*
 * ==============================
 *  迷宫判断阈值
 * ==============================
 * 本版算法采用左手原则。FRONT_SAFE_DISTANCE_CM 用于判断直行、
 * 转弯前短距离前进和转弯后短距离前进是否安全。
 */
#define FRONT_SAFE_DISTANCE_CM   18      /* 前方距离大于该值，认为可以继续前进 */
#define FRONT_SLOW_DISTANCE_CM   28      /* 前方距离较近但仍安全时，降低前进速度 */
#define ULTRASONIC_MAX_CM        300
#define ULTRASONIC_MIN_CM        2
#define ULTRASONIC_TIMEOUT_US    30000
#define ULTRASONIC_ECHO_IDLE_TIMEOUT_US 1000
#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U

/*
 * ==============================
 *  动作时间参数
 * ==============================
 * 90°转弯和死路掉头使用低速脉冲式差速转弯，转弯时间通常需要重新实测。
 * 若转角不足，增大对应时间；若转角过大，减小对应时间。
 */
#define MAZE_LOOP_DELAY_MS        35
#define MAZE_FORWARD_CHECK_STEP_MS 40
#define MAZE_FORWARD_CHECK_PAUSE_MS 20
#define MAZE_STOP_BEFORE_TURN_MS  120

/* 延迟转弯时间 */
#define MAZE_TURN_DELAY_FORWARD_MS 320

#define MAZE_POST_TURN_FORWARD_MS 220
#define MAZE_TURN_STEP_MS         220
#define MAZE_TURN_STEP_PAUSE_MS   20
#define MAZE_TURN_90_MS           720
#define MAZE_TURN_BACK_MS         1500
#define MAZE_SENSOR_FAULT_RETRY_MS 100

#endif
