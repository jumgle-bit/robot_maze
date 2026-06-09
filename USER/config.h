#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * ==============================
 *  工程基本配置
 * ==============================
 */

/* 运行模式：
 * APP_MODE_MAZE        ：正式迷宫寻迹
 * APP_MODE_MOTOR_TEST  ：电机单模块测试
 * APP_MODE_SENSOR_TEST ：传感器联动测试
 */
#define APP_MODE_MAZE          0
#define APP_MODE_MOTOR_TEST    1
#define APP_MODE_SENSOR_TEST   2
#define APP_RUN_MODE           APP_MODE_MAZE

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
#define MOTOR_FORWARD_PWM         800    /* 正常直行 PWM，已调低 */
#define MOTOR_SLOW_PWM            720    /* 前方较近时减速 PWM，已调低 */

/* 差速 90°转弯 PWM：内侧轮慢，外侧轮快 */
#define MOTOR_TURN_INNER_PWM      600
#define MOTOR_TURN_OUTER_PWM      800

/* 差速掉头 PWM：仍然双轮向前差速，不采用正反转 */
#define MOTOR_TURN_BACK_INNER_PWM 600
#define MOTOR_TURN_BACK_OUTER_PWM 800

/* 测试模式用 PWM */
#define MOTOR_TEST_FORWARD_PWM    450
#define MOTOR_TEST_TURN_INNER_PWM 180
#define MOTOR_TEST_TURN_OUTER_PWM 560

/* 左右轮 PWM 微调量：正数表示略微增加该侧，负数表示略微减小该侧 */
#define MOTOR_LEFT_PWM_TRIM       0
#define MOTOR_RIGHT_PWM_TRIM      0

/* 兼容旧函数名，正常使用时不用改这里 */
#define MOTOR_BASE_SPEED          MOTOR_FORWARD_PWM
#define MOTOR_SLOW_SPEED          MOTOR_SLOW_PWM
#define MOTOR_TURN_SPEED          MOTOR_TURN_OUTER_PWM
#define MOTOR_TEST_SPEED          MOTOR_TEST_FORWARD_PWM


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
 * 本版算法采用左手原则。FRONT_LEFT_TURN_CLEAR_CM 用于判断转弯前是否允许短距离前进，
 * FRONT_SAFE_DISTANCE_CM 用于判断直行和转弯后的前方是否安全。
 */
#define FRONT_LEFT_TURN_CLEAR_CM 10      /* 前方距离大于该值，转弯前允许短距离前进 */
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
 * 差速转弯半径比原地正反转更大，转弯时间通常需要重新实测。
 * 若转角不足，增大对应时间；若转角过大，减小对应时间。
 */
#define MAZE_LOOP_DELAY_MS        35
#define MAZE_STOP_BEFORE_TURN_MS  120
#define MAZE_PRE_TURN_FORWARD_MS  160
#define MAZE_POST_TURN_FORWARD_MS 220
#define MAZE_TURN_90_MS           650
#define MAZE_TURN_BACK_MS         1350
#define MAZE_SENSOR_FAULT_RETRY_MS 100

#endif
