#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * ==============================
 *  红外传感器逻辑配置
 * ==============================
 * 已按要求设置：红外检测到障碍物为低电平。
 * 若后续更换模块，实测为高电平触发，再把 IR_BLOCKED_LEVEL 改成 1。
 */
#define IR_BLOCKED_LEVEL       0       /* 红外输出等于该电平时，认为检测到墙/障碍 */

/*
 * ==============================
 *  电机方向修正
 * ==============================
 * 如果执行 Motor_Forward() 时某一侧电机反转，
 * 只修改下面对应宏，不建议直接改算法。
 */
#define MOTOR_LEFT_REVERSE     0       /* 左电机方向反了就改为 1 */
#define MOTOR_RIGHT_REVERSE    0       /* 右电机方向反了就改为 1 */

/*
 * ==============================
 *  PWM 与速度参数：全部在这里改
 * ==============================
 * TIM2 PWM 周期默认为 1000，因此下面 PWM 值范围建议 0~1000。
 * 左右 90°转弯采用“差速转弯”：左右轮均向前转，只是内侧轮慢、外侧轮快。
 * 死路掉头采用一侧正转、一侧反转，便于在狭窄空间完成原地掉头。
 * 本次已把差速驱动 PWM 整体调低，若实车动力不足可只调大下面几个宏。
 */
#define MOTOR_PWM_MAX             1000  /* PWM 最大值，对应 TIM2 ARR+1 */

/* 正式迷宫运行用 PWM */
#define MOTOR_FORWARD_PWM         600    /* 正常直行 PWM */
#define MOTOR_SLOW_PWM            580    /* 前方较近时减速 PWM */
#define MOTOR_TURN_LOST_PWM       560    /* 转弯中两侧都丢墙时，补位前进 PWM */

/* 差速 90°转弯 PWM：内侧轮慢，外侧轮快，避免原地甩头 */
#define MOTOR_TURN_INNER_PWM      550   /* 90°差速转弯时，内侧轮 PWM */
#define MOTOR_TURN_OUTER_PWM      950   /* 90°差速转弯时，外侧轮 PWM */

/* 原地掉头 PWM：一侧正转、一侧反转 */
#define MOTOR_TURN_BACK_SPIN_PWM  650   /* 原地掉头时，两侧轮正反转 PWM */

/* 左右轮 PWM 微调量：正数表示略微增加该侧，负数表示略微减小该侧 */
#define MOTOR_LEFT_PWM_TRIM       0     /* 左轮 PWM 微调，正数增大、负数减小 */
#define MOTOR_RIGHT_PWM_TRIM      0     /* 右轮 PWM 微调，正数增大、负数减小 */

/*
 * ==============================
 *  串口实时状态输出配置
 * ==============================
 * USART1：PA9=TX，PA10=RX。默认 115200-8-N-1。
 * 用 USB-TTL 模块接线时：PA9 接 USB-TTL RX，PA10 接 USB-TTL TX，GND 共地。
 * 每 0.3 s 输出超声波、左右红外状态和左右电机实际 PWM。
 */
#define UART_STATUS_ENABLE        1       /* 1=开启串口状态输出，0=关闭 */
#define UART1_BAUDRATE            115200U /* USART1 波特率 */
#define UART_STATUS_INTERVAL_MS   300U    /* 串口状态输出周期 */

/*
 * ==============================
 *  迷宫判断阈值
 * ==============================
 * 本版算法采用左手原则。FRONT_SAFE_DISTANCE_CM 用于判断直行、
 * 转弯后短距离前进是否安全。
 * 红外状态需要连续确认，避免瞬时抖动误触发转弯。
 */
#define FRONT_SAFE_DISTANCE_CM   10      /* 前方距离大于该值，认为可以继续前进 */
#define FRONT_SLOW_DISTANCE_CM   15      /* 前方距离较近但仍安全时，降低前进速度 */
#define FRONT_RIGHT_TURN_DISTANCE_CM 25  /* 右侧可通且前方距离不大于该值时，提前进入右转 */
#define FRONT_TURN_BACK_CLEAR_CM 15      /* 掉头时前方距离大于该值，认为已转到安全方向 */
#define MAZE_IR_CONFIRM_COUNT    2       /* 红外连续确认次数 */
#define ULTRASONIC_MAX_CM        300     /* 超声波有效距离上限，超过认为无效 */
#define ULTRASONIC_MIN_CM        2       /* 超声波有效距离下限，低于认为无效 */
#define ULTRASONIC_TIMEOUT_US    30000   /* 等待/测量 ECHO 的最大超时时间 */
#define ULTRASONIC_ECHO_IDLE_TIMEOUT_US 1000 /* 触发前等待 ECHO 回到低电平的时间 */
#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U   /* 两次超声波触发的最小间隔 */

/*
 * ==============================
 *  动作时间参数
 * ==============================
 * 90°转弯由状态机分段执行：达到最小时间后，锁定侧红外重新检测到墙即退出。
 * 若传感器未确认完成，则达到最大时间后兜底退出。
 * 死路掉头使用分段原地左旋，前方超声波恢复到安全距离后提前退出。
 */
#define MAZE_LOOP_DELAY_MS        35      /* 普通前进状态每次动作持续时间 */
#define MAZE_FORWARD_CHECK_STEP_MS 40     /* 分段前进时，每小段前进时间 */
#define MAZE_FORWARD_CHECK_PAUSE_MS 20    /* 分段前进/转弯之间的停车暂停时间 */
#define MAZE_STOP_BEFORE_TURN_MS  120     /* 检测到需要转弯后，先停车稳定的时间 */

#define MAZE_POST_TURN_FORWARD_MS 220     /* 转弯完成后，若前方安全则短距离前进时间 */
#define MAZE_TURN_STEP_MS         45      /* 左/右转状态每次差速转弯的小段时间 */
#define MAZE_TURN_STEP_PAUSE_MS   20      /* 左/右转每小段之间的停车暂停时间 */
#define MAZE_TURN_LOST_FORWARD_MS 90      /* 转弯中两侧都检测不到墙时，前进补位时间 */
#define MAZE_TURN_MIN_MS          260     /* 左/右转最小执行时间，未达到前不允许退出 */
#define MAZE_TURN_MAX_MS          1200    /* 左/右转最大执行时间，超时后兜底退出 */
#define MAZE_TURN_BACK_MIN_MS     260     /* 掉头最小执行时间，未达到前不允许退出 */
#define MAZE_TURN_BACK_STEP_MS    80      /* 掉头状态每次原地旋转的小段时间 */
#define MAZE_TURN_BACK_CHECK_PAUSE_MS 20  /* 掉头每小段后停车等待测距稳定时间 */
#define MAZE_TURN_BACK_MAX_MS     1800    /* 掉头最大执行时间，超时后兜底退出 */
#define MAZE_SENSOR_FAULT_RETRY_MS 100    /* 超声波读数无效时停车等待重试时间 */

#endif
