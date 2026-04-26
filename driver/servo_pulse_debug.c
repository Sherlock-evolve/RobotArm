#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>

#define MAX_SERVOS 6
#define STEP_SIZE_NS 100000  // 每次调整0.1ms = 100000ns

// 舵机配置：pwmchip, min_us_ns, max_us_ns, zero_us_ns, 名称
typedef struct {
    int pwmchip;
    uint32_t min_us_ns;
    uint32_t max_us_ns;
    uint32_t zero_us_ns;
    const char* name;
} servo_pulse_config_t;

static servo_pulse_config_t servo_configs[MAX_SERVOS] = {
    {0, 400000, 2600000, 900000, "底座（肩膀）"},
    {1, 400000, 1900000, 1200000, "肩关节"},
    {3, 1000000, 2600000, 2000000, "肘关节"},
    {6, 300000, 2200000, 900000, "腕关节1"},
    {7, 300000, 2600000, 1300000, "腕关节2（手腕）"},
    {8, 1300000, 2600000, 1600000, "夹持器"},
};

// 写入文件
static int write_file(const char* path, const char* value) {
    FILE* fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "Error: Cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    if (fprintf(fp, "%s", value) < 0) {
        fprintf(stderr, "Error: Cannot write to %s: %s\n", path, strerror(errno));
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

// 写入数值
static int write_uint(const char* path, uint32_t value) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%u", value);
    return write_file(path, buf);
}

// 读取数值
static uint32_t read_uint(const char* path) {
    FILE* fp = fopen(path, "r");
    if (!fp) {
        return 0;
    }
    uint32_t value = 0;
    if (fscanf(fp, "%u", &value) != 1) {
        value = 0;
    }
    fclose(fp);
    return value;
}

// 初始化PWM通道
static int init_pwm(int servo_id) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        return -1;
    }
    
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    char path[256];
    
    // 检查通道是否已存在
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0", config->pwmchip);
    if (access(path, F_OK) == 0) {
        return 0; // 已存在
    }
    
    // 导出通道
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", config->pwmchip);
    if (write_uint(path, 0) != 0) {
        return -1;
    }
    
    sleep(1); // 等待设备创建
    return 0;
}

// 设置PWM参数
static int setup_pwm(int servo_id) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        return -1;
    }
    
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    char path[256];
    
    // 禁用
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/enable", config->pwmchip);
    write_file(path, "0");
    
    // 设置周期20ms
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/period", config->pwmchip);
    write_uint(path, 20000000);
    
    // 设置极性normal
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/polarity", config->pwmchip);
    if (access(path, F_OK) == 0) {
        write_file(path, "normal");
    }
    
    return 0;
}

// 设置脉宽
static int set_duty(int servo_id, uint32_t duty_ns) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        return -1;
    }
    
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    
    // 限制在范围内
    if (duty_ns < config->min_us_ns) duty_ns = config->min_us_ns;
    if (duty_ns > config->max_us_ns) duty_ns = config->max_us_ns;
    
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", config->pwmchip);
    if (write_uint(path, duty_ns) != 0) {
        return -1;
    }
    
    // 启用
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/enable", config->pwmchip);
    write_file(path, "1");
    
    return 0;
}

// 获取当前脉宽
static uint32_t get_duty(int servo_id) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        return 0;
    }
    
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", config->pwmchip);
    return read_uint(path);
}

// 打印菜单
static void print_menu(int servo_id, uint32_t current_duty) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        return;
    }
    
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    float duty_us = current_duty / 1000.0f;
    float duty_ms = duty_us / 1000.0f;
    
    printf("\n");
    printf("========================================\n");
    printf("舵机 %d: %s (pwmchip%d)\n", servo_id, config->name, config->pwmchip);
    printf("========================================\n");
    printf("当前脉宽: %u ns (%.2f us, %.3f ms)\n", current_duty, duty_us, duty_ms);
    printf("脉宽范围: %u ~ %u ns (%.2f ~ %.2f ms)\n", 
           config->min_us_ns, config->max_us_ns,
           config->min_us_ns/1000.0f, config->max_us_ns/1000.0f);
    printf("特殊值:   %u ns (%.2f ms)\n", 
           config->zero_us_ns, config->zero_us_ns/1000.0f);
    printf("----------------------------------------\n");
    printf("[+] 增加 0.1ms (100000ns)\n");
    printf("[-] 减少 0.1ms (100000ns)\n");
    printf("[r] 重置到特殊值\n");
    printf("[1-6] 切换舵机\n");
    printf("[q] 退出\n");
    printf("请选择: ");
    fflush(stdout);
}

// 选择舵机
static int select_servo(void) {
    printf("\n");
    printf("请选择要控制的舵机:\n");
    printf("--- ------------ ------------\n");
    for (int i = 0; i < MAX_SERVOS; i++) {
        printf("%d   %-12s (pwmchip%d)\n", 
               i + 1, 
               servo_configs[i].name,
               servo_configs[i].pwmchip);
    }
    printf("--- ------------ ------------\n");
    printf("请输入舵机编号 (1-%d): ", MAX_SERVOS);
    fflush(stdout);
    
    char buf[16];
    if (fgets(buf, sizeof(buf), stdin) == NULL) {
        return -1;
    }
    
    int servo_id = atoi(buf);
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
        printf("无效的舵机编号\n");
        return -1;
    }
    
    return servo_id;
}

int main(void) {
    // 检查root权限
    if (geteuid() != 0) {
        fprintf(stderr, "请使用 sudo 运行本程序（需要访问 /sys/class/pwm）\n");
        return 1;
    }
    
    // 选择舵机
    int servo_id = select_servo();
    if (servo_id < 1) {
        return 1;
    }
    
    // 初始化PWM
    if (init_pwm(servo_id) != 0) {
        fprintf(stderr, "初始化PWM失败\n");
        return 1;
    }
    
    if (setup_pwm(servo_id) != 0) {
        fprintf(stderr, "设置PWM参数失败\n");
        return 1;
    }
    
    // 设置为特殊值（初始值）
    servo_pulse_config_t* config = &servo_configs[servo_id - 1];
    uint32_t current_duty = config->zero_us_ns;
    set_duty(servo_id, current_duty);
    printf("已初始化为特殊值: %u ns (%.2f ms)\n", 
           current_duty, current_duty/1000.0f);
    
    // 主循环
    while (1) {
        // 读取当前实际值
        uint32_t actual_duty = get_duty(servo_id);
        if (actual_duty > 0) {
            current_duty = actual_duty;
        }
        
        print_menu(servo_id, current_duty);
        
        int c = getchar();
        // 吃掉换行符
        while (c == '\n') c = getchar();
        
        if (c == '+') {
            current_duty += STEP_SIZE_NS;
            if (set_duty(servo_id, current_duty) == 0) {
                printf("已增加 0.1ms\n");
            }
        } else if (c == '-') {
            if (current_duty > STEP_SIZE_NS) {
                current_duty -= STEP_SIZE_NS;
            } else {
                current_duty = config->min_us_ns;
            }
            if (set_duty(servo_id, current_duty) == 0) {
                printf("已减少 0.1ms\n");
            }
        } else if (c == 'r' || c == 'R') {
            current_duty = config->zero_us_ns;
            if (set_duty(servo_id, current_duty) == 0) {
                printf("已重置到特殊值\n");
            }
        } else if (c >= '1' && c <= '6') {
            int new_servo = c - '0';
            if (new_servo >= 1 && new_servo <= MAX_SERVOS) {
                servo_id = new_servo;
                config = &servo_configs[servo_id - 1];
                
                // 初始化新舵机
                if (init_pwm(servo_id) == 0 && setup_pwm(servo_id) == 0) {
                    current_duty = config->zero_us_ns;
                    set_duty(servo_id, current_duty);
                    printf("已切换到舵机 %d\n", servo_id);
                } else {
                    fprintf(stderr, "切换舵机失败\n");
                }
            }
        } else if (c == 'q' || c == 'Q') {
            printf("退出程序\n");
            break;
        } else {
            printf("无效选项\n");
        }
        
        // 清空输入缓冲区
        int ch;
        while ((ch = getchar()) != '\n' && ch != EOF) { }
    }
    
    return 0;
}

