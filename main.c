#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "pid/pid.h"

#define PIN_SWITCH_UP 10
#define PIN_SWITCH_DOWN 11
#define PIN_SERVO 2
#define PIN_TACH 12

#define NUMBER_OF_FLYWHEEL_REFLECTORS 8
#define SERVO_ZERO_POSITION 990
#define SERVO_FULL_POSITION 2030

typedef enum
{
    CHOKE,
    IDLE,
    SLOW,
    FAST,
} state;

volatile state g_state = CHOKE;

volatile alarm_id_t g_switch_alarm_id = 0;
volatile alarm_id_t g_tachometer_detection_alarm_id = 0;
volatile alarm_id_t g_tachometer_timeout_alarm_id = 0;

volatile bool g_is_turning = false;
volatile uint32_t g_one_revolution_time = 0;

int64_t tachometer_alarm_timeout_callback(alarm_id_t id, void *user_data)
{
    g_tachometer_timeout_alarm_id = 0;
    g_is_turning = false;

    // if this alarm fires while the engine is in SLOW/FAST state it is likely a sensor failure, cut throttle to idle
    if (g_state == SLOW || g_state == FAST)
    {
        g_state = IDLE;
    }

    return 0;
}

int64_t tachometer_alarm_callback(alarm_id_t id, void *user_data)
{
    g_tachometer_detection_alarm_id = 0;

    if (gpio_get(PIN_TACH) == 0)
    {
        static uint32_t sensor_times[NUMBER_OF_FLYWHEEL_REFLECTORS + 1];
        static size_t pointer_position;
        sensor_times[pointer_position] = time_us_32();
        uint8_t next_pointer_position = (pointer_position + 1) % NUMBER_OF_FLYWHEEL_REFLECTORS;
        g_one_revolution_time = sensor_times[pointer_position] - sensor_times[next_pointer_position];
        pointer_position = next_pointer_position;

        g_is_turning = true;
        if (g_tachometer_timeout_alarm_id != 0)
        {
            cancel_alarm(g_tachometer_timeout_alarm_id);
        }
        g_tachometer_timeout_alarm_id = add_alarm_in_us(100000, &tachometer_alarm_timeout_callback, NULL, false);
    }

    return 0;
}

void core1_gpio_callback(uint gpio, uint32_t events)
{
    if (g_tachometer_detection_alarm_id != 0)
    {
        cancel_alarm(g_tachometer_detection_alarm_id);
    }
    g_tachometer_detection_alarm_id = add_alarm_in_us(100, &tachometer_alarm_callback, NULL, false);
}

// core 1 handles interrupts from tachometer sensor
void core_1_main()
{
    gpio_init(PIN_TACH);
    gpio_set_dir(PIN_TACH, GPIO_IN);
    gpio_pull_up(PIN_TACH);
    gpio_set_irq_enabled_with_callback(PIN_TACH, GPIO_IRQ_EDGE_FALL, true, &core1_gpio_callback);
}

int64_t switch_alarm_callback(alarm_id_t id, void *user_data)
{
    g_switch_alarm_id = 0;

    if (gpio_get(PIN_SWITCH_DOWN) == 0)
    {
        switch (g_state)
        {
        case CHOKE:
            // nothing to do
            break;
        case IDLE:
            // only allow the choke to be put on if the engine is not turning
            if (!g_is_turning)
            {
                g_state = CHOKE;
            }
            break;
        case SLOW:
            g_state = IDLE;
            break;
        case FAST:
            g_state = SLOW;
            break;
        }
    }
    else if (gpio_get(PIN_SWITCH_UP) == 0)
    {
        switch (g_state)
        {
        case CHOKE:
            g_state = IDLE;
            break;
        case IDLE:
            // only allow the governor to be engaged if the engine is running
            if (g_is_turning)
            {
                g_state = SLOW;
            }
            break;
        case SLOW:
            // only allow the governor to be engaged if the engine is running
            if (g_is_turning)
            {
                g_state = FAST;
            }
            break;
        case FAST:
            // nothing to do
            break;
        }
    }

    return 0;
}

void core0_gpio_callback(uint gpio, uint32_t events)
{
    if (g_switch_alarm_id != 0)
    {
        cancel_alarm(g_switch_alarm_id);
    }
    g_switch_alarm_id = add_alarm_in_us(100000, &switch_alarm_callback, NULL, false);
}

int main()
{
    stdio_init_all();

    multicore_launch_core1(core_1_main);

    gpio_init(PIN_SWITCH_DOWN);
    gpio_set_dir(PIN_SWITCH_DOWN, GPIO_IN);
    gpio_pull_up(PIN_SWITCH_DOWN);
    gpio_set_irq_enabled_with_callback(PIN_SWITCH_DOWN, GPIO_IRQ_EDGE_FALL, true, &core0_gpio_callback);

    gpio_init(PIN_SWITCH_UP);
    gpio_set_dir(PIN_SWITCH_UP, GPIO_IN);
    gpio_pull_up(PIN_SWITCH_UP);
    gpio_set_irq_enabled(PIN_SWITCH_UP, GPIO_IRQ_EDGE_FALL, true);

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_SERVO);
    pwm_set_clkdiv(slice_num, 150.0f);  // 1_000_000/sec
    pwm_set_wrap(slice_num, 20000 - 1); // 50Hz
    pwm_set_enabled(slice_num, true);

    Controller controller = {
        .proportionalGain = 4.f,
        .integralGain = 0.02f,
        .derivativeGain = 0.f,
        .integralMin = -50.f,
        .integralMax = 50.f,
    };

    uint32_t interval = 10000;
    uint32_t next = time_us_32();
    float rpm = 0;
    float scale_factor = SERVO_FULL_POSITION - SERVO_ZERO_POSITION;
    float command = 0;
    for (uint32_t i = 0;; i++)
    {
        rpm = 60000000.f / (float)g_one_revolution_time;
        if (i % 10 == 0)
        {
            printf(">rpm:%f %u\r\n", rpm, g_is_turning);
        }

        switch (g_state)
        {
        case CHOKE:
            if (g_is_turning && rpm > 2000)
            {
                g_state = SLOW;
            }

            pwm_set_gpio_level(PIN_SERVO, SERVO_FULL_POSITION); // Full throttle
            break;

        case IDLE:
            pwm_set_gpio_level(PIN_SERVO, SERVO_ZERO_POSITION); // Closed throttle
            break;

        case SLOW:
            command = update(&controller, 0.75f, rpm);
            if (command < 0)
            {
                command = 0;
            }
            else if (command > 1)
            {
                command = 1;
            }
            command *= scale_factor;
            pwm_set_gpio_level(PIN_SERVO, SERVO_ZERO_POSITION + (uint16_t)command);
            break;

        case FAST:
            command = update(&controller, 1.f, rpm);
            if (command < 0)
            {
                command = 0;
            }
            else if (command > 1)
            {
                command = 1;
            }
            command *= scale_factor;
            pwm_set_gpio_level(PIN_SERVO, SERVO_ZERO_POSITION + (uint16_t)command);
            break;

        default:                                                // Should never be called
            pwm_set_gpio_level(PIN_SERVO, SERVO_ZERO_POSITION); // Closed throttle
            break;
        }

        next += interval;
        sleep_us(next - time_us_32());
    }
}
