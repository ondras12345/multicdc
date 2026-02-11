/*
 * Led strip control from Home Assistant via modbus. Heavily inspired by
 * zephyr/samples/subsys/modbus/rtu_server.
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
//#include <zephyr/modbus/modbus.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(multicdc_leds, LOG_LEVEL_DBG);

K_SEM_DEFINE(leds_transition_sem, 0, 1);

#define TRANSITION_PERIOD_MS 10
#define LEDS_STACK 512
#define LEDS_PRIO 10  // lower priority than uart

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

static const struct led_dt_spec led_rgbcct_r = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_r));
static const struct led_dt_spec led_rgbcct_g = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_g));
static const struct led_dt_spec led_rgbcct_b = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_b));
static const struct led_dt_spec led_rgbcct_c = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_c));
static const struct led_dt_spec led_rgbcct_w = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_w));
static const struct led_dt_spec led_backlight = LED_DT_SPEC_GET(DT_NODELABEL(pwm_led_backlight));
static const struct gpio_dt_spec gpio_backlight_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(led_backlight_enable), gpios);

static const struct led_dt_spec * led_dev[] = {
    &led_rgbcct_r,
    &led_rgbcct_g,
    &led_rgbcct_b,
    &led_rgbcct_c,
    &led_rgbcct_w,
    &led_backlight,
};

// see tools/leds_gamma.py
static const uint8_t gamma255_100[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 54, 54, 55, 56, 57, 57, 58, 59, 60, 60, 61, 62, 63, 64, 64, 65, 66, 67, 68, 69, 70, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 93, 94, 95, 96, 97, 98, 99, 100
};
static const uint8_t gamma100_100[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11, 11, 12, 13, 14, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 33, 34, 35, 37, 38, 40, 41, 43, 45, 46, 48, 50, 52, 54, 55, 57, 59, 61, 63, 66, 68, 70, 72, 74, 77, 79, 82, 84, 87, 89, 92, 95, 97, 100
};

struct led_transition {
    // hardware configuration
    const uint8_t brightness_full;  //< maximum input brightness, 100 or 255
    const struct led_dt_spec * const led_pwm;  //< PWM channel
    const struct gpio_dt_spec * const led_power;  //< power switch. Can be NULL.

    // internal state
    int16_t step;  //< brightness step to be added every (interval * TRANSITION_PERIOD_MS)
    uint16_t interval;  //< how often to add step to brightness
    uint8_t target;  //< target brightness
    uint8_t brightness;  //< current brightness
    uint16_t interval_counter;
};

bool led_transition_done(const struct led_transition * tran)
{
    return (tran->step == 0 ||
            (tran->step > 0 && tran->brightness >= tran->target) ||
            (tran->step < 0 && tran->brightness <= tran->target));
}

void led_transition_set(struct led_transition * tran, uint8_t target, uint16_t duration_ms)
{
    if (!tran) return;

    uint8_t start = tran->brightness;
    int16_t diff = target - start;
    uint16_t duration_periods = duration_ms / TRANSITION_PERIOD_MS;
    if (duration_periods == 0) duration_periods = 1;

    uint16_t interval = duration_periods / abs(diff);
    if (interval < 1) interval = 1;

    int16_t step = (interval * diff) / duration_periods;
    if (step == 0) step = (diff > 0) ? 1 : -1;

    LOG_DBG("transition_set interval %u step %d target %u", interval, step, target);
    tran->interval = interval;
    tran->step = step;
    tran->target = target;
}

void led_transition_start(struct led_transition * tran)
{
    if (tran->target > tran->brightness_full) tran->target = tran->brightness_full;
    if (tran->brightness > tran->brightness_full) tran->brightness = tran->brightness_full;

    if (led_transition_done(tran)) return;

    tran->interval_counter = 0;  // execute immediately
    // retrieve current brightness

    // turn on power
    if (tran->led_power) gpio_pin_set_dt(tran->led_power, 1);

    // unblock the transition loop
    k_sem_give(&leds_transition_sem);
}

/// @return true if still running
bool led_transition_step(struct led_transition * tran)
{
    if (!tran || led_transition_done(tran)) return false;

    if (tran->interval_counter > 0) {
        tran->interval_counter--;
        return true;
    }
    tran->interval_counter = tran->interval;

    int16_t v = tran->brightness + tran->step;
    if (tran->step > 0 && v >= tran->target) v = tran->target;
    if (tran->step < 0 && v <= tran->target) v = tran->target;
    tran->brightness = v;

    // apply gamma correction
    if (v > tran->brightness_full) v = tran->brightness_full;  // just to be safe
    if (tran->brightness_full == 100) v = gamma100_100[v];
    else v = gamma255_100[v];

    // set led brightness
    if (tran->led_pwm) led_set_brightness_dt(tran->led_pwm, v);

    if (led_transition_done(tran)) {
        // we have just finished the transition
        if (v == 0 && tran->led_power) gpio_pin_set_dt(tran->led_power, 0);
        return false;
    }
    return true;
}


static struct led_transition tran_rgbcct_r = {
    .brightness_full = 255,
    .led_pwm = &led_rgbcct_r,
};

static struct led_transition tran_rgbcct_g = {
    .brightness_full = 255,
    .led_pwm = &led_rgbcct_g,
};

static struct led_transition tran_rgbcct_b = {
    .brightness_full = 255,
    .led_pwm = &led_rgbcct_b,
};

static struct led_transition tran_rgbcct_c = {
    .brightness_full = 255,
    .led_pwm = &led_rgbcct_c,
};

static struct led_transition tran_rgbcct_w = {
    .brightness_full = 255,
    .led_pwm = &led_rgbcct_w,
};

static struct led_transition tran_backlight = {
    .brightness_full = 255,  // should be 100 for modbus, but we aren't using that
    .led_pwm = &led_backlight,
    .led_power = &gpio_backlight_enable,
};


static struct led_transition * led_transitions[] = {
    &tran_rgbcct_r,
    &tran_rgbcct_g,
    &tran_rgbcct_b,
    &tran_rgbcct_c,
    &tran_rgbcct_w,
    &tran_backlight,
};


// Modbus was never tested. We don't have enough CDC channels. See note in
// app.overlay.
//enum holding_reg {
//    // RGBCCT will be a Home Assistant template light. Home Assistant uses
//    // 0-255 brightness values internally, so it is probably best to use them
//    // here too.
//    HR_RGBCCT_R = 0,    ///< LED strip red channel brightness 0-255
//    HR_RGBCCT_G,        ///< LED strip green channel brightness 0-255
//    HR_RGBCCT_B,        ///< LED strip blue channel brightness 0-255
//    HR_RGBCCT_C,        ///< LED strip cool while channel brightness 0-255
//    HR_RGBCCT_W,        ///< LED strip warm white channel brightness 0-255
//    HR_RGBCCT_DURATION, ///< 0-65535 ms. Writing the duration register starts the transition.
//    // backlight needs to be compatible with Home Assistant's modbus light
//    // integration.
//    // Home Assistant modbus lights use 0-100 brightness, and so does the
//    // pwm-leds driver.
//    HR_BACKLIGHT_DURATION, ///< 0-65535ms brightness transition duration
//    HR_BACKLIGHT_PWM,   ///< backlight brightness 0-100. Writing this value starts the transition.
//    HR_BACKLIGHT_ENA,   ///< backlight enable 0/1 (disables PWM when 0). Home Assistant modbus light requires this.
//    HR_COUNT_,          ///< this must be the last value in the enum
//};
//
//static uint16_t holding_reg[HR_COUNT_] = { 0 };
//
//static int holding_reg_rd(uint16_t addr, uint16_t *reg)
//{
//    if (addr >= ARRAY_SIZE(holding_reg)) return -ENOTSUP;
//    *reg = holding_reg[addr];
//    LOG_INF("HR read, addr %u", addr);
//    return 0;
//}
//
//
//static int holding_reg_wr(uint16_t addr, uint16_t reg)
//{
//    if (addr >= ARRAY_SIZE(holding_reg)) return -ENOTSUP;
//    holding_reg[addr] = reg;
//    LOG_DBG("HR write, addr %u, value %x", addr, reg);
//    switch (addr) {
//        case HR_RGBCCT_DURATION:
//            led_transition_set(&tran_rgbcct_r, holding_reg[HR_RGBCCT_R], holding_reg[HR_RGBCCT_DURATION]);
//            led_transition_start(&tran_rgbcct_r);
//            led_transition_set(&tran_rgbcct_g, holding_reg[HR_RGBCCT_G], holding_reg[HR_RGBCCT_DURATION]);
//            led_transition_start(&tran_rgbcct_g);
//            led_transition_set(&tran_rgbcct_b, holding_reg[HR_RGBCCT_B], holding_reg[HR_RGBCCT_DURATION]);
//            led_transition_start(&tran_rgbcct_b);
//            led_transition_set(&tran_rgbcct_c, holding_reg[HR_RGBCCT_C], holding_reg[HR_RGBCCT_DURATION]);
//            led_transition_start(&tran_rgbcct_c);
//            led_transition_set(&tran_rgbcct_w, holding_reg[HR_RGBCCT_W], holding_reg[HR_RGBCCT_DURATION]);
//            led_transition_start(&tran_rgbcct_w);
//            break;
//
//        case HR_BACKLIGHT_ENA:
//        case HR_BACKLIGHT_PWM:
//        {
//            uint16_t duty = holding_reg[HR_BACKLIGHT_ENA] ? holding_reg[HR_BACKLIGHT_PWM] : 0;
//            uint16_t duration_ms = holding_reg[HR_BACKLIGHT_ENA] ? holding_reg[HR_BACKLIGHT_DURATION] : 0;
//            led_transition_set(&tran_backlight, duty, duration_ms);
//            led_transition_start(&tran_backlight);
//            break;
//        }
//
//        default:
//            break;
//    }
//    return 0;
//}
//
//
//static struct modbus_user_callbacks mbs_cbs = {
//    .holding_reg_rd = holding_reg_rd,
//    .holding_reg_wr = holding_reg_wr,
//};
//
//const static struct modbus_iface_param server_param = {
//    .mode = MODBUS_MODE_RTU,
//    .server = {
//        .user_cb = &mbs_cbs,
//        .unit_id = 1,
//    },
//    .serial = {
//        .baud = 19200,
//        .parity = UART_CFG_PARITY_NONE,
//    },
//};
//
//
//static int init_modbus_server(void)
//{
//    const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};
//    int iface;
//
//    iface = modbus_iface_get_by_name(iface_name);
//
//    if (iface < 0) {
//        LOG_ERR("Failed to get iface index for %s", iface_name);
//        return iface;
//    }
//
//    return modbus_init_server(iface, server_param);
//}


static void leds_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (int i = 0; i < ARRAY_SIZE(led_dev); i++) {
        if (!led_is_ready_dt(led_dev[i])) {
            LOG_ERR("LED%u GPIO device not ready", i);
            return;
        }
        // Some LEDs have inverted PWM (need to be pulled to 3V3 to turn off).
        // That does not happen until the first set_brightness call.
        led_set_brightness_dt(led_dev[i], 0);
    }
    if (!gpio_is_ready_dt(&gpio_backlight_enable)) {
        LOG_ERR("backlight_ENA gpio is not ready");
        return;
    }

    //holding_reg[HR_BACKLIGHT_DURATION] = 2000;  // default backlight duration
    //int ret = init_modbus_server();
    //if (ret) {
    //    LOG_ERR("modbus init failed: %d", ret);
    //    return;
    //}

    for (;;)
    {
        int64_t next = k_uptime_get();
        bool transitions_running;
        do {
            // execute transitions
            transitions_running = false;
            for (int i = 0; i < ARRAY_SIZE(led_transitions); i++) {
                transitions_running |= led_transition_step(led_transitions[i]);
            }

            // sleep
            int64_t now = k_uptime_get();
            next += TRANSITION_PERIOD_MS;
            int64_t delay = next - now;
            if (delay > 0) k_sleep(K_MSEC(delay));
            else {
                LOG_ERR("leds thread cannot keep up %" PRIi64, delay);
            }
        } while (transitions_running);

        LOG_DBG("all transitions done");
        // wait until another transition is triggered
        k_sem_take(&leds_transition_sem, K_FOREVER);
    }
}

K_THREAD_DEFINE(leds, LEDS_STACK, leds_thread, NULL, NULL, NULL, LEDS_PRIO, 0, 0);


static int cmd_leds_rgbcct(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t r,g,b,c,w;
    uint16_t duration_ms;

    int ret = 0;
    r = shell_strtoul(argv[1], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse r");
        return ret;
    }

    g = shell_strtoul(argv[2], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse g");
        return ret;
    }

    b = shell_strtoul(argv[3], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse b");
        return ret;
    }

    c = shell_strtoul(argv[4], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse c");
        return ret;
    }

    w = shell_strtoul(argv[5], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse w");
        return ret;
    }

    duration_ms = shell_strtoul(argv[6], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse duration_ms");
        return ret;
    }

    shell_print(sh, "\n!rgbcct r=%u g=%u b=%u c=%u w=%u duration_ms=%u\n", r, g, b, c, w, duration_ms);

    led_transition_set(&tran_rgbcct_r, r, duration_ms);
    led_transition_start(&tran_rgbcct_r);
    led_transition_set(&tran_rgbcct_g, g, duration_ms);
    led_transition_start(&tran_rgbcct_g);
    led_transition_set(&tran_rgbcct_b, b, duration_ms);
    led_transition_start(&tran_rgbcct_b);
    led_transition_set(&tran_rgbcct_c, c, duration_ms);
    led_transition_start(&tran_rgbcct_c);
    led_transition_set(&tran_rgbcct_w, w, duration_ms);
    led_transition_start(&tran_rgbcct_w);
    return 0;
}
SHELL_CMD_ARG_REGISTER(rgbcct, NULL, "Set led strip brightness\nUsage: rgbcct <r> <g> <b> <c> <w> <duration_ms>", cmd_leds_rgbcct, 7, 0);


static int cmd_leds_backlight(const struct shell *sh, size_t argc, char **argv)
{

    int ret = 0;
    uint8_t duty = shell_strtoul(argv[1], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse duty");
        return ret;
    }

    uint16_t duration_ms = shell_strtoul(argv[2], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse duration_ms");
        return ret;
    }

    shell_print(sh, "\n!backlight duty=%u duration_ms=%u\n", duty, duration_ms);

    led_transition_set(&tran_backlight, duty, duration_ms);
    led_transition_start(&tran_backlight);
    return 0;
}
SHELL_CMD_ARG_REGISTER(backlight, NULL, "Set backlight LED brightness\nUsage: backlight <duty> <duration_ms>", cmd_leds_backlight, 3, 0);
