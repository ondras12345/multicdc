/*
 * Led strip control from Home Assistant via modbus. Heavily inspired by
 * zephyr/samples/subsys/modbus/rtu_server.
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(multicdc_leds, LOG_LEVEL_DBG);

K_SEM_DEFINE(leds_transition_sem, 0, 1);

#define LEDS_STACK 512
#define LEDS_PRIO 10  // lower priority than uart
#define TRANSITION_PERIOD_MS 15

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

// see tools/leds_gamma.py
static const uint16_t gamma255_65535[] = {
    0, 0, 0, 0, 1, 1, 2, 3, 4, 6, 8, 10, 13, 16, 19, 24, 28, 33, 39, 46, 53, 60, 69, 78, 88, 98, 110, 122, 135, 149, 164, 179, 196, 214, 232, 252, 273, 295, 317, 341, 366, 393, 420, 449, 478, 510, 542, 575, 610, 647, 684, 723, 764, 806, 849, 894, 940, 988, 1037, 1088, 1140, 1194, 1250, 1307, 1366, 1427, 1489, 1553, 1619, 1686, 1756, 1827, 1900, 1975, 2051, 2130, 2210, 2293, 2377, 2463, 2552, 2642, 2734, 2829, 2925, 3024, 3124, 3227, 3332, 3439, 3548, 3660, 3774, 3890, 4008, 4128, 4251, 4376, 4504, 4634, 4766, 4901, 5038, 5177, 5319, 5464, 5611, 5760, 5912, 6067, 6224, 6384, 6546, 6711, 6879, 7049, 7222, 7397, 7576, 7757, 7941, 8128, 8317, 8509, 8704, 8902, 9103, 9307, 9514, 9723, 9936, 10151, 10370, 10591, 10816, 11043, 11274, 11507, 11744, 11984, 12227, 12473, 12722, 12975, 13230, 13489, 13751, 14017, 14285, 14557, 14833, 15111, 15393, 15678, 15967, 16259, 16554, 16853, 17155, 17461, 17770, 18083, 18399, 18719, 19042, 19369, 19700, 20034, 20372, 20713, 21058, 21407, 21759, 22115, 22475, 22838, 23206, 23577, 23952, 24330, 24713, 25099, 25489, 25884, 26282, 26683, 27089, 27499, 27913, 28330, 28752, 29178, 29608, 30041, 30479, 30921, 31367, 31818, 32272, 32730, 33193, 33660, 34131, 34606, 35085, 35569, 36057, 36549, 37046, 37547, 38052, 38561, 39075, 39593, 40116, 40643, 41175, 41711, 42251, 42796, 43346, 43899, 44458, 45021, 45588, 46161, 46737, 47319, 47905, 48495, 49091, 49691, 50295, 50905, 51519, 52138, 52761, 53390, 54023, 54661, 55303, 55951, 56604, 57261, 57923, 58590, 59262, 59939, 60621, 61308, 62000, 62697, 63399, 64106, 64818, 65535
};

struct led_transition {
    // hardware configuration
    const struct pwm_dt_spec * const led_pwm;  //< PWM channel
    const struct gpio_dt_spec * const led_power;  //< power switch. Can be NULL.

    // internal state
    uint8_t brightness; //< current brightness
    uint8_t target;     //< target brightness
    uint8_t start;      //< start brightness (captured at set())
    uint32_t duration_ms; //< total transition duration
    uint32_t elapsed_ms;  //< elapsed time
    bool active;
};

void led_transition_start(struct led_transition * t, uint8_t target, uint32_t duration_ms)
{
    if (!t) return;

    t->start = t->brightness;
    t->target = target;
    t->duration_ms = duration_ms;
    t->elapsed_ms = 0;

    if (t->start == t->target) {
        t->active = false;
        return;
    }

    t->active = true;

    // turn the power on
    if (t->led_power && (t->start > 0 || t->target > 0)) {
        (void)gpio_pin_set_dt(t->led_power, 1);
    }

    // unblock transition loop
    k_sem_give(&leds_transition_sem);
}

void led_transition_apply_brightness(struct led_transition * t)
{
    if (!t || !t->led_pwm) return;
    // gamma-compensated brightness 0-65535
    uint16_t o = gamma255_65535[t->brightness];
    // set led brightness
    uint32_t pulse = (uint64_t)t->led_pwm->period * o / 65535;
    pwm_set_pulse_dt(t->led_pwm, pulse);
}

/**
 * Execute LED brightness transition. To be called every TRANSITION_PERIOD_MS.
 * @retval true if still running
 * @retval false if finished / not active
 */
bool led_transition_step(struct led_transition * t)
{
    if (!t || !t->active) return false;

    uint64_t elapsed = t->elapsed_ms + TRANSITION_PERIOD_MS;

    if (elapsed >= t->duration_ms) {
        // finish
        t->elapsed_ms = t->duration_ms;
        t->brightness = t->target;
        led_transition_apply_brightness(t);
        t->active = false;
        if (t->brightness == 0 && t->led_power) {
            (void)gpio_pin_set_dt(t->led_power, 0);
        }
        return false;
    }

    t->elapsed_ms = elapsed;

    // Interpolate: b = start + (target-start) * elapsed / duration
    int16_t delta = (int16_t)t->target - (int16_t)t->start;

    int64_t num = delta * (int64_t)t->elapsed_ms;
    int64_t den = t->duration_ms;
    // round-to-nearest
    int64_t adj = (num >= 0) ? (den / 2u) : -(den / 2u);
    int64_t b = (int64_t)t->start + (num + adj) / den;
    uint8_t next = CLAMP(b, 0, 255);

    if (next != t->brightness) {
        t->brightness = next;
        led_transition_apply_brightness(t);
    }
    return true;
}


static const struct pwm_dt_spec led_rgbcct_r = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_r));
static const struct pwm_dt_spec led_rgbcct_g = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_g));
static const struct pwm_dt_spec led_rgbcct_b = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_b));
static const struct pwm_dt_spec led_rgbcct_c = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_c));
static const struct pwm_dt_spec led_rgbcct_w = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_rgbcct_w));
static const struct pwm_dt_spec led_backlight = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led_backlight));
static const struct gpio_dt_spec gpio_backlight_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(led_backlight_enable), gpios);

static struct led_transition tran_rgbcct_r = {
    .led_pwm = &led_rgbcct_r,
};

static struct led_transition tran_rgbcct_g = {
    .led_pwm = &led_rgbcct_g,
};

static struct led_transition tran_rgbcct_b = {
    .led_pwm = &led_rgbcct_b,
};

static struct led_transition tran_rgbcct_c = {
    .led_pwm = &led_rgbcct_c,
};

static struct led_transition tran_rgbcct_w = {
    .led_pwm = &led_rgbcct_w,
};

static struct led_transition tran_backlight = {
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


static void leds_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!gpio_is_ready_dt(&gpio_backlight_enable)) {
        LOG_ERR("backlight_ENA gpio is not ready");
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(led_transitions); i++) {
        if (!pwm_is_ready_dt(led_transitions[i]->led_pwm)) {
            LOG_ERR("LED%u PWM device not ready", i);
            return;
        }
        // Some LEDs have inverted PWM (need to be pulled to 3V3 to turn off).
        // That does not happen until the first pwm_set call.
        led_transition_apply_brightness(led_transitions[i]);
    }

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

    uint32_t duration_ms = shell_strtoul(argv[6], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse duration_ms");
        return ret;
    }

    shell_print(sh, "\n!rgbcct r=%u g=%u b=%u c=%u w=%u duration_ms=%u\n", r, g, b, c, w, duration_ms);

    led_transition_start(&tran_rgbcct_r, r, duration_ms);
    led_transition_start(&tran_rgbcct_g, g, duration_ms);
    led_transition_start(&tran_rgbcct_b, b, duration_ms);
    led_transition_start(&tran_rgbcct_c, c, duration_ms);
    led_transition_start(&tran_rgbcct_w, w, duration_ms);
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

    uint32_t duration_ms = shell_strtoul(argv[2], 10, &ret);
    if (ret) {
        shell_error(sh, "failed to parse duration_ms");
        return ret;
    }

    shell_print(sh, "\n!backlight duty=%u duration_ms=%u\n", duty, duration_ms);

    led_transition_start(&tran_backlight, duty, duration_ms);
    return 0;
}
SHELL_CMD_ARG_REGISTER(backlight, NULL, "Set backlight LED brightness\nUsage: backlight <duty> <duration_ms>", cmd_leds_backlight, 3, 0);
