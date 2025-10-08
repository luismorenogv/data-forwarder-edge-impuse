#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SAMPLE_PERIOD_MS 20   // 50 Hz

/* ===== Thingy:52 LED (SX1509B) =====
 * Pins on expander: 5=GREEN, 6=BLUE, 7=RED
 */
#define LED_BLUE   6
#define LED_GREEN  5
#define LED_RED    7

static const struct device *sx1509b;
static struct k_work_delayable adv_blink_work;
static bool adv_led_on;

/* ===== IMU + BLE ===== */
static const struct device *imu;
static struct bt_conn *cur_conn;

/* ---------- LED helpers ---------- */
static int leds_init(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(semtech_sx1509b)
    sx1509b = DEVICE_DT_GET_ANY(semtech_sx1509b);
    if (!sx1509b || !device_is_ready(sx1509b)) {
        LOG_WRN("SX1509B not ready; LED feedback disabled");
        sx1509b = NULL;
        return -ENODEV;
    }
    /* Configure three LED pins as outputs (inactive) */
    gpio_pin_configure(sx1509b, LED_BLUE,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(sx1509b, LED_GREEN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(sx1509b, LED_RED,   GPIO_OUTPUT_INACTIVE);
    return 0;
#else
    LOG_WRN("No semtech,sx1509b in DT; LED feedback disabled");
    return -ENODEV;
#endif
}

static inline void led_set(int pin, int value) {
    if (sx1509b) gpio_pin_set(sx1509b, pin, value);
}

static inline void led_all_off(void) {
    led_set(LED_BLUE, 0);
    led_set(LED_GREEN, 0);
    led_set(LED_RED, 0);
}

/* Advertising blink: toggle BLUE every 200 ms while not connected */
static void adv_blink_fn(struct k_work *work) {
    ARG_UNUSED(work);
    if (!sx1509b || cur_conn) return;     // stop if connected or LED unavailable
    adv_led_on = !adv_led_on;
    led_set(LED_BLUE, adv_led_on);
    k_work_reschedule(&adv_blink_work, K_MSEC(200));
}

/* ---------- BLE callbacks ---------- */
static void connected(struct bt_conn *conn, uint8_t err) {
    if (!err) {
        cur_conn = bt_conn_ref(conn);
        /* stop advertising blink, show brief RED */
        k_work_cancel_delayable(&adv_blink_work);
        led_all_off();
        led_set(LED_RED, 1);
        k_sleep(K_MSEC(50));
        led_set(LED_RED, 0);
        LOG_INF("BLE connected");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);
    if (cur_conn) { bt_conn_unref(cur_conn); cur_conn = NULL; }
    LOG_INF("BLE disconnected");
    /* resume advertising blink */
    adv_led_on = false;
    led_set(LED_BLUE, 0);
    k_work_reschedule(&adv_blink_work, K_MSEC(200));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static int ble_init(void) {
    int err = bt_enable(NULL);
    if (err) return err;

    err = bt_nus_init(NULL);
    if (err) return err;

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (!err) {
        /* start BLUE blink to indicate advertising */
        adv_led_on = false;
        k_work_reschedule(&adv_blink_work, K_MSEC(200));
    }
    return err;
}

/* ---------- IMU init ---------- */
static int imu_init(void) {
#if DT_HAS_COMPAT_STATUS_OKAY(invensense_mpu9250)
    imu = DEVICE_DT_GET_ANY(invensense_mpu9250);
#elif DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
    imu = DEVICE_DT_GET_ANY(st_lsm6dsl);
#else
#error "No supported IMU present"
#endif
    if (!imu || !device_is_ready(imu)) {
        LOG_ERR("IMU not ready");
        return -ENODEV;
    }
    return 0;
}

/* ---------- TX sample ---------- */
static int send_sample(void) {
    if (!cur_conn) return -ENOTCONN;

    struct sensor_value ax, ay, az, gx, gy, gz;
    int err = sensor_sample_fetch(imu);
    if (err) return err;

    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_X,  &gx);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y,  &gy);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z,  &gz);

    char line[96];
    int n = snprintk(line, sizeof(line),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        sensor_value_to_double(&ax), sensor_value_to_double(&ay),
        sensor_value_to_double(&az), sensor_value_to_double(&gx),
        sensor_value_to_double(&gy), sensor_value_to_double(&gz));
    if (n <= 0) return -EIO;

    err = bt_nus_send(cur_conn, line, n);
    /* quick GREEN tick on successful TX */
    if (!err && sx1509b) {
        led_set(LED_GREEN, 1);
        /* 5 ms is visible but won’t impact the 20 ms loop */
        k_sleep(K_MSEC(5));
        led_set(LED_GREEN, 0);
    }
    return err;
}

/* ---------- main ---------- */
int main(void) {
    /* LED first, so you get visible life-signs even if BLE/IMU fails */
    leds_init();
    k_work_init_delayable(&adv_blink_work, adv_blink_fn);

    if (ble_init()) { LOG_ERR("BLE init failed"); return -1; }
    if (imu_init()) { LOG_ERR("IMU init failed"); return -1; }
    LOG_INF("Ready. Advertising & waiting for BLE central...");

    int64_t next = k_uptime_get() + SAMPLE_PERIOD_MS;
    while (1) {
        if (cur_conn) {
            int err = send_sample();
            if (err && err != -ENOTCONN) {
                LOG_WRN("send_sample err %d", err);
            }
        }
        int64_t now = k_uptime_get();
        if (next <= now) {
            next = now + SAMPLE_PERIOD_MS;
        } else {
            k_sleep(K_MSEC(next - now));
            next += SAMPLE_PERIOD_MS;
        }
    }
}
