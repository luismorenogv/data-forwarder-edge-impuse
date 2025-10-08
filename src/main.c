#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SAMPLE_PERIOD_MS 20  // 50 Hz

static const struct device *imu;
static struct bt_conn *cur_conn;

static void connected(struct bt_conn *conn, uint8_t err) {
    if (!err) {
        cur_conn = bt_conn_ref(conn);
        LOG_INF("BLE connected");
    }
}
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);
    if (cur_conn) { bt_conn_unref(cur_conn); cur_conn = NULL; }
    LOG_INF("BLE disconnected");
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

    /* Fast advertising */
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    return err;
}

static int imu_init(void) {
    #if DT_HAS_COMPAT_STATUS_OKAY(invensense_mpu9250)
        imu = DEVICE_DT_GET_ANY(invensense_mpu9250);
    #elif DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
        imu = DEVICE_DT_GET_ANY(st_lsm6dsl);
    #else
    #error "No supported IMU present"
    #endif
        if (!device_is_ready(imu)) {
            LOG_ERR("IMU not ready");
            return -ENODEV;
        }
        return 0;
}


static int send_sample(void) {
    if (!cur_conn) return -ENOTCONN;

    struct sensor_value ax, ay, az, gx, gy, gz;
    int err = sensor_sample_fetch(imu);
    if (err) return err;

    /* Accel */
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
    /* Gyro */
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gx);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gy);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gz);

    char line[96];
    int n = snprintk(line, sizeof(line),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        sensor_value_to_double(&ax), sensor_value_to_double(&ay),
        sensor_value_to_double(&az), sensor_value_to_double(&gx),
        sensor_value_to_double(&gy), sensor_value_to_double(&gz));

    if (n <= 0) return -EIO;
    /* Send over BLE NUS; central_uart on the DK will forward over USB CDC */
    err = bt_nus_send(cur_conn, line, n);
    return err;
}

int main(void) {
    if (ble_init()) { LOG_ERR("BLE init failed"); return -1; }
    if (imu_init()) { LOG_ERR("IMU init failed"); return -1; }
    LOG_INF("Ready. Waiting for BLE central...");

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
            /* If you hit this often, lower sample rate or reduce print length */
            next = now + SAMPLE_PERIOD_MS;
        } else {
            k_sleep(K_MSEC(next - now));
            next += SAMPLE_PERIOD_MS;
        }
    }
}
