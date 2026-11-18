#include "imu.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

#define MPU6050_ADDR_7BIT       0x68U
#define MPU6050_ADDR            (MPU6050_ADDR_7BIT << 1)
#define MPU6050_TIMEOUT_MS      5U
#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_CONFIG      0x1AU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_DLPF_CFG        0x04U /* 20 Hz；10 Hz 用 0x05U，42 Hz 用 0x03U */
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define IMU_TILT_FILTER_TAU_S   0.5f
#define IMU_ACCEL_FUSION_MIN_G  0.8f
#define IMU_ACCEL_FUSION_MAX_G  1.2f
#define IMU_RAD_TO_DEG          57.2957795f

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} IMU_Raw_t;

static volatile uint32_t s_tick;
static IMU_Data_t s_d;
static float s_bias[3];
static float s_gx;
static float s_gy;
static float s_gz;
static uint32_t s_last_tick;//注释：上一次更新的时间戳
static uint16_t s_still_cnt;//注释：静止计数器，用于判断IMU是否处于静止状态
static uint16_t s_cool_ms;//注释：冷却时间计数器，用于在大动作后冻结零偏学习
static uint16_t s_read_err_streak;//注释：连续读取错误计数器，用于判断IMU是否处于错误状态
static uint8_t s_learn_en = 0U;
static uint8_t s_chip_id = 0x68U;

static uint8_t imu_write_reg(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U,
                              MPU6050_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t imu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if ((buffer == NULL) || (length == 0U)) {
        return 1U;
    }

    return (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buffer, length,
                             MPU6050_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t imu_read_reg(uint8_t reg, uint8_t *value)//注释：读取单个寄存器的值
{
    return imu_read_regs(reg, value, 1U);
}

static uint8_t imu_read_raw(IMU_Raw_t *raw)//注释：读取原始传感器数据
{
    uint8_t buffer[14];

    if (raw == NULL) {
        return 1U;
    }
    if (imu_read_regs(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer)) != 0U) {
        return 1U;
    }

    raw->ax = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    raw->ay = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    raw->az = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    raw->temp = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);
    raw->gx = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    raw->gy = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    raw->gz = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
    return 0U;
}

static uint8_t imu_sensor_init(void)
{
    uint8_t id;
    uint8_t value;

    if (imu_read_reg(MPU6050_REG_WHO_AM_I, &id) != 0U) {
        return 1U;
    }
    if ((id != 0x68U) && (id != 0x70U) &&
        (id != 0x71U) && (id != 0x73U)) {
        return 1U;
    }
    s_chip_id = id;

    if (imu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80U) != 0U) {
        return 1U;
    }
    HAL_Delay(100U);
    if (imu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01U) != 0U) {
        return 1U;
    }
    HAL_Delay(10U);

    if (imu_write_reg(MPU6050_REG_SMPLRT_DIV, 0x00U) != 0U ||
        imu_write_reg(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG) != 0U ||
        imu_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00U) != 0U ||
        imu_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00U) != 0U) {
        return 1U;
    }
    HAL_Delay(10U);

    if (imu_read_reg(MPU6050_REG_CONFIG, &value) != 0U ||
        value != MPU6050_DLPF_CFG) {
        return 1U;
    }
    if (imu_read_reg(MPU6050_REG_GYRO_CONFIG, &value) != 0U || value != 0x00U) {
        return 1U;
    }
    if (imu_read_reg(MPU6050_REG_ACCEL_CONFIG, &value) != 0U || value != 0x00U) {
        return 1U;
    }
    if (imu_read_reg(MPU6050_REG_PWR_MGMT_1, &value) != 0U ||
        (value & 0x40U) != 0U) {
        return 1U;
    }
    return 0U;
}

static float imu_raw_temp_to_c(int16_t raw_temp)
{
    if ((s_chip_id == 0x70U) || (s_chip_id == 0x71U) ||
        (s_chip_id == 0x73U)) {
        return ((float)raw_temp / 333.87f) + 21.0f;
    }
    return ((float)raw_temp / 340.0f) + 36.53f;
}

static float imu_wrap180(float angle)
{
    if (!(angle > -1.0e9f && angle < 1.0e9f)) {
        return 0.0f;
    }
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void imu_get_accel_angles(const IMU_Raw_t *raw,
                                 float *roll_deg, float *pitch_deg)
{
    float ax = (float)raw->ax;
    float ay = (float)raw->ay;
    float az = (float)raw->az;

    *roll_deg = atan2f(ay, az) * IMU_RAD_TO_DEG;
    *pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * IMU_RAD_TO_DEG;
}

void IMU_Tick1ms(void)
{
    s_tick++;
}

uint8_t IMU_Init(void)
{
    IMU_Raw_t raw;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float minimum[3] = {1.0e9f, 1.0e9f, 1.0e9f};
    float maximum[3] = {-1.0e9f, -1.0e9f, -1.0e9f};
    float span;
    uint16_t i;
    uint16_t k;
    uint16_t used = 0U;

    memset(&s_d, 0, sizeof(s_d));
    memset(s_bias, 0, sizeof(s_bias));
    s_gx = 0.0f;
    s_gy = 0.0f;
    s_gz = 0.0f;
    s_still_cnt = 0U;
    s_cool_ms = 0U;
    s_read_err_streak = 0U;
    s_learn_en = 0U;

    if (imu_sensor_init() != 0U) {
        s_d.state = IMU_STATE_ERROR;
        return 1U;
    }

    s_d.state = IMU_STATE_CALIBRATING;
    for (i = 0U; i < IMU_CALIB_SAMPLES; i++) {
        float sample[3];

        if (imu_read_raw(&raw) != 0U) {
            s_d.read_errors++;
            HAL_Delay(1U);
            continue;
        }

        sample[0] = (float)raw.gx / MPU6050_GYRO_LSB_PER_DPS;
        sample[1] = (float)raw.gy / MPU6050_GYRO_LSB_PER_DPS;
        sample[2] = (float)raw.gz / MPU6050_GYRO_LSB_PER_DPS;
        for (k = 0U; k < 3U; k++) {
            sum[k] += sample[k];
            if (sample[k] < minimum[k]) {
                minimum[k] = sample[k];
            }
            if (sample[k] > maximum[k]) {
                maximum[k] = sample[k];
            }
        }
        used++;
        HAL_Delay(1U);
    }

    if (used < (IMU_CALIB_SAMPLES / 4U)) {
        s_d.state = IMU_STATE_ERROR;
        return 1U;
    }

    for (k = 0U; k < 3U; k++) {
        s_bias[k] = sum[k] / (float)used;
    }
    span = 0.0f;
    for (k = 0U; k < 3U; k++) {
        if ((maximum[k] - minimum[k]) > span) {
            span = maximum[k] - minimum[k];
        }
    }

    s_d.bias_z_dps = s_bias[2];
    s_d.calib_span_dps = span;
    imu_get_accel_angles(&raw, &s_d.roll_deg, &s_d.pitch_deg);
    s_d.yaw_cont_deg = 0.0f;
    s_d.yaw_deg = 0.0f;
    s_d.gyro_z_dps = 0.0f;
    s_d.tick_ms = s_tick;
    s_last_tick = s_tick;
    s_d.state = IMU_STATE_READY;
    return 0U;
}

void IMU_Update(void)
{
    IMU_Raw_t raw;
    uint32_t now;
    uint32_t dt_ticks;
    float dt_s;
    float accel_norm;
    float accel_roll_deg;
    float accel_pitch_deg;
    float alpha;
    float predicted_roll_deg;
    float predicted_pitch_deg;
    uint8_t still;

    if (s_d.state != IMU_STATE_READY) {
        return;
    }

    now = s_tick;
    dt_ticks = now - s_last_tick;
    if (dt_ticks == 0U) {
        return;
    }
    s_last_tick = now;
    if (dt_ticks > IMU_GAP_MAX_TICKS) {
        s_d.missed_ticks += dt_ticks;
        return;
    }

    if (imu_read_raw(&raw) != 0U) {
        s_d.read_errors++;
        s_read_err_streak++;
        if (s_read_err_streak > 50U) {
            s_d.state = IMU_STATE_ERROR;
        }
        return;
    }
    s_read_err_streak = 0U;

    s_gx = (float)raw.gx / MPU6050_GYRO_LSB_PER_DPS - s_bias[0];
    s_gy = (float)raw.gy / MPU6050_GYRO_LSB_PER_DPS - s_bias[1];
    s_gz = (float)raw.gz / MPU6050_GYRO_LSB_PER_DPS - s_bias[2];
    accel_norm = sqrtf((float)raw.ax * (float)raw.ax +
                       (float)raw.ay * (float)raw.ay +
                       (float)raw.az * (float)raw.az) /
                  MPU6050_ACCEL_LSB_PER_G;

    if ((fabsf(s_gx) > IMU_MOTION_GYRO_DPS) ||
        (fabsf(s_gy) > IMU_MOTION_GYRO_DPS) ||
        (fabsf(s_gz) > IMU_MOTION_GYRO_DPS)) {
        s_cool_ms = IMU_COOLDOWN_TICKS;
    } else if (s_cool_ms > 0U) {
        s_cool_ms = (s_cool_ms > dt_ticks) ?
                    (uint16_t)(s_cool_ms - dt_ticks) : 0U;
    }

    still = 1U;
    if (s_cool_ms > 0U ||
        fabsf(s_gx) >= IMU_STILL_GYRO_DPS ||
        fabsf(s_gy) >= IMU_STILL_GYRO_DPS ||
        fabsf(s_gz) >= IMU_STILL_GYRO_DPS ||
        fabsf(accel_norm - 1.0f) >= IMU_STILL_ACCEL_ERR) {
        still = 0U;
    }
    if (still == 0U) {
        s_still_cnt = 0U;
    } else if (s_still_cnt < 60000U) {
        s_still_cnt++;
    }

#if IMU_ONLINE_BIAS_ENABLE
    if (s_learn_en && (s_still_cnt > IMU_STILL_TICKS)) {
        s_bias[0] += s_gx * IMU_BIAS_ALPHA;
        s_bias[1] += s_gy * IMU_BIAS_ALPHA;
        s_bias[2] += s_gz * IMU_BIAS_ALPHA;
    }
    s_d.bias_z_dps = s_bias[2];
#endif

    dt_s = (float)dt_ticks / (float)IMU_SAMPLE_HZ;
    imu_get_accel_angles(&raw, &accel_roll_deg, &accel_pitch_deg);
    alpha = IMU_TILT_FILTER_TAU_S / (IMU_TILT_FILTER_TAU_S + dt_s);
    predicted_roll_deg = s_d.roll_deg + s_gx * dt_s;
    predicted_pitch_deg = s_d.pitch_deg + s_gy * dt_s;
    if ((accel_norm >= IMU_ACCEL_FUSION_MIN_G) &&
        (accel_norm <= IMU_ACCEL_FUSION_MAX_G)) {
        predicted_roll_deg += (1.0f - alpha) *
            imu_wrap180(accel_roll_deg - predicted_roll_deg);
        predicted_pitch_deg += (1.0f - alpha) *
            imu_wrap180(accel_pitch_deg - predicted_pitch_deg);
    }
    s_d.roll_deg = imu_wrap180(predicted_roll_deg);
    s_d.pitch_deg = imu_wrap180(predicted_pitch_deg);
    s_d.yaw_cont_deg += s_gz * dt_s;
    s_d.yaw_deg = imu_wrap180(s_d.yaw_cont_deg);
    s_d.gyro_z_dps = s_gz;
    s_d.accel_norm_g = accel_norm;
    s_d.temp_c = imu_raw_temp_to_c(raw.temp);
    s_d.tick_ms = now;
}

void IMU_GetData(IMU_Data_t *out)
{
    if (out != NULL) {
        *out = s_d;
    }
}

float IMU_GetYawDeg(void)
{
    return s_d.yaw_deg;
}

void IMU_GetGyroDps(float *gx, float *gy, float *gz)
{
    if (gx != NULL) {
        *gx = s_gx;
    }
    if (gy != NULL) {
        *gy = s_gy;
    }
    if (gz != NULL) {
        *gz = s_gz;
    }
}

void IMU_ZeroYaw(void)
{
    s_d.yaw_cont_deg = 0.0f;
    s_d.yaw_deg = 0.0f;
}

void IMU_SetBiasLearn(uint8_t en)
{
    s_learn_en = (en != 0U) ? 1U : 0U;
}

uint8_t IMU_GetBiasLearn(void)
{
    return s_learn_en;
}

uint8_t IMU_IsSettled(void)
{
    if ((s_d.state != IMU_STATE_READY) || (s_cool_ms > 0U)) {
        return 0U;
    }
    return (s_still_cnt > IMU_SETTLED_TICKS) ? 1U : 0U;
}

uint8_t IMU_IsReady(void)
{
    return (s_d.state == IMU_STATE_READY) ? 1U : 0U;
}
