#include "mpu6050.h"
#include "app_config.h"
#include "main.h"

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_WHO_AM_I_VALUE    0x68U
#define MPU6500_WHO_AM_I_VALUE     0x70U
#define MPU9250_WHO_AM_I_VALUE     0x71U
#define MPU9255_WHO_AM_I_VALUE     0x73U
#define MPU6050_I2C_TIMEOUT_MS    100U

static I2C_HandleTypeDef *s_i2c;
static bool s_connected;
static uint32_t s_last_i2c_error;
static uint16_t s_device_addr_shifted = (MPU6050_I2C_ADDRESS_7BIT << 1);

static MPU6050_Status_t WriteRegister(uint8_t reg, uint8_t value)
{
  if (s_i2c == NULL) return MPU6050_STATUS_PARAM_ERROR;
  if (HAL_I2C_Mem_Write(s_i2c, s_device_addr_shifted, reg,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U,
                        MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    s_last_i2c_error = HAL_I2C_GetError(s_i2c);
    return MPU6050_STATUS_I2C_ERROR;
  }
  return MPU6050_STATUS_OK;
}

static MPU6050_Status_t ReadBytes(uint8_t reg, uint8_t *data, uint16_t size)
{
  if (s_i2c == NULL) return MPU6050_STATUS_PARAM_ERROR;
  if (HAL_I2C_Mem_Read(s_i2c, s_device_addr_shifted, reg,
                        I2C_MEMADD_SIZE_8BIT, data, size,
                        MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    s_last_i2c_error = HAL_I2C_GetError(s_i2c);
    return MPU6050_STATUS_I2C_ERROR;
  }
  return MPU6050_STATUS_OK;
}

static MPU6050_Status_t WriteAndVerify(uint8_t reg, uint8_t value)
{
  uint8_t readback;
  MPU6050_Status_t status = WriteRegister(reg, value);
  if (status != MPU6050_STATUS_OK) return status;
  HAL_Delay(2U);
  status = ReadBytes(reg, &readback, 1U);
  return ((status == MPU6050_STATUS_OK) && (readback == value)) ?
         MPU6050_STATUS_OK : MPU6050_STATUS_I2C_ERROR;
}

static int16_t Signed16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void MPU6050_Bind(I2C_HandleTypeDef *hi2c)
{
  s_i2c = hi2c;
  s_connected = false;
  s_last_i2c_error = HAL_I2C_ERROR_NONE;
}

MPU6050_Status_t MPU6050_Initialize(void)
{
  uint8_t who_am_i = 0U;
  MPU6050_Status_t status = MPU6050_STATUS_I2C_ERROR;
  static const uint16_t candidate_addrs[] = { (0x68U << 1), (0x69U << 1) };
  bool found_valid = false;
  if (s_i2c == NULL) return MPU6050_STATUS_PARAM_ERROR;

  for (uint32_t index = 0U; index < 2U; ++index) {
    s_device_addr_shifted = candidate_addrs[index];
    status = ReadBytes(MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
    if ((status == MPU6050_STATUS_OK) &&
        ((who_am_i == MPU6050_WHO_AM_I_VALUE) ||
         (who_am_i == MPU6500_WHO_AM_I_VALUE) ||
         (who_am_i == MPU9250_WHO_AM_I_VALUE) ||
         (who_am_i == MPU9255_WHO_AM_I_VALUE))) {
      found_valid = true;
      break;
    }
  }
  if (!found_valid) return (status != MPU6050_STATUS_OK) ? status : MPU6050_STATUS_IDENTITY_ERROR;
  status = WriteAndVerify(MPU6050_REG_PWR_MGMT_1, 0U);       /* wake, PLL internal */
  if (status != MPU6050_STATUS_OK) return status;
  HAL_Delay(50U);
  status = WriteAndVerify(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV);
  if (status != MPU6050_STATUS_OK) return status;
  status = WriteAndVerify(MPU6050_REG_CONFIG, MPU6050_CONFIG);
  if (status != MPU6050_STATUS_OK) return status;
  status = WriteAndVerify(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG);
  if (status != MPU6050_STATUS_OK) return status;
  status = WriteAndVerify(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG);
  s_connected = (status == MPU6050_STATUS_OK);
  return status;
}

MPU6050_Status_t MPU6050_Read(MPU6050_Measurement_t *measurement)
{
  uint8_t raw[14];
  MPU6050_Status_t status;
  if ((s_i2c == NULL) || (measurement == NULL)) return MPU6050_STATUS_PARAM_ERROR;
  if (!s_connected) return MPU6050_STATUS_IDENTITY_ERROR;
  status = ReadBytes(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
  if (status != MPU6050_STATUS_OK) return status;
  measurement->accel_x_g = (float)Signed16(&raw[0]) / MPU6050_ACCEL_LSB_PER_G;
  measurement->accel_y_g = (float)Signed16(&raw[2]) / MPU6050_ACCEL_LSB_PER_G;
  measurement->accel_z_g = (float)Signed16(&raw[4]) / MPU6050_ACCEL_LSB_PER_G;
  measurement->gyro_x_dps = (float)Signed16(&raw[8]) / MPU6050_GYRO_LSB_PER_DPS;
  measurement->gyro_y_dps = (float)Signed16(&raw[10]) / MPU6050_GYRO_LSB_PER_DPS;
  measurement->gyro_z_dps = (float)Signed16(&raw[12]) / MPU6050_GYRO_LSB_PER_DPS;
  measurement->timestamp_ms = HAL_GetTick();
  return MPU6050_STATUS_OK;
}

MPU6050_Status_t MPU6050_Recover(void)
{
  if (s_i2c == NULL) return MPU6050_STATUS_PARAM_ERROR;
  s_connected = false;
  (void)HAL_I2C_DeInit(s_i2c);
  HAL_Delay(5U);
  I2C1_ClearBus();
  HAL_Delay(5U);
  if (HAL_I2C_Init(s_i2c) != HAL_OK) {
    s_last_i2c_error = HAL_I2C_GetError(s_i2c);
    return MPU6050_STATUS_I2C_ERROR;
  }
  (void)HAL_I2CEx_ConfigAnalogFilter(s_i2c, I2C_ANALOGFILTER_ENABLE);
  (void)HAL_I2CEx_ConfigDigitalFilter(s_i2c, 0U);
  HAL_Delay(10U);
  return MPU6050_Initialize();
}

bool MPU6050_IsConnected(void) { return s_connected; }
uint32_t MPU6050_LastI2cError(void) { return s_last_i2c_error; }
