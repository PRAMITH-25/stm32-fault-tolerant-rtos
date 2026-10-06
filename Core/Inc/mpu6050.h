#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdbool.h>

typedef enum {
  MPU6050_STATUS_OK = 0,
  MPU6050_STATUS_PARAM_ERROR,
  MPU6050_STATUS_I2C_ERROR,
  MPU6050_STATUS_IDENTITY_ERROR
} MPU6050_Status_t;

typedef struct {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  uint32_t timestamp_ms;
} MPU6050_Measurement_t;

void MPU6050_Bind(I2C_HandleTypeDef *hi2c);
MPU6050_Status_t MPU6050_Initialize(void);
MPU6050_Status_t MPU6050_Read(MPU6050_Measurement_t *measurement);
MPU6050_Status_t MPU6050_Recover(void);
bool MPU6050_IsConnected(void);
uint32_t MPU6050_LastI2cError(void);

#endif /* MPU6050_H */
