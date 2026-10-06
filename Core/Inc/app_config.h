#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* MPU6050 on I2C1.  Address 0x68 applies when the module AD0 pin is low. */
#define MPU6050_I2C_ADDRESS_7BIT             0x68U
#define MPU6050_ACCEL_LSB_PER_G              16384.0f /* +/-2 g */
#define MPU6050_GYRO_LSB_PER_DPS             131.0f   /* +/-250 degrees/s */
#define MPU6050_SMPLRT_DIV                   9U       /* 100 Hz with the selected DLPF */
#define MPU6050_CONFIG                       3U       /* DLPF approximately 44 Hz */
#define MPU6050_ACCEL_CONFIG                 0U       /* +/-2 g */
#define MPU6050_GYRO_CONFIG                  0U       /* +/-250 degrees/s */

#define APP_SENSOR_PERIOD_MS                 100U
#define APP_VALIDATION_PERIOD_MS             250U
#define APP_SUPERVISOR_PERIOD_MS             500U
#define APP_HEARTBEAT_TIMEOUT_MS             1500U
#define APP_MEASUREMENT_TIMEOUT_MS           1500U
#define APP_I2C_FAILURE_LIMIT                3U
#define APP_RECOVERY_LIMIT                   3U

/* IWDG uses the imprecise internal LSI oscillator: nominally about 8 seconds. */
#define APP_IWDG_PRESCALER_BITS              6U     /* divide LSI by 256 */
#define APP_IWDG_RELOAD                      999U

/* Runtime UART fault-injection modes. */
#define APP_FAULT_INJECTION_NONE             0U
#define APP_FAULT_INJECTION_SENSOR_STALL     1U
#define APP_FAULT_INJECTION_I2C_ERROR        2U
/* Values 3 and 4 match the PuTTY demonstration menu. */
#define APP_FAULT_INJECTION_INVALID_DATA      3U
#define APP_FAULT_INJECTION_SUPERVISOR_STALL 4U
/* Retained only as the boot default; runtime selection controls behavior. */
#define APP_FAULT_INJECTION_MODE             APP_FAULT_INJECTION_NONE
#define APP_FAULT_INJECTION_START_MS         10000U
#define APP_FAULT_INJECTION_DURATION_MS      1000U

#define APP_STACK_WARNING_WORDS               32U
#define APP_PERSISTENT_MAGIC                  0x46545254UL /* "FTRT" */
#define APP_PERSISTENT_VERSION                2U
#define APP_PERSISTENT_HISTORY_LENGTH         8U

#endif /* APP_CONFIG_H */
