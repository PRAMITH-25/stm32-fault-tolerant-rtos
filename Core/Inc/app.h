#ifndef APP_H
#define APP_H

#include "main.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

typedef enum { APP_TASK_SENSOR, APP_TASK_VALIDATION, APP_TASK_SUPERVISOR, APP_TASK_UART_COMMAND, APP_TASK_COUNT } AppTaskId_t;
typedef enum { APP_SYSTEM_STARTING, APP_SYSTEM_HEALTHY, APP_SYSTEM_RECOVERING, APP_SYSTEM_SAFE } AppSystemState_t;
typedef enum { APP_SENSOR_OK, APP_SENSOR_I2C_ERROR, APP_SENSOR_IDENTITY_ERROR, APP_SENSOR_ERROR } AppSensorStatus_t;
typedef enum { APP_VALIDATION_PENDING, APP_VALIDATION_OK, APP_VALIDATION_STALE, APP_VALIDATION_INVALID } AppValidationStatus_t;
typedef enum { APP_FAULT_NONE, APP_FAULT_SENSOR_HEARTBEAT, APP_FAULT_VALIDATION_HEARTBEAT, APP_FAULT_I2C, APP_FAULT_SENSOR_IDENTITY, APP_FAULT_STALE_MEASUREMENT, APP_FAULT_INVALID_MEASUREMENT, APP_FAULT_RECOVERY_FAILURE, APP_FAULT_RECOVERY_SUCCESS, APP_FAULT_SUPERVISOR_STALL } AppFaultType_t;

typedef struct { TickType_t tick; uint32_t count; } AppHeartbeat_t;
typedef struct { AppFaultType_t type; AppTaskId_t task; uint32_t count; uint32_t recovery_attempt; uint32_t i2c_error; } AppFaultRecord_t;
typedef struct {
  MPU6050_Measurement_t measurement;
  AppHeartbeat_t heartbeat[APP_TASK_COUNT];
  AppFaultRecord_t fault;
  AppSystemState_t state;
  AppSensorStatus_t sensor_status;
  AppValidationStatus_t validation_status;
  uint8_t measurement_ready;
  uint32_t consecutive_i2c_failures;
  uint32_t recovery_attempt;
  uint32_t reset_flags;
} AppSnapshot_t;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t boot_count;
  uint32_t iwdg_reset_count;
  AppFaultRecord_t watchdog_fault;
  AppFaultRecord_t latest_fault;
  uint32_t last_state;
  uint32_t history_next;
  AppFaultRecord_t history[8];
} AppPersistentDiagnostics_t;

void App_CaptureResetDiagnostics(void);
void App_Initialize(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart);
BaseType_t App_CreateTasks(void);
void App_SetFaultInjectionMode(uint8_t mode);
void App_Log(const char *format, ...);

#endif /* APP_H */
