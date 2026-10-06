# Fault-Tolerant Redundant Real-Time System 
 
Fault-tolerant embedded system on the STM32H723VGT6 using FreeRTOS. 
 
## Hardware 
- STM32H723VGT6 development board 
- MPU6050 motion sensor over I2C 
 
## Features 
- Task-based firmware: sensor acquisition, validation, processing, supervision, fault logging and recovery 
- Heartbeat monitoring and deadline checks to detect task failures and abnormal behavior 
- Tested fault conditions and recovery behavior on real hardware 
 
## Tools 
STM32CubeIDE, STM32CubeMX, STM32 HAL, FreeRTOS 
