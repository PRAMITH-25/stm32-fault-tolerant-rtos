#include "app.h"
#include "app_config.h"
#include "semphr.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define IWDG_WAIT_LOOPS 10000000UL
static AppSnapshot_t state;
static SemaphoreHandle_t state_lock, uart_lock;
static UART_HandleTypeDef *uart;
static TaskHandle_t tasks[APP_TASK_COUNT];
static uint32_t captured_reset_flags;
static volatile uint8_t fault_injection_mode=APP_FAULT_INJECTION_MODE;
static volatile uint32_t fault_injection_started_ms=0U;
static volatile bool fault_injection_consumed=false;
static volatile bool demo_completion_pending=false;
static volatile bool validation_demo_latched=false;
static volatile AppPersistentDiagnostics_t *const persistent=(volatile AppPersistentDiagnostics_t *)D3_BKPSRAM_BASE;

static void SensorTask(void *argument); static void ValidationTask(void *argument); static void SupervisorTask(void *argument); static void UartCommandTask(void *argument); static void PrintFaultMenu(void);
static void Lock(void){(void)xSemaphoreTake(state_lock,portMAX_DELAY);} static void Unlock(void){(void)xSemaphoreGive(state_lock);}
static void BootLog(const char *s){if(uart!=NULL)(void)HAL_UART_Transmit(uart,(uint8_t *)s,(uint16_t)strlen(s),100U);}
static BaseType_t WaitSet(volatile uint32_t *r,uint32_t m){uint32_t n=IWDG_WAIT_LOOPS;while(((*r&m)==0U)&&(n--!=0U)){}return((*r&m)!=0U)?pdPASS:pdFAIL;}
static BaseType_t WaitClear(volatile uint32_t *r,uint32_t m){uint32_t n=IWDG_WAIT_LOOPS;while(((*r&m)!=0U)&&(n--!=0U)){}return((*r&m)==0U)?pdPASS:pdFAIL;}

static bool ResetWasIwdg(uint32_t f){
#if defined(RCC_RSR_IWDG1RSTF)
 return(f&RCC_RSR_IWDG1RSTF)!=0U;
#elif defined(RCC_RSR_IWDGRSTF)
 return(f&RCC_RSR_IWDGRSTF)!=0U;
#else
 (void)f;return false;
#endif
}
static const char *ResetCauseName(uint32_t f){if(ResetWasIwdg(f))return "IWDG";
#if defined(RCC_RSR_WWDG1RSTF)
if((f&RCC_RSR_WWDG1RSTF)!=0U)return "WWDG";
#endif
#if defined(RCC_RSR_SFTRSTF)
if((f&RCC_RSR_SFTRSTF)!=0U)return "SOFTWARE";
#endif
#if defined(RCC_RSR_PINRSTF)
if((f&RCC_RSR_PINRSTF)!=0U)return "PIN";
#endif
#if defined(RCC_RSR_BORRSTF)
if((f&RCC_RSR_BORRSTF)!=0U)return "BROWNOUT";
#endif
#if defined(RCC_RSR_PORRSTF)
if((f&RCC_RSR_PORRSTF)!=0U)return "POWER ON";
#endif
return "UNKNOWN";}
static const char *StateName(AppSystemState_t s){return(s==APP_SYSTEM_STARTING)?"STARTING":(s==APP_SYSTEM_HEALTHY)?"HEALTHY":(s==APP_SYSTEM_RECOVERING)?"RECOVERING":"SAFE";}
static const char *FaultName(AppFaultType_t f){switch(f){case APP_FAULT_SENSOR_HEARTBEAT:return "SENSOR HEARTBEAT TIMEOUT";case APP_FAULT_VALIDATION_HEARTBEAT:return "VALIDATION HEARTBEAT TIMEOUT";case APP_FAULT_I2C:return "MPU6050 I2C FAILURE";case APP_FAULT_SENSOR_IDENTITY:return "MPU6050 WHO_AM_I FAILED";case APP_FAULT_STALE_MEASUREMENT:return "STALE IMU MEASUREMENT";case APP_FAULT_INVALID_MEASUREMENT:return "INVALID IMU MEASUREMENT";case APP_FAULT_RECOVERY_FAILURE:return "RECOVERY EXHAUSTED";case APP_FAULT_RECOVERY_SUCCESS:return "RECOVERY SUCCESS";case APP_FAULT_SUPERVISOR_STALL:return "SUPERVISOR STALL";default:return "UNKNOWN";}}

static void PersistentClear(void){volatile uint32_t *w=(volatile uint32_t *)persistent;for(uint32_t i=0U;i<(sizeof(*persistent)/sizeof(uint32_t));++i)w[i]=0U;persistent->magic=APP_PERSISTENT_MAGIC;persistent->version=APP_PERSISTENT_VERSION;}
void App_CaptureResetDiagnostics(void){captured_reset_flags=RCC->RSR;HAL_PWR_EnableBkUpAccess();
#if defined(__HAL_RCC_BKPRAM_CLK_ENABLE)
__HAL_RCC_BKPRAM_CLK_ENABLE();
#endif
if((persistent->magic!=APP_PERSISTENT_MAGIC)||(persistent->version!=APP_PERSISTENT_VERSION)){PersistentClear();}persistent->boot_count++;if(ResetWasIwdg(captured_reset_flags)){persistent->iwdg_reset_count++;}__HAL_RCC_CLEAR_RESET_FLAGS();}
static void PersistentRecord(AppFaultType_t type,AppTaskId_t task,uint32_t error,uint32_t attempts,AppSystemState_t system_state){AppFaultRecord_t r={type,task,0U,attempts,error};taskENTER_CRITICAL();r.count=(persistent->latest_fault.count==UINT32_MAX)?UINT32_MAX:persistent->latest_fault.count+1U;persistent->latest_fault=r;persistent->last_state=(uint32_t)system_state;persistent->history[persistent->history_next%APP_PERSISTENT_HISTORY_LENGTH]=r;persistent->history_next=(persistent->history_next+1U)%APP_PERSISTENT_HISTORY_LENGTH;taskEXIT_CRITICAL();}
static void PersistentSetWatchdogFault(AppFaultType_t type,AppTaskId_t task,uint32_t error,uint32_t attempts,AppSystemState_t system_state){AppFaultRecord_t r={type,task,0U,attempts,error};taskENTER_CRITICAL();r.count=(persistent->watchdog_fault.count==UINT32_MAX)?UINT32_MAX:persistent->watchdog_fault.count+1U;persistent->watchdog_fault=r;persistent->last_state=(uint32_t)system_state;taskEXIT_CRITICAL();}
static void Beat(AppTaskId_t i){Lock();state.heartbeat[i].tick=xTaskGetTickCount();state.heartbeat[i].count++;Unlock();}
static void Snapshot(AppSnapshot_t *s){Lock();*s=state;Unlock();if((xTaskGetCurrentTaskHandle()==tasks[APP_TASK_SUPERVISOR])&&validation_demo_latched){s->validation_status=APP_VALIDATION_INVALID;validation_demo_latched=false;}}
static void Transition(AppSystemState_t s){Lock();state.state=s;Unlock();persistent->last_state=(uint32_t)s;}
static void Fault(AppFaultType_t f,AppTaskId_t t,uint32_t e){uint32_t a;AppSystemState_t s;bool changed;Lock();changed=(state.fault.type!=f)||(state.fault.task!=t)||(state.fault.i2c_error!=e);state.fault.type=f;state.fault.task=t;state.fault.i2c_error=e;if(state.fault.count!=UINT32_MAX)state.fault.count++;a=state.recovery_attempt;s=state.state;Unlock();if(changed)PersistentRecord(f,t,e,a,s);}

void App_Log(const char *fmt,...){char b[192];va_list a;int n;if(uart==NULL)return;va_start(a,fmt);n=vsnprintf(b,sizeof(b)-3U,fmt,a);va_end(a);if(n<0)return;if(n>(int)(sizeof(b)-3U))n=(int)(sizeof(b)-3U);b[n++]='\r';b[n++]='\n';if((uart_lock!=NULL)&&(xSemaphoreTake(uart_lock,pdMS_TO_TICKS(100U))==pdTRUE)){(void)HAL_UART_Transmit(uart,(uint8_t *)b,(uint16_t)n,200U);(void)xSemaphoreGive(uart_lock);}}
static BaseType_t WatchdogInit(void){BootLog("[IWDG] LSI enable\r\n");RCC->CSR|=RCC_CSR_LSION;if(WaitSet(&RCC->CSR,RCC_CSR_LSIRDY)!=pdPASS)return pdFAIL;IWDG1->KR=0xCCCCU;IWDG1->KR=0x5555U;IWDG1->PR=APP_IWDG_PRESCALER_BITS;if(WaitClear(&IWDG1->SR,IWDG_SR_PVU)!=pdPASS)return pdFAIL;IWDG1->RLR=APP_IWDG_RELOAD;if(WaitClear(&IWDG1->SR,IWDG_SR_RVU)!=pdPASS)return pdFAIL;IWDG1->KR=0xAAAAU;BootLog("[IWDG] Initialized\r\n");return pdPASS;}
static void WatchdogRefresh(void){if(xTaskGetCurrentTaskHandle()==tasks[APP_TASK_SUPERVISOR])IWDG1->KR=0xAAAAU;}

static const char *FaultModeName(uint8_t mode){switch(mode){case APP_FAULT_INJECTION_SENSOR_STALL:return "SENSOR_STALL";case APP_FAULT_INJECTION_I2C_ERROR:return "I2C_ERROR";case APP_FAULT_INJECTION_INVALID_DATA:return "INVALID_DATA";case APP_FAULT_INJECTION_SUPERVISOR_STALL:return "SUPERVISOR_STALL";default:return "NONE";}}
static void EndRecoverableDemo(uint8_t mode){if(fault_injection_mode==mode){if(mode==APP_FAULT_INJECTION_INVALID_DATA)validation_demo_latched=true;fault_injection_consumed=true;fault_injection_mode=APP_FAULT_INJECTION_NONE;demo_completion_pending=true;}}
static bool FaultWindowActive(uint8_t mode){uint32_t elapsed;if((fault_injection_mode!=mode)||fault_injection_consumed)return false;elapsed=HAL_GetTick()-fault_injection_started_ms;if(elapsed<APP_FAULT_INJECTION_DURATION_MS)return true;if(mode!=APP_FAULT_INJECTION_SUPERVISOR_STALL)EndRecoverableDemo(mode);return false;}
static void PrintFaultMenu(void){App_Log("------------------------------------------------------------");App_Log(" DEMONSTRATION MENU");App_Log("------------------------------------------------------------");App_Log(" [0] Normal Operation");App_Log(" [1] Sensor Task Stall");App_Log(" [2] I2C Communication Error");App_Log(" [3] Invalid Sensor Data");App_Log(" [4] Supervisor Task Stall");App_Log("------------------------------------------------------------");App_Log(" Enter selection:");}
void App_SetFaultInjectionMode(uint8_t mode){if(mode>APP_FAULT_INJECTION_SUPERVISOR_STALL)return;taskENTER_CRITICAL();fault_injection_mode=mode;fault_injection_started_ms=HAL_GetTick();fault_injection_consumed=false;demo_completion_pending=false;taskEXIT_CRITICAL();if(mode==APP_FAULT_INJECTION_NONE){App_Log("[FAULT INJECT] Mode set: NONE");PrintFaultMenu();return;}App_Log("============================================================");App_Log(" DEMO %u : %s",(unsigned)mode,(mode==APP_FAULT_INJECTION_SENSOR_STALL)?"SENSOR TASK FAILURE":(mode==APP_FAULT_INJECTION_I2C_ERROR)?"I2C COMMUNICATION FAULT":(mode==APP_FAULT_INJECTION_INVALID_DATA)?"INVALID SENSOR DATA":"SUPERVISOR FAILURE");App_Log("============================================================");App_Log("[FAULT INJECT] %s armed",FaultModeName(mode));}

void App_Initialize(I2C_HandleTypeDef *i2c,UART_HandleTypeDef *u){AppFaultRecord_t previous;memset(&state,0,sizeof(state));state_lock=xSemaphoreCreateMutex();uart_lock=xSemaphoreCreateMutex();uart=u;state.state=APP_SYSTEM_STARTING;state.sensor_status=APP_SENSOR_ERROR;state.validation_status=APP_VALIDATION_PENDING;state.reset_flags=captured_reset_flags;MPU6050_Bind(i2c);previous=ResetWasIwdg(captured_reset_flags)&&persistent->watchdog_fault.type!=APP_FAULT_NONE?persistent->watchdog_fault:persistent->latest_fault;App_Log("============================================================");App_Log("        STM32H723 FAULT-TOLERANT IMU MONITOR");App_Log("============================================================");App_Log(" Sensor    : MPU6050");App_Log(" RTOS      : FreeRTOS");App_Log(" I2C       : PB8=SCL  PB9=SDA");App_Log(" UART      : USART1 115200 8N1");App_Log(" Watchdog  : IWDG");App_Log("============================================================");App_Log("[BOOT]");App_Log(" Reset Cause      : %s",ResetCauseName(captured_reset_flags));App_Log(" Boot Count       : %lu",(unsigned long)persistent->boot_count);App_Log(" IWDG Reset Count : %lu",(unsigned long)persistent->iwdg_reset_count);App_Log(" Previous Fault   : %s",previous.type!=APP_FAULT_NONE?FaultName(previous.type):"NONE");App_Log("============================================================");if(ResetWasIwdg(captured_reset_flags)){App_Log(" POST-RESET DIAGNOSTICS");App_Log(" Reset Cause      : IWDG");App_Log(" IWDG Reset Count : %lu",(unsigned long)persistent->iwdg_reset_count);App_Log(" Previous Fault   : %s",previous.type!=APP_FAULT_NONE?FaultName(previous.type):"NONE");App_Log("============================================================");}PrintFaultMenu();}
BaseType_t App_CreateTasks(void){if(xTaskCreate(SensorTask,"MPU6050",384U,NULL,3U,&tasks[APP_TASK_SENSOR])!=pdPASS)return pdFAIL;if(xTaskCreate(ValidationTask,"Validate",384U,NULL,2U,&tasks[APP_TASK_VALIDATION])!=pdPASS)return pdFAIL;if(xTaskCreate(SupervisorTask,"Supervisor",512U,NULL,4U,&tasks[APP_TASK_SUPERVISOR])!=pdPASS)return pdFAIL;if(xTaskCreate(UartCommandTask,"UartCmd",256U,NULL,1U,&tasks[APP_TASK_UART_COMMAND])!=pdPASS)return pdFAIL;return WatchdogInit();}

static void SensorTask(void *a){MPU6050_Status_t status;MPU6050_Measurement_t m;uint32_t fails=0U;(void)a;App_Log("[TASK] Sensor started");status=MPU6050_Initialize();if(status==MPU6050_STATUS_OK){App_Log("[MPU6050] WHO_AM_I = 0x68");App_Log("[MPU6050] Initialization OK");}else App_Log("[MPU6050] Initialization failed");for(;;){
if(FaultWindowActive(APP_FAULT_INJECTION_SENSOR_STALL))vTaskDelay(pdMS_TO_TICKS(APP_HEARTBEAT_TIMEOUT_MS+APP_SENSOR_PERIOD_MS));
Beat(APP_TASK_SENSOR);status=MPU6050_Read(&m);
if(FaultWindowActive(APP_FAULT_INJECTION_I2C_ERROR))status=MPU6050_STATUS_I2C_ERROR;
if((status==MPU6050_STATUS_OK)&&FaultWindowActive(APP_FAULT_INJECTION_INVALID_DATA))m.accel_x_g=NAN;
Lock();if(status==MPU6050_STATUS_OK){state.measurement=m;state.measurement_ready=1U;state.sensor_status=APP_SENSOR_OK;state.consecutive_i2c_failures=0U;fails=0U;}else{state.sensor_status=(status==MPU6050_STATUS_IDENTITY_ERROR)?APP_SENSOR_IDENTITY_ERROR:APP_SENSOR_I2C_ERROR;state.consecutive_i2c_failures=++fails;}Unlock();if(status!=MPU6050_STATUS_OK)Fault((status==MPU6050_STATUS_IDENTITY_ERROR)?APP_FAULT_SENSOR_IDENTITY:APP_FAULT_I2C,APP_TASK_SENSOR,MPU6050_LastI2cError());if(fails>=APP_I2C_FAILURE_LIMIT){App_Log("[RECOVERY] MPU6050/I2C attempt");status=MPU6050_Recover();if(status==MPU6050_STATUS_OK){App_Log("[RECOVERY] MPU6050/I2C success");EndRecoverableDemo(APP_FAULT_INJECTION_I2C_ERROR);PersistentRecord(APP_FAULT_RECOVERY_SUCCESS,APP_TASK_SENSOR,0U,0U,APP_SYSTEM_RECOVERING);fails=0U;Lock();state.sensor_status=APP_SENSOR_OK;state.consecutive_i2c_failures=0U;Unlock();}else{App_Log("[RECOVERY] MPU6050/I2C failed");Fault(APP_FAULT_RECOVERY_FAILURE,APP_TASK_SENSOR,MPU6050_LastI2cError());}}vTaskDelay(pdMS_TO_TICKS(APP_SENSOR_PERIOD_MS));}}

static void ValidationTask(void *a){AppSnapshot_t s;AppValidationStatus_t v;float g;(void)a;App_Log("[TASK] Validation started");for(;;){Beat(APP_TASK_VALIDATION);Snapshot(&s);if(!s.measurement_ready){Lock();state.validation_status=APP_VALIDATION_PENDING;Unlock();vTaskDelay(pdMS_TO_TICKS(APP_VALIDATION_PERIOD_MS));continue;}v=APP_VALIDATION_OK;if((s.sensor_status!=APP_SENSOR_OK)||(s.measurement.timestamp_ms==0U)||(HAL_GetTick()-s.measurement.timestamp_ms>APP_MEASUREMENT_TIMEOUT_MS))v=APP_VALIDATION_STALE;else if(!isfinite(s.measurement.accel_x_g)||!isfinite(s.measurement.accel_y_g)||!isfinite(s.measurement.accel_z_g)||!isfinite(s.measurement.gyro_x_dps)||!isfinite(s.measurement.gyro_y_dps)||!isfinite(s.measurement.gyro_z_dps)||fabsf(s.measurement.accel_x_g)>2.2f||fabsf(s.measurement.accel_y_g)>2.2f||fabsf(s.measurement.accel_z_g)>2.2f||fabsf(s.measurement.gyro_x_dps)>275.0f||fabsf(s.measurement.gyro_y_dps)>275.0f||fabsf(s.measurement.gyro_z_dps)>275.0f)v=APP_VALIDATION_INVALID;else{g=sqrtf(s.measurement.accel_x_g*s.measurement.accel_x_g+s.measurement.accel_y_g*s.measurement.accel_y_g+s.measurement.accel_z_g*s.measurement.accel_z_g);if((g<0.20f)||(g>3.20f))v=APP_VALIDATION_INVALID;}Lock();state.validation_status=v;Unlock();if(v==APP_VALIDATION_STALE)Fault(APP_FAULT_STALE_MEASUREMENT,APP_TASK_VALIDATION,0U);else if(v==APP_VALIDATION_INVALID){Fault(APP_FAULT_INVALID_MEASUREMENT,APP_TASK_VALIDATION,0U);EndRecoverableDemo(APP_FAULT_INJECTION_INVALID_DATA);}vTaskDelay(pdMS_TO_TICKS(APP_VALIDATION_PERIOD_MS));}}

static BaseType_t RestartTask(AppTaskId_t id){TaskFunction_t e=(id==APP_TASK_SENSOR)?SensorTask:ValidationTask;TaskHandle_t old=tasks[id];tasks[id]=NULL;if(old!=NULL)vTaskDelete(old);return xTaskCreate(e,(id==APP_TASK_SENSOR)?"MPU6050":"Validate",384U,NULL,(id==APP_TASK_SENSOR)?3U:2U,&tasks[id]);}
static void ReportStacks(void){for(uint32_t i=0U;i<APP_TASK_COUNT;++i)if(tasks[i]!=NULL){UBaseType_t m=uxTaskGetStackHighWaterMark(tasks[i]);if(m<APP_STACK_WARNING_WORDS)App_Log("[STACK] Warning task %lu margin %lu words",(unsigned long)i,(unsigned long)m);}}
static void Print(AppSnapshot_t *s){App_Log("[IMU]");App_Log(" ACC : X=%ld mg  Y=%ld mg  Z=%ld mg",(long)(s->measurement.accel_x_g*1000.0f),(long)(s->measurement.accel_y_g*1000.0f),(long)(s->measurement.accel_z_g*1000.0f));App_Log(" GYR : X=%ld dps Y=%ld dps Z=%ld dps",(long)s->measurement.gyro_x_dps,(long)s->measurement.gyro_y_dps,(long)s->measurement.gyro_z_dps);App_Log("[RTOS]");App_Log(" Sensor      : %s",s->sensor_status==APP_SENSOR_OK?"OK":"FAULT");App_Log(" Validation  : %s",s->validation_status==APP_VALIDATION_OK?"OK":"FAULT");App_Log(" Supervisor  : OK");App_Log(" System      : %s",StateName(s->state));}
static void UartCommandTask(void *a){uint8_t character;(void)a;for(;;){if((uart!=NULL)&&(HAL_UART_Receive(uart,&character,1U,20U)==HAL_OK)){if((character>='0')&&(character<='4'))App_SetFaultInjectionMode((uint8_t)(character-'0'));else if((character=='m')||(character=='M'))PrintFaultMenu();}vTaskDelay(pdMS_TO_TICKS(10U));}}
static void SupervisorTask(void *a){AppSnapshot_t s;TickType_t last=0U;uint32_t tries=0U;bool stall_recorded=false;(void)a;App_Log("[TASK] Supervisor started");for(;;){
if(fault_injection_mode==APP_FAULT_INJECTION_SUPERVISOR_STALL){if(!stall_recorded){PersistentSetWatchdogFault(APP_FAULT_SUPERVISOR_STALL,APP_TASK_SUPERVISOR,0U,tries,APP_SYSTEM_RECOVERING);Fault(APP_FAULT_SUPERVISOR_STALL,APP_TASK_SUPERVISOR,0U);stall_recorded=true;}vTaskDelay(pdMS_TO_TICKS(1000U));continue;}
Beat(APP_TASK_SUPERVISOR);Snapshot(&s);if(s.state==APP_SYSTEM_SAFE){vTaskDelay(pdMS_TO_TICKS(APP_SUPERVISOR_PERIOD_MS));continue;}if(!s.measurement_ready&&(s.sensor_status==APP_SENSOR_ERROR)){Transition(APP_SYSTEM_STARTING);WatchdogRefresh();vTaskDelay(pdMS_TO_TICKS(APP_SUPERVISOR_PERIOD_MS));continue;}bool sl=(xTaskGetTickCount()-s.heartbeat[APP_TASK_SENSOR].tick)>pdMS_TO_TICKS(APP_HEARTBEAT_TIMEOUT_MS);bool vl=(xTaskGetTickCount()-s.heartbeat[APP_TASK_VALIDATION].tick)>pdMS_TO_TICKS(APP_HEARTBEAT_TIMEOUT_MS);bool healthy=!sl&&!vl&&s.sensor_status==APP_SENSOR_OK&&s.validation_status==APP_VALIDATION_OK;if(healthy){if(s.state==APP_SYSTEM_RECOVERING){App_Log("[RECOVERY] SUCCESS");PersistentRecord(APP_FAULT_RECOVERY_SUCCESS,APP_TASK_SUPERVISOR,0U,tries,APP_SYSTEM_HEALTHY);}tries=0U;Lock();state.recovery_attempt=0U;Unlock();Transition(APP_SYSTEM_HEALTHY);if(demo_completion_pending){demo_completion_pending=false;App_Log("SYSTEM STATUS : HEALTHY");App_Log("============================================================");App_Log(" DEMO COMPLETE");App_Log("============================================================");PrintFaultMenu();}WatchdogRefresh();}else if(tries<APP_RECOVERY_LIMIT){AppFaultType_t f;AppTaskId_t t;tries++;Lock();state.recovery_attempt=tries;Unlock();Transition(APP_SYSTEM_RECOVERING);if(sl){f=APP_FAULT_SENSOR_HEARTBEAT;t=APP_TASK_SENSOR;}else if(vl){f=APP_FAULT_VALIDATION_HEARTBEAT;t=APP_TASK_VALIDATION;}else if(s.sensor_status==APP_SENSOR_IDENTITY_ERROR){f=APP_FAULT_SENSOR_IDENTITY;t=APP_TASK_SENSOR;}else if(s.sensor_status==APP_SENSOR_I2C_ERROR){f=APP_FAULT_I2C;t=APP_TASK_SENSOR;}else if(s.validation_status==APP_VALIDATION_STALE){f=APP_FAULT_STALE_MEASUREMENT;t=APP_TASK_VALIDATION;}else{f=APP_FAULT_INVALID_MEASUREMENT;t=APP_TASK_VALIDATION;}Fault(f,t,MPU6050_LastI2cError());App_Log("*** FAULT DETECTED *** %s",FaultName(f));App_Log("[SUPERVISOR] State : RECOVERING");App_Log("[RECOVERY] Attempt %lu/%u",(unsigned long)tries,APP_RECOVERY_LIMIT);if(sl||vl){if(RestartTask(t)==pdPASS)App_Log("[RECOVERY] %s task restarted",t==APP_TASK_SENSOR?"Sensor":"Validation");else Fault(APP_FAULT_RECOVERY_FAILURE,t,0U);}WatchdogRefresh();}else{Fault(APP_FAULT_RECOVERY_FAILURE,APP_TASK_SUPERVISOR,MPU6050_LastI2cError());Transition(APP_SYSTEM_SAFE);App_Log("[RECOVERY] FAILED");App_Log("SYSTEM STATUS : SAFE");App_Log("[SUPERVISOR] State : SAFE");App_Log("[IWDG] Refresh stopped");if(demo_completion_pending){demo_completion_pending=false;App_Log("============================================================");App_Log(" DEMO COMPLETE");App_Log("============================================================");}}Snapshot(&s);if((xTaskGetTickCount()-last)>=pdMS_TO_TICKS(2000U)){Print(&s);ReportStacks();last=xTaskGetTickCount();}vTaskDelay(pdMS_TO_TICKS(APP_SUPERVISOR_PERIOD_MS));}}
