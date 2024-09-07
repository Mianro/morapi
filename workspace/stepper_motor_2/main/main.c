#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

//PRIVATE INCLUDES
#include "sdkconfig.h"

#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
//#include "esp_clk.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/spi_slave.h"

#include "driver/ledc.h"

#include "driver/timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"

#include "esp32/rom/rtc.h"
#include "esp32/rtc.h"
#include "esp_err.h"




/* Private constants  ---------------------------------------------------------*/
#define HIGH			1
#define LOW 			0


#define SPI_FREQUENCY   1000000 // 1 MHz
#define TIMER_BASE_CLK  80000000

// TIMER DEFINITIONS
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_OUTPUT_IO          (18) // Pin de salida del PWM
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Resolución de 13 bits
#define TIMER_PRESCALER         1
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X_FREQ_RESCALER 1
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X LEDC_CHANNEL_0



//DRIVER SPI PINS
#define HSPI_MISO   26
#define HSPI_MOSI   27
#define HSPI_SCLK   25
#define CHIP_SELECT 5
#define HSPI_SS     32

#define LIGHT       16
/// Timer handler for PWMX
timer_config_t hTimPwm;

/// Error while initialising the SPI
#define L6474_ERROR_0   (0x8000)
/// Error: Bad SPI transaction
#define L6474_ERROR_1   (0x8001)

/// Maximum number of steps
#define MAX_STEPS         (0x7FFFFFFF)

/// Maximum frequency of the PWMs in Hz
#define L6474_MAX_PWM_FREQ   (10000)

/// Minimum frequency of the PWMs in Hz
#define L6474_MIN_PWM_FREQ   (2)

/// L6474 max number of bytes of command & arguments to set a parameter
#define L6474_CMD_ARG_MAX_NB_BYTES              (4)

/// L6474 command + argument bytes number for GET_STATUS command
#define L6474_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L6474 response bytes number
#define L6474_RSP_NB_BYTES_GET_STATUS           (2)

/// L6474 value mask for ABS_POS register
#define L6474_ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)

/// L6474 sign bit mask for ABS_POS register
#define L6474_ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)

/************************ Phase Current Control *******************************/

/// TVAL register value for device 0 (range 31.25mA to 4000mA)
#define L6474_CONF_PARAM_TVAL_DEVICE_0  (625)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 0 (range 2us to 32us)
#define L6474_CONF_PARAM_TOFF_FAST_DEVICE_0  (L6474_TOFF_FAST_8us)
/// Fall time value (T_FAST field of T_FAST register) for device 0 (range 2us to 32us)
#define L6474_CONF_PARAM_FAST_STEP_DEVICE_0  (L6474_FAST_STEP_12us)
/// Minimum ON time (TON_MIN register) for device 0 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TON_MIN_DEVICE_0 (3)
/// Minimum OFF time (TOFF_MIN register) for device 0 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TOFF_MIN_DEVICE_0 (21)

/******************************* Others ***************************************/

/// Overcurrent threshold settings for device 0 (OCD_TH register)
#define L6474_CONF_PARAM_OCD_TH_DEVICE_0  (L6474_OCD_TH_1125mA)
/// Step selection settings for device 0 (STEP_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_STEP_SEL_DEVICE_0  (L6474_STEP_SEL_1_16)
/// Synch. selection settings for device 0 (SYNC_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_SYNC_SEL_DEVICE_0  (L6474_SYNC_SEL_1_2)
/// Alarm settings for device 0 (ALARM_EN register)
#define L6474_CONF_PARAM_ALARM_EN_DEVICE_0  (L6474_ALARM_EN_OVERCURRENT |\
                                             L6474_ALARM_EN_THERMAL_SHUTDOWN |\
                                             L6474_ALARM_EN_THERMAL_WARNING |\
                                             L6474_ALARM_EN_UNDERVOLTAGE |\
                                             L6474_ALARM_EN_SW_TURN_ON |\
                                             L6474_ALARM_EN_WRONG_NPERF_CMD)

/// Clock setting for device 0 (OSC_CLK_SEL field of CONFIG register)
#define L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0  (L6474_CONFIG_INT_16MHZ)
/// Torque regulation method for device 0 (EN_TQREG field of CONFIG register)
#define L6474_CONF_PARAM_TQ_REG_DEVICE_0  (L6474_CONFIG_EN_TQREG_TVAL_USED)
/// Over current shutwdown enabling for device 0 (OC_SD field of CONFIG register)
#define L6474_CONF_PARAM_OC_SD_DEVICE_0  (L6474_CONFIG_OC_SD_ENABLE)
/// Slew rate for device 0 (POW_SR field of CONFIG register)
#define L6474_CONF_PARAM_SR_DEVICE_0  (L6474_CONFIG_SR_320V_us)
/// Target Swicthing Period for device 0 (field TOFF of CONFIG register)
#define L6474_CONF_PARAM_TOFF_DEVICE_0  (L6474_CONFIG_TOFF_044us)

/************************ Speed Profile  *******************************/

/// Acceleration rate in step/s2 for device 0 (must be greater than 0)
#define L6474_CONF_PARAM_ACC_DEVICE_0        (160)
/// Deceleration rate in step/s2 for device 0 (must be greater than 0)
#define L6474_CONF_PARAM_DEC_DEVICE_0        (160)
/// Maximum speed in step/s for device 0 (30 step/s < Maximum speed <= 10 000 step/s )
#define L6474_CONF_PARAM_MAX_SPEED_DEVICE_0  (1600)
/// Minimum speed in step/s for device 0 (30 step/s <= Minimum speed < 10 000 step/s)
#define L6474_CONF_PARAM_MIN_SPEED_DEVICE_0  (800)


/** @defgroup L6474_Command_Set
  * @{
  */
/// L6474 command set
typedef enum {
  L6474_NOP           = ((uint8_t) 0x00),
  L6474_SET_PARAM     = ((uint8_t) 0x00),
  L6474_GET_PARAM     = ((uint8_t) 0x20),
  L6474_ENABLE        = ((uint8_t) 0xB8),
  L6474_DISABLE       = ((uint8_t) 0xA8),
  L6474_GET_STATUS    = ((uint8_t) 0xD0),
  L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6474_Commands_t;
/**
  * @}
  */

/** @defgroup L6474_Internal_Register_Addresses
  * @{
  */
/// Internal L6474 register addresses
typedef enum {
  L6474_ABS_POS        = ((uint8_t) 0x01),
  L6474_EL_POS         = ((uint8_t) 0x02),
  L6474_MARK           = ((uint8_t) 0x03),
  L6474_RESERVED_REG01 = ((uint8_t) 0x04),
  L6474_RESERVED_REG02 = ((uint8_t) 0x05),
  L6474_RESERVED_REG03 = ((uint8_t) 0x06),
  L6474_RESERVED_REG04 = ((uint8_t) 0x07),
  L6474_RESERVED_REG05 = ((uint8_t) 0x08),
  L6474_RESERVED_REG06 = ((uint8_t) 0x15),
  L6474_TVAL           = ((uint8_t) 0x09),
  L6474_RESERVED_REG07 = ((uint8_t) 0x0A),
  L6474_RESERVED_REG08 = ((uint8_t) 0x0B),
  L6474_RESERVED_REG09 = ((uint8_t) 0x0C),
  L6474_RESERVED_REG10 = ((uint8_t) 0x0D),
  L6474_T_FAST         = ((uint8_t) 0x0E),
  L6474_TON_MIN        = ((uint8_t) 0x0F),
  L6474_TOFF_MIN       = ((uint8_t) 0x10),
  L6474_RESERVED_REG11 = ((uint8_t) 0x11),
  L6474_ADC_OUT        = ((uint8_t) 0x12),
  L6474_OCD_TH         = ((uint8_t) 0x13),
  L6474_RESERVED_REG12 = ((uint8_t) 0x14),
  L6474_STEP_MODE      = ((uint8_t) 0x16),
  L6474_ALARM_EN       = ((uint8_t) 0x17),
  L6474_CONFIG         = ((uint8_t) 0x18),
  L6474_STATUS         = ((uint8_t) 0x19),
  L6474_RESERVED_REG13 = ((uint8_t) 0x1A),
  L6474_RESERVED_REG14 = ((uint8_t) 0x1B),
  L6474_INEXISTENT_REG = ((uint8_t) 0x1F)
} L6474_Registers_t;
/**
  * @}
  */

/** @defgroup L6474_Fast_Decay_Time_Options
  * @{
  */
///TOFF_FAST values for T_FAST register
typedef enum {
  L6474_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  L6474_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  L6474_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  L6474_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  L6474_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  L6474_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  L6474_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  L6474_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  L6474_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  L6474_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  L6474_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  L6474_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  L6474_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  L6474_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  L6474_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  L6474_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} L6474_TOFF_FAST_t;
/**
  * @}
  */

/** @defgroup L6474_Fall_Step_Time_Options
  * @{
  */
///FAST_STEP values for T_FAST register
typedef enum {
  L6474_FAST_STEP_2us = ((uint8_t) 0x00),
  L6474_FAST_STEP_4us = ((uint8_t) 0x01),
  L6474_FAST_STEP_6us = ((uint8_t) 0x02),
  L6474_FAST_STEP_8us = ((uint8_t) 0x03),
  L6474_FAST_STEP_10us = ((uint8_t) 0x04),
  L6474_FAST_STEP_12us = ((uint8_t) 0x05),
  L6474_FAST_STEP_14us = ((uint8_t) 0x06),
  L6474_FAST_STEP_16us = ((uint8_t) 0x07),
  L6474_FAST_STEP_18us = ((uint8_t) 0x08),
  L6474_FAST_STEP_20us = ((uint8_t) 0x09),
  L6474_FAST_STEP_22us = ((uint8_t) 0x0A),
  L6474_FAST_STEP_24us = ((uint8_t) 0x0B),
  L6474_FAST_STEP_26us = ((uint8_t) 0x0C),
  L6474_FAST_STEP_28us = ((uint8_t) 0x0D),
  L6474_FAST_STEP_30us = ((uint8_t) 0x0E),
  L6474_FAST_STEP_32us = ((uint8_t) 0x0F)
} L6474_FAST_STEP_t;
/**
  * @}
  */
/** @defgroup L6474_Overcurrent_Threshold_options
  * @{
  */

///OCD_TH register
typedef enum {
  L6474_OCD_TH_375mA  = ((uint8_t) 0x00),
  L6474_OCD_TH_750mA  = ((uint8_t) 0x01),
  L6474_OCD_TH_1125mA = ((uint8_t) 0x02),
  L6474_OCD_TH_1500mA = ((uint8_t) 0x03),
  L6474_OCD_TH_1875mA = ((uint8_t) 0x04),
  L6474_OCD_TH_2250mA = ((uint8_t) 0x05),
  L6474_OCD_TH_2625mA = ((uint8_t) 0x06),
  L6474_OCD_TH_3000mA = ((uint8_t) 0x07),
  L6474_OCD_TH_3375mA = ((uint8_t) 0x08),
  L6474_OCD_TH_3750mA = ((uint8_t) 0x09),
  L6474_OCD_TH_4125mA = ((uint8_t) 0x0A),
  L6474_OCD_TH_4500mA = ((uint8_t) 0x0B),
  L6474_OCD_TH_4875mA = ((uint8_t) 0x0C),
  L6474_OCD_TH_5250mA = ((uint8_t) 0x0D),
  L6474_OCD_TH_5625mA = ((uint8_t) 0x0E),
  L6474_OCD_TH_6000mA = ((uint8_t) 0x0F)
} L6474_OCD_TH_t;
/**
  * @}
  */

/** @defgroup L6474_STEP_SEL_Options_For_STEP_MODE_Register
  * @{
  */
///STEP_SEL field of STEP_MODE register
typedef enum {
  L6474_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  L6474_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  L6474_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  L6474_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  L6474_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} L6474_STEP_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_SYNC_SEL_Options_For_STEP_MODE_Register
  * @{
  */
///SYNC_SEL field of STEP_MODE register
typedef enum {
  L6474_SYNC_SEL_1_2    = ((uint8_t) 0x80),
  L6474_SYNC_SEL_1      = ((uint8_t) 0x90),
  L6474_SYNC_SEL_2      = ((uint8_t) 0xA0),
  L6474_SYNC_SEL_4      = ((uint8_t) 0xB0),
  L6474_SYNC_SEL_8      = ((uint8_t) 0xC0),
  L6474_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} L6474_SYNC_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_ALARM_EN_Register_Options
  * @{
  */
///ALARM_EN register
typedef enum {
  L6474_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  L6474_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  L6474_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  L6474_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  L6474_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  L6474_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} L6474_ALARM_EN_t;
/**
  * @}
  */

/** @defgroup L6474_CONFIG_Register_Masks
  * @{
  */
///CONFIG register
typedef enum {
  L6474_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  L6474_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  L6474_CONFIG_OC_SD    = ((uint16_t) 0x0080),
  L6474_CONFIG_POW_SR   = ((uint16_t) 0x0300),
  L6474_CONFIG_TOFF      = ((uint16_t) 0x7C00)
} L6474_CONFIG_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_Clock_Source_Options_For_CONFIG_Register
  * @{
  */
///Clock source option for CONFIG register
typedef enum {
  L6474_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
  L6474_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
  L6474_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  L6474_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  L6474_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  L6474_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  L6474_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  L6474_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  L6474_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  L6474_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  L6474_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  L6474_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} L6474_CONFIG_OSC_MGMT_t;
/**
  * @}
  */

/** @defgroup L6474_External_Torque_Regulation_Options_For_CONFIG_Register
  * @{
  */
///External Torque regulation options for CONFIG register
typedef enum {
  L6474_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  L6474_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} L6474_CONFIG_EN_TQREG_t;
/**
  * @}
  */

/** @defgroup L6474_Over_Current_Shutdown_Options_For_CONFIG_Register
  * @{
  */
///Over Current Shutdown options for CONFIG register
typedef enum {
  L6474_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  L6474_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} L6474_CONFIG_OC_SD_t;
/**
  * @}
  */

/** @defgroup L6474_Power_Bridge_Output_Slew_Rate_Options
  * @{
  */
/// POW_SR values for CONFIG register
typedef enum {
  L6474_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  L6474_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  L6474_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  L6474_CONFIG_SR_260V_us    =((uint16_t)0x0300)
} L6474_CONFIG_POW_SR_t;
/**
  * @}
  */

/** @defgroup L6474_Off_Time_Options
  * @{
  */
/// TOFF values for CONFIG register
typedef enum {
  L6474_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  L6474_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  L6474_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  L6474_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  L6474_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  L6474_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  L6474_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  L6474_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  L6474_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  L6474_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  L6474_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  L6474_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  L6474_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  L6474_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  L6474_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  L6474_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  L6474_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  L6474_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  L6474_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  L6474_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  L6474_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  L6474_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  L6474_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  L6474_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  L6474_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  L6474_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  L6474_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  L6474_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  L6474_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  L6474_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  L6474_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} L6474_CONFIG_TOFF_t;
/**
  * @}
  */

/** @defgroup Device_Direction_Options
  * @{
  */
/// Direction options
typedef enum {
  BACKWARD = 0,
  FORWARD = 1,
  UNKNOW_DIR = ((uint8_t)0xFF)
} motorDir_t;
/**
  * @}
  */


/** @defgroup Device_States
  * @{
  */
/// Device states
typedef enum {
  ACCELERATING       = 0,
  DECELERATINGTOSTOP = 1,
  DECELERATING       = 2,
  STEADY             = 3,
  INDEX_ACCEL        = 4,
  INDEX_RUN          = 5,
  INDEX_DECEL        = 6,
  INDEX_DWELL        = 7,
  INACTIVE           = 8
} motorState_t;
/**
  * @}
  */


/** @defgroup Device_Commands
  * @{
  */
/// Device commands
typedef enum {
  RUN_CMD,
  MOVE_CMD,
  SOFT_STOP_CMD,
  NO_CMD
} deviceCommand_t;
/**
  * @}
  */


/** @defgroup Device_Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
    /// accumulator used to store speed increase smaller than 1 pps
    volatile uint32_t accu;
    /// Position in steps at the start of the goto or move commands
    volatile int32_t currentPosition;
    /// position in step at the end of the accelerating phase
    volatile uint32_t endAccPos;
    /// nb steps performed from the beggining of the goto or the move command
    volatile uint32_t relativePos;
    /// position in step at the start of the decelerating phase
    volatile uint32_t startDecPos;
    /// nb steps to perform for the goto or move commands
    volatile uint32_t stepsToTake;

    /// acceleration in pps^2
    volatile uint16_t acceleration;
    /// deceleration in pps^2
    volatile uint16_t deceleration;
    /// max speed in pps (speed use for goto or move command)
    volatile uint16_t maxSpeed;
    /// min speed in pps
    volatile uint16_t minSpeed;
    /// current speed in pps
    volatile uint16_t speed;

    /// command under execution
    volatile deviceCommand_t commandExecuted;
    /// FORWARD or BACKWARD direction
    volatile motorDir_t direction;
    /// Current State of the device
    volatile motorState_t motionState;
}deviceParams_t;
/**
  * @}
  */


/* Private variables ---------------------------------------------------------*/

/** @defgroup L6474_Private_Variables
  * @{
  */

/// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
/// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);

static uint8_t spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][0];
static uint8_t spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][0];
static volatile bool spiPreemtionByIsr = false;
static volatile bool isrFlag = false;
/// L6474 Device Paramaters structure
deviceParams_t devicePrm;

const TickType_t xDelay = 40 / portTICK_PERIOD_MS;

/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6474_Private_functions
  * @{
  */
void L6474_ApplySpeed(uint16_t newSpeed);
void L6474_ComputeSpeedProfile(uint32_t nbSteps);
int32_t L6474_ConvertPosition(uint32_t abs_position_reg);
void L6474_ErrorHandler(uint16_t error);
void L6474_FlagInterruptHandler(void);
void L6474_SendCommand(uint8_t param);
void L6474_SetRegisterToPredefinedValues(void);
void L6474_WriteBytes(uint8_t *pByteToTransmit);
void L6474_SetDeviceParamsToPredefinedValues(void);
void L6474_StartMovement(void);
void L6474_StepClockHandler(void);
uint8_t L6474_Tval_Current_to_Par(double Tval);
uint8_t L6474_Tmin_Time_to_Par(double Tmin);

//void spi_init(void){
//    spi_bus_config_t buscfg = {
//        .miso_io_num = HSPI_MISO,
//        .mosi_io_num = HSPI_MOSI,
//        .sclk_io_num = HSPI_SCLK,
//        .quadwp_io_num = -1,
//        .quadhd_io_num = -1,
//    };
//
//    spi_device_interface_config_t devcfg = {
//        .clock_speed_hz = SPI_FREQUENCY,
//        .mode = 0,
//        .spics_io_num = HSPI_SS,
//        .queue_size = 1,
//    };
//
//    spi_bus_initialize(HSPI_HOST, &buscfg, 1);
//    spi_bus_add_device(HSPI_HOST, &devcfg, &device);
//}


void configure_pin (void){
    gpio_reset_pin(LIGHT);
    gpio_reset_pin(HSPI_MISO);
    gpio_reset_pin(HSPI_MOSI);
    gpio_reset_pin(HSPI_SCLK);
    gpio_reset_pin(HSPI_SS);
    gpio_reset_pin(CHIP_SELECT);

    gpio_set_direction(LIGHT,		GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_MISO, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_MOSI, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_SCLK, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(HSPI_SS, 	GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT, GPIO_MODE_OUTPUT);
}

/******************************************************//**
 * @brief This function disable the interruptions
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_DisableIrq(void)
{
	portDISABLE_INTERRUPTS();
}

/******************************************************//**
 * @brief This function enable the interruptions
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_EnableIrq(void)
{
	  portENABLE_INTERRUPTS();
}


/******************************************************//**
 * @brief  Sets the frequency of PWM_X used by device 0
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmSetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = 80000000;
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * (uint32_t)newFreq)) - 1;

  // Configurar el temporizador LEDC con la nueva frecuencia
      ledc_timer_config_t ledc_timer = {
          .duty_resolution = LEDC_DUTY_RES,
          .freq_hz = newFreq,
          .speed_mode = LEDC_MODE,
          .timer_num = LEDC_TIMER,
          .clk_cfg = LEDC_AUTO_CLK
      };
      ledc_timer_config(&ledc_timer);

      // Configurar el canal LEDC
      ledc_channel_config_t ledc_channel = {
          .channel    = LEDC_CHANNEL,
          .duty       = period / 2, // Establecer el duty cycle al 50%
          .gpio_num   = LEDC_OUTPUT_IO,
          .speed_mode = LEDC_MODE,
          .hpoint     = 0,
          .timer_sel  = LEDC_TIMER
      };
      ledc_channel_config(&ledc_channel);

      // Iniciar el PWM
      ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/******************************************************//**
 * @brief  Stops the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_PwmStop(void)
{
	ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
}


/******************************************************//**
 * @brief  Issues the Enable command to the L6474 of the specified device
 * @retval None
 **********************************************************/
void L6474_CmdEnable(void)
{
  L6474_SendCommand(L6474_ENABLE);
}

/******************************************************//**
 * @brief  Issue the Disable command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_CmdDisable(void)
{
  L6474_SendCommand(L6474_DISABLE);
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_HardStop(void)
{
  /* Disable corresponding PWM */
  BSP_MotorControlBoard_PwmStop();

  /* Disable power stage */
  L6474_CmdDisable();

  /* Set inactive state */
  devicePrm.motionState = INACTIVE;
  devicePrm.commandExecuted = NO_CMD;
  devicePrm.stepsToTake = MAX_STEPS;
}

/******************************************************//**
 * @brief  Updates the current speed of the device
 * @param[in] newSpeed in pps
 * @retval None
 **********************************************************/
void L6474_ApplySpeed(uint16_t newSpeed)
{
  if (newSpeed < L6474_MIN_PWM_FREQ)
  {
    newSpeed = L6474_MIN_PWM_FREQ;
  }
  if (newSpeed > L6474_MAX_PWM_FREQ)
  {
    newSpeed = L6474_MAX_PWM_FREQ;
  }

  devicePrm.speed = newSpeed;

  BSP_MotorControlBoard_PwmSetFreq(newSpeed);
}

/******************************************************//**
 * @brief  Write and read SPI byte to the L6474
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval ESP_OK (0) if SPI transaction is OK, ESP_FAIL (-1) else
 **********************************************************/
uint8_t BSP_MotorControlBoard_SpiWriteBytes(uint8_t *pByteToTransmit)
{
    spi_transaction_t t;
    spi_device_handle_t device;
    esp_err_t status = ESP_OK;


	//CONFIGURAR TRANSACION SPI
    memset(&t, 0, sizeof(t)); // Limpiar la estructura de la transacción
    t.length = 8; // Longitud de la transacción en bits
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA; // Usar los datos de TX y RX proporcionados
    t.tx_data[0] = *pByteToTransmit; // Configurar el primer byte a transmitir
    t.tx_buffer = pByteToTransmit;
    t.rxlength = 8; // Longitud de la recepción en bits

    memset(&device, 0, sizeof(device));
//  device->cfg =
//	device->dev_lock =
//	device->hal_dev =
//	device->host =
//	device->id =
//	device->ret_queue =
//	device->trans_queue =


	gpio_set_level(CHIP_SELECT, LOW);
	//comprobar esta funcion
//	status = spi_device_acquire_bus(SPI_DEVICE_ID, portMAX_DELAY);

	status = spi_device_transmit(device, &t);

	if (status != ESP_OK)
	{
		return (uint8_t) status;
	}
	pByteToTransmit++;

	//Comprobar esta funcion
//	spi_device_release_bus(handle); // Liberar el bus SPI
	gpio_set_level(CHIP_SELECT, HIGH);

	return (uint8_t) status;
}

/******************************************************//**
 * @brief Converts mA in compatible values for TVAL register
 * @param[in] Tval
 * @retval TVAL values
 **********************************************************/
inline uint8_t L6474_Tval_Current_to_Par(double Tval)
{
  return ((uint8_t)(((Tval - 31.25)/31.25)+0.5));
}

/******************************************************//**
 * @brief Convert time in us in compatible values
 * for TON_MIN register
 * @param[in] Tmin
 * @retval TON_MIN values
 **********************************************************/
inline uint8_t L6474_Tmin_Time_to_Par(double Tmin)
{
  return ((uint8_t)(((Tmin - 0.5)*2)+0.5));
}

/******************************************************//**
 * @brief  Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 **********************************************************/
void L6474_WriteBytes(uint8_t *pByteToTransmit)
{
//  if (BSP_MotorControlBoard_SpiWriteBytes(pByteToTransmit))
//  {
//    L6474_ErrorHandler(L6474_ERROR_1);
//  }
  BSP_MotorControlBoard_SpiWriteBytes(pByteToTransmit);

  if (isrFlag)
  {
    spiPreemtionByIsr = true;
  }
}

/******************************************************//**
 * @brief  Issues the SetParam command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void L6474_CmdSetParam(uint32_t param,
                       uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = 0;
  bool itDisable = false;
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable BSP_MotorControlBoard_EnableIrq if disable in previous iteration */
      BSP_MotorControlBoard_EnableIrq();
      itDisable = false;
    }

      spiTxBursts[0][0] = L6474_NOP;
      spiTxBursts[1][0] = L6474_NOP;
      spiTxBursts[2][0] = L6474_NOP;
      spiTxBursts[3][0] = L6474_NOP;

    switch (param)
  {
    case L6474_ABS_POS: ; //FALL-THROUGH PARA SALTAR AL SIGUIENTE CASO
    case L6474_MARK:
        spiTxBursts[0][spiIndex] = param;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
    case L6474_EL_POS: ; //FALL-THROUGH PARA SALTAR AL SIGUIENTE CASO
    case L6474_CONFIG:
        spiTxBursts[1][spiIndex] = param;
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
    default:
        spiTxBursts[2][spiIndex] = param;
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    BSP_MotorControlBoard_DisableIrq();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  /* SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spiTxBursts[3][0]);
  }
  /* re-enable BSP_MotorControlBoard_EnableIrq after SPI transfers*/
  BSP_MotorControlBoard_EnableIrq();
}

/******************************************************//**
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the device,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 **********************************************************/
void L6474_ComputeSpeedProfile(uint32_t nbSteps)
{
  uint32_t reqAccSteps;
  uint32_t reqDecSteps;

  /* compute the number of steps to get the targeted speed */
  uint16_t minSpeed = devicePrm.minSpeed;
  reqAccSteps = (devicePrm.maxSpeed - minSpeed);
  reqAccSteps *= (devicePrm.maxSpeed + minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)devicePrm.acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)devicePrm.deceleration;
  reqDecSteps /= 2;

	if(( reqAccSteps + reqDecSteps ) > nbSteps)
	{
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
    uint32_t dec = devicePrm.deceleration;
    uint32_t acc = devicePrm.acceleration;

    reqDecSteps =  ((uint32_t) dec * nbSteps) / (acc + dec);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }
    }
    else
    {
      reqAccSteps = 0;
    }
    devicePrm.endAccPos = reqAccSteps;
    devicePrm.startDecPos = reqDecSteps;
	}
	else
	{
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    devicePrm.endAccPos = reqAccSteps;
    devicePrm.startDecPos = nbSteps - reqDecSteps - 1;
	}
}

/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position
 **********************************************************/
int32_t L6474_ConvertPosition(uint32_t abs_position_reg)
{
	int32_t operation_result;

  if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK)
  {
		/* Negative register value */
		abs_position_reg = ~abs_position_reg;
		abs_position_reg += 1;

		operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
		operation_result = -operation_result;
  }
  else
  {
		operation_result = (int32_t) abs_position_reg;
	}
	return operation_result;
}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6474_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void L6474_FlagInterruptHandler(void)
{
  if (flagInterruptCallback != 0)
  {
    /* Set isr flag */
    isrFlag = true;

    flagInterruptCallback();

    /* Reset isr flag */
    isrFlag = false;
  }
}

/******************************************************//**
 * @brief  Sends a command without arguments to the L6474 via the SPI
 * @param[in] param Command to send
 * @retval None
 **********************************************************/
void L6474_SendCommand(uint8_t param)
{
  uint8_t spiIndex = 0;
  bool itDisable = false;

  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable BSP_MotorControlBoard_EnableIrq if disable in previous iteration */
      BSP_MotorControlBoard_EnableIrq();
//      esp_intr_enable_source(inum);
      itDisable = false;
    }

//    for (i = 0; i < numberOfDevices; i++)
//    {
//      spiTxBursts[3][i] = L6474_NOP;
//    }
    spiTxBursts[3][spiIndex] = param;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    BSP_MotorControlBoard_DisableIrq();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  L6474_WriteBytes(&spiTxBursts[3][spiIndex]);

  /* re-enable BSP_MotorControlBoard_EnableIrq after SPI transfers*/
  BSP_MotorControlBoard_EnableIrq();

}


/******************************************************//**
 * @brief  Sets the registers of the L6474 to their predefined values
 * from l6474_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_SetRegisterToPredefinedValues(void)
{
	L6474_CmdSetParam(
				L6474_ABS_POS,
				0);
	L6474_CmdSetParam(
				L6474_EL_POS,
				0);
	L6474_CmdSetParam(
				L6474_MARK,
				0);

	L6474_CmdSetParam(
					L6474_TVAL,
					L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_0));
	L6474_CmdSetParam(
						  L6474_T_FAST,
						  (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_0 |
						  (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_0);
	L6474_CmdSetParam(
						  L6474_TON_MIN,
						  L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_0)
							);
	L6474_CmdSetParam(
						  L6474_TOFF_MIN,
						  L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_0));
	L6474_CmdSetParam(
					L6474_OCD_TH,
					L6474_CONF_PARAM_OCD_TH_DEVICE_0);
	L6474_CmdSetParam(
					L6474_STEP_MODE,
					(uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_0 |
					(uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_0);
	L6474_CmdSetParam(
					L6474_ALARM_EN,
					L6474_CONF_PARAM_ALARM_EN_DEVICE_0);
	L6474_CmdSetParam(
					L6474_CONFIG,
					(uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
					(uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_0 |
					(uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_0 |
					(uint16_t)L6474_CONF_PARAM_SR_DEVICE_0 |
					(uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_0);
}


/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from l6474_target_config.h
 * @param None
 * @retval None
 **********************************************************/
void L6474_SetDeviceParamsToPredefinedValues(void)
{
	devicePrm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_0;
	devicePrm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_0;
	devicePrm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_0;
	devicePrm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_0;

	devicePrm.accu = 0;
	devicePrm.currentPosition = 0;
	devicePrm.endAccPos = 0;
	devicePrm.relativePos = 0;
	devicePrm.startDecPos = 0;
	devicePrm.stepsToTake = 0;
	devicePrm.speed = 0;
	devicePrm.commandExecuted = NO_CMD;
	devicePrm.direction = FORWARD;
	devicePrm.motionState = INACTIVE;

    L6474_SetRegisterToPredefinedValues();
}


/******************************************************//**
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_StartMovement(void)
{
  /* Enable L6474 powerstage */
  L6474_CmdEnable();
  if (devicePrm.endAccPos != 0)
  {
    devicePrm.motionState = ACCELERATING;
  }
  else
  {
    devicePrm.motionState = DECELERATING;
  }
  devicePrm.accu = 0;
  devicePrm.relativePos = 0;
  L6474_ApplySpeed(devicePrm.minSpeed);
}

/******************************************************//**
 * @brief  Handles the device state machine at each ste
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void L6474_StepClockHandler(void)
{
  /* Set isr flag */
  isrFlag = true;

  /* Incrementation of the relative position */
  devicePrm.relativePos++;

  switch (devicePrm.motionState)
  {
    case ACCELERATING:
    {
        uint32_t relPos = devicePrm.relativePos;
        uint32_t endAccPos = devicePrm.endAccPos;
        uint16_t speed = devicePrm.speed;
        uint32_t acc = ((uint32_t)devicePrm.acceleration << 16);

        if ((devicePrm.commandExecuted == SOFT_STOP_CMD)||
            ((devicePrm.commandExecuted != RUN_CMD)&&
             (relPos == devicePrm.startDecPos)))
        {
          devicePrm.motionState = DECELERATING;
          devicePrm.accu = 0;
        }
        else if ((speed >= devicePrm.maxSpeed)||
                 ((devicePrm.commandExecuted != RUN_CMD)&&
                  (relPos == endAccPos)))
        {
          devicePrm.motionState = STEADY;
        }
        else
        {
          bool speedUpdated = false;
          /* Go on accelerating */
          if (speed == 0) speed =1;
          devicePrm.accu += acc / speed;
          while (devicePrm.accu >= (0X10000L))
          {
            devicePrm.accu -= (0X10000L);
            speed +=1;
            speedUpdated = true;
          }

          if (speedUpdated)
          {
            if (speed > devicePrm.maxSpeed)
            {
              speed = devicePrm.maxSpeed;
            }
            devicePrm.speed = speed;
            L6474_ApplySpeed(devicePrm.speed);
          }
        }
        break;
    }
    case STEADY:
    {
      uint16_t maxSpeed = devicePrm.maxSpeed;
      uint32_t relativePos = devicePrm.relativePos;
      if  ((devicePrm.commandExecuted == SOFT_STOP_CMD)||
           ((devicePrm.commandExecuted != RUN_CMD)&&
            (relativePos >= (devicePrm.startDecPos))) ||
           ((devicePrm.commandExecuted == RUN_CMD)&&
            (devicePrm.speed > maxSpeed)))
      {
        devicePrm.motionState = DECELERATING;
        devicePrm.accu = 0;
      }
      else if ((devicePrm.commandExecuted == RUN_CMD)&&
               (devicePrm.speed < maxSpeed))
      {
        devicePrm.motionState = ACCELERATING;
        devicePrm.accu = 0;
      }
      break;
    }
    case DECELERATING:
    {
      uint32_t relativePos = devicePrm.relativePos;
      uint16_t speed = devicePrm.speed;
      uint32_t deceleration = ((uint32_t)devicePrm.deceleration << 16);
      if (((devicePrm.commandExecuted == SOFT_STOP_CMD)&&(speed <=  devicePrm.minSpeed))||
          ((devicePrm.commandExecuted != RUN_CMD)&&
           (relativePos >= devicePrm.stepsToTake)))
      {
        /* Motion process complete */
        L6474_HardStop();
      }
      else if ((devicePrm.commandExecuted == RUN_CMD)&&
               (speed <= devicePrm.maxSpeed))
      {
        devicePrm.motionState = STEADY;
      }
      else
      {
        /* Go on decelerating */
        if (speed > devicePrm.minSpeed)
        {
          bool speedUpdated = false;
          if (speed == 0) speed =1;
          devicePrm.accu += deceleration / speed;
          while (devicePrm.accu >= (0X10000L))
          {
            devicePrm.accu -= (0X10000L);
            if (speed > 1)
            {
              speed -=1;
            }
            speedUpdated = true;
          }

          if (speedUpdated)
          {
            if (speed < devicePrm.minSpeed)
            {
              speed = devicePrm.minSpeed;
            }
            devicePrm.speed = speed;
            L6474_ApplySpeed(devicePrm.speed);
          }
        }
      }
      break;
    }
    default:
    {
      break;
    }
  }
  /* Set isr flag */
  isrFlag = false;
}

/* END OF Private function bodies -----------------------------------------------*/






//Main program
void app_main(void)
{
	//VARIABLES
	uint16_t minSpeed = 2000;
	uint16_t maxSpeed = 8000;
	uint16_t newSpeed = 5000;
	uint32_t nbSteps = 100;
	uint32_t abs_position_reg = 0;
//	uint16_t error = 0;
	uint8_t param = 0b10111000;
//	uint8_t *pByteToTransmit = 0;
//	double Tval = 0;
//	double Tmin = 0;

	//CONFIGURAR LOS PINES
	configure_pin();

	L6474_SetDeviceParamsToPredefinedValues();
//	L6474_ApplySpeed(newSpeed);
//	L6474_ConvertPosition(abs_position_reg);

//	L6474_SendCommand(param);
/*	ESTE SEND COMAND SE HACE DESDE L6474_StartMovement();
 * 	DONDE INTERNAMENTE LLAMA A L6474_CmdEnable();
 * 	ESTA FUNCION HACE L6474_SendCommand(L6474_ENABLE);
 * 	DONDE L6474_ENABLE = ((uint8_t) 0xB8),ES NUESTRO VALOR DE
 * 	param = 0b10111000 QUE ES 184 EN DECIMAL
 *
 * 	LOS VALORES COINCIDEN ASI QUE TIENE QUE FUNCIONAR
 *
 * */


	L6474_StartMovement();

    while (1) {
//    	L6474_StartMovement();
//      printf("Comienzo a mover el motor""\n");
//      L6474_StepClockHandler();
//      L6474_ComputeSpeedProfile(nbSteps);
    	vTaskDelay(xDelay);
    }
}
