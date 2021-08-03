#include "Arduino.h"


#define VERSION_STRING          "v0.1"
#define BAUD_RATE               115200
/* self test verification borders */
#define MAX_DAC_OUT_DIFFERENCE  0.007  /* in decimal percent points IS DEPENDENT FROM ARDUINO REFERENCE PRECISISION  */
#define ARDUINO_MAX_ADC_IN      0x3FF
#define ARDUINO_REF_VOLTAGE     5.000
#define DAC_OUT_READBACK        A1
#define DAC_OUT_FS_READBACK     A2

/* MCP4821 Defines */
#define DAC_MCP4821_CS_Offset   2
#define DAC_MCP4821_CS_Mask     (1 << DAC_MCP4821_CS_Offset)
#define DAC_MCP4821_SHDN_Offset 1
#define DAC_MCP4821_SHDN_Mask   (1 << DAC_MCP4821_SHDN_Offset)
#define DAC_MCP4821_GAIN_1      0x1
#define DAC_MCP4821_GAIN_2      0x0
#define DAC_MCP4821_BITs        12
#define DAC_REAL_FS_OUT         5.000
#define DAC_MAX_COUNT           (pow(2, DAC_MCP4821_BITs) -1)
#define DAC_MAX_VAL             (DAC_MAX_COUNT/1000)
/* tolerance values for upper and lower limit at self test */
#define DAC_MAX_TOL_VAL         DAC_MAX_VAL + (DAC_MAX_VAL * MAX_DAC_OUT_DIFFERENCE)
#define DAC_MIN_TOL_VAL         DAC_MAX_VAL - (DAC_MAX_VAL * MAX_DAC_OUT_DIFFERENCE)
#define DAC_FS_MAX_TOL_VAL      DAC_REAL_FS_OUT + (DAC_REAL_FS_OUT * MAX_DAC_OUT_DIFFERENCE)
#define DAC_FS_MIN_TOL_VAL      DAC_REAL_FS_OUT - (DAC_REAL_FS_OUT * MAX_DAC_OUT_DIFFERENCE)

/* PCF8574 Defines */
#define PCF8574_ADDRESS_A0      0
#define PCF8574_ADDRESS_A1      0
#define PCF8574_ADDRESS_A2      0
#define PCF8574_BASE_ADDRESS    0x20
#define PCF8574_ADDRESS         PCF8574_BASE_ADDRESS | (PCF8574_ADDRESS_A0 | (PCF8574_ADDRESS_A1 << 1) | (PCF8574_ADDRESS_A2 << 2))
#define RELAY_20mA_Offset       0
#define RELAY_20mA_Mask         ~(1 << RELAY_20mA_Offset)
#define RELAY_100mA_Offset      1
#define RELAY_100mA_Mask        ~(1 << RELAY_100mA_Offset)
#define RELAY_1000mA_Offset     2
#define RELAY_1000mA_Mask       ~(1 << RELAY_1000mA_Offset)
#define RELAY_10000mA_Offset    3
#define RELAY_10000mA_Mask      ~(1 << RELAY_10000mA_Offset)

/* ERROR Defines */
#define ERROR_I2C_Offset        0
#define ERROR_I2C_Mask          (1 << ERROR_I2C_Offset)
#define ERROR_DAC_OUT_Offset    1
#define ERROR_DAC_OUT_Mask      (1 << ERROR_DAC_OUT_Offset)
#define ERROR_DAC_OUT_FS_Offset 1
#define ERROR_DAC_OUT_FS_Mask   (1 << ERROR_DAC_OUT_FS_Offset)

/* SCPI Commands */
#define SCPI_IDENTIFY_CMD       F("*IDN?")
#define SCPI_ERROR_READ_CMD     F("*ERR?")
#define SCPI_SELF_TEST_CMD      F("*TEST?")
#define SCPI_RANGE_CMD_REQ      F("RANGE?")
#define SCPI_RANGE_CMD_TREE     F("RANGE")
#define SCPI_RANGE_20mA         F(":20MA")
#define SCPI_RANGE_100mA        F(":100MA")
#define SCPI_RANGE_1000mA       F(":1A")
#define SCPI_RANGE_10000mA      F(":10A")
#define SCPI_TEMP_CMD_TREE      F("TEMP")
#define SCPI_TEMP_INTERNAL      F(":INTernal?")   /* query for internal temprature */
#define SCPI_TEMP_MOSFET        F(":MOSfet?")     /* query for tempretaure at high side mosfet */
#define SCPI_TEMP_REF           F(":REFerence?")  /* query for tempretaure at reference voltage */

/* MCP4821 Command Struct */
typedef struct {
uint16_t value : DAC_MCP4821_BITs;
  unsigned int shdn : 1;
  unsigned int gain : 1;
} MCP4821_COMMAND_T;

/* global ressource structure */
typedef struct {
  unsigned char range;
  uint16_t gusError;
} RSC_STRUCT_T;
