#include "Arduino.h"


#define VERSION_STRING          "v0.1"
#define BAUD_RATE               115200

/* set debounce delay to 20ms */
#define BUTTON_DEBOUNCE_DELAY   20

/* range up & range down button pins */
#define BUTTON_RANGE_UP_PIN     4
#define BUTTON_RANGE_DOWN_PIN   5

/* self test verification borders */
#define MAX_DAC_OUT_DIFFERENCE  0.007  /* in decimal percent points 0.7%  */
#define ARDUINO_MAX_ADC_IN      0x3FF
#define ARDUINO_REF_VOLTAGE     5.000
#define TEMP_R_FEEDBACK         388.0
#define TEMP_R_IN               99.0
#define TEMP_REF_TEMP_COEF      1.96  /* 1.96mV / °C */
#define TEMP_REF_BASE_TEMP      25.0  /* °C */
#define TEMP_REF_BASE_MVOLT     550   /* mV at TEMP_REF_BASE_TEMP */
#define DIGITS_TO_SHOW          4
#define AVERAGING_SAMPLES       50  /* standard averaging samples */

/* maximum current ratings per range in mA */
#define MAX_CURR_20MA           ((float)20.0)
#define MAX_CURR_100MA          ((float)100.0)
#define MAX_CURR_1000MA         ((float)1000.0)
#define MAX_CURR_10000MA        ((float)10000.0)

/* self test adc inputs */
#define DAC_OUT_READBACK        A1
#define DAC_OUT_FS_READBACK     A2
#define TEMP_REFERENCE_VOLT_PIN A3

/* MCP3202 Defines */
#define ADC_MCP3202_CS          8
#define ADC_MCP3202_REF_V       5.000 /* from ref02cp */
#define ADC_CHANNEL_SENSE_A     0
#define ADC_CHANNEL_SENSE_B     1
#define VOLT_DIV_ATTENTUATION   8
#define DAC_MCP3202_BITs        12
#define ADC_MAX_COUNT           (pow(2, DAC_MCP3202_BITs) -1)

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
#define ERROR_DAC_OUT_FS_Offset 2
#define ERROR_DAC_OUT_FS_Mask   (1 << ERROR_DAC_OUT_FS_Offset)
#define ERROR_SCPI_PARAM_Offset 3
#define ERROR_SCPI_PARAM_Mask   (1 << ERROR_SCPI_PARAM_Offset)


/* SCPI Commands */
#define SCPI_IDENTIFY_CMD       F("*IDN?")
#define SCPI_ERROR_READ_CMD     F("*ERR?")
#define SCPI_SELF_TEST_CMD      F("*TEST?")
#define SCPI_RANGE_CMD_REQ      F("*RANGE?")
#define SCPI_RANGE_20mA         F("RANGE:20MA")
#define SCPI_RANGE_100mA        F("RANGE:100MA")
#define SCPI_RANGE_1000mA       F("RANGE:1A")
#define SCPI_RANGE_10000mA      F("RANGE:10A")
#define SCPI_TEMP_INTERNAL      F("TEMP:INTernal?")   /* query for internal temprature */
#define SCPI_TEMP_MOSFET        F("TEMP:MOSfet?")     /* query for tempretaure at high side mosfet */
#define SCPI_TEMP_REF           F("TEMP:REFerence?")  /* query for tempretaure at reference voltage */
#define SCPI_VOLT_SENSE_A       F("VOLT:A?")
#define SCPI_VOLT_SENSE_B       F("VOLT:B?")
#define SCPI_VOLT_ABSOLUTE      F("VOLT:ABS?")
#define SCPI_CURR_SET           F("CURR:SET")
#define SCPI_CURR_GET           F("CURR:GET?")

/* MCP4821 Command Struct */
typedef struct {
uint16_t value : DAC_MCP4821_BITs;
  unsigned int shdn : 1;
  unsigned int gain : 1;
} MCP4821_COMMAND_T;

/* global ressource structure */
typedef struct {
  uint8_t range;        /* active range consisting of RELAY_10000mA_Mask etc... */
  uint16_t gusError;    /* global error "short" */
  double mosfetTemp;    /* tempreature at FET */
  double referenceTemp;
  double atmelTemp;
  float voltageA;
  float voltageB;
  float absoluteVoltage;
  float setCurrent;
  float maximumCurrentInRange;
  bool measurementDone;
} RSC_STRUCT_T;
