#include <Arduino.h>
#include "Main.hpp"
#include <SPI.h>
#include "PCF8574.h"
#include "InputDebounce.h"
#include "MCP3202.h"

/* can deactivate for less ram / flash consumption */
#define __SCPI__

#ifdef __SCPI__
/* got more commands than the libraray supports by default */
#include "Vrekrer_scpi_parser.h"
#include "scpi_handler.hpp"
#endif

/*
   Global Variables
*/
/* IO Expander PCF8574 Object */
PCF8574 IO_Expander(PCF8574_ADDRESS);

/* ADC MCP3202 */
MCP3202 _adc;

/* SCPI Parser Global Object */
#ifdef __SCPI__
SCPI_Parser SCPI;
#endif __SCPI__

/* global ressource pointer */
static RSC_STRUCT_T tRscStruct = {0};
RSC_STRUCT_T *ptRscStruct = &tRscStruct;

/* debounced pin objects */
static InputDebounce RangeUpButton;
static InputDebounce RangeDownButton;

int setDACOutput(bool gain, bool enable, uint16_t val)
{
  MCP4821_COMMAND_T tCommandBuffer;
  uint16_t buffer = 0;

  /* fill command struct */
  tCommandBuffer.gain = gain;
  tCommandBuffer.value = val;
  tCommandBuffer.shdn = enable;

  /* copy to uint16_t (aka convert) */
  memcpy(&buffer, &tCommandBuffer, sizeof(MCP4821_COMMAND_T));

  /* byteorder in struct is wrong so switch high byte and low byte */
  buffer = __builtin_bswap16((uint16_t)buffer);

  /* drive chip select low */
  PORTB &= ~DAC_MCP4821_CS_Mask;

  /* transfer the two bytes via spi */
  SPI.transfer(&buffer, sizeof(MCP4821_COMMAND_T));

  /* drive chip select high */
  PORTB |= DAC_MCP4821_CS_Mask;

}



/*  my own map with double precisision */
float _map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef __SCPI__
void scpi_self_test_cb(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  // executes self test
  uint16_t usDACOutMeas = 0;
  double fRealVoltage = 0;

  /* activate dac */
  PORTB |= DAC_MCP4821_SHDN_Mask;

  interface.println(F("Test DAC Output:"));
  setDACOutput(DAC_MCP4821_GAIN_2, true, DAC_MAX_COUNT);
  interface.print(F("Set DAC Out (V):\t")); interface.println(DAC_MAX_VAL);
  usDACOutMeas = analogRead(DAC_OUT_READBACK);
  interface.print(F("Read back (V):\t"));
  fRealVoltage = _map(usDACOutMeas, 0, ARDUINO_MAX_ADC_IN, 0.0, ARDUINO_REF_VOLTAGE);

  interface.println(fRealVoltage);
  interface.print(F("Upper Limit: ")); interface.print(DAC_MAX_TOL_VAL); interface.print(F(" / Lower Limit:")); interface.println(DAC_MIN_TOL_VAL);
  if ((fRealVoltage <= DAC_MAX_TOL_VAL) && (fRealVoltage >= DAC_MIN_TOL_VAL))
  {
    ptRscStruct->gusError &= ~ERROR_DAC_OUT_Mask;
    interface.println(F("Test Pass"));
  }
  else
  {
    ptRscStruct->gusError |= ERROR_DAC_OUT_Mask;
    interface.println(F("Test Fail"));
  }
  interface.println(F(""));
  interface.println(F("Test DAC OP-Amp Output (DAC-Out * 1.22):"));
  interface.print(F("Set DAC Out (V):\t")); interface.println(DAC_MAX_VAL);
  usDACOutMeas = analogRead(DAC_OUT_FS_READBACK);
  interface.print(F("Read back (V):\t"));
  fRealVoltage = _map(usDACOutMeas, 0, ARDUINO_MAX_ADC_IN, 0.0, ARDUINO_REF_VOLTAGE);

  interface.println(fRealVoltage);
  interface.print(F("Upper Limit: ")); interface.print(DAC_FS_MAX_TOL_VAL); interface.print(F(" / Lower Limit:")); interface.println(DAC_FS_MIN_TOL_VAL);
  if ((fRealVoltage <= DAC_FS_MAX_TOL_VAL) && (fRealVoltage >= DAC_FS_MIN_TOL_VAL))
  {
    ptRscStruct->gusError &= ~ERROR_DAC_OUT_FS_Mask;
    interface.println(F("Test Pass"));
  }
  else
  {
    ptRscStruct->gusError |= ERROR_DAC_OUT_FS_Mask;
    interface.println("Test Fail");
  }

  PORTB &= ~DAC_MCP4821_SHDN_Mask;
}
#endif

void range_up_down_btn_released_cb(uint8_t pinIn)
{
  if(pinIn == BUTTON_RANGE_UP_PIN)
  {

  }
  else if(pinIn == BUTTON_RANGE_DOWN_PIN)
  {

  }
}

void setup()
{
  /* debounce callback registration */
  RangeUpButton.registerCallbacks(NULL, range_up_down_btn_released_cb, NULL, NULL);
  RangeDownButton.registerCallbacks(NULL, range_up_down_btn_released_cb, NULL, NULL);

  /* debounce pin initialization */
  RangeUpButton.setup(BUTTON_RANGE_UP_PIN, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
  RangeDownButton.setup(BUTTON_RANGE_DOWN_PIN, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);

  /* initialize mcp3202 */
  _adc = MCP3202(ADC_MCP3202_CS);
  _adc.begin();

  /* set default range to 20ma */
  ptRscStruct->range = RELAY_20mA_Mask;

  /* initialization routines for DAC */
  /* set DAC_MCP4821_CS pin mode to output */
  DDRB |= DAC_MCP4821_CS_Mask | DAC_MCP4821_SHDN_Mask;

  /* set chip select for DAC to high */
  /* set DAC SHDN (Shutdown) to low to deactivate the DAC output */
  PORTB |= DAC_MCP4821_CS_Mask;
  PORTB &= ~DAC_MCP4821_SHDN_Mask;

  /* use external analog reference voltage (REF02CP) */
  analogReference(EXTERNAL);

  /* Initialization of serial connection to host */
  Serial.begin(BAUD_RATE);

  /* spi setup */
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

#ifdef __SCPI__
  /* scpi request setup */
  SCPI.RegisterCommand(SCPI_IDENTIFY_CMD, &scpi_identify_cb);
  SCPI.RegisterCommand(SCPI_ERROR_READ_CMD, &scpi_error_read_cb);
  SCPI.RegisterCommand(SCPI_SELF_TEST_CMD, &scpi_self_test_cb);
  /* range swichting command registrations */
  SCPI.RegisterCommand(SCPI_RANGE_CMD_REQ, &scpi_get_range);
  SCPI.RegisterCommand(SCPI_RANGE_20mA, &scpi_select_range_20ma);
  SCPI.RegisterCommand(SCPI_RANGE_100mA, &scpi_select_range_100ma);
  SCPI.RegisterCommand(SCPI_RANGE_1000mA, &scpi_select_range_1000ma);
  SCPI.RegisterCommand(SCPI_RANGE_10000mA, &scpi_select_range_10000ma);
  /* temperature readouts */
  SCPI.RegisterCommand(SCPI_TEMP_INTERNAL, &scpi_internal_temp);
  SCPI.RegisterCommand(SCPI_TEMP_MOSFET, &scpi_mosfet_temp);
  SCPI.RegisterCommand(SCPI_TEMP_REF, &scpi_reference_temp);
  /* voltage reading */
  SCPI.RegisterCommand(SCPI_VOLT_SENSE_A, &scpi_get_voltage_a);
  SCPI.RegisterCommand(SCPI_VOLT_SENSE_B, &scpi_get_voltage_b);
  SCPI.RegisterCommand(SCPI_VOLT_ABSOLUTE, &scpi_get_absolute_voltage);
  /* current setting */
  SCPI.RegisterCommand(SCPI_CURR_SET, &scpi_set_current);
  SCPI.RegisterCommand(SCPI_CURR_GET, &scpi_get_current);
#endif

  /* start communication with PCF8574 */
  if (!IO_Expander.begin())
  {
    ptRscStruct->gusError |= ERROR_I2C_Mask;
  }
  else
  {
    if (!IO_Expander.isConnected())
    {
      ptRscStruct->gusError |= ERROR_I2C_Mask;
    }
    else
    {
      /* reset (seto to low) all outputs */
      IO_Expander.write8(0xFF);
    }
  }

  /* execute self test */
  //self_test();
}

void read_temperature()
{
  /* temperature of reference voltage */
  double tempRaw = 0;
  /* read value of adc */
  uint16_t usRefTempRaw = analogRead(TEMP_REFERENCE_VOLT_PIN);
  /* map analog reading to reference voltage in millivolt */
  tempRaw = _map(usRefTempRaw, 0, ARDUINO_MAX_ADC_IN, 0, ARDUINO_REF_VOLTAGE*1000);
  /* calculate real voltage at ref (divide by real op amp gain) */
  tempRaw = tempRaw / ((TEMP_R_FEEDBACK / TEMP_R_IN) + 1);
  /* calculate tempreature in degree above TEMP_REF_BASE_TEMP° or below */
  tempRaw = ((tempRaw - TEMP_REF_BASE_MVOLT) / TEMP_REF_TEMP_COEF);
  /* offeset value by TEMP_REF_BASE_TEMP */
  tempRaw = tempRaw + TEMP_REF_BASE_TEMP;

  ptRscStruct->referenceTemp = tempRaw;
}


void read_input_voltage()
{
  static uint8_t counter = 0;
  static float values_a;
  static float values_b;
  static float values_abs;

  float fAbsoluteValue;
  float usRawChannelA = _adc.readChannel(ADC_CHANNEL_SENSE_A);
  float usRawChannelB = _adc.readChannel(ADC_CHANNEL_SENSE_B);

  /* convert 0 .. ADC_MAX_COUNT to 0 .. ADC_MCP3202_REF_V */
  usRawChannelA = _map(usRawChannelA, (float)0, (float)ADC_MAX_COUNT, (float)0, (float)ADC_MCP3202_REF_V);
  usRawChannelB = _map(usRawChannelB, (float)0, (float)ADC_MAX_COUNT, (float)0, (float)ADC_MCP3202_REF_V);

  /* muliply by the attentuation of the input stage VOLT_DIV_ATTENTUATION */
  usRawChannelA = usRawChannelA * VOLT_DIV_ATTENTUATION;
  usRawChannelB = usRawChannelB * VOLT_DIV_ATTENTUATION;

  fAbsoluteValue = usRawChannelA > usRawChannelB ? usRawChannelA - usRawChannelB : usRawChannelB - usRawChannelA;


  /* save to global ressource pointer */
  //ptRscStruct->voltageA = usRawChannelA;
  //ptRscStruct->voltageB = usRawChannelB;
  if(counter < AVERAGING_SAMPLES)
  {
    values_a = values_a + usRawChannelA;
    values_b = values_b + usRawChannelB;
    values_abs = values_abs + fAbsoluteValue;
    counter++;
    ptRscStruct->measurementDone = false;
  }
  else
  {
    ptRscStruct->voltageA = values_a / (float)AVERAGING_SAMPLES;
    values_a = 0;
    ptRscStruct->voltageB = values_b / (float)AVERAGING_SAMPLES;
    values_b = 0;
    ptRscStruct->absoluteVoltage = values_abs / (float)AVERAGING_SAMPLES;
    values_abs = 0;
    counter = 0;
    ptRscStruct->measurementDone = true;
  }

  /* get absolute value of substraction of input A and B */


  /* save absolute value to ressource pointer */
  //ptRscStruct->absoluteVoltage = fAbsoluteValue;
}

void update_ressources()
{
  switch(ptRscStruct->range)
  {
    case (uint8_t)RELAY_20mA_Mask:
      ptRscStruct->maximumCurrentInRange = MAX_CURR_20MA;
      break;
    case (uint8_t)RELAY_100mA_Mask:
      ptRscStruct->maximumCurrentInRange = MAX_CURR_100MA;
      break;
    case (uint8_t)RELAY_1000mA_Mask:
      ptRscStruct->maximumCurrentInRange = MAX_CURR_1000MA;
      break;
    case (uint8_t)RELAY_10000mA_Mask:
      ptRscStruct->maximumCurrentInRange = MAX_CURR_10000MA;
      break;
  }
}

void update_outputs()
{
  if(ptRscStruct->measurementDone)
  {
    /* update relay state via pcf8574 */
    if(IO_Expander.valueOut() != ptRscStruct->range)
    {
      /* reset output current to 0 in case of range change */
      ptRscStruct->setCurrent = 0;
      setDACOutput(DAC_MCP4821_GAIN_2, true, _map(ptRscStruct->setCurrent, 0.0, ptRscStruct->maximumCurrentInRange, 0.0, (float)DAC_MAX_COUNT));
      IO_Expander.write8(ptRscStruct->range);
    }
    /* update dac output */
    setDACOutput(DAC_MCP4821_GAIN_2, true, _map(ptRscStruct->setCurrent, 0.0, ptRscStruct->maximumCurrentInRange, 0.0, (float)DAC_MAX_COUNT));
  }
}

void loop()
{
  static int temp = 0;

#ifdef __SCPI__
  if(ptRscStruct->measurementDone)
  {
    /* process scpi commands */
    SCPI.ProcessInput(Serial, "\n");
  }
#endif

  /* update ressource pointer */
  update_ressources();

  /* process temperature reading from reference voltage */
  read_temperature();

  /* read analog voltage A & B and convert raw to voltage */
  read_input_voltage();

  /* process debouncing of range up/down buttons */
  RangeUpButton.process(millis());
  RangeDownButton.process(millis());


  /* actiavte dac output */
  PORTB |= DAC_MCP4821_SHDN_Mask;

  /* update outputs */
  update_outputs();

}
