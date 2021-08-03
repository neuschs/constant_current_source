#include <Arduino.h>
#include "Main.hpp"
#include <SPI.h>
#include "Vrekrer_scpi_parser.h"
#include "PCF8574.h"
#include "scpi_handler.hpp"

/*
   Global Variables
*/
/* IO Expander PCF8574 Object */
PCF8574 IO_Expander(PCF8574_ADDRESS);

/* SCPI Parser Global Object */
SCPI_Parser SCPI;

/* */

/* global ressource pointer */
static RSC_STRUCT_T tRscStruct = {0};
RSC_STRUCT_T *ptRscStruct = &tRscStruct;



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


void scpi_self_test_cb(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  // executes self test
  uint16_t usDACOutMeas = 0;
  double fRealVoltage = 0;

  /* activate dac */
  PORTB |= DAC_MCP4821_SHDN_Mask;

  interface.println("Test DAC Output:");
  setDACOutput(DAC_MCP4821_GAIN_2, true, DAC_MAX_COUNT);
  interface.print("Set DAC Out (V):\t"); interface.println(DAC_MAX_VAL);
  usDACOutMeas = analogRead(DAC_OUT_READBACK);
  interface.print("Read back (V):\t");
  fRealVoltage = _map(usDACOutMeas, 0, ARDUINO_MAX_ADC_IN, 0.0, ARDUINO_REF_VOLTAGE);

  interface.println(fRealVoltage);
  interface.print("Upper Limit: "); interface.print(DAC_MAX_TOL_VAL); interface.print(" / Lower Limit:"); interface.println(DAC_MIN_TOL_VAL);
  if ((fRealVoltage <= DAC_MAX_TOL_VAL) && (fRealVoltage >= DAC_MIN_TOL_VAL))
  {
    ptRscStruct->gusError &= ~ERROR_DAC_OUT_Mask;
    interface.println("Test Pass");
  }
  else
  {
    ptRscStruct->gusError |= ERROR_DAC_OUT_Mask;
    interface.println("Test Fail");
  }
  interface.println("");
  interface.println("Test DAC OP-Amp Output (DAC-Out * 1.22):");
  interface.print("Set DAC Out (V):\t"); interface.println(DAC_MAX_VAL);
  usDACOutMeas = analogRead(DAC_OUT_FS_READBACK);
  interface.print("Read back (V):\t");
  fRealVoltage = _map(usDACOutMeas, 0, ARDUINO_MAX_ADC_IN, 0.0, ARDUINO_REF_VOLTAGE);

  interface.println(fRealVoltage);
  interface.print("Upper Limit: "); interface.print(DAC_FS_MAX_TOL_VAL); interface.print(" / Lower Limit:"); interface.println(DAC_FS_MIN_TOL_VAL);
  if ((fRealVoltage <= DAC_FS_MAX_TOL_VAL) && (fRealVoltage >= DAC_FS_MIN_TOL_VAL))
  {
    ptRscStruct->gusError &= ~ERROR_DAC_OUT_FS_Mask;
    interface.println("Test Pass");
  }
  else
  {
    ptRscStruct->gusError |= ERROR_DAC_OUT_FS_Mask;
    interface.println("Test Fail");
  }

  PORTB &= ~DAC_MCP4821_SHDN_Mask;
}



void setup()
{
  /* initialization routines for DAC */
  /* set DAC_MCP4821_CS pin mode to output */
  DDRB |= DAC_MCP4821_CS_Mask | DAC_MCP4821_SHDN_Mask;
  /* set chip select for DAC to high
     set DAC SHDN (Shutdown) to low to deactivate the DAC output */
  PORTB |= DAC_MCP4821_CS_Mask;
  PORTB &= ~DAC_MCP4821_SHDN_Mask;

  /* use external analog reference voltage (REF02CP) */
  analogReference(EXTERNAL);

  /* Initialization of serial connection to host */
  Serial.begin(BAUD_RATE);

  /* spi setup */
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  /* scpi request setup */
  SCPI.RegisterCommand(SCPI_IDENTIFY_CMD, &scpi_identify_cb);
  SCPI.RegisterCommand(SCPI_ERROR_READ_CMD, &scpi_error_read_cb);
  SCPI.RegisterCommand(SCPI_SELF_TEST_CMD, &scpi_self_test_cb);
  SCPI.SetCommandTreeBase(SCPI_RANGE_CMD_TREE);
  //SCPI.RegisterCommand(SCPI_RANGE_20mA, &select_range_20ma);
  //SCPI.RegisterCommand(SCPI_RANGE_100mA, &select_range_100ma);
  //SCPI.RegisterCommand(SCPI_RANGE_1000mA, &select_range_1000ma);
  //SCPI.RegisterCommand(SCPI_RANGE_10000mA, &select_range_10000ma);
  SCPI.SetCommandTreeBase(SCPI_TEMP_CMD_TREE);
  SCPI.RegisterCommand(SCPI_TEMP_INTERNAL, &scpi_internal_temp);
  SCPI.RegisterCommand(SCPI_TEMP_MOSFET, &scpi_mosfet_temp);
  SCPI.RegisterCommand(SCPI_TEMP_REF, &scpi_reference_temp);


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

void loop()
{
  static int temp = 0;

  /* process scpi commands */
  SCPI.ProcessInput(Serial, "\n");

  if (temp == 0)
  {
    /* actiavte dac output */
    PORTB |= DAC_MCP4821_SHDN_Mask;
    //setDACOutput(DAC_MCP4821_GAIN_2, true, 0x000);
    //temp = 0;
    //IO_Expander.write(RELAY_20mA, 0);
    //setDACOutput(DAC_MCP4821_GAIN_2, true, 2047);

  }
}
