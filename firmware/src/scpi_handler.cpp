#include "scpi_handler.hpp"
#include "Main.hpp"

extern RSC_STRUCT_T *ptRscStruct;

void scpi_identify_cb(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  // SCPI Command "*IDN?"
  // Returns the instrument's identification string
  interface.println(F("neuschs.,Constant Current Source,#00,v0.1"));
}

void scpi_error_read_cb(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  // Returns the instrument's error status short
  interface.println((uint16_t)ptRscStruct->gusError, HEX);
}

void scpi_internal_temp(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println((double)ptRscStruct->atmelTemp);
}

void scpi_mosfet_temp(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println((double)ptRscStruct->mosfetTemp);
}

void scpi_reference_temp(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println((double)ptRscStruct->referenceTemp);
}

/*
 * RANGE SELECTION
 */
void scpi_select_range_20ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->range = RELAY_20mA_Mask;
}

void scpi_select_range_100ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->range = RELAY_100mA_Mask;
}

void scpi_select_range_1000ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->range = RELAY_1000mA_Mask;
}

void scpi_select_range_10000ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->range = RELAY_10000mA_Mask;
}

void scpi_get_range(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  switch(ptRscStruct->range)
  {
    case (uint8_t)RELAY_20mA_Mask:
    {
      interface.println(F("20MA"));
      break;
    }
    case (uint8_t)RELAY_100mA_Mask:
    {
      interface.println(F("100MA"));
      break;
    }
    case (uint8_t)RELAY_1000mA_Mask:
    {
      interface.println(F("1A"));
      break;
    }
    case (uint8_t)RELAY_10000mA_Mask:
    {
      interface.println(F("10A"));
      break;
    }
  }
}
void scpi_get_voltage_a(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println(ptRscStruct->voltageA, DIGITS_TO_SHOW);
}
void scpi_get_voltage_b(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println(ptRscStruct->voltageB, DIGITS_TO_SHOW);
}
void scpi_get_absolute_voltage(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println(ptRscStruct->absoluteVoltage, DIGITS_TO_SHOW);
}
void scpi_set_current(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  float convertedValue = 0;
  if(parameters.Size() == 1)
  {
    convertedValue = String(parameters[0]).toFloat();
    if(convertedValue != 0)
    {
      convertedValue = constrain(convertedValue, 0.0, ptRscStruct->maximumCurrentInRange);
      ptRscStruct->setCurrent = convertedValue;
    }
    else
    {
      ptRscStruct->gusError |= ERROR_SCPI_PARAM_Mask;
    }
  }
  else
  {
    ptRscStruct->gusError |= ERROR_SCPI_PARAM_Mask;
  }
}
void scpi_get_current(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  interface.println(ptRscStruct->setCurrent);
}
