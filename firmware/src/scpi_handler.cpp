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
  ptRscStruct->referenceTemp = RELAY_20mA_Mask;
}

void scpi_select_range_100ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->referenceTemp = RELAY_100mA_Mask;
}

void scpi_select_range_1000ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->referenceTemp = RELAY_1000mA_Mask;
}

void scpi_select_range_10000ma(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  ptRscStruct->referenceTemp = RELAY_10000mA_Mask;
}
