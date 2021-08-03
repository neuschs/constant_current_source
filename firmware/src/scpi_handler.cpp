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
}y
