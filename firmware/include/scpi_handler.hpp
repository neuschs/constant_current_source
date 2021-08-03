#include "Vrekrer_scpi_parser.h"
#include "Arduino.h"
void scpi_identify_cb(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_error_read_cb(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_self_test_cb(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_internal_temp(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_mosfet_temp(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_reference_temp(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_select_range_20ma(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_select_range_100ma(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_select_range_1000ma(SCPI_C commands, SCPI_P parameters, Stream& interface);
void scpi_select_range_10000ma(SCPI_C commands, SCPI_P parameters, Stream& interface);
