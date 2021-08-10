# Constant Current Source
"Precise" Constant Current Source based on TI XTR110AG

# Main Components 

## Revision 0.1
### Arduino Nano
Simple Arduino for processing of the SCPI commands and coordinating the range switching.
### MCP3202
Two channel SPI 12 Bit ADC with external reference voltage.
### REF02
Reference voltage source used for internal "calibration" and as reference for arduino ADC and MCP3202

# Measurments
## Revision 0.1
### Stability over 2 Hours
Measured against reference voltage source REF02

#### Parameters:
- Duration: 2 Hours
- Frequency: 1s (approx.)
- Attentuatuation before LM324N: 8 (rough setting)
- LSB at ADC: approx 9,765mV (with Attentuation of 8)
- Internal Averaging Samples: 50

### Voltage A (Sense A):
| Value | Measurment |
| ------------------| ---------- |
|Standard Deviation| 1,456 mV    |
|Maximum Value|      5.216500 V  |
|Minimum Value|      5.207500 V  |
|Mean Value|         5.213059 V  |
|25% of Counts|      5.212000 V  |
|50% of Coutns|      5.213200 V  |
|75% of Counts|      5.214200 V  |

### Boxplot and Histogram
![alt_text](https://github.com/neuschs/constant_current_source/blob/be0c6ac58962ab518f57ccae12046b3fc8509949/measurements/revision_0.1/histogram_boxplot_2hours_voltage_a.png)

### Voltage B (Sense B):
| Value | Measurment |
| ------------------| ----------  |
|Standard Deviation| 411 uV       |
|Maximum Value:    |  5.150300 V  |
|Minimum Value:    |  5.146000 V  |
|Mean Value:       |  5.147937 V  |
|25% of Counts:    |  5.147700 V  |
|50% of Coutns:    |  5.147900 V  |
|75% of Counts:    |  5.148100 V  |

### Boxplot and Histogram
![alt_text](https://github.com/neuschs/constant_current_source/blob/be0c6ac58962ab518f57ccae12046b3fc8509949/measurements/revision_0.1/histogram_boxplot_2hours_voltage_b.png)

### Overview over 2 Hours
![alt text](https://github.com/neuschs/constant_current_source/blob/be0c6ac58962ab518f57ccae12046b3fc8509949/measurements/revision_0.1/stability_over_2hours.png)

### Result
> In some mysterious way, Sense A voltage measrument is way more bad than Sense B. Next measurment will be done over 24h with better shielding against the USB connection which is directly over the input attentuation voltage divider.


# Credits
Used Librarys:

[vrekrer/Vrekrer SCPI parser@0.4.1](https://github.com/Vrekrer/Vrekrer_scpi_parser)

[robtillaart/PCF8574@0.3.2](https://github.com/RobTillaart/PCF8574)

[mokolea/InputDebounce@1.6.0](https://github.com/Mokolea/InputDebounce)

[ouviksaha97/MCP3202@1.0.2](https://github.com/souviksaha97/MCP3202)

