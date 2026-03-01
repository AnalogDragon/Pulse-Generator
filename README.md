**Pulse Generator**  
  
[中文](./README-zh.md)  

Used for testing and calibrating dose rate instruments such as survey meters or calibrators. STM32G0, KiCad.  
  
This device was built because I purchased a LUDLUM Model 3 and a Model 2360, both of which require calibration of multiple threshold levels, so I made this small tool for that purpose.  
  
Functions:  
- 1. Generates adjustable pulses with amplitude 2 mV–1000 mV, pulse width 50 nS–500 µS, adjustable frequency, and selectable positive/negative polarity, for calibration or performance verification;  
- 2. Multiple operating modes;  
- 3. Output interface changed to MHV connector (previous low-quality BNC connectors could suffer dielectric breakdown);  
- 4. Supports USB power supply and battery power; when USB is connected, the battery is automatically disconnected;  

## Buttons  

![keys](./IMG.jpg)  

**Button function description**  

- A: **Increment** button. When a digit is selected, pressing it increases the value by +1;  
- B: **Select / Mode** button. **Short press** switches the selected digit position; **long press** switches the **mode** (see [Modes](#modes));  
- C: **Decrement** button. When a digit is selected, pressing it decreases the value by −1;  
- D: **Pulse Width** button. Short press to set, long press to change the multiplier. Range: 500 µS–50 nS, minimum step 50 nS, 4 significant digits;  
- E: **Frequency** button. Short press to set, long press to change the multiplier. Range: 5 MHz–0.1 Hz, 3 or 4 significant digits;  
- F: **Voltage** button. Short press to set, long press to switch between **output pulse voltage** and **input high-voltage value**;  
- G: **Output** button. Press to start outputting the configured pulses;  

***Other operations***  
- A + B: Switch output pulse polarity to **positive polarity** pulses (display shows `POS_┌┐_`)  
- C + B: Switch output pulse polarity to **negative polarity** pulses (display shows `NEG‾└┘‾`)  

***High-voltage calibration***  
How to enter: during power-on, hold F + B to enter calibration mode.  

Single-point calibration by default. You can press D / E / F to switch the number of calibration points to 1 / 3 / 5 (updated in V1.2).  
During calibration, adjust the displayed voltage to match the input voltage. Keep the voltage unchanged, then long-press the **Output** button (G) to save the calibration point.  
After the required number of calibration points is saved, calibration ends automatically. If the result displays `DONE`, calibration is successful. If `FAILED RETRY` is displayed, calibration has failed.  

Possible reasons for calibration failure:  
- The voltage difference between calibration points is less than 100 V (for example, point 1 is 100 V and point 2 is 150 V);  
- No input voltage during calibration;  
- The input voltage is not correlated with the displayed voltage;  

## Modes  

Three modes are supported:  

- Normal mode (`NORMAL MODE`): the configured parameters output **continuous** pulses. During output, a rapidly blinking indicator appears at the lower-right corner of the display to show status;  
- Repeat burst mode (`REPEAT BURST`): the configured parameters are output at **100 CPS**;  
- Single burst mode (`SINGEL BURST`): with the configured parameters, each press of the **Output** button generates exactly **100 pulses**;  

## Schematic  

[Schematic](./SCH.pdf)  

## Hardware  

![img](./IMG2.jpg)  

## TEST  
- Normal mode set 1.67CPS(100KCPM)  
![img](./IMG100KPM.jpg)  
  
- Repeat burst mode (6KCPM)  
![img](./IMG100CPS.jpg)  
