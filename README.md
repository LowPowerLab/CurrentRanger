# CurrentRanger
Precision auto-ranging current meter (ammeter)

## [Please see the full guide and latest updates posted here.](https://lowpowerlab.com/guide/currentranger/)

![CurrentRanger](https://lowpowerlab.com/wp-content/uploads/2018/09/DSC_2631-768x579.jpg)

## Quick Highlights
Here are some of the features of this instrument which sets it apart:

* Low noise zero-offset with 3-ranges (1mV output per nA/µA/mA)
* Low input burden voltage, high precision & bandwidth analog outputs
* Increased flexibility and usability with several input and output terminal options
* Auto-ranging capable
* Use standalone with a small OLED display or with a multimeter/oscilloscope
* Ultra fast range switching between any ranges (even nA to mA) without any glitching/bouncing of a mechanical switch
* Low Pass Filter mode – very useful to capture low noise  signals on oscilloscopes
* Unidirectional mode – most used mode in measuring DC currents ranging from [0, 3.3A]
* Bidirectional mode – split supply biasing allows AC currents measurement ranging from [-1.65A, 1.65A]
* LiPo battery powered – long life and extended measurement range
* Auto-power-off
* Full digital control for power & range switching via touch pads
* OLED display option to read output with usable precision
* Datalogging possible via Bluetooth serial module
* SAMD21 Cortex M0+ powered, change firmware to your needs
* Optional buzzer for audible feedback

* Current ranges output:
  - 0-3300 nA/µA/mA (Unidirectional mode)
  - +/- 0-1650 nA/µA/mA (Bidirectional mode)
  - Burden voltage:
  - 17µV/mA
  - 10µV/µA
  - 10µV/nA
* Output offset voltage¹: typically <10µV, max 50µV
* Maximum input voltage differential (see Safety): 33mV
* Accuracy:
  - +/-0.05% (µA, nA ranges)
  - +/-0.1% (mA range)
* Highest resolution (nA range):
  - 100pA (3.5digit meter)
  - 10pA (4.5 digit meter)
  - 1pA (5.5 digit meter)
* Cascaded MAX4239 amplifiers with 100x output gain
  - Bandwidth: >300KHz (-3dB)

![CurrentRanger](https://lowpowerlab.com/wp-content/uploads/2018/09/DSC_2642.jpg)

Sample no-load mA range comparison to µCurrent GOLD:
![mA compare µCurrent](https://lowpowerlab.com/wp-content/uploads/2018/09/DS1Z_QuickPrint11_2.png)

Sample measurement of a [Moteino](https://lowpowerlab.com/guide/moteino/) waking up and transmitting a  RFM69 transceiver packet:

![CurrentRanger](https://lowpowerlab.com/wp-content/uploads/2018/09/CurrentRanger_LPF.gif)
