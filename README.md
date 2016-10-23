# ISAscale_gauge_driver
Custom electric vehicle dash display using an ISAScale high-precision CANBus shunt

This program reads CAN data coming from an ISAscale CAN shunt and presents it on a 4DSystems LCD screen. It
  will also--using a SparkFun mini-fet shield and an auxiliary PnP transistor--create pulses to drive a normal
  Tachometer to show current AMP draw and variable resistance (PWN to ground) to show SOC on a conventional
  fuel gauge. It has two auxiliary on/off outputs (again to ground) to light some indicators. One for low battery
  and the second as a warning light in case of voltage mismatch between the various segments of the battery pack.
  Our poor-man's BMS.

  It's meant to run on an EVTVDue CAN microcontroller although it should work on other arduinos with an appropriate
  CAN shield.

  Note that EVTVDue CAN microcontroller ONLY provides CAN0 and does not provide a CAN1 output.   It also ONLY supports
  the NATIVE USB port and so this code will not work on the programming port as written.
  
  copyright 2016 Rick Beebe
  portions copyright 2015 Jack Rickard, Collin Kidder
    
  This sketch uses the EVTVDue Microcontroller and is compatible with Arduino Due  Use BOARD select Arduino Due (Native USB port).

  5/24/2016 Rick Beebe
