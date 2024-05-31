# rfcarrierwave

---

Example Summary
---------------
The carrier wave (CW) example sends a continuous carrier wave or pseudo-random
modulated signal on a fixed frequency.

Board Specific Settings
-----------------------
1. The default frequency is:
    - 2440 MHz on the LP_EM_CC2340R5
In order to change frequency, modify the FREQUENCY define in rfCarrierWave.c.

Example Usage
-------------
1. Run the continuous receive using SmartRf Studio.
2. Run the example.

Application Design Details
--------------------------
This examples consists of a single task and setting up RCL to transmit a continuous
wave.

To switch between carrier wave (1) and modulated signal (0) set the following
in the code (CW is set as default):

    rclPacketTxCmdGenericTxTest.config.sendCw = 1; // CW
    rclPacketTxCmdGenericTxTest.config.sendCw = 0; // modulated signal

When the task is executed it:

1. Initializes and opens RCL with proprietary radio setup.
2. Sets up the radio for generic transmit test.
3. Explicitly configures CW (1) or Modulated (0). Whiten mode is set to default
   whitening. FS is turned off.
4. Sets up runtime callback and triggers for RCL events.
4. Submits command.
5. Pends on command completion.

Switching PHYs
--------------
The PHY used for TX operations can be configured in SysConfig by opening the
rfPacketTx.syscfg file and navigating to the "Custom" RF Stack. The PHY used
can be selected from the provided drop downs. The example only supports one phy
at a time.
