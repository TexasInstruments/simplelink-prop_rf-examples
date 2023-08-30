# rfPacketTx

---

Example Summary
---------------
The Packet TX example illustrates how to do simple packet transmission using
the RCL driver. This example is meant to be used with the Packet RX
example.

Board Specific Settings
-----------------------
1. The default frequency is:
    - 2440 MHz on the LP_EM_CC2340R5
In order to change frequency, modify the FREQUENCY define in rfPacketTx.c.

Example Usage
-------------
Run the example. On another board, run the Packet RX example.

Application Design Details
--------------------------
This examples consists of a single task and setting up RCL to transmit a packet.

When the task is executed it:

1. Initializes and opens RCL with proprietary radio setup. 
2. Sets up the radio for generic transmit.
3. Configures the command settings to turn off FS.
4. Creates the TX packet.
5. Sets up runtime callback and triggers for RCL events.
6. Puts packet in RCL TX buffer.
7. Submits the command.
8. Pends on command completion.

Switching PHYs
--------------
Add the respective predefine symbol by navigating in CCS to Project Properties ->
Build -> Arm Compiler -> Predefined Symbols. After nagivating to Predefined Symbols,
use the appropriate symbol as described below to define the PHY used.
	- By default, Generic BLE 1M PHY is used.
	- To use 250KBPS MSK, define predefine symbol **USE_250KBPS_MSK**.
	- To use 500KBPS MSK, define predefine symbol **USE_250KBPS_MSK_FEC**.
	
PHY Packet Length Configuration
--------------
By default, the application compiles with variable packet length set up for each PHY. However, **variable
packet length is not supported for the 500KBPS MSK PHY** in this application. 500KBPS MSK is always set up for fixed packet length.
Variable packet length is set up for the PHY when the VARIABLE_LENGTH_SETUP symbol is defined.
For fixed packet length set up,  navigate in CCS to Project Properties ->
Build -> Arm Compiler -> Predefined Symbols and change the VARIABLE_LENGTH_SETUP symbol to **FIXED_LENGTH_SETUP**.