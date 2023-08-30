# rfPacketRx

---

Example Summary
---------------
The Packet RX example illustrates how to do simple packet RX using the
RCL driver. This example is meant to be used with the Packet TX example.

Board Specific Settings
-----------------------
1. The default frequency is:
    - 2440 MHz on the LP_EM_CC2340R5
In order to change frequency, modify the FREQUENCY define in rfPacketRx.c.

Example Usage
-------------
Run the example. On another board, run the Packet TX example.

Application Design Details
--------------------------
This examples consists of a single task and setting up RCL to receive a packet.

When the task is executed it:

1. Initializes and opens RCL with proprietary radio setup. 
2. Sets up the radio for generic receive.
3. Configures the command settings to turn off FS, store received packets,
   maximum packet length, run forever until completion, and end after receiving
   one packet.
4. Sets up runtime callback and triggers for RCL events.
5. Sets up the generic statistics command to store the RX command statistics.
6. Initializes the multi buffer to allow RCL to store the RX packet.
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