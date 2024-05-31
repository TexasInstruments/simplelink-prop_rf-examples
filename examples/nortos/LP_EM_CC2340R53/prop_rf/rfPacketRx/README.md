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
The PHY used for TX operations can be configured in SysConfig by opening the
rfPacketTx.syscfg file and navigating to the "Custom" RF Stack. The PHY used
can be selected from the provided drop downs. The example only supports one phy
at a time.

In addition, several register fields must be exposed as defines so that
the example can appropriately set the length field in the packet header. This
can be accomplished by navigating in SysConfig to the Custom RF Stack --> Your
Selected PHY --> Code Export Configuration --> Register Field Defines and setting
it's value to "PBE_GENERIC_RAM_LENCFG_LENPOS,PBE_GENERIC_RAM_LENCFG_NUMLENBITS".
