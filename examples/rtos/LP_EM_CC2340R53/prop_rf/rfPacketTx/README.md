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
The PHY used for TX operations can be configured in SysConfig by opening the
rfPacketTx.syscfg file and navigating to the "Custom" RF Stack. The PHY used
can be selected from the provided drop downs. The example only supports one phy
at a time.

In addition, several register fields must be exposed as defines so that
the example can appropriately set the length field in the packet header. This
can be accomplished by navigating in SysConfig to the Custom RF Stack --> Your
Selected PHY --> Code Export Configuration --> Register Field Defines and setting
it's value to "PBE_GENERIC_RAM_LENCFG_LENPOS,PBE_GENERIC_RAM_LENCFG_NUMLENBITS".
