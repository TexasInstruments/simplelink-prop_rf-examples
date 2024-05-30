# rfEchoRx

---

Example Summary
---------------
In this example you will learn how to setup the radio to do bi-directional
communication. The Echo RX example start off by putting the radio in receiver
mode and waits for a packet to be transmitted. When it receives a packet it
switches the radio to transmitter mode and re-transmits the received
packet (echo). This example is meant to be used with the Echo TX example or
SmartRF Studio. For every packet echo, CONFIG_PIN_RLED is toggled. The
frequency and other RF settings can be modified using SmartRF Studio.


Board Specific Settings
-----------------------
1. The default frequency is:
    - 2440 MHz on the LP_EM_CC2340R5
In order to change frequency, modify the FREQUENCY define in rfEchoRx.c.

Example Usage
-------------
The user will require two launchpads, one running rfEchoTx (`Board_1`),
another running rfEchoRx (`Board_2`). Run Board_2 first, followed by
Board_1. Board_1 is set to transmit a packet every second while Board_2 is
set to receive the packet and then turnaround and transmit it after a delay of
100ms. CONFIG_PIN_GLED on Board_1 will toggle when it's able to successfully
transmits a packet, and when it receives its echo. CONFIG_PIN_RLED on Board_2
will toggle when it receives a packet, and once again when it's able to
re-transmit it (see [figure 1]).

![perfect_echo_ref][figure 1]

If there is an issue in receiving a packet then CONFIG_PIN_GLED on Board_2 is
toggled while CONFIG_PIN_RLED is cleared, to indicate an error condition

![echo_error_ref][figure 2]

Application Design Details
--------------------------
This examples consists of a single task and the exported SmartRF Studio radio
settings.

When the task is executed it:

1. Configures the radio for Proprietary mode
2. Get access to the radio via the RCL drivers RCL_open
3. Configure the proprietary RCL_CmdGenericTx and RCL_CmdGenericRx commands.
4. Set the output power using TX_POWER.
5. Set the frequency by editing the command's rfFrequency attribute
6. Run the RCL_CmdGenericRx command and wait for a packet to be transmitted. The
   receive command is chained with a transmit command, RCL_CmdGenericTx, which runs
   once a packet is received
7. When a packet is successfully received CONFIG_PIN_RLED is toggled, the radio
   switches over to the transmit mode and schedules the packet for transmission
   immediately
8. If there is an issue either with the receive or transmit, an error, both
   LEDs are set
9. The devices repeat steps 6-8 forever.

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


[figure 1]:rfEcho_PerfectEcho.png "Perfect Echo"
[figure 2]:rfEcho_ErrorTxRx.png "Echo Error"