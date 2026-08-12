# rfEchoTx

---

Example Summary
---------------
In this example you will learn how to setup the radio to do bi-directional
communication. In the Echo TX example the radio to transmit a packet and then
switch over to the receiver mode and wait for its echo. This example is meant
to be used with the Echo RX example or SmartRF Studio. For every packet echo,
CONFIG_PIN_GLED is toggled. The frequency and other RF settings can be modified
using SmartRF Studio.

Board Specific Settings
-----------------------
1. The default frequency is:
    - 2440 MHz on the LP_EM_CC2340R5
In order to change frequency, modify the FREQUENCY define in rfEchoTx.c.

Example Usage
-------------
Run the example. On another board, run the Packet RX example.

The user will require two launchpads, one running rfEchoTx (`Board_1`),
another running rfEchoRx (`Board_2`). Run Board_2 first, followed by
Board_1. Board_1 is set to transmit a packet every second while Board_2 is
set to receive the packet and then turnaround and transmit it. CONFIG_PIN_GLED on Board_1 will toggle when it's able to successfully
transmits a packet, and when it receives its echo. CONFIG_PIN_RLED on Board_2
will toggle when it receives a packet, and then when its able to
re-transmit it (see [figure 1]).

![perfect_echo_ref][figure 1]

If the receiver (`Board_2`) is turned off and the rfEchoTx (`Board_1`) begins
transmitting, it switches over to the receiver mode waiting for an echo that
will never come; in this situation a timeout timer is started and if no
packet is received within 500ms the receiver operation is aborted. This
condition is indicated by CONFIG_PIN_GLED being cleared and CONFIG_PIN_RLED
being set (see [figure 2]).

![missed_first_ref][figure 2]

If the receiver continues to stay turned off then the rfEchoTx example will
alternate between transmitting and aborted receiver operations. CONFIG_PIN_GLED
and CONFIG_PIN_RLED will start alternating, as seen in [figure 3].

![missed_first_few_ref][figure 3]

An error in transmission of a packet, or the reception of its echo, is
indicated by both LEDs going high (see [figure 4]).

![echo_error_ref][figure 4]

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
6. Create packet (with random content)
7. Transmit packet using RCL_CmdGenericTx command by submitting and pending RCL driver call. Once the TX command completed successfully, submit and pend the RCL_CmdGenericRx to recieve the echoed packet.
8. Transmit packet using RCL_CmdGenericTx command with blocking RF driver call, in
   which case CONFIG_PIN_GLED is toggled. The transmit command is followed with
   the receive command, RCL_CmdGenericRx, which runs immediately after the packet
   is transmitted
9. The radio waits for the echo and there are three possibilities. In the callback, CONFIG_PIN_GLED and CONFIG_PIN_RLED will be configured based on the transmit-receive status:
   a. If the echo-back packet is received and matches the transmit packet, Board_PIN_LED1 is toggled and Board_PIN_LED2 is kept off
   b. If no echo-back packet is received, Board_PIN_LED1 is turned off and Board_PIN_LED2 is turned on
   c. If the echo-back packet is received but doesn’t match the transmit packet or an unexpected error occurs, both LEDs are turned on
10. Transmit packets forever by repeating step 6-9

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
[figure 2]:rfEcho_MissedFirstPacket.png "Missed First Packet"
[figure 3]:rfEcho_MissingFirstCouplePackets.png "Missing First Couple of Packets"
[figure 4]:rfEcho_ErrorTxRx.png "Echo Error"