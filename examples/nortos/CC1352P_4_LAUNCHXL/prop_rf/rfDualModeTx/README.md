# rfDualModeTx

---

Project Setup using the System Configuration Tool (SysConfig)
-------------------------
The purpose of SysConfig is to provide an easy to use interface for configuring
drivers, RF stacks, and more. The .syscfg file provided with each example
project has been configured and tested for that project. Changes to the .syscfg
file may alter the behavior of the example away from default. Some parameters
configured in SysConfig may require the use of specific APIs or additional
modifications in the application source code. More information can be found in
SysConfig by hovering over a configurable and clicking the question mark (?)
next to it's name.

Example Summary
---------------
In this example you will learn how to setup multi-mode radio driver to send
data to a receiver. The Dual Mode TX example configures a BLE and proprietary
radio mode to transmit the same random packet. The packet is first
transmitted using the proprietary radio driver, then the same packet is then
transmitted  500 ms later using the BLE radio driver. This example is meant
to be used with the Dual Mode RX example or Smart RF Studio. For every proprietary
packet sent CONFIG_PIN_RLED is turned on and CONFIG_PIN_GLED is turned off.
When the same BLE packet is sent, CONFIG_PIN_RLED is turned off and CONFIG_PIN_GLED
is turned on. The frequency and other RF settings can be modified using SmartRF Studio.

Peripherals Exercised
---------------------
* `CONFIG_PIN_RLED` - Blinking indicates a successful transmission of the
proprietary packet (500 ms)
* `CONFIG_PIN_GLED` - Blinking indicates a successful transmission of the
BLE packet (500 ms)

Resources & Jumper Settings
---------------------------
> If you're using an IDE (such as CCS or IAR), please refer to Board.html in your project
directory for resources used and board-specific jumper settings. Otherwise, you can find
Board.html in the directory &lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

Example Usage
-------------
The use of this example will require two launchpads,  one running rfDualModeTx (`Board_1`)
and another running rfDualModeRx (`Board_2`). Run Board_2 first, followed by Board_1.
Board_1 is set to alternate between transmitting a proprietary packet and BLE packet
every 500 ms. CONFIG_PIN_RLED on Board_1 will turn on when a proprietary packet is
transmitted. CONFIG_PIN_RLED on Board_2 will turn on to indicate the packet was successfully
received. On Board_1, CONFIG_PIN_RLED will turn off and CONFIG_PIN_GLED will turn on to
indicate the BLE packet was successfully sent. The behavior on Board_2 will mimic the
LEDs on Board_1 with CONFIG_PIN_RLED turning off and CONFIG_PIN_GLED turning on. This indicates
the BLE packet was successfully received and matches the proprietary packet. The cycle
will then restart with the new packet.

If a packet is missed from the receiver, the current status of the lights will remain. The extended
duration that the LED is on ( > 500 ms) indicates that the receiver is stuck in its previous state
(see [figure 1]). The next proprietary will cause the receiver and transmitter to resync.

![missed_prop_packet_ref][figure 1]

If the BLE packet is received, but the packet is not the same as the proprietary packet both
CONFIG_PIN_GLED and CONFIG_PIN_RLED will turn on (see [figure 2]). This means both a proprietary packet and BLE packet
were received but interference or a missed packet has caused the transmitter and receiver to become
out of sync. If this happens the receiver and transmitter will automatically resync with the
next proprietary packet.

![missed_ble_packet_ref][figure 2]


[figure 1]:rfDualMode_MissedPropPacket.png "Missed Prop Packet"
[figure 2]:rfDualMode_MissedBlePacket.png "Missed BLE Packet"