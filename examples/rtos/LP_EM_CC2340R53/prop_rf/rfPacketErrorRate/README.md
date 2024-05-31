# rfpacketerrorrate

---

Example Summary
---------------
The Packet Error Rate (PER) example showcases different RF transfer modes of the
CC23xx. It combines tasks, events and several peripherals to a platform-
independent application.

Test cases are provided for the following RF modes:

- `MSK 250K` : Minimum shift keying 250 KBits/s
- `BLE` : Bluetooth Low Energy

Peripherals Exercised
---------------------
This example uses the following peripherals:

- 2 GPIOs for buttons
- UART2_0 (115200 Baud) as an alternative display on VT100-compatible
  terminal emulators



  | Peripheral        | Identifier        |
  | ----------------- | ----------------- |
  | Select button     | CONFIG_PIN_BTN1   |
  | Navigate button   | CONFIG_PIN_BTN2   |

Example Usage
-------------

This example requires two boards, each running the PER Test application.
However, the packet format is identical to the default one in SmartRF Studio, so
that any compatible hardware can be used as well.

1. Use the UART and hook it to a VT100-compatible terminal emulator at 115200 Baud.
   Use PuTTY or TeraTerm on Microsoft Windows. After a splash screen, you will see the
   main menu:

        Main Menu
        >Test: Rx
         Mode: MSK 250K
         Freq: 2405
         Pkts: 10
         Interval: No
         Length: 30
         Start...


2. Navigate through the rows with BTN-2/DOWN, modify a value or start the
 selected test with BTN-1/UP.

3. Use a second board with the PER test application as test companion for
   transmissions. Once started, the current progress is shown with these menus
   (TX mode on the left side, RX on the right):

        Sending...      	|  Receiving...
        MSK 250K  2405  	|  MSK 250K  2405
                        	|  Pkts ok   : 10
        Pkts sent: 10   	|  RSSI [dBm]: -26
                        	|  PER  [%]  : 0.00
                        	|  TP[bps]: 20867
                        	|
                        	|  ...finished.
        Push a button...	|  Push a button...

   The receiver prints the amount of successfully received packets (Pkts ok),
   the Signal strength of the current packet (RSSI), the observed packet
   error rate (PER) in percent, and the observed throughput in bits per second.
   Please note that the PER is n/a when sending more packets than configured in
   the receiver.

5. You can always abort a running test case by pushing any button and go back
   to the main menu.

Application Design Details
--------------------------

The PER test application contains of one main task and an event handler to
synchronize the task with buttons. After setting up all resources, the menu task
is started and runs in an endless loop. It shows the menu and invokes test cases
in either `rx.c` or `tx.c`.

PHY Packet Length Configuration
--------------
By default, the application compiles with variable packet length set up for each PHY.
