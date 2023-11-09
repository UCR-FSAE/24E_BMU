**A BRIEF SUMMARY**

The BQ79600-Q1 is designed to serve as a communication interface in automotive systems. It supports both Serial Peripheral Interface (SPI) and Universal Asynchronous Receiver/Transmitter (UART) protocols. This dual compatibility makes it a versatile choice for facilitating data transmission between various electronic components in a vehicle. 

The BQ79600-Q1 is a communication (bridge) IC designed to interface between a microcontroller (MCU) and TI battery monitoring ICs. The information from the MCU is translated by the device to signals recognized by TI’s battery management daisy chain protocol, and transmitted out. And signals from the daisy chain are decoded to bit stream and then sent back to the microcontroller unit.

DOCOUMENTATION LINK: https://www.ti.com/lit/ds/symlink/bq79600-q1.pdf?HQS=dis-mous-null-mousermode-dsf-pf-null-wwe&ts=1683319556475&ref_url=https%3A%2F%2Fwww.ti.com%2Fgeneral%2Fdocs%2Fsuppproductinfo.tsp%3FdistId%3D26%26gotoUrl%3Dhttps%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Fbq79600-q1

-------------------------------------------------------------------------------------------------------------

**NEW MEMBERS: START HERE**

_There are three major sections you must understand to begin the BMU (Battery Management Unit) for the accelerator. To begin, open up the documentation and begin reading the three main sections of the BQ79600-Q1._

1. Open documentation. Scroll to PAGE 10 and read the information about the power states. Or, read the summary below underneath this section. 

2. Go to the Time Requirements section of the SOP below. Then, scroll to PAGE 6 and understand the timing requirements.

3. Go to the Register Maps section of the SOP below. Then, scroll to PAGE 37 and briefly understand the primary pins involved. They will be listed in the README TLDR.

-------------------------------------------------------------------------------------------------------------

**POWER STATES**

_Here are all the possible states the BQ79600-Q1 can be in. Refer to PAGE 10 for more information about the Power States._

COMPLETE OFF: All circuits are powered off when the voltage at the BAT pin is less than the minimum required voltage (VBAT min).

SHUTDOWN: This is the lowest power mode. Without VIO (Voltage Input/Output), the device can only transition to the "VALIDATE" mode if SNIFFER value is present.

SLEEP: A low-power mode where transitioning to the "ACTIVE" mode is much faster compared to when in "SHUTDOWN" mode.

ACTIVE: This is the full-power mode where the device can communicate between the MCU (Microcontroller Unit) and the daisy chain of connected devices.

VALIDATE: This state is used to validate if there is a real fault tone from stack devices. If a fault tone is validated, it cannot transition back to ACTIVE mode. If all tests pass, it can proceed to ACTIVE mode. The device goes back to "SHUTDOWN" if a specific timeout occurs.

<img width="521" alt="Screenshot 2023-11-09 at 14 53 05" src="https://github.com/UCR-FSAE/24E_BMU/assets/117234817/df336d67-37d7-438a-8555-baaca77cdb7e">

-------------------------------------------------------------------------------------------------------------

**TIME REQUIREMENTS**

_We look at the time required for the signals to send. MIN = “Minimum” | TYP = “Typical” | MAX = “Maximum.” Please refer to the bottom of PAGE 6 of the documentation. The importance of the Time Requirements is to prevent corruption of signals. For more explanation, please ask Alex (CS Lead)._

Further Explanation: To explain simply, the goal of the BMU is that we want to avoid sending and receiving signals at the same time. If two signals are sent down the BUS, the signal becomes corrupt and therefore causing problems. Hence, time requirements are introduced: introducing when to send signals so they do not collide and allowing information to travel untouched. 

-------------------------------------------------------------------------------------------------------------

**REGISTER MAPS**

_We look at the registers of the BQ79600-Q1 and its purposes. Refer to PAGE 37. For the register and their definitions._

<img width="468" alt="Screenshot 2023-11-09 at 14 54 18" src="https://github.com/UCR-FSAE/24E_BMU/assets/117234817/f5ecd912-05de-41d0-a513-3dc147aa6c36">

_Important Pins: CONTROL1, FAULT_REG, DIE_ID#... It is recommended to read all registers and their purpose to gain a general understanding._

