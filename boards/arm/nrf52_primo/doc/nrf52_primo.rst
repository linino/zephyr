.. _nrf52_primo:

Linino Primo
############

Overview
********

The Linino Primo combines the processing power from the Nordic nRF52 processor,
an Espressif ESP8266 for WiFi, as well as several on-board sensors and a battery charger.
The nRF52 includes NFC (Near Field Communication) and Bluetooth Smart.
The sensors include an on-board button, LED and infrared receiver and transmitter.
It provides support for the Nordic Semiconductor nRF52832 ARM Cortex-M4F CPU and
the following devices:

* :abbr:`ADC (Analog to Digital Converter)`
* CLOCK
* FLASH
* :abbr:`GPIO (General Purpose Input Output)`
* :abbr:`I2C (Inter-Integrated Circuit)`
* :abbr:`MPU (Memory Protection Unit)`
* :abbr:`NVIC (Nested Vectored Interrupt Controller)`
* :abbr:`PWM (Pulse Width Modulation)`
* RADIO (Bluetooth Low Energy and 802.15.4)
* :abbr:`RTC (nRF RTC System Clock)`
* :abbr:`SPI (Serial Peripheral Interface)`
* :abbr:`UART (Universal asynchronous receiver-transmitter)`
* :abbr:`WDT (Watchdog Timer)`

.. figure:: img/linino_primo.jpg
     :width: 800px
     :align: center
     :alt: Linino Primo

     Linino Primo

Hardware
********

The Linino Primo is the first board developed in cooperation with Nordic Semiconductor.
It brings new benefits for the IoT world all on one platform: advanced 32-bit microcontroller architecture,
bluetooth low energy (BLE), Wi-Fi, near-field communications (NFC), and infrared (IR) transmit and receive capability.

There are three onboard microcontrollers:

* nRF52832, the main Arduino microcontroller with integrated BLE and NFC
* STM32f103, a service microcontroller used for advanced debugging and programming of the other microcontrollers
* ESP8266, for Wi-Fi and related internet connectivity functions.

Supported Features
==================

The Linino Primo board configuration supports the following
hardware features:

+-----------+------------+----------------------+
| Interface | Controller | Driver/Component     |
+===========+============+======================+
| ADC       | nRF52832   | adc                  |
+-----------+------------+----------------------+
| CLOCK     | nRF52832   | clock_control        |
+-----------+------------+----------------------+
| FLASH     | nRF52832   | flash                |
+-----------+------------+----------------------+
| GPIO      | nRF52832   | gpio                 |
+-----------+------------+----------------------+
| I2C(M)    | nRF52832   | i2c                  |
+-----------+------------+----------------------+
| MPU       | nRF52832   | arch/arm             |
+-----------+------------+----------------------+
| NVIC      | nRF52832   | arch/arm             |
+-----------+------------+----------------------+
| PWM       | nRF52832   | pwm                  |
+-----------+------------+----------------------+
| RADIO     | ESP8266    | WiFi                 |
+-----------+------------+----------------------+
| RADIO     | nRF52832   | Bluetooth            |
+-----------+------------+----------------------+
| RTC       | nRF52832   | system clock         |
+-----------+------------+----------------------+
| SPI(M/S)  | nRF52832   | spi                  |
+-----------+------------+----------------------+
| UART      | nRF52832   | serial               |
+-----------+------------+----------------------+
| WDT       | nRF52832   | watchdog             |
+-----------+------------+----------------------+
| IR        | STM32f103  | swd                  |
+-----------+------------+----------------------+
| CMSIS-DAP | STM32f103  | infrared             |
+-----------+------------+----------------------+

Connections and IOs
===================

LED
---

* ON    (Orange)
* L9    (Orange) = P0.20
* USER2 (white)
* WIFI  (green)
* BLE   (blue)
* CHG   (red)

Push buttons
------------

* USER1    = P0.07
* USER2
* ESP_BOOT
* RESET

Audio
-----

* Buzzer = P0.08

External Connectors
-------------------

STM32 SWD

+-------+----------------+
| PIN # | Signal Name    |
+=======+================+
| 1     | VCC            |
+-------+----------------+
| 2     | SWDIO          |
+-------+----------------+
| 3     | GND            |
+-------+----------------+
| 4     | SWDCLK         |
+-------+----------------+
| 5     | GND            |
+-------+----------------+
| 6     | Cut off        |
+-------+----------------+
| 7     | GND            |
+-------+----------------+
| 8     | Cut off        |
+-------+----------------+
| 9     | GND            |
+-------+----------------+
| 10    | RESET          |
+-------+----------------+

nRF52 SWD

+-------+----------------+
| PIN # | Signal Name    |
+=======+================+
| 1     | VCC            |
+-------+----------------+
| 2     | EXT_SWDIO      |
+-------+----------------+
| 3     | GND            |
+-------+----------------+
| 4     | EXT_SWDCLK     |
+-------+----------------+
| 5     | GND            |
+-------+----------------+
| 6     | Cut off        |
+-------+----------------+
| 7     | Cut off        |
+-------+----------------+
| 8     | Cut off        |
+-------+----------------+
| 9     | EXT_GND_DETECT |
+-------+----------------+
| 10    | EXT_RESET      |
+-------+----------------+

Headers
-------

Power

+-------+--------------+-------------------------+
| PIN # | Signal Name  | NRF52832 Functions      |
+=======+==============+=========================+
| 1     | N/A          | N/A                     |
+-------+--------------+-------------------------+
| 2     | IOREF        | N/A                     |
+-------+--------------+-------------------------+
| 3     | RESET        | P0.21 / RESET           |
+-------+--------------+-------------------------+
| 4     | 3.3V         | N/A                     |
+-------+--------------+-------------------------+
| 5     | 5V           | N/A                     |
+-------+--------------+-------------------------+
| 6     | GND          | N/A                     |
+-------+--------------+-------------------------+
| 7     | GND          | N/A                     |
+-------+--------------+-------------------------+
| 8     | VIN          | N/A                     |
+-------+--------------+-------------------------+

Analog in

+-------+--------------+-------------------------+
| PIN # | Signal Name  | NRF52832 Functions      |
+=======+==============+=========================+
| 1     | A0           | P0.03 / AIN1            |
+-------+--------------+-------------------------+
| 2     | A1           | P0.04 / AIN2            |
+-------+--------------+-------------------------+
| 3     | A2           | P0.28 / AIN4            |
+-------+--------------+-------------------------+
| 4     | A3           | P0.29 / AIN5            |
+-------+--------------+-------------------------+
| 5     | A4           | P0.30 / AIN6            |
+-------+--------------+-------------------------+
| 6     | A5           | P0.31 / AIN7            |
+-------+--------------+-------------------------+

Digital I/O

+-------+--------------+-------------------------+
| PIN # | Signal Name  | NRF52832 Functions      |
+=======+==============+=========================+
| 1     | D0 (RX)      | P0.11                   |
+-------+--------------+-------------------------+
| 2     | D1 (TX)      | P0.12                   |
+-------+--------------+-------------------------+
| 3     | D2           | P0.13                   |
+-------+--------------+-------------------------+
| 4     | D3           | P0.14 / TRACEDATA[3]    |
+-------+--------------+-------------------------+
| 5     | D4           | P0.15 / TRACEDATA[2]    |
+-------+--------------+-------------------------+
| 6     | D5           | P0.16 / TRACEDATA[1]    |
+-------+--------------+-------------------------+
| 7     | D6           | P0.17                   |
+-------+--------------+-------------------------+
| 8     | D7           | P0.18 / TRACEDATA[0]  / |
|       |              | SWO                     |
+-------+--------------+-------------------------+

Digital I/O

+-------+--------------+-------------------------+
| PIN # | Signal Name  | NRF52832 Functions      |
+=======+==============+=========================+
| 1     | D8           | P0.19                   |
+-------+--------------+-------------------------+
| 2     | D9           | P0.20 / TRACECLK        |
+-------+--------------+-------------------------+
| 3     | D10 (SS)     | P0.22                   |
+-------+--------------+-------------------------+
| 4     | D11 (MOSI)   | P0.23                   |
+-------+--------------+-------------------------+
| 5     | D12 (MISO)   | P0.24                   |
+-------+--------------+-------------------------+
| 6     | D13 (SCK)    | P0.25                   |
+-------+--------------+-------------------------+
| 7     | GND          | N/A                     |
+-------+--------------+-------------------------+
| 8     | AREF         | P0.02 / AIN0            |
+-------+--------------+-------------------------+
| 9     | SDA          | P0.26                   |
+-------+--------------+-------------------------+
| 10    | SCL          | P0.27                   |
+-------+--------------+-------------------------+

ISCP

+-------+--------------+-------------------------+
| PIN # | Signal Name  | NRF52832 Functions      |
+=======+==============+=========================+
| 1     | D12 (MISO)   | P0.24                   |
+-------+--------------+-------------------------+
| 2     | 5V           | N/A                     |
+-------+--------------+-------------------------+
| 3     | D13 (SCK)    | P0.25                   |
+-------+--------------+-------------------------+
| 4     | D11 (MOSI)   | P0.23                   |
+-------+--------------+-------------------------+
| 5     | RESET        | N/A                     |
+-------+--------------+-------------------------+
| 6     | GND          | N/A                     |
+-------+--------------+-------------------------+

Programming and Debugging
*************************

Applications for the ``nrf52_primo`` board configuration can be built and
flashed in the usual way (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Flashing
========

Here is an example for the :ref:`hello_world` application.

First, run your favorite terminal program to listen for output.

.. code-block:: console

   $ minicom -D <tty_device> -b 115200

Replace :code:`<tty_device>` with the port where the board Linino Primo
can be found. For example, under Linux, :code:`/dev/ttyACM0`.

Then build and flash the application in the usual way.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nrf52_primo
   :goals: build flash

Debugging
=========

Debugging an application in the usual way. Here is an example for
the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nrf52_primo
   :maybe-skip-config:
   :goals: debug



Testing the LEDs and buttons in the Linino Primo
************************************************

There are 2 samples that allow you to test that the buttons (switches) and LEDs on
the board are working properly with Zephyr:

.. code-block:: console

   samples/basic/blinky
   samples/basic/button

You can build and flash the examples to make sure Zephyr is running correctly on
your board. The button and LED definitions can be found in :file:`boards/arm/nrf52_primo/board.h`.

References
**********

.. target-notes::
