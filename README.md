# Flowmaster

This is an automated system to control the radiator fans of a water cooled computer.  Fan speed is regulated according to the temperature of the water in the system.

The system is designed to run the fans at their lowest (quietest) setting so that the system remains as silent as possible without overheating.

The MCU can provide information on fan speed, pump speed, water temperature and ambient temperature

## Requirements

This uses an AVR atmega88 microcontroller.  Temperature sensing is done via cheaply available 10k ohm thermistors.

Radiator fans must be of the 4 wire PWM type.

## todo

* Provide a circuit schematic
* Add support for a flow rate sensor
