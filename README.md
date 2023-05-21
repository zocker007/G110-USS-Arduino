# G110-USS-Arduino
Arduino library for communication to Siemens SINAMICS G110 inverter over USS protocol (RS-485)
currently uses Serial1, so its only compatible with boards that have one like native USB devices Leonardo, Micro or Mega
 
SINAMICS V20 can be also compatible but its not tested.
 
You need a level shifter (for example with MAX485 chip) for RS-485 voltage levels and must wire it to 5V/GND and Serial1 TX/RX and a pin you can choose for switching between sending and receiving (only on MAX485)

## important registers/parameters on G110 for testing
[r2024] Counter for error-free telegrams