## This is to send a button press command to the roomba
uint8 buttoncode
## 0 is Power (opcode 133) to put it to sleep
## 1 is Spot  (opcode 134) to run a spot cleaning cycle
## 2 is Clean (opcode 135) to run a standard cleaning cycle
## 3 is Max   (opcode 136) to run a max length cleaning cycle
## 10 is Dock (opcode 143) to return to the dock.
## Any of these will return the SCI to passive mode
