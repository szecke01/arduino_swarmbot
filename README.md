Arduino Swarmbot for Toxic Waste First-Response
Specification

Modules – Logic, collision detection, toxin detection, communication, drive, power

Power
•	Input: 9V battery
•	Output:
    o	Power Supply H (Vcc_H) – high current power supply (250 mA to 2A)
    o	Power Supply L (Vcc_L) – low current power supply (0-250 mA)
•	Functionality: deliver power to all modules
Collision
•	Input: Vcc_L, actuator
•	Output: collision interrupt to main logic
•	Functionality:
    o	Sensor – 8 button or paddle actuators located on the perimeter of the bot
    o	All sensor outputs will be logic “OR”ed together connected to an interrupt pin in main logic.  Each sensor output will also be connected to its own digital input pin in main logic.
    o	Decoding occurs in software.
Toxin Detection
•	Input: Vcc_L, photosensor
•	Output: analog voltage to main logic
•	Functionality:
    o   Detect color on the ground with photosensor and map a voltage to color (wavelength).
    o	Software will determine type of “material”.
    o	Flash LED upon detection.
Communication
•	Input: Vcc_L, IR photosensor
•	Output: IR signal, communication interrupt to main logic 
•	Functionality:
    o	Transmit and receive a specific communication protocol (TBD) with companion swarm bot.
    o	Input decoded in software.

Drive 
•	Input: Vcc_H, right wheel logic (2 lines), left wheel logic (2 lines)
•	Output: current to drive motors
•	Functionality:
    o	Controls H-bridge to determine movement of swarm bot.
    o	Movements include stopping, forward, reverse, clockwise and counter-clockwise rotation.
Logic
•	Input: Vcc_L, sensor signals from sub-modules
•	Output: control logic signal to sub-modules
•	Functionality:
    o	Process inputs to determine state transitions in finite state machine.
    o	Performs necessary computations for state operation.
