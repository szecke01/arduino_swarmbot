# Arduino Swarmbot for Toxic Waste First-Response  
## Specification

Our swarmbot is composed of six independent modules designed to promote the singular purpose of toxic waste identification for large-scale disaster first response. The robot must perform several actions that represent prototypical situations that would arise in a natural or man-made disaster, such as a toxic oil spill. The robot must be able to identify the surface of the floor beneath it to determine its chemical safety, recover from collisions with various obstacles, and communicate unsafe zones with another robot. Ideally, the robot must be lightweight to maximize battery efficiency, small enough volumetrically to maneuver throughout the environment, and must move quickly enough to perform the task in a timely fashion (roughly 90 seconds), but not so fast as to be inaccurate in its analysis or self-damaging (TBD). 

#### Power:  
*	Input: 9V battery, ON/OFF Switch  
*	Output:  
  * Power Supply HP (Vcc_HP)  9V – high current positive voltage power supply (250 mA to 2A)  
  * Power Supply HN (Vcc_HN) -9V - high current negative voltage power supply (250 mA to 2A)
  * Power Supply L (Vcc_L)    5V – low current power supply (0-250 mA)  
*	Functionality: deliver power to all modules  

#### Collision:  
*	Input: Vcc_L, actuator  
*	Output: collision interrupt to main logic  
*	Functionality:  
  *	Sensor – 6 button or paddle actuators located on the perimeter of the bot  
  *	All sensor outputs will be logic “OR”ed together connected to an interrupt pin in main logic.  Each sensor output will also be connected to its own digital input pin in main logic.  
  *	Decoding occurs in software.  

#### Toxin Detection:  
*	Input: Vcc_L, photosensor  
*	Output: analog voltage to main logic  
*	Functionality:  
  *   Detect color on the ground with photosensor and map a voltage to color (wavelength).  
  *	Software will determine type of “material”.  
  *	Flash LED upon detection.  

#### Communication:    
*	Input: Vcc_L, Vcc_HN, Vcc_HP, IR photosensor  
*	Output: IR signal, Recieved Signal
*	Functionality:  
  *	Transmit and receive serial communication over IR at 600 baud AM by 20 KHz

#### Drive:   
*	Input: Vcc_H, right wheel logic (2 lines), left wheel logic (2 lines)  
*	Output: current to drive motors  
*	Functionality:  
  *	Controls H-bridge to determine movement of swarm bot.  
  *	Movements include stopping, forward, reverse, clockwise and counter-clockwise rotation.  
 
#### Logic:  
*	Input: Vcc_L, sensor signals from sub-modules  
*	Output: control logic signal to sub-modules  
*	Functionality:  
  *	Process inputs to determine state transitions in finite state machine.  
  *	Performs necessary computations for state operation.  
