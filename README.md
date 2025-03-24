# BluetoothLE_RC_Car

📌 Description

This project was developed as the final project for the graduate course EGEC 558B: Microprocessors and Systems Applications in the Computer Engineering program at California State University, Fullerton.

The project requirement was to work in a team of one or two students to design an ARM-based circuit system capable of interfacing with external devices—either through architecture-based design or by reading inputs from buttons or sensors to control output devices.

This project was solely designed and implemented by me. It features an RC model car controlled via a remote control using a Bluetooth Low Energy (BLE) connection. Specifically, the Nordic UART Service (NUS) was used to establish wireless communication between the remote and the car.

Both the car and the remote control utilize the nRF5340 DK as their microcontroller unit (MCU). The remote control reads input from two joysticks, measuring their positions and transmitting the data over the BLE connection. This input determines the car's speed and direction, allowing for smooth and responsive control.

🚀 Features

Wireless control via BLE - The car receives commands from the remote over Bluetooth Low Energy (NUS protocol).

Joystick-Based Steering - Dual joystick control for determining speed and direction.

Speed Control - Two different speeds modes based on the position of speed joystick.

LED indicators - Onboard LEDs to indicate connectin status (blinking LED 2 for pairing, solid LED 2 for connected).

🛠️ Technologies Used

Software:

  -Nordic UART Service 
  
  -nRF connect for Vscode (Software development kit)
  
  -Bluetooth Low Energy 
  
Hardware:

  -nRF5340 DK (Microcontroller Unit)
  
  -DRI0002 (Motor driver)
  
  -Ovonic 2s LiPo Batteries 35C (2)
  
  -Joysticks (2)
  
  -DC Gearbox motor - TT Motor (4)
  

📄 Additional Resources

For more information about the project consult the project conference paper and presentation slides provided.

demonstration video comming soon
