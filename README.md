# Homemade Manipulator Arm Project

Welcome to the Homemade Manipulator Arm project repository! This project features a DIY robotic arm with 6 degrees of freedom (6DoF) and a gripper. The arm is controlled using 4 MG1501 servos and 3 SG90 servos. The project includes SolidWorks 3D part designs, electrical schemas created with EasyEDA, and the ESP code for controlling the arm.
Use PlatformIO in Visual Studio Code (VSCode) to manage and upload the code to the ESP microcontroller.

## Contents

- [Description](#description)
- [Components](#components)
- [Folder Structure](#folder-structure)
- [Usage](#usage)
- [Credits](#credits)
- [License](#license)

## Description

This project is a homemade manipulator arm designed for educational purposes and hobbyist robotics enthusiasts. The arm is built using 3D-printed parts designed in SolidWorks and controlled using ESP32 microcontroller boards. The control methods include Bluetooth Low Energy (BLE) using the RemoteXY library or manual control using encoders (potentiometers).

## Components

The manipulator arm features the following components:

- **Servos**:
  - 4 x [MG1501](https://www.pololu.com/file/0J729/HD-1501MG.pdf) servos (for joints)
  - 3 x [SG90 servos](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf) (for gripper and the last wrists)
  
- **Electronics**:
  - ESP32 microcontroller board
  - PCB Board to control the robot
  - PCB Board with Potentiometers (for manual control using encoders)
  
- **3D Parts**:
  - Various parts designed in SolidWorks for the arm's structure
  
- **Electrical Schemas**:
  - Schemas created in EasyEDA for the wiring and electronic components layout
  
## Folder Structure

The repository is organized as follows:

/Code
<br>&nbsp;&nbsp;&nbsp;&nbsp;├── [src/main.cpp](Code/src/main.cpp)  # Main file of the Project
<br>&nbsp;&nbsp;&nbsp;&nbsp;├── [lib](Code/lib)                    # Folder containing help libraries
<br>/Cad
<br>&nbsp;&nbsp;&nbsp;&nbsp;   ├── *.sldprt                            # SolidWorks part files for 3D-printed components
<br>/Electrical_Schemas
<br>&nbsp;&nbsp;&nbsp;&nbsp;    ├── schemas\Main Board     # Electrical schemas created in EasyEDA for the robot control PCB
<br>&nbsp;&nbsp;&nbsp;&nbsp;    ├── schemas\Potentiometer Board   # Electrical schemas created in EasyEDA for the Potentiometer PCB
/README.md                             
## Usage

To replicate or modify this project, follow these steps:

1. **3D Printing**:
   - Print the 3D parts from the `\Cad\stl` directory using a 3D printer.

2. **Assembly**:
   - Assemble the printed parts and servos according to the design.

3. **Electronics**:
    <BR>Choose one of the following methods:
     - **Option 1: Order PCB Boards**:
       - Use the provided PCB design files (`*.zip`) in the repository's attachments to order custom PCB boards.
       <br>* For Manipulator Main PCB - schemas\Main Board\Gerber_Manip_PCB.zip
       <br>* For Potentiometer PCB schemas\Potentiometer\Gerber_Potensiometers_PCB.zip
     - **Option 2: Manual Wiring**:
       - Refer to the electrical schemas (`Electrical_Schemas`) and wire up the servos, microcontroller, and other components manually based on the schematic PDFs (`*.pdf`).

4. **Programming**:
   - Use PlatformIO in Visual Studio Code (VSCode) to manage and upload the code

5. **Control**:
   - Use the RemoteXY library for BLE control, or wire up potentiometers for manual control.

## Credits

This project was created by Tamir Mhabary. Feel free to contribute by submitting pull requests or reporting issues.

## License

This project is licensed under the [MIT License](LICENSE). Feel free to use, modify, or distribute this code and design for personal or educational purposes.
