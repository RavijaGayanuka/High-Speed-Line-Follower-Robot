# Hardware Design & CAD 🛠️

This folder contains the structural and electronic design details of the Dixon Mark 1 Line Follower Robot.

## Chassis Design
Instead of standard 3D printed parts, our team opted for a custom-cut **Dot Board (Glass Epoxy)** chassis. This decision provided two main advantages:
1. **Integrated Prototyping:** Allowed direct soldering of minor components without additional perfboards.
2. **Weight Reduction:** Kept the overall weight extremely low for faster acceleration.

## Circuit Layout
The electrical system integrates the ESP32 microcontroller with a TB6612FNG motor driver and a QTR-8RC sensor array. Power is managed via a 2S Li-ion configuration with a dedicated BMS and stepped down using an MP1584 Buck Converter.

*(Refer to the images in this directory for detailed visual layouts).*
