# Raspberry PI Autonomous Bot

![This is a image](/img/Lane_Following_R_Trim.gif)

**Project Summary**:

This project involved building an autonomous car powered by a Raspberry Pi and equipped with various features, such as **lane detection**, **traffic light detection**, **stop sign detection**, **U-turn sign detection**, and **vehicle detection**.

Key Project Features:

- Lane Detection: **OpenCV** was used for lane detection, while a **PID controller** was used to steer the vehicle. The controller calculated the steering angle by taking the difference between the frame center and lane center.

- Object Detection: Cascade training was used to train the model to detect traffic lights, stop signs, U-turn signs, and other vehicles.

- Bluetooth Remote Control: A script was written to enable **remote control** of the car using Bluetooth, allowing the car to be controlled via a mobile phone. The script captured images of the driving and steering input, which were stored in a CSV file.

- Data Collection: The main requirement was to drive carefully on the track, as this information was used to train an AI model to output a steering angle by analyzing the camera frame.

- Hardware: The Raspberry Pi was connected to an Arduino, which was connected to the motor driver. **I2C communication** was used between the Raspberry Pi and Arduino. The car was equipped with a 4-wheel drive double layer chassis, and a power bank was used to power the Raspberry Pi, while the motor controller and Arduino were powered by lithium-ion cells.

- Android Application: The project utilized an Android application called Bluetooth Electronics that provides various controls, including joystick control.

Overall, this project was successful in building a Raspberry Pi-powered autonomous car with remote control functionality, and demonstrated the use of various computer vision techniques and hardware components to achieve this goal.