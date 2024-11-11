				      




### Smart Parking System
								~CB.EN.U4CSE22522 Kanishka.D
								~CB.EN.U4CSE22530 Nandhakumar.P
								~CB.EN.U4CSE22544 Shreya.J.V
								~CB.EN.U4CSE22546 Srinidhi.K



















## Introduction
The ultrasonic distance measurement system is designed to measure distances using an HC-SR04 ultrasonic sensor interfaced with an STM32F401 microcontroller. The system utilizes timer and interrupt capabilities to calculate the distance and subsequently activates a set of LEDs based on the measured range. This project demonstrates key concepts in embedded systems, including GPIO configuration, timer operations, and external interrupts.

## Problem statement:
Our project is a smart parking system designed to assist drivers with precise object detection while reversing. When an object is detected close to the back of the car, all three LEDs will blink to indicate an immediate proximity warning. Based on varying distances, different LED patterns and behaviors provide additional alerts, helping drivers maintain safe distances and avoid obstacles.

## COMPONENTS:

# STM32F401 Microcontroller
•	Overview: The STM32F401 is a powerful 32-bit microcontroller based on the ARM Cortex-M4 architecture, designed for high performance and energy efficiency. It is commonly used in embedded systems and IoT applications.

•	Key Features:
o	Clock Speed: Can operate up to 84 MHz, ensuring fast data processing.
o	Memory: Includes up to 512 KB of Flash memory for code storage and up to 96 KB of SRAM for data storage.
o	Peripherals: Comes with timers, analog-to-digital converters (ADCs), digital-to-analog converters (DACs), and various communication interfaces (USART, SPI, I2C).
o	GPIOs: Configurable pins for interfacing with external hardware components such as sensors, actuators, and LEDs.
•	Role in Project: The STM32F401 serves as the main control unit, handling the ultrasonic sensor's input, processing timing data, and controlling LED outputs based on distance measurements.

2. HC-SR04 Ultrasonic Distance Sensor
•	Overview: The HC-SR04 is an ultrasonic sensor that measures distance by using sound waves. It is popular for its reliability and ease of use.
•	Key Features:
o	TRIG Pin: Used to send a short, high pulse that triggers the ultrasonic sound wave.
o	ECHO Pin: Outputs a pulse that reflects the time taken for the ultrasonic wave to return after hitting an object.
o	Range: Measures distances from 2 cm to 400 cm, with an accuracy of about ±3 mm.
•	Working Principle: The sensor emits an ultrasonic wave at 40 kHz, which bounces back when it hits an object. The ECHO pin outputs a high pulse whose duration corresponds to the round-trip travel time of the sound wave. This duration is used to calculate the distance using: Distance (cm)=Time (μs)×0.03432\text{Distance (cm)} = \frac{\text{Time (μs)} \times 0.0343}{2}Distance (cm)=2Time (μs)×0.0343
•	Role in Project: The microcontroller sends a signal through the TRIG pin to start the measurement and reads the ECHO pin to capture the pulse width, which is then used to calculate the distance.

4. GPIO Pins (General Purpose Input/Output)
•	Overview: GPIOs on the STM32F401 are versatile pins that can be configured as inputs or outputs for various applications.
•	Configuration in Project:
o	PA0 (TRIG): Configured as an output to trigger the ultrasonic sensor.
o	PA1 (ECHO): Configured as an input with interrupt capability to read the pulse duration.
o	PC14, PB9, PC13: Configured as output pins for controlling LEDs that provide visual feedback based on distance measurements.
•	Role in Project: The GPIOs are used to control the flow of data to and from the sensor, trigger the sensor, and manage LED status based on measured distance.

6. TIM2 Timer (STM32 Peripheral)
•	Overview: TIM2 is a general-purpose 32-bit timer used for precise timing and pulse-width measurements.
•	Key Features:
o	Resolution: Configured to run at 1 MHz, providing a 1 μs resolution.
o	Functionality: Counts time intervals and stops upon command, allowing accurate measurement of the ECHO pulse duration.
•	Role in Project: TIM2 is started when a rising edge on the ECHO pin is detected and stopped when a falling edge occurs. The count value is used to determine the time taken for the ultrasonic pulse to return.

8. LEDs
•	Overview: Light-emitting diodes provide a simple visual indication of the measured distance.
•	Functionality:
o	Close Range (≤ 100 cm): Specific LEDs light up or flash rapidly to indicate proximity.
o	Medium Range (101–400 cm): LEDs flash at a slower rate to indicate an intermediate distance.
o	Far Range (401–750 cm): LEDs show a different pattern for moderate distances.
o	No Detection (> 750 cm): LEDs remain off, indicating that no object is detected within range.
•	Role in Project: The LEDs offer visual feedback to quickly assess the measured distance without needing additional output devices.

10. SYSCFG (System Configuration Controller)
•	Overview: A peripheral that enables system-level configuration, including external interrupt routing.
•	Role in Project: Configures the external interrupt line (EXTI) for the ECHO pin (PA1), allowing the microcontroller to respond to the signal's rising and falling edges.

12. NVIC (Nested Vectored Interrupt Controller)
•	Overview: NVIC is responsible for managing interrupts and exceptions, allowing the microcontroller to handle events with priority and nesting capabilities.
•	Role in Project: Ensures that the EXTI1 interrupt (for the ECHO pin) is enabled and handled appropriately when triggered, allowing the microcontroller to capture timing data with minimal latency.
•	External Interrupt (EXTI):
•	The EXTI protocol allows the system to detect changes on the ECHO pin (connected to PA1) and trigger an interrupt for measuring the time the echo pulse is high. This interrupt helps in capturing the start and end of the pulse from the ultrasonic sensor.
•	This protocol is essential for real-time response in measuring the distance based on the duration of the echo pulse.

Supporting Software and Tools
 Keil uVision IDE
•	Overview: A development environment used for coding, compiling, and debugging projects based on ARM microcontrollers.
•	Role in Project: Provides an integrated platform for developing, compiling, and debugging the STM32F401-based project code.


## Challenges:
The main challenges include:
1.	Accurate Timing and Measurement: Using the ultrasonic sensor's ECHO signal to measure the time accurately and calculate the distance precisely in centimeters.
2.	Interrupt Handling: Properly handling interrupts for the ECHO pin to capture the pulse duration without missing any signal transitions.
3.	Real-Time Processing: Ensuring the system continuously measures distance and updates LED states without significant delays.
4.	Effective Feedback Mechanism: Implementing a clear and immediate response through LEDs to indicate object proximity, aiding in user interaction or situational awareness.

## Registers Configured:
•	GPIO Registers
•	EXTI (External Interrupt) Registers
•	Timer (TIM2) Registers


![WhatsApp Image 2024-11-11 at 20 21 13_0f6b5bdf](https://github.com/user-attachments/assets/06eb7781-36bb-4494-a85b-aef475d04e18)

![WhatsApp Image 2024-11-11 at 20 21 14_38d93a33](https://github.com/user-attachments/assets/aa199d9b-a585-4bb4-b9de-b414fd791a3b)

![WhatsApp Image 2024-11-11 at 20 21 20_9e1298fc](https://github.com/user-attachments/assets/19786e2d-ea2d-4c9f-91b4-4e4a4736c2e5)






