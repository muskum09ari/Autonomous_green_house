

# Autonomous Greenhouse System

An intelligent greenhouse monitoring and control system built on the STM32F407 microcontroller platform. This project implements automated environmental control with security features, real-time monitoring, and adaptive climate management.

## Features

### Environmental Monitoring
- **Temperature & Humidity Sensing**: DHT11 sensor for real-time climate data
- **Light Level Detection**: LDR (Light Dependent Resistor) with ADC conversion
- **Visual Feedback**: 5-LED bar graph display for light intensity levels
- **LCD Display**: 16x2 character display for system status and sensor readings

### Automated Climate Control
- **Temperature Management**: DC motor fan control with PWM speed adjustment (activates when temperature ≥ 25°C)
- **Humidity Control**: Relay-controlled irrigation/misting system (activates when humidity ≥ 75%)
- **Adaptive Response**: Real-time adjustments based on sensor feedback

### Security System
- **Dynamic Authentication**: Random 4-digit security code generation using TIM3 timer
- **TM1637 Display**: 7-segment display shows the current access code
- **Keypad Input**: 4x4 matrix keypad for code entry
- **Access Control**: Green/Red LED indicators for access status
- **Auto-Refresh**: Security code updates every 20 seconds when system is locked

## Hardware Components

### Microcontroller
- **STM32F407VGT6** (ARM Cortex-M4, 168 MHz)

### Sensors
- **DHT11**: Temperature and humidity sensor (Digital, PC3)
- **LDR**: Light-dependent resistor with ADC (Analog, PA6)

### Actuators
- **DC Motor**: PWM-controlled ventilation fan (PA3 - PWM, PA2 - Direction)
- **Relay Module**: Controls irrigation/misting system (PA8)

### User Interface
- **16x2 LCD**: Status display (I2C/Parallel interface)
- **4x4 Matrix Keypad**: Security code input (GPIOB pins 2-9)
- **TM1637 Display**: 4-digit 7-segment display for security code (PC0 - CLK, PC1 - DIO)
- **LED Indicators**:
  - 5x LEDs for light level bar graph (PC6, PC7, PC8, PC12, PD2)
  - Access status LEDs (PA10 - Red, PA11 - Green)

## Pin Configuration

| Component | Pin | Function |
|-----------|-----|----------|
| DHT11 | PC3 | Digital I/O |
| LDR | PA6 | ADC1_IN6 |
| Motor PWM | PA3 | TIM2_CH4 |
| Motor Direction | PA2 | GPIO Output |
| Relay | PA8 | GPIO Output |
| Keypad Rows | PB2-5 | GPIO Output |
| Keypad Columns | PB6-9 | GPIO Input (Pull-up) |
| Red LED | PA10 | GPIO Output |
| Green LED | PA11 | GPIO Output |
| TM1637 CLK | PC0 | GPIO Output |
| TM1637 DIO | PC1 | GPIO Output |
| Light LEDs | PC6, PC7, PC8, PC12, PD2 | GPIO Output |

## System Thresholds

```c
#define TEMP_THRESHOLD 25   // Temperature in °C
#define HUM_THRESHOLD  75   // Humidity in %
```

## Software Architecture

### Timers
- **TIM2**: PWM generation for DC motor speed control (1 kHz frequency)
- **TIM3**: Security code refresh timer (20-second intervals)
- **SysTick**: Microsecond/millisecond delay generation

### Key Functions

#### Sensor Reading
- `DHT11_start()`: Initiates DHT11 communication
- `DHT11_read_byte()`: Reads 8-bit data from DHT11
- `read_adc_pa6()`: Reads LDR analog value via ADC

#### Actuator Control
- `Motor_SetSpeed(uint16_t speed_percent)`: Sets fan speed (0-100%)
- `Relay_ON()` / `Relay_OFF()`: Controls irrigation system

#### Security
- `update_random_code()`: Generates new 4-digit access code
- `Keypad_Scan()`: Scans 4x4 matrix for key press
- `TM1_display_number(uint16_t num)`: Displays code on TM1637

#### Display
- `display_status(uint8_t temperature, uint8_t humidity)`: Updates LCD with sensor data
- `set_led_level(uint8_t level)`: Updates light level bar graph

## Operation Flow

### 1. System Initialization
```
- Configure GPIO pins
- Initialize LCD display
- Start TIM2 for PWM
- Start TIM3 for code generation
- Display welcome message
- Generate initial random code
```

### 2. Security Phase
```
- Display random code on TM1637 (updates every 20s)
- Wait for keypad input (* to start entry)
- User enters 4-digit code (actual password = displayed code + 2)
- Validate entry with # key
- Grant/deny access based on correctness
```

### 3. Monitoring Phase (After Access Granted)
```
- Stop random code generation
- Clear TM1637 display
- Continuously read sensors:
  - LDR: Update 5-LED bar graph (adaptive calibration)
  - DHT11: Read every 3 seconds
- Display temperature and humidity on LCD
- Activate fan if temperature ≥ 25°C (40% speed)
- Activate relay if humidity ≥ 75%
```

## Code Highlights

### Adaptive LDR Calibration
The system auto-calibrates light levels over the first 2000 readings to establish min/max thresholds, ensuring accurate LED bar graph representation regardless of ambient conditions.

### Security Code Algorithm
```c
// Displayed code: 1234
// Actual password: 1236 (displayed + 2)
uint16_t actual_password = system_random_code + 2;
```

### PWM Motor Control
```c
// 40% speed when temperature exceeds threshold
if (Temp_int >= TEMP_THRESHOLD) 
    Motor_SetSpeed(40);
else 
    Motor_SetSpeed(0);
```

## Building and Flashing

### Requirements
- STM32CubeIDE or Keil MDK
- STM32 ST-LINK Utility
- ARM GCC Compiler

### Build Steps
```bash
# Clone repository
git clone https://github.com/muskum09ari/STM32.git
cd STM32_coode

# Open project in STM32CubeIDE
# Build: Project → Build All (Ctrl+B)
# Flash: Run → Debug (F11) or use ST-LINK Utility
```

## Future Enhancements
- [ ] Wi-Fi connectivity (ESP8266/ESP32 integration)
- [ ] Mobile app for remote monitoring
- [ ] Data logging to SD card
- [ ] Soil moisture sensor integration
- [ ] Automated fertilizer dispensing
- [ ] Multi-zone control
- [ ] Cloud integration for analytics

## Troubleshooting

### DHT11 Not Responding
- Check PC3 connection
- Verify 5V power supply
- Ensure proper timing (20ms low pulse)

### Motor Not Running
- Verify PA3 PWM signal with oscilloscope
- Check motor driver connections (AIN1, AIN2)
- Confirm motor power supply is adequate

### Keypad Not Working
- Test each row/column with multimeter
- Verify pull-up resistors on columns
- Check for shorts or open connections

### Random Code Not Updating
- Verify TIM3 is running: `TIM3->CR1 & TIM_CR1_CEN`
- Check interrupt enable: `NVIC_IsEnabled(TIM3_IRQn)`

## License
This project is open-source and available under the MIT License.



## Acknowledgments
- STM32 HAL Library Documentation
- DHT11 Protocol Specification
- TM1637 Display Driver Reference

---

**Project Status**: Active Development  
**Last Updated**: December 2025
