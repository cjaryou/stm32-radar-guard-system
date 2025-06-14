# STM32F411RE Radar Guard System - Configuration Guide

## IMMEDIATE ACTION REQUIRED - Timer 2 Setup

### Step-by-Step TIM2 Configuration:

1. **Open STM32CubeIDE** and go to your .ioc file
2. **In the left panel**, find "Timers" section
3. **Click on TIM2** (you should see it in the list)
4. **Enable TIM2** by setting:
   - Clock Source: "Internal Clock"
5. **In the Parameter Settings tab**, set:
   - Prescaler: **83**
   - Counter Period (AutoReload Register): **4294967295**
   - Counter Mode: **Up**
   - Clock Division: **1**
   - Auto-reload preload: **Disable**

6. **Generate Code** (Ctrl+S and generate)
7. **Rebuild and flash** your project

### What This Fixes:
- Provides 1µs precision timing for ultrasonic sensor
- Enables proper pulse width measurement
- Fixes the 3.9cm maximum reading issue

## Your Current Configuration Status:

✅ **TIM1 (Servo PWM):** Correctly configured
✅ **I2C1 (LCD):** Correctly configured  
✅ **TIM2 (Ultrasonic):** **CORRECTLY CONFIGURED!**
? **GPIO Pins:** Need to verify
? **USART2:** Need to verify

### Critical Settings for STM32CubeIDE

### 1. Timer 2 Configuration (For Ultrasonic Sensor Timing) ✅ **CORRECTLY CONFIGURED**
**Purpose:** Microsecond precision timing for HC-SR04

Your TIM2 is properly configured with:
- **Timer:** TIM2
- **Prescaler:** 83 (to get 1µs per tick) ✅
- **Counter Period:** 4294967295 (maximum for 32-bit timer) ✅
- **Clock Division:** 1 ✅
- **Counter Mode:** Up ✅
- **Auto-reload preload:** Disable ✅
- **All Channels:** Disabled ✅ (This is correct for ultrasonic timing)

**Calculation:**
- System Clock: 84 MHz
- Prescaler: 83+1 = 84
- Timer frequency: 84MHz / 84 = 1MHz = 1µs per tick

**Status:** This configuration is perfect for ultrasonic sensor timing!

### 2. Timer 1 Configuration (For Servo PWM)
**Purpose:** 50Hz PWM signal for servo motor control

- **Timer:** TIM1
- **Prescaler:** 839 (to get 50Hz)
- **Counter Period:** 1999 (20ms period)
- **PWM Mode:** PWM Generation CH1
- **Output Compare Preload:** Enable
- **Auto-reload preload:** Disable

**Calculation:**
- System Clock: 84 MHz
- Prescaler: 839+1 = 840
- Timer frequency: 84MHz / 840 = 100kHz
- Period: 1999+1 = 2000 counts = 20ms (50Hz)
- 1ms pulse = 100 counts, 2ms pulse = 200 counts

### 3. GPIO Configuration

#### Ultrasonic Sensor Pins:
- **PC0 (TRIG):** GPIO_Output, No Pull-up/Pull-down, Low Speed
- **PC1 (ECHO):** GPIO_Input, No Pull-up/Pull-down

#### LED Pins:
- **PB0 (RED LED):** GPIO_Output, No Pull-up/Pull-down, Low Speed
- **PB1 (GREEN LED):** GPIO_Output, No Pull-up/Pull-down, Low Speed

#### Buzzer Pin:
- **PC4 (BUZZER):** GPIO_Output, No Pull-up/Pull-down, Low Speed

#### Servo Pin:
- **PA8:** TIM1_CH1 (PWM Output)

### 4. I2C Configuration (For LCD)
- **I2C1**
- **Speed:** 100 kHz (Standard Mode)
- **Pins:** PB8 (SCL), PB9 (SDA)
- **Pull-up resistors:** External (4.7kΩ recommended)

### 5. UART Configuration (For Debug)
- **USART2**
- **Baud Rate:** 115200
- **Word Length:** 8 Bits
- **Stop Bits:** 1
- **Parity:** None
- **Pins:** PA2 (TX), PA3 (RX)

## Pin Configuration Summary

| Function | Pin | Configuration |
|----------|-----|---------------|
| TRIG | PC0 | GPIO Output |
| ECHO | PC1 | GPIO Input |
| Servo PWM | PA8 | TIM1_CH1 |
| Red LED | PB0 | GPIO Output |
| Green LED | PB1 | GPIO Output |
| Buzzer | PC4 | GPIO Output |
| LCD SDA | PB9 | I2C1_SDA |
| LCD SCL | PB8 | I2C1_SCL |
| UART TX | PA2 | USART2_TX |
| UART RX | PA3 | USART2_RX |

## Clock Configuration
- **System Clock:** 84 MHz
- **APB1 Timer Clock:** 84 MHz
- **APB2 Timer Clock:** 84 MHz

## Troubleshooting Tips

### If sensor still shows 3.9cm max:

1. **Check Timer2 Prescaler:**
   - Ensure prescaler is exactly 83 for 1µs resolution
   - Verify system clock is 84MHz

2. **Check GPIO Configuration:**
   - TRIG pin must be output
   - ECHO pin must be input with no pull-up/down

3. **Hardware Checks:**
   - Verify HC-SR04 power supply (5V recommended)
   - Check wiring connections
   - Ensure HC-SR04 VCC is connected to 5V, not 3.3V
   - Add 1kΩ resistor in series with ECHO pin if using 5V sensor

4. **Code Debugging:**
   - Monitor UART output for distance readings
   - Check if timeout values are appropriate
   - Verify timer counter is incrementing correctly

### If objects are not detected:

1. **Reduce MIN_DETECTION_DISTANCE** to 1.0cm
2. **Increase scan frequency** by reducing servo step size
3. **Adjust PROXIMITY_ALARM_DISTANCE** based on your needs
4. **Check sensor orientation** and mounting

## Testing Procedure

1. **Upload code** to STM32F411RE
2. **Connect UART** to PC and open serial monitor at 115200 baud
3. **Observe debug output** showing angle and distance readings
4. **Test with objects** at various distances (5cm, 10cm, 20cm, etc.)
5. **Verify servo movement** and LCD display updates

## Expected Behavior

- Servo should sweep from 0° to 180° and back
- Distance readings should be accurate within ±1cm
- Objects closer than 15cm should trigger detection
- LCD should show current angle and distance
- Debug UART should show continuous readings
