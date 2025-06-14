/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F411RE Radar Guard System - Optimized Version
  * @description    : Object detection system with HC-SR04 ultrasonic sensor
  *                   Servo motor scanning, LCD display, LED and buzzer alerts
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_SCANNING,
    STATE_WARNING,
    STATE_OBJECT_DETECTED
} RadarState;

typedef struct {
    float distance;
    uint8_t angle;
    uint32_t timestamp;
} DetectionInfo;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Distance and detection settings
#define PROXIMITY_ALARM_DISTANCE    15.0f   // Alarm distance (cm)
#define MIN_DETECTION_DISTANCE      2.0f    // Minimum detection distance (cm) - lower threshold
#define MAX_DETECTION_DISTANCE      400.0f  // Maximum detection distance (cm) - higher range
#define SENSOR_READINGS_COUNT       3       // Number of readings for averaging

// Timing settings
#define SERVO_UPDATE_INTERVAL       25      // Servo movement interval (ms)
#define DISTANCE_READ_INTERVAL      50      // Distance reading interval (ms)
#define WARNING_BLINK_INTERVAL      200     // Warning blink interval (ms)
#define CLEAR_CHECK_INTERVAL        1000    // Area clear check interval (ms)

// Detection filtering
#define TRIGGER_THRESHOLD           3       // Consecutive detections required for alarm
#define CLEAR_THRESHOLD             5       // Consecutive readings required for clear

// Servo settings
#define SERVO_MIN_ANGLE             0
#define SERVO_MAX_ANGLE             180
#define SERVO_STEP                  2       // How many degrees to move per step

// Pin definitions (pins used in code)
#define TRIG_PIN                    GPIO_PIN_0  // PC0
#define ECHO_PIN                    GPIO_PIN_1  // PC1
#define BUZZER_PIN                  GPIO_PIN_4  // PC4
#define RED_LED_PIN                 GPIO_PIN_0  // PB0
#define GREEN_LED_PIN               GPIO_PIN_1  // PB1
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// System state variables
RadarState current_state = STATE_SCANNING;
DetectionInfo last_detection = {0};

// Servo control variables
int current_angle = 0;
int angle_increment = SERVO_STEP;
uint32_t last_servo_time = 0;

// Distance reading variables
float current_distance = 0.0f;
uint32_t last_distance_time = 0;

// Detection counters
int trigger_count = 0;
int clear_count = 0;

// Warning system variables
uint32_t last_warning_time = 0;
bool warning_led_state = false;
int warning_blink_count = 0;

// LCD update
uint32_t last_lcd_update = 0;
char lcd_buffer[17]; // 16 chars + null terminator for LCD
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
// Servo motor control functions
void servo_set_angle(uint8_t angle);
void servo_update(void);

// Distance reading functions
float ultrasonic_read_distance(void);
float ultrasonic_read_distance_filtered(void);
bool is_valid_distance(float distance);

// State management functions
void state_change_to_scanning(void);
void state_change_to_warning(void);
void state_change_to_object_detected(void);

// System update functions
void system_update_scanning(void);
void system_update_warning(void);
void system_update_object_detected(void);

// LCD and visual feedback
void lcd_update_display(void);
void led_control(bool red_state, bool green_state);
void buzzer_control(bool state);

// Helper functions
void system_reset_counters(void);
uint32_t get_system_time(void);
void debug_print_distance(float distance, uint8_t angle);
void test_ultrasonic_sensor(void); // New test function
void hardware_diagnostic_check(void); // Hardware diagnostic function
void safe_lcd_print(uint8_t row, const char* format, ...); // Safe LCD print function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Sets servo motor angle
 * @param angle: angle value between 0-180 degrees
 */
void servo_set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;

    // PWM duty cycle calculation (for 50Hz)
    // 0° = 1ms pulse, 180° = 2ms pulse
    // Timer period = 1999, which means 20ms (50Hz)
    // 1ms = 1999/20 = 100, 2ms = 200
    uint16_t pulse = (uint16_t)(100 + ((float)angle / 180.0f * 100.0f));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

/**
 * @brief Updates servo motor position
 */
void servo_update(void) {
    uint32_t current_time = get_system_time();

    if (current_time - last_servo_time >= SERVO_UPDATE_INTERVAL) {
        last_servo_time = current_time;

        servo_set_angle(current_angle);        // Angle update
        current_angle += angle_increment;

        if (current_angle >= SERVO_MAX_ANGLE) {
            current_angle = SERVO_MAX_ANGLE;
            angle_increment = -SERVO_STEP;
        } else if (current_angle <= SERVO_MIN_ANGLE) {
            current_angle = SERVO_MIN_ANGLE;
            angle_increment = SERVO_STEP;
        }
    }
}

/**
 * @brief Reads distance from HC-SR04 ultrasonic sensor - Improved version
 * @return Measured distance (cm), returns 999.0 on error
 */
float ultrasonic_read_distance(void) {
    uint32_t timeout_start = 0;
    uint32_t pulse_duration = 0;
    uint32_t start_time = 0;

    /* 1. Ensure TRIG is low and sensor is settled */
    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_RESET);
    HAL_Delay(5); // Increased stabilization time

    /* 2. Check if ECHO is already high (sensor busy) */
    if (HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) == GPIO_PIN_SET) {
        // Wait for ECHO to go low first
        timeout_start = HAL_GetTick();
        while (HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) == GPIO_PIN_SET) {
            if (HAL_GetTick() - timeout_start > 50) {
                return 999.0f; // Sensor stuck
            }
        }
        HAL_Delay(2); // Additional settle time
    }

    /* 3. Send 10 µs trigger pulse */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_SET);

    // Precise 10 µs delay using timer
    start_time = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start_time) < 10);

    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_RESET);

    /* 4. Wait for ECHO to go HIGH (start of the pulse) */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - timeout_start > 100) { // Increased timeout
            return 999.0f; // No echo received
        }
    }

    /* 5. Start measuring – reset counter exactly at rising edge */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 6. Wait until ECHO goes LOW (end of the pulse) */
    timeout_start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - timeout_start > 100) { // Increased timeout
            return 999.0f; // Echo stuck high
        }
    }

    /* 7. Read pulse width in micro-seconds */
    pulse_duration = __HAL_TIM_GET_COUNTER(&htim2);

    /* 8. Check for valid pulse duration */
    if (pulse_duration < 100 || pulse_duration > 30000) { // 1.7cm to 500cm range
        return 999.0f; // Invalid pulse duration
    }

    /* 9. Convert to distance (cm) – speed of sound 343 m/s = 0.0343 cm/µs */
    float distance_cm = (pulse_duration * 0.0343f) / 2.0f;

    /* 10. Final sanity check */
    if (distance_cm < 1.0f || distance_cm > 500.0f) {
        return 999.0f;
    }

    return distance_cm;
}

/**
 * @brief Filtered distance reading - multiple readings to prevent false positives
 * @return Average distance (cm), returns 999.0 on error
 */
float ultrasonic_read_distance_filtered(void) {
    float readings[SENSOR_READINGS_COUNT];
    int valid_count = 0;
    float total = 0.0f;

    // Take multiple readings
    for (int i = 0; i < SENSOR_READINGS_COUNT; i++) {
        readings[i] = ultrasonic_read_distance();
        HAL_Delay(20); // Increased wait time for sensor stabilization

        // Record if reading is valid
        if (readings[i] < 999.0f && readings[i] > 0.5f && readings[i] < 450.0f) {
            valid_count++;
        }
    }

    // Average if at least 1 valid reading exists
    if (valid_count > 0) {
        // Apply median filter (for more stable results)
        for (int i = 0; i < SENSOR_READINGS_COUNT; i++) {
            if (readings[i] < 999.0f) {
                total += readings[i];
            }
        }
        return total / valid_count;
    }

    return 999.0f; // No valid readings
}

/**
 * @brief Checks if distance value is valid
 */
bool is_valid_distance(float distance) {
    return (distance >= MIN_DETECTION_DISTANCE &&
            distance <= MAX_DETECTION_DISTANCE);
}

/**
 * @brief Transition to scanning mode
 */
void state_change_to_scanning(void) {
    current_state = STATE_SCANNING;

    // Set LEDs
    led_control(false, true); // Green LED on, red LED off
    buzzer_control(false);    // Buzzer off// LCD update
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Radar Scanning");
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("Area Safe");

    system_reset_counters();
}

/**
 * @brief Transition to warning mode
 */
void state_change_to_warning(void) {current_state = STATE_WARNING;    // LCD update
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("*** WARNING ***");
    safe_lcd_print(1, "Dist:%.1fcm", last_detection.distance);    last_warning_time = get_system_time();
    warning_led_state = false;
    warning_blink_count = 0;  // Reset blink counter
}

/**
 * @brief Transition to object detected mode
 */
void state_change_to_object_detected(void) {
    current_state = STATE_OBJECT_DETECTED;

    // Save detection information
    last_detection.distance = current_distance;
    last_detection.angle = current_angle;last_detection.timestamp = get_system_time();    // LCD update
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("OBJECT DETECTED");
    safe_lcd_print(1, "%.1fcm %ddeg", last_detection.distance, last_detection.angle);
}

/**
 * @brief Scanning mode system update - Improved error handling
 */
void system_update_scanning(void) {
    uint32_t current_time = get_system_time();

    // Update servo motor
    servo_update();

    // Distance reading - more frequent measurement
    if (current_time - last_distance_time >= DISTANCE_READ_INTERVAL) {
        last_distance_time = current_time;

        // Measure distance at every 5 degree angle (more frequent scanning)
        if (current_angle % 5 == 0) {
            // Multiple attempts for reliability
            float raw_distance = 999.0f;
            int attempts = 0;

            // Try up to 3 times to get a valid reading
            while (raw_distance >= 999.0f && attempts < 3) {
                raw_distance = ultrasonic_read_distance();
                attempts++;
                if (raw_distance >= 999.0f && attempts < 3) {
                    HAL_Delay(10); // Small delay between attempts
                }
            }

            current_distance = raw_distance;

            // Enhanced debug output
            char debug_msg[120];            sprintf(debug_msg, "Angle: %d°, Distance: %.2f cm, Attempts: %d, Valid: %s\r\n",
                    current_angle, raw_distance, attempts,
                    (raw_distance < 999.0f) ? "YES" : "NO");
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 1000);

            if (is_valid_distance(current_distance) &&
                current_distance < PROXIMITY_ALARM_DISTANCE) {
                trigger_count++;

                if (trigger_count >= TRIGGER_THRESHOLD) {
                    state_change_to_object_detected();
                    return;
                }
            } else {
                trigger_count = 0;
            }            // LCD display angle and distance info - safe display
            if (current_time - last_lcd_update >= 300) {
                last_lcd_update = current_time;                if (raw_distance >= 999.0f) {
                    safe_lcd_print(1, "A:%3d ERR(%d)", current_angle, attempts);
                } else {
                    safe_lcd_print(1, "A:%3d %.1fcm", current_angle, raw_distance);
                }
            }
        }
    }
}

/**
 * @brief Warning mode system update
 */
void system_update_warning(void) {
    uint32_t current_time = get_system_time();

    // LED and buzzer blinking - switch to scanning mode after 4 blinks
    if (current_time - last_warning_time >= WARNING_BLINK_INTERVAL) {
        last_warning_time = current_time;
        warning_led_state = !warning_led_state;

        // Increment counter when LED turns on (i.e., every complete blink cycle)
        if (warning_led_state) {
            warning_blink_count++;
        }

        led_control(warning_led_state, false);
        buzzer_control(warning_led_state);

        // Switch to scanning mode after 4 blinks
        if (warning_blink_count >= 4 && !warning_led_state) {
            // Make transition after last blink is complete (when LED is off)
            state_change_to_scanning();
            return;
        }
    }
}

/**
 * @brief Object detected mode system update
 */
void system_update_object_detected(void) {
    uint32_t current_time = get_system_time();

    // Red LED steady on, buzzer off
    led_control(true, false);
    buzzer_control(false);

    // Switch to warning mode after 3 seconds
    if (current_time - last_detection.timestamp >= 3000) {
        state_change_to_warning();
    }
}

/**
 * @brief LED control
 */
void led_control(bool red_state, bool green_state) {
    HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, red_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GREEN_LED_PIN, green_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Buzzer control
 */
void buzzer_control(bool state) {
    HAL_GPIO_WritePin(GPIOC, BUZZER_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Reset system counters
 */
void system_reset_counters(void) {
    trigger_count = 0;
    clear_count = 0;
}

/**
 * @brief Get system time
 */
uint32_t get_system_time(void) {
    return HAL_GetTick();
}

/**
 * @brief Print distance information to UART for debugging
 */
void debug_print_distance(float distance, uint8_t angle) {
    char debug_buffer[64];
    sprintf(debug_buffer, "Angle: %d°, Distance: %.2f cm\r\n", angle, distance);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 1000);
}

/**
 * @brief Safe LCD print function to prevent buffer overflow
 */
void safe_lcd_print(uint8_t row, const char* format, ...) {
    char temp_buffer[32];
    va_list args;
    va_start(args, format);

    // Format to temporary buffer
    vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);

    // Ensure string doesn't exceed LCD width
    temp_buffer[16] = '\0';

    // Clear the row and print
    HD44780_SetCursor(0, row);
    HD44780_PrintStr("                "); // Clear with spaces
    HD44780_SetCursor(0, row);
    HD44780_PrintStr(temp_buffer);
}

/**
 * @brief Ultrasonic sensor test function - Enhanced diagnostics
 */
void test_ultrasonic_sensor(void) {
    char test_msg[150];
    sprintf(test_msg, "\r\n=== ENHANCED ULTRASONIC SENSOR TEST ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    // Test 1: Timer functionality
    sprintf(test_msg, "Test 1: Timer functionality\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    for (int i = 0; i < 5; i++) {
        uint32_t timer_before = __HAL_TIM_GET_COUNTER(&htim2);
        HAL_Delay(10); // 10ms delay
        uint32_t timer_after = __HAL_TIM_GET_COUNTER(&htim2);

        sprintf(test_msg, "Timer %d: Before=%lu, After=%lu, Diff=%lu (Expected ~10000)\r\n",
                i, timer_before, timer_after, timer_after - timer_before);
        HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);
    }

    // Test 2: GPIO functionality
    sprintf(test_msg, "\r\nTest 2: GPIO functionality\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    // Test TRIG pin
    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    sprintf(test_msg, "TRIG LOW, ECHO state: %s\r\n",
            HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    sprintf(test_msg, "TRIG HIGH, ECHO state: %s\r\n",
            HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    HAL_GPIO_WritePin(GPIOC, TRIG_PIN, GPIO_PIN_RESET);

    // Test 3: Distance measurements
    sprintf(test_msg, "\r\nTest 3: Distance measurements\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    float distances[10];
    int valid_count = 0;

    for (int i = 0; i < 10; i++) {
        distances[i] = ultrasonic_read_distance();
        if (distances[i] < 999.0f) {
            valid_count++;
        }

        sprintf(test_msg, "Test %d: Distance = %.2f cm %s\r\n",
                i+1, distances[i], (distances[i] < 999.0f) ? "(VALID)" : "(ERROR)");
        HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

        HAL_Delay(500);
    }

    // Test summary
    sprintf(test_msg, "\r\n=== TEST SUMMARY ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    sprintf(test_msg, "Valid readings: %d/10 (%.1f%%)\r\n",
            valid_count, (valid_count * 100.0f) / 10.0f);
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

    if (valid_count > 0) {
        float min_dist = 999.0f, max_dist = 0.0f, avg_dist = 0.0f;
        for (int i = 0; i < 10; i++) {
            if (distances[i] < 999.0f) {
                if (distances[i] < min_dist) min_dist = distances[i];
                if (distances[i] > max_dist) max_dist = distances[i];
                avg_dist += distances[i];
            }
        }
        avg_dist /= valid_count;

        sprintf(test_msg, "Range: %.2f cm to %.2f cm, Average: %.2f cm\r\n",
                min_dist, max_dist, avg_dist);
        HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);
    }

    sprintf(test_msg, "=== TEST COMPLETED ===\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);
}

/**
 * @brief Hardware diagnostic check function
 */
void hardware_diagnostic_check(void) {
    char diag_msg[120];
    sprintf(diag_msg, "\r\n=== HARDWARE DIAGNOSTIC CHECK ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    // Check system clock
    sprintf(diag_msg, "System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);    // Check timer status and counter
    if (htim2.Instance->CR1 & TIM_CR1_CEN) {
        uint32_t counter_val = __HAL_TIM_GET_COUNTER(&htim2);
        sprintf(diag_msg, "TIM2: Running ✓ (Counter: %lu)\r\n", counter_val);
    } else {
        sprintf(diag_msg, "TIM2: NOT RUNNING ✗\r\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    // Check timer prescaler and period
    sprintf(diag_msg, "TIM2 Prescaler: %u, Period: %lu\r\n",
            htim2.Init.Prescaler, htim2.Init.Period);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    // Check GPIO configuration
    GPIO_TypeDef* port = GPIOC;

    // Check PC0 (TRIG) configuration
    uint32_t pc0_mode = (port->MODER >> (0 * 2)) & 0x3;
    sprintf(diag_msg, "PC0 (TRIG) Mode: %s\r\n",
            (pc0_mode == 1) ? "Output ✓" : "NOT Output ✗");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    // Check PC1 (ECHO) configuration
    uint32_t pc1_mode = (port->MODER >> (1 * 2)) & 0x3;
    sprintf(diag_msg, "PC1 (ECHO) Mode: %s\r\n",
            (pc1_mode == 0) ? "Input ✓" : "NOT Input ✗");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);
      // Test continuous ECHO reading for 3 seconds
    sprintf(diag_msg, "ECHO pin monitor (3 seconds):\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    uint32_t start_monitor = HAL_GetTick();
    GPIO_PinState last_state = HAL_GPIO_ReadPin(GPIOC, ECHO_PIN);
    int state_changes = 0;

    while ((HAL_GetTick() - start_monitor) < 3000) {
        GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOC, ECHO_PIN);
        if (current_state != last_state) {
            state_changes++;
            sprintf(diag_msg, "ECHO: %s->%s at %lu ms\r\n",
                    last_state ? "HIGH" : "LOW",
                    current_state ? "HIGH" : "LOW",
                    HAL_GetTick() - start_monitor);
            HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);
            last_state = current_state;
        }
        HAL_Delay(1);
    }

    sprintf(diag_msg, "ECHO state changes detected: %d\r\n", state_changes);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    sprintf(diag_msg, "Final ECHO state: %s\r\n",
            HAL_GPIO_ReadPin(GPIOC, ECHO_PIN) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);

    sprintf(diag_msg, "=== DIAGNOSTIC COMPLETED ===\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), 1000);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // LCD initialization
  HD44780_Init(2);
  HD44780_Backlight();

  // Start timers
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);  // Startup screen
  HD44780_Clear();
  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("STM32 Radar Guard");
  HD44780_SetCursor(0, 1);
  HD44780_PrintStr("Starting...");
  HAL_Delay(2000);

  // Hardware diagnostic check (for debugging)
  hardware_diagnostic_check();

  // Ultrasonic sensor test (for debugging)
  test_ultrasonic_sensor();

  // System startup
  state_change_to_scanning();

  // Set initial times
  last_servo_time = get_system_time();
  last_distance_time = get_system_time();
  last_lcd_update = get_system_time();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    // State machine based system update
    switch(current_state) {
      case STATE_SCANNING:
        system_update_scanning();
        break;

      case STATE_WARNING:
        system_update_warning();
        break;

      case STATE_OBJECT_DETECTED:
        system_update_object_detected();
        break;

      default:
        state_change_to_scanning();
        break;
    }    // Small delay (to reduce CPU load)
    HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 839;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */