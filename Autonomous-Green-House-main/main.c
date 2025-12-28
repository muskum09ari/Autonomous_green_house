//Final Code - Modified with proper timer integration

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdlib.h>
#include "lcd.h"

// === Pin Definitions ===
#define DHT11_PORT GPIOC
#define DHT11_PIN  3

#define MOTOR_GPIO GPIOA
#define AIN1_PIN   3
#define AIN2_PIN   2

#define RELAY_GPIO GPIOA
#define RELAY_PIN  8

#define LDR_PORT  GPIOA
#define LDR_PIN   6  // ADC1_IN6

#define LED1_PORT GPIOC
#define LED1_PIN  6
#define LED2_PORT GPIOC
#define LED2_PIN  7
#define LED3_PORT GPIOC
#define LED3_PIN  8
#define LED4_PORT GPIOC
#define LED4_PIN  12
#define LED5_PORT GPIOD
#define LED5_PIN  2

#define TEMP_THRESHOLD 25
#define HUM_THRESHOLD  75

#define ROW1_PIN 2
#define ROW2_PIN 3
#define ROW3_PIN 4
#define ROW4_PIN 5
#define COL1_PIN 6
#define COL2_PIN 7
#define COL3_PIN 8
#define COL4_PIN 9
#define ROW_PORT GPIOB
#define COL_PORT GPIOB

#define RED_LED_PIN 10
#define GREEN_LED_PIN 11

#define TM1_DIO_PIN 1
#define TM1_CLK_PIN 0
#define TM1_PORT GPIOC

// === Structures ===
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} LedPin;

LedPin led_pins[5] = {
    {LED1_PORT, LED1_PIN},
    {LED2_PORT, LED2_PIN},
    {LED3_PORT, LED3_PIN},
    {LED4_PORT, LED4_PIN},
    {LED5_PORT, LED5_PIN}
};

const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static uint8_t segment_map[10] = {
    0x3F, 0x06, 0x5B, 0x4F,
    0x66, 0x6D, 0x7D, 0x07,
    0x7F, 0x6F
};

uint16_t system_random_code = 0;
volatile uint8_t access_granted = 0;
volatile uint8_t update_code_flag = 0;
volatile uint8_t keypad_input_active = 0;  // Flag to prevent code updates during input

// === Delay Functions ===
void delay_us(uint32_t us) {
    SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick->CTRL = 0;
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 4000; i++) __NOP();
}

void soft_delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 5000; i++) {
        __NOP();
    }
}

// === GPIO & Peripherals Init ===
void gpio_init_leds(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    GPIOC->MODER |= (1 << (LED1_PIN * 2)) | (1 << (LED2_PIN * 2)) |
                    (1 << (LED3_PIN * 2)) | (1 << (LED4_PIN * 2));
    GPIOD->MODER |= (1 << (LED5_PIN * 2));
}

void adc_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (3 << (LDR_PIN * 2));
    ADC->CCR = 0;
    ADC1->CR2 = 0;
    ADC1->SQR3 = LDR_PIN;
    ADC1->SMPR2 |= (7 << 18);
    ADC1->CR2 |= ADC_CR2_ADON;
}

void GPIO_Motor_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA2 -> Direction (always LOW for clockwise rotation)
    MOTOR_GPIO->MODER &= ~(3 << (AIN2_PIN * 2));
    MOTOR_GPIO->MODER |= (1 << (AIN2_PIN * 2));   // Set PA2 as output
    MOTOR_GPIO->BSRR = (1 << (AIN2_PIN + 16));     // Set PA2 LOW (direction control)

    // PA3 -> PWM Output (AF for controlling speed)
    MOTOR_GPIO->MODER &= ~(3 << (AIN1_PIN * 2));  // Clear bits for PA3
    MOTOR_GPIO->MODER |= (2 << (AIN1_PIN * 2));   // Set PA3 to AF mode
    MOTOR_GPIO->AFR[0] &= ~(0xF << (AIN1_PIN * 4));
    MOTOR_GPIO->AFR[0] |= (1 << (AIN1_PIN * 4));  // Use TIM2 for PWM control
}

// === TIM2 PWM Functions (Motor Control Only) ===
void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 84 - 1;           // 84MHz / 84 = 1MHz
    TIM2->ARR = 1000 - 1;         // 1kHz PWM frequency
    TIM2->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;  // PWM mode 1
    TIM2->CCER |= TIM_CCER_CC4E;  // Enable channel 4 output
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;  // Enable auto-reload and counter
}

void Motor_SetSpeed(uint16_t speed_percent) {
    if (speed_percent > 100) speed_percent = 100;
    uint32_t pwm_value = (TIM2->ARR + 1) * speed_percent / 100;
    TIM2->CCR4 = pwm_value;
}

// === TIM3 Timer Functions (Random Code Generation) ===
void Timer3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 16000 - 1;       // 16MHz / 16000 = 1kHz (1ms tick)
    TIM3->ARR = 20000 - 1;       // 20s = 20000 * 1ms
    TIM3->CNT = 0;
    TIM3->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM3->CR1 |= TIM_CR1_CEN;    // Enable counter
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;  // Clear interrupt flag

        // Only update code if access is not granted and no keypad input is active
        if (!access_granted && !keypad_input_active) {
            update_code_flag = 1;
        }
    }
}

void stop_code_generation_timer(void) {
    TIM3->CR1 &= ~TIM_CR1_CEN;  // Stop TIM3
    NVIC_DisableIRQ(TIM3_IRQn);
}

void restart_code_generation_timer(void) {
    TIM3->CNT = 0;              // Reset counter
    TIM3->CR1 |= TIM_CR1_CEN;   // Start TIM3
    NVIC_EnableIRQ(TIM3_IRQn);
}

// === TM1637 Display Functions ===
void TM1_start(void) {
    TM1_PORT->ODR |= (1 << TM1_CLK_PIN) | (1 << TM1_DIO_PIN);
    delay_us(10);
    TM1_PORT->ODR &= ~(1 << TM1_DIO_PIN);
    delay_us(10);
}

void TM1_stop(void) {
    TM1_PORT->ODR &= ~(1 << TM1_CLK_PIN);
    TM1_PORT->ODR &= ~(1 << TM1_DIO_PIN);
    delay_us(10);
    TM1_PORT->ODR |= (1 << TM1_CLK_PIN);
    delay_us(10);
    TM1_PORT->ODR |= (1 << TM1_DIO_PIN);
}

void TM1_write_byte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        TM1_PORT->ODR &= ~(1 << TM1_CLK_PIN);
        if (b & 0x01) TM1_PORT->ODR |= (1 << TM1_DIO_PIN);
        else TM1_PORT->ODR &= ~(1 << TM1_DIO_PIN);
        delay_us(10);
        TM1_PORT->ODR |= (1 << TM1_CLK_PIN);
        delay_us(10);
        b >>= 1;
    }
    TM1_PORT->ODR &= ~(1 << TM1_CLK_PIN);
    TM1_PORT->ODR |= (1 << TM1_DIO_PIN);
    delay_us(10);
    TM1_PORT->ODR |= (1 << TM1_CLK_PIN);
    delay_us(10);
    TM1_PORT->ODR &= ~(1 << TM1_CLK_PIN);
}

void TM1_display_number(uint16_t num) {
    uint8_t d[4] = {
        segment_map[(num / 1000) % 10],
        segment_map[(num / 100) % 10],
        segment_map[(num / 10) % 10],
        segment_map[num % 10]
    };
    TM1_start();
    TM1_write_byte(0x40);
    TM1_stop();

    TM1_start();
    TM1_write_byte(0xC0);
    for (int i = 0; i < 4; i++) TM1_write_byte(d[i]);
    TM1_stop();

    TM1_start();
    TM1_write_byte(0x88 | 0x07);
    TM1_stop();
}

void GPIO_Relay_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RELAY_GPIO->MODER |= (1 << (RELAY_PIN * 2));
}

// === Helper Functions ===
uint16_t read_adc_pa6(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void Relay_ON(void) {
    RELAY_GPIO->BSRR = (1 << RELAY_PIN);
}

void Relay_OFF(void) {
    RELAY_GPIO->BSRR = (1 << (RELAY_PIN + 16));
}

void set_led_level(uint8_t level) {
    for (uint8_t i = 0; i < 5; i++) {
        if (i < level)
            led_pins[i].port->BSRR = (1 << led_pins[i].pin);
        else
            led_pins[i].port->BSRR = (1 << (led_pins[i].pin + 16));
    }
}

// === DHT11 Section ===
void DHT11_set_pin_output(void) {
    DHT11_PORT->MODER |= (1 << (DHT11_PIN * 2));
}

void DHT11_set_pin_input(void) {
    DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));
}

void DHT11_start(void) {
    DHT11_set_pin_output();
    DHT11_PORT->ODR &= ~(1 << DHT11_PIN);
    delay_us(20000);
    DHT11_PORT->ODR |= (1 << DHT11_PIN);
    delay_us(30);
    DHT11_set_pin_input();
}

uint8_t DHT11_check_response(void) {
    uint32_t timeout = 0;
    while (DHT11_PORT->IDR & (1 << DHT11_PIN)) { delay_us(1); if (++timeout > 250) return 0; }
    timeout = 0;
    while (!(DHT11_PORT->IDR & (1 << DHT11_PIN))) { delay_us(1); if (++timeout > 250) return 0; }
    timeout = 0;
    while (DHT11_PORT->IDR & (1 << DHT11_PIN)) { delay_us(1); if (++timeout > 250) return 0; }
    return 1;
}

uint8_t DHT11_read_byte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        while (!(DHT11_PORT->IDR & (1 << DHT11_PIN)));
        delay_us(40);
        if (DHT11_PORT->IDR & (1 << DHT11_PIN)) byte |= (1 << (7 - i));
        while (DHT11_PORT->IDR & (1 << DHT11_PIN));
    }
    return byte;
}

void display_status(uint8_t temperature, uint8_t humidity) {
    lcd(0x01, 0);
    lcd_string("Temp: ");
    single_print(temperature);
    lcd_string("C ");
    lcd_string((temperature >= TEMP_THRESHOLD) ? "Mild Hot" : "Cool");
    lcd(0xC0, 0);
    lcd_string("Hum: ");
    single_print(humidity);
    lcd_string("% ");
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;

    // Keypad rows as outputs
    ROW_PORT->MODER &= ~((3 << (ROW1_PIN * 2)) | (3 << (ROW2_PIN * 2)) | (3 << (ROW3_PIN * 2)) | (3 << (ROW4_PIN * 2)));
    ROW_PORT->MODER |= ((1 << (ROW1_PIN * 2)) | (1 << (ROW2_PIN * 2)) | (1 << (ROW3_PIN * 2)) | (1 << (ROW4_PIN * 2)));

    // Keypad columns as inputs with pull-up
    COL_PORT->MODER &= ~((3 << (COL1_PIN * 2)) | (3 << (COL2_PIN * 2)) | (3 << (COL3_PIN * 2)) | (3 << (COL4_PIN * 2)));
    COL_PORT->PUPDR &= ~((3 << (COL1_PIN * 2)) | (3 << (COL2_PIN * 2)) | (3 << (COL3_PIN * 2)) | (3 << (COL4_PIN * 2)));
    COL_PORT->PUPDR |= ((1 << (COL1_PIN * 2)) | (1 << (COL2_PIN * 2)) | (1 << (COL3_PIN * 2)) | (1 << (COL4_PIN * 2)));

    // LED indicators
    GPIOA->MODER &= ~((3 << (RED_LED_PIN * 2)) | (3 << (GREEN_LED_PIN * 2)));
    GPIOA->MODER |= ((1 << (RED_LED_PIN * 2)) | (1 << (GREEN_LED_PIN * 2)));
    GPIOA->ODR &= ~((1 << RED_LED_PIN) | (1 << GREEN_LED_PIN));

    // TM1637 display pins
    GPIOC->MODER &= ~((3 << (TM1_CLK_PIN * 2)) | (3 << (TM1_DIO_PIN * 2)));
    GPIOC->MODER |= ((1 << (TM1_CLK_PIN * 2)) | (1 << (TM1_DIO_PIN * 2)));
}

void disable_JTAG_enable_SWD(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    DBGMCU->CR &= ~(1 << 0);
}

char Keypad_Scan(void) {
    uint16_t rowPins[4] = {ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN};
    uint16_t colPins[4] = {COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN};

    ROW_PORT->ODR |= (1 << ROW1_PIN) | (1 << ROW2_PIN) | (1 << ROW3_PIN) | (1 << ROW4_PIN);

    for (int r = 0; r < 4; r++) {
        ROW_PORT->ODR |= (1 << ROW1_PIN) | (1 << ROW2_PIN) | (1 << ROW3_PIN) | (1 << ROW4_PIN);
        ROW_PORT->ODR &= ~(1 << rowPins[r]);

        delay_us(50);

        uint16_t col_state = COL_PORT->IDR & ((1 << COL1_PIN) | (1 << COL2_PIN) | (1 << COL3_PIN) | (1 << COL4_PIN));
        if (col_state != ((1 << COL1_PIN) | (1 << COL2_PIN) | (1 << COL3_PIN) | (1 << COL4_PIN))) {
            for (int c = 0; c < 4; c++) {
                if (!(COL_PORT->IDR & (1 << colPins[c]))) {
                    delay_us(5000);
                    if (!(COL_PORT->IDR & (1 << colPins[c]))) {
                        while (!(COL_PORT->IDR & (1 << colPins[c])));
                        return keymap[r][c];
                    }
                }
            }
        }
    }
    return 0;
}

void update_random_code(void) {
    system_random_code = (rand() % 9000) + 1000;  // Generate 4-digit code (1000-9999)
    TM1_display_number(system_random_code);
}

uint32_t get_adc_random_seed(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3 = 0; // Channel 0 (floating)
    ADC1->SMPR2 |= (7 << 0); // Max sampling time for channel 0 (480 cycles)
    ADC1->CR2 |= ADC_CR2_ADON;
    soft_delay_ms(10);

    uint32_t seed = 0;
    for (int i = 0; i < 4; i++) { // Take multiple samples
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC));
        seed ^= (ADC1->DR << (i * 8)); // XOR samples to combine
        soft_delay_ms(1); // Small delay to allow noise variation
    }
    seed ^= SysTick->VAL; // Add SysTick for extra variability
    ADC1->CR2 &= ~ADC_CR2_ADON;
    return seed;
}

// === Main ===
int main(void) {
    uint16_t ldr_min = 4095, ldr_max = 0;
    uint32_t ldr_counter = 0;
    uint32_t last_dht_time = 0;

    // Initialize SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Initialize all peripherals
    gpio_init_leds();
    GPIO_Init();
    GPIO_Motor_Init();
    TIM2_PWM_Init();        // TIM2 for PWM only
    GPIO_Relay_Init();
    DHT11_set_pin_input();
    lcd_gpio_init();
    lcd_init();
    disable_JTAG_enable_SWD();
    Timer3_Init();          // TIM3 for random code generation

    // Initialize random number generator
    srand(get_adc_random_seed());
    update_random_code();

    // Welcome message
    lcd(0x80, 0);
    lcd_string("Welcome");
    soft_delay_ms(200);
    lcd(0x01, 0);
    lcd(0x80, 0);
    lcd_string("Press * key ");

    uint8_t entered_code[4] = {0};
    uint8_t code_index = 0;
    uint8_t in_code_entry = 0;

    while (1) {
        // Handle code update flag from timer interrupt
        if (update_code_flag) {
            update_code_flag = 0;
            update_random_code();
        }

        char key = Keypad_Scan();
        if (!key) continue;

        if (key == '*') {
            keypad_input_active = 1;  // Prevent code updates during input
            lcd(0x01, 0);
            lcd(0x80, 0);
            lcd_string("Security Code:");
            lcd(0xC0, 0);
            code_index = 0;
            in_code_entry = 1;
        }
        else if (in_code_entry && key >= '0' && key <= '9') {
            if (code_index < 4) {
                entered_code[code_index++] = key;
                lcd(key, 1);
            }
        }
        else if (in_code_entry && key == '#') {
            keypad_input_active = 0;  // Allow code updates again

            if (code_index == 4) {
                uint16_t user_code = (entered_code[0] - '0') * 1000 +
                                   (entered_code[1] - '0') * 100 +
                                   (entered_code[2] - '0') * 10 +
                                   (entered_code[3] - '0');

                lcd(0x01, 0);
                lcd(0x80, 0);

                uint16_t actual_password = system_random_code + 2;

                if (user_code == actual_password) {
                    lcd_string("Access Granted");
                    adc_init();

                    // Stop random code generation
                    stop_code_generation_timer();
                    access_granted = 1;
                    TM1_display_number(0);

                    // LED indication
                    GPIOA->ODR |= (1 << GREEN_LED_PIN);
                    GPIOA->ODR &= ~(1 << RED_LED_PIN);
                    delay_us(1000000);
                    GPIOA->ODR &= ~(1 << GREEN_LED_PIN);

                    lcd(0x01, 0);
                    lcd(0x80, 0);
                    lcd_string("Green vibes only.");
                    delay_us(1000000);

                    lcd(0x01, 0);
                    lcd_string("System Live ");
                    delay_us(1000000);

                    // Main system loop
                    while (1) {
                        // LDR LED Bar Logic
                        uint16_t light = read_adc_pa6();
                        if (ldr_counter < 2000) {
                            if (light < ldr_min) ldr_min = light;
                            if (light > ldr_max) ldr_max = light;
                            ldr_counter++;
                        }
                        uint16_t range = (ldr_max - ldr_min < 300) ? 300 : (ldr_max - ldr_min);
                        uint8_t level = (light > ldr_min) ? ((light - ldr_min) * 5 / range) : 0;
                        if (level > 5) level = 5;
                        set_led_level(level);
                        delay_ms(200);

                        // DHT11 Read Every 3 Seconds
                        if ((last_dht_time++ >= 15)) {  // 15 * 200ms = 3s
                            last_dht_time = 0;
                            DHT11_start();
                            if (DHT11_check_response()) {
                                uint8_t Rh_int = DHT11_read_byte();
                                uint8_t Rh_dec = DHT11_read_byte();
                                uint8_t Temp_int = DHT11_read_byte();
                                uint8_t Temp_dec = DHT11_read_byte();
                                uint8_t checksum = DHT11_read_byte();
                                if (checksum == (Rh_int + Rh_dec + Temp_int + Temp_dec)) {
                                    display_status(Temp_int, Rh_int);
                                    if (Temp_int >= TEMP_THRESHOLD) Motor_SetSpeed(40);
                                    else Motor_SetSpeed(0);
                                    if (Rh_int >= HUM_THRESHOLD) Relay_ON();
                                    else Relay_OFF();
                                } else {
                                    lcd(0x01, 0);
                                    lcd_string("Checksum Error");
                                }
                            } else {
                                lcd(0x01, 0);
                                lcd_string("Sensor Not Found");
                            }
                        }
                    }
                } else {
                    // Access denied
                    lcd_string("Access Denied!");
                    lcd(0xC0, 0);
                    lcd_string("Try Again!");

                    GPIOA->ODR |= (1 << RED_LED_PIN);
                    GPIOA->ODR &= ~(1 << GREEN_LED_PIN);
                    delay_us(500000);
                    GPIOA->ODR &= ~(1 << RED_LED_PIN);

                    lcd(0x01, 0);
                    lcd(0x80, 0);
                    lcd_string("Re-enter pass");
                    lcd(0xC0, 0);
                    code_index = 0;
                    in_code_entry = 1;  // Stay in code entry mode
                }
            } else {
                // Incomplete entry
                lcd(0x01, 0);
                lcd(0x80, 0);
                lcd_string("Incomplete Entry");
                lcd(0xC0, 0);
                lcd_string("Try Again!");
                delay_us(500000);

                lcd(0x01, 0);
                lcd(0x80, 0);
                lcd_string("Re-enter pass");
                lcd(0xC0, 0);
                code_index = 0;
                in_code_entry = 1;  // Stay in code entry mode
            }
        }
        else if (in_code_entry && key == '*') {
            // Reset code entry on * press
            keypad_input_active = 0;
            lcd(0x01, 0);
            lcd(0x80, 0);
            lcd_string("Press * key ");
            in_code_entry = 0;
            code_index = 0;
        }
    }
}
