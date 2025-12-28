//DHT_11 + LDR _
// DHT11 + Relay + motor driver
 
#include "stm32f4xx.h"
#include "lcd.h"
 
// DHT11 Configuration
#define DHT11_PORT GPIOC
#define DHT11_PIN 3
 
// Motor (Fan) Configuration
#define MOTOR_GPIO GPIOA
#define AIN1_PIN  3   // PA3 -> TIM2_CH4
#define AIN2_PIN  2   // PA2 always LOW
 
#define TIM2_PSC_VALUE  84 - 1  // 1 MHz PWM clock
#define TIM2_ARR_VALUE  1000 - 1 // PWM frequency ~1kHz
 
// Relay (Bulb) Configuration
#define RELAY_GPIO GPIOA
#define RELAY_PIN 8
 
// Control Thresholds
#define TEMP_THRESHOLD 30    // X°C -> Fan ON if Temp >= 30°C
#define HUM_THRESHOLD 60    // Arbitrary humidity threshold -> Bulb ON if Hum >= 60%
 
// LED Pins
#define LED1_PORT GPIOC
#define LED1_PIN 6
#define LED2_PORT GPIOC
#define LED2_PIN 7
#define LED3_PORT GPIOC
#define LED3_PIN 8
#define LED4_PORT GPIOC
#define LED4_PIN 12
#define LED5_PORT GPIOD
#define LED5_PIN 2
 
// LDR ADC Input
#define LDR_PORT GPIOA
#define LDR_PIN  6  // ADC1_IN6
 
// ========= LED STRUCTURE =========
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
 
// Function Prototypes
void delay_us(uint32_t us);
void SysTick_Init(void);
void DHT11_init(void);
void DHT11_start(void);
uint8_t DHT11_check_response(void);
uint8_t DHT11_read_byte(void);
void display_status(uint8_t temperature, uint8_t humidity);
void GPIO_Motor_Init(void);
void TIM2_PWM_Init(void);
void Motor_SetSpeed(uint16_t speed_percent);
void GPIO_Relay_Init(void);
void Relay_ON(void);
void Relay_OFF(void);
void Delay_ms(uint32_t ms);
void gpio_init(void);
void adc_init(void);
uint16_t read_adc_pa6(void);
void set_led_level(uint8_t level);
void turn_off_leds(void);
 
// Delay microseconds using SysTick
void delay_us(uint32_t us) {
   SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
   SysTick->VAL = 0;
   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
 
   while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
   SysTick->CTRL = 0;
}
 
// SysTick Basic Init
void SysTick_Init(void) {
   SysTick->CTRL = 0;
   SysTick->LOAD = 0;
   SysTick->VAL = 0;
}
 
// DHT11 Pin Mode Control
void DHT11_set_pin_output(void) {
   DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));
   DHT11_PORT->MODER |= (1 << (DHT11_PIN * 2));
}
 
void DHT11_set_pin_input(void) {
   DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));
}
 
void DHT11_init(void) {
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  DHT11_set_pin_input();
   SysTick_Init();
 
   lcd_gpio_init();
   lcd_init();
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
 
   while (DHT11_PORT->IDR & (1 << DHT11_PIN)) {
       delay_us(1);
       if (++timeout > 250) return 0;
   }
 
   timeout = 0;
   while (!(DHT11_PORT->IDR & (1 << DHT11_PIN))) {
       delay_us(1);
       if (++timeout > 250) return 0;
   }
 
   timeout = 0;
   while (DHT11_PORT->IDR & (1 << DHT11_PIN)) {
       delay_us(1);
       if (++timeout > 250) return 0;
   }
 
   return 1;
}
 
uint8_t DHT11_read_byte(void) {
   uint8_t byte = 0;
 
   for (int i = 0; i < 8; i++) {
       uint32_t timeout = 0;
 
       while (!(DHT11_PORT->IDR & (1 << DHT11_PIN))) {
          delay_us(1);
           if (++timeout > 250) break;
       }
 
       delay_us(40);
 
       if (DHT11_PORT->IDR & (1 << DHT11_PIN)) {
           byte |= (1 << (7 - i));
       }
 
       timeout = 0;
       while (DHT11_PORT->IDR & (1 << DHT11_PIN)) {
          delay_us(1);
           if (++timeout > 250) break;
       }
   }
 
   return byte;
}
 
// Display temperature, humidity, and system status
void display_status(uint8_t temperature, uint8_t humidity) {
   lcd(0x01, 0); // Clear display
 
   lcd_string("Temp: ");
  single_print(temperature);
   lcd_string("C ");
 
   if (temperature >= TEMP_THRESHOLD) {
       lcd_string("Mild Hot");
   } else {
       lcd_string("Cool");
   }
 
   lcd(0xC0, 0); // Second line
   lcd_string("Hum: ");
  single_print(humidity);
   lcd_string("% ");
}
 
// Motor (Fan) Initialization
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
 
 
void TIM2_PWM_Init(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
 
   TIM2->PSC = TIM2_PSC_VALUE;
   TIM2->ARR = TIM2_ARR_VALUE;
 
   TIM2->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
   TIM2->CCER |= TIM_CCER_CC4E;
   TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}
 
void Motor_SetSpeed(uint16_t speed_percent) {
   if (speed_percent > 100) speed_percent = 100;
 
   uint32_t pwm_value = (TIM2->ARR + 1) * speed_percent / 100;
   TIM2->CCR4 = pwm_value;
}
 
// Relay (Bulb) Initialization
void GPIO_Relay_Init(void) {
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
 
   RELAY_GPIO->MODER &= ~(0x3 << (RELAY_PIN * 2));
   RELAY_GPIO->MODER |= (0x1 << (RELAY_PIN * 2)); // Output mode
}
 
void Relay_ON(void) {
   RELAY_GPIO->BSRR = (1 << RELAY_PIN);
}
 
void Relay_OFF(void) {
   RELAY_GPIO->BSRR = (1 << (RELAY_PIN + 16));
}
 
// Crude millisecond delay
void Delay_ms(uint32_t ms) {
   for (uint32_t i = 0; i < ms * 2000; i++) {
       __NOP();
   }
}
 
void turn_off_leds(void)
{
   for (uint8_t i = 0; i < 5; i++) {
       led_pins[i].port->BSRR = (1 << (led_pins[i].pin + 16));  // Reset bit
   }
}
 
void set_led_level(uint8_t level)
{
   for (uint8_t i = 0; i < 5; i++) {
       if (i < level)
          led_pins[i].port->BSRR = (1 << led_pins[i].pin);           // ON
       else
          led_pins[i].port->BSRR = (1 << (led_pins[i].pin + 16));    // OFF
   }
}
 
// ========= GPIO CONFIGURATION =========
void gpio_init(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
 
   // PC6, PC7, PC8, PC12 as output
   GPIOC->MODER &= ~((3 << (LED1_PIN * 2)) | (3 << (LED2_PIN * 2)) |
                    (3 << (LED3_PIN * 2)) | (3 << (LED4_PIN * 2)));
   GPIOC->MODER |=  ((1 << (LED1_PIN * 2)) | (1 << (LED2_PIN * 2)) |
                    (1 << (LED3_PIN * 2)) | (1 << (LED4_PIN * 2)));
   GPIOC->OTYPER &= ~((1 << LED1_PIN) | (1 << LED2_PIN) |
                     (1 << LED3_PIN) | (1 << LED4_PIN));
 
   // PD2 as output
   GPIOD->MODER &= ~(3 << (LED5_PIN * 2));
   GPIOD->MODER |=  (1 << (LED5_PIN * 2));
   GPIOD->OTYPER &= ~(1 << LED5_PIN);
}
 
// ========= ADC INITIALIZATION =========
void adc_init(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
 
   GPIOA->MODER |= (3 << (LDR_PIN * 2));    // Analog mode
   GPIOA->PUPDR &= ~(3 << (LDR_PIN * 2));   // No pull
 
   ADC->CCR = 0;
   ADC1->CR2 = 0;
   ADC1->SQR3 = LDR_PIN;
   ADC1->SMPR2 |= (7 << 18);
   ADC1->CR2 |= ADC_CR2_ADON;
}
 
// ========= ADC READ =========
uint16_t read_adc_pa6(void)
{
   ADC1->CR2 |= ADC_CR2_SWSTART;
   while (!(ADC1->SR & ADC_SR_EOC));
   return ADC1->DR;
}
 
// ========= DELAY =========
void delay_ms(uint32_t ms)
{
   for (volatile uint32_t i = 0; i < ms * 4000; i++) {
       __NOP();
   }
}
 
int main(void) {
   uint8_t Rh_int, Rh_dec, Temp_int, Temp_dec, checksum;
 
   DHT11_init();
   GPIO_Motor_Init();
   TIM2_PWM_Init();
   GPIO_Relay_Init();
   gpio_init();
              adc_init();
 
              uint16_t ldr_min = 4095;
              uint16_t ldr_max = 0;
              uint32_t startup_counter = 0;
              uint16_t light;
              uint16_t range;
              uint8_t level;
 
   while (1) {
              light = read_adc_pa6();
 
                             // Allow calibration only for first 20 seconds (~100 samples/sec)
                             if (startup_counter < 2000) {
                                            if (light < ldr_min) ldr_min = light;
                                            if (light > ldr_max) ldr_max = light;
                                            startup_counter++;
                             }
 
                             range = ldr_max - ldr_min;
                             if (range < 300) range = 300; // Artificial stretch to ensure mapping
 
                             // Map light to 0–5 LED level
                             level = 0;
                             if (light > ldr_min) {
                                            level = (uint8_t)((light - ldr_min) * 5 / range);
                                            if (level > 5) level = 5;
                             }
 
                             set_led_level(level);
                             delay_ms(200);
 
       DHT11_start();
 
       if (DHT11_check_response()) {
           Rh_int = DHT11_read_byte();
           Rh_dec = DHT11_read_byte();
           Temp_int = DHT11_read_byte();
           Temp_dec = DHT11_read_byte();
           checksum = DHT11_read_byte();
 
           if (checksum == (Rh_int + Rh_dec + Temp_int + Temp_dec)) {
              display_status(Temp_int, Rh_int);
 
               // Fan Control
               if (Temp_int >= TEMP_THRESHOLD) {
                  Motor_SetSpeed(40); // Run fan at 70% speed
               } else {
                  Motor_SetSpeed(0); // Stop fan
               }
 
               // Bulb (Relay) Control
               if (Rh_int >= HUM_THRESHOLD) {
                  Relay_ON();
               } else {
                  Relay_OFF();
               }
           } else {
              lcd(0x01, 0);
              lcd_string("Checksum Error");
           }
       } else {
           lcd(0x01, 0);
          lcd_string("Sensor Not Found");
       }
 
      Delay_ms(3000); // 3-second delay between readings
   }
}
