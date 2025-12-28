//keypad + 7 segment+ green, red LED 
 
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdlib.h>
#include "lcd.h"
 
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
#define RED_LED_PIN 5
#define GREEN_LED_PIN 4
#define TM1_DIO_PIN 1
#define TM1_CLK_PIN 0
#define TM1_PORT GPIOC
 
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
uint8_t access_granted = 0;
 
void soft_delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 5000; i++) {
        __NOP();
    }
}
 
void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 5; i++) {
        __NOP();
    }
}
 
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
 
void Timer2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16000 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CNT = 0;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}
 
void Stop_Timer2(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
 
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;
 
    ROW_PORT->MODER &= ~((3 << (ROW1_PIN * 2)) | (3 << (ROW2_PIN * 2)) | (3 << (ROW3_PIN * 2)) | (3 << (ROW4_PIN * 2)));
    ROW_PORT->MODER |= ((1 << (ROW1_PIN * 2)) | (1 << (ROW2_PIN * 2)) | (1 << (ROW3_PIN * 2)) | (1 << (ROW4_PIN * 2)));
 
    COL_PORT->MODER &= ~((3 << (COL1_PIN * 2)) | (3 << (COL2_PIN * 2)) | (3 << (COL3_PIN * 2)) | (3 << (COL4_PIN * 2)));
    COL_PORT->PUPDR &= ~((3 << (COL1_PIN * 2)) | (3 << (COL2_PIN * 2)) | (3 << (COL3_PIN * 2)) | (3 << (COL4_PIN * 2)));
    COL_PORT->PUPDR |= ((1 << (COL1_PIN * 2)) | (1 << (COL2_PIN * 2)) | (1 << (COL3_PIN * 2)) | (1 << (COL4_PIN * 2)));
 
    GPIOA->MODER &= ~((3 << (RED_LED_PIN * 2)) | (3 << (GREEN_LED_PIN * 2)));
    GPIOA->MODER |= ((1 << (RED_LED_PIN * 2)) | (1 << (GREEN_LED_PIN * 2)));
 
    GPIOA->ODR &= ~((1 << RED_LED_PIN) | (1 << GREEN_LED_PIN));
 
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
    if (!access_granted) {
        system_random_code = (rand() % 9000) + 1000;
        TM1_display_number(system_random_code);
    }
}
 
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        update_random_code();
    }
}
 
uint32_t get_adc_random_seed(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3 = 0; // Channel 0 (floating)
    ADC1->CR2 |= ADC_CR2_ADON;
    soft_delay_ms(10);
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint32_t seed = ADC1->DR;
    ADC1->CR2 &= ~ADC_CR2_ADON;
    return seed;
}
 
int main(void) {
    disable_JTAG_enable_SWD();
    Timer2_Init();
    GPIO_Init();
    lcd_gpio_init();
    lcd_init();
 
    uint32_t seed = get_adc_random_seed();
    srand(seed);
 
    lcd(0x80, 0);
    lcd_string("Welcome");
    soft_delay_ms(200);
    lcd(0x01, 0);
    lcd(0x80, 0);
    lcd_string("Press * key ");
 
    update_random_code();
 
    uint8_t entered_code[4] = {0};
    uint8_t code_index = 0;
    uint8_t in_code_entry = 0;
 
    while (1) {
        char key = Keypad_Scan();
        if (!key) continue;
 
        if (key == '*') {
            lcd(0x01, 0);
            lcd(0x80, 0);
            lcd_string("Security Code:");
            lcd(0xC0, 0);
            code_index = 0;
            in_code_entry = 1;
        } else if (in_code_entry && key >= '0' && key <= '9') {
            if (code_index < 4) {
                entered_code[code_index++] = key;
                lcd(key, 1);
            }
        } else if (in_code_entry && key == '#') {
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
                    GPIOA->ODR |= (1 << GREEN_LED_PIN);
                    GPIOA->ODR &= ~(1 << RED_LED_PIN);
 
                    Stop_Timer2();
                    access_granted = 1;
                    TM1_display_number(0);
 
                    delay_us(1000000);
                    GPIOA->ODR &= ~(1 << GREEN_LED_PIN);
 
                    lcd(0x01, 0);
                    lcd(0x80, 0);
                    lcd_string("Green vibes only.");
                    delay_us(1000000);
 
                    lcd(0x01, 0);
                    lcd_string("System Live ");
                    delay_us(50000000);
 
                    break;
                } else {
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
                }
            } else {
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
            }
        }
    }
}

 
