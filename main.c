#include "peripherals.h"

int main(void) {
    Motor_Init();
    LCD_init();
    PWM_Init();
    ADC0_init();
    LED_init();
    UART0_init();
    NVIC_EnableIRQ(UART0_IRQn);
    keypad_init();
    EmergencyButton_init();
    NVIC_EnableIRQ(PORTA_IRQn);

    LCD_updateCounts();
    Motor_forward();

    while (1) {
        if(modo_manual){
            char key = keypad_getkey();
            int decoded = decoder_teclado(key);
            switch (decoded) {
                case 1: setServoAngle1(configTable[0

