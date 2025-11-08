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
            case 1: setServoAngle1(configTable[0].angle1); setServoAngle2(configTable[0].angle2); break;
            case 2: setServoAngle1(configTable[1].angle1); setServoAngle2(configTable[1].angle2); break;
            case 3: setServoAngle1(configTable[2].angle1); setServoAngle2(configTable[2].angle2); break;
            case 4: setServoAngle1(configTable[3].angle1); setServoAngle2(configTable[3].angle2); break;
            case 5: Motor_setSpeed(80); break;
            case 6: modo_manual = 0;
        }
    }
    else{
        uint8_t bin = peek();
        ServoConfig cfg = configTable[bin];
        setServoAngle1(cfg.angle1);
        setServoAngle2(cfg.angle2);
    }
}

