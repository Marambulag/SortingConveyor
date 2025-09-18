#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "MKL25Z4.h"
#include <stdint.h>
#include <stdio.h>

// --- Defines ---
#define MAX_QUEUE_SIZE 25
#define ADC_CHANNEL 0       // ADC0_SE0 = PTE20
#define LED_PIN     18      // LED rojo = PTB18
#define THRESHOLD   400     // Umbral para detectar corte del láser

#define MOTOR_PWM_PIN     12  // PTA12 → TPM1_CH0
#define MOTOR_DIR1_PIN    10  // PTA10
#define MOTOR_DIR2_PIN    11  // PTA11

#define EMERGENCY_BUTTON_PIN 17  // PTA17
#define MANUAL_BUTTON_PIN 16     // PTA16

// --- Structs ---
typedef struct {
    int angle1;
    int angle2;
} ServoConfig;

// --- Variables externas ---
extern int c1, c2, c3, c4;
extern ServoConfig configTable[5];
extern uint8_t queue[MAX_QUEUE_SIZE];
extern int front, rear, count;
extern uint16_t last_adc_value;
extern uint8_t trigger_flag;
extern int modo_manual;

// --- Prototipos ---
// Utils
void delayMs(int n);

// UART
void UART0_init(void);
void UART0_IRQHandler(void);

// LED
void LED_init(void);

// ADC
void ADC0_init(void);
uint16_t readADC0(uint8_t channel);
void ADC0_IRQHandler(void);

// PWM y Servos
void PWM_Init(void);
void setServoAngle1(int angle);
void setServoAngle2(int angle);

// Motor
void Motor_Init(void);
void Motor_setSpeed(uint8_t duty);
void Motor_forward(void);
void Motor_reverse(void);
void Motor_stop(void);

// LCD
void LCD_init(void);
void LCD_command(uint8_t cmd);
void LCD_command_noWait(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_nibble(uint8_t nibble);
void LCD_updateCounts(void);

// Keypad
void keypad_init(void);
char keypad_getkey(void);
int decoder_teclado(char input);

// Cola
void enqueue(uint8_t config);
void dequeue(void);
uint8_t peek(void);

// Botones
void EmergencyButton_init(void);
void PORTA_IRQHandler(void);

#endif

