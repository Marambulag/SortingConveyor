#include "peripherals.h"

// --- Variables globales ---
int c1 = -1, c2 = 0, c3 = 0, c4 = 0;

ServoConfig configTable[5] = {
    {10, 125},    // 00
    {30, 140},    // 01
    {45, 152},    // 10
    {58, 180},    // 11
    {1, 180}      // default
};

uint8_t queue[MAX_QUEUE_SIZE];
int front = 0, rear = -1, count = 0;
uint16_t last_adc_value = 0;
uint8_t trigger_flag = 0;

int modo_manual = 0;

// --- Utils ---
void delayMs(int n) {
    SysTick->LOAD = 41940 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    for (int i = 0; i < n; i++) {
        while ((SysTick->CTRL & 0x10000) == 0);
    }
    SysTick->CTRL = 0;
}

// --- UART ---
void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;           
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);          

    UART0->C2 = 0;                                

    UART0->BDH = 0x00;
    UART0->BDL = 0x0B;                            
    UART0->C4  = 0x0F;                            

    UART0->C1 = 0x00;
    UART0->C2 = UART_C2_RE_MASK | UART_C2_RIE_MASK;  

    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    PORTA->PCR[1] = PORT_PCR_MUX(2);              
}

void UART0_IRQHandler(void) {
    if (UART0->S1 & UART_S1_RDRF_MASK) {
        uint8_t value = UART0->D;
        UART0->D = value; // eco
        if(modo_manual == 0){
            enqueue(value & 0x03); 
        }
    }
}

// --- Cola ---
void enqueue(uint8_t config) {
    if (count < MAX_QUEUE_SIZE) {
        rear = (rear + 1) % MAX_QUEUE_SIZE;
        queue[rear] = config;
        count++;
    }
}

uint8_t peek(void) {
    if (count > 0){
        Motor_setSpeed(100);
        return queue[front];
    }
    Motor_setSpeed(100);
    return 4;
}

void dequeue(void) {
    uint8_t curr = peek();
    if(curr == 0) c1++;
    else if(curr == 1) c2++;
    else if(curr == 2) c3++;
    else if (curr == 3)c4++;
    else return;

    if (count > 0) {
        front = (front + 1) % MAX_QUEUE_SIZE;
        count--;
    }

    LCD_updateCounts();  
}

// --- PWM / Servo ---
void PWM_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;       
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;        
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);        

    PORTD->PCR[0] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[0] |= PORT_PCR_MUX(4);         

    PORTD->PCR[1] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[1] |= PORT_PCR_MUX(4);         

    TPM0->SC = 0;                  
    TPM0->MOD = 83999;            

    TPM0->CONTROLS[0].CnSC = 0x28;
    TPM0->CONTROLS[1].CnSC = 0x28;

    TPM0->CONTROLS[0].CnV = 0;
    TPM0->CONTROLS[1].CnV = 0;

    TPM0->SC = 0x08 | 0x01;        
}

void setServoAngle1(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float pwm = 79.72f * angle + 3650.0f;
    TPM0->CONTROLS[0].CnV = (uint16_t)pwm;  
}

void setServoAngle2(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float pwm = 79.72f * angle + 3650.0f;
    TPM0->CONTROLS[1].CnV = (uint16_t)pwm;  
}

// --- ADC ---
void ADC0_init(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[20] &= ~PORT_PCR_MUX_MASK;

    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
    ADC0->SC3 |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
    ADC0->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP_MASK;

    ADC0->SC1[0] = ADC_CHANNEL;
    ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK;
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
    NVIC_EnableIRQ(ADC0_IRQn);
}

uint16_t readADC0(uint8_t channel) {
    ADC0->SC1[0] = channel & ADC_SC1_ADCH_MASK;
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

void ADC0_IRQHandler(void) {
    uint16_t current_adc = ADC0->R[0];

    if (current_adc > THRESHOLD && last_adc_value <= THRESHOLD && !trigger_flag) {
        dequeue();
        trigger_flag = 1;
    }

    if (current_adc <= THRESHOLD) {
        trigger_flag = 0;
    }

    last_adc_value = current_adc;
    ADC0->SC1[0] = ADC_CHANNEL | ADC_SC1_AIEN_MASK;
}

// --- LED ---
void LED_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[LED_PIN] = PORT_PCR_MUX(1);
    PTB->PDDR |= (1 << LED_PIN);
    PTB->PSOR = (1 << LED_PIN);
}

// --- LCD ---
void LCD_init(void){
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK;

    for (int i = 4; i <= 7; i++) {
        PORTD->PCR[i] = PORT_PCR_MUX(1);
        PTD->PDDR |= (1 << i);
    }

    PORTA->PCR[2] = PORT_PCR_MUX(1);
    PORTA->PCR[4] = PORT_PCR_MUX(1);
    PORTA->PCR[5] = PORT_PCR_MUX(1);
    PTA->PDDR |= (1 << 2) | (1 << 4) | (1 << 5);

    delayMs(20);
    LCD_command_noWait(0x33);
    LCD_command_noWait(0x32);
    LCD_command(0x28);
    LCD_command(0x06);
    LCD_command(0x01);
    LCD_command(0x0F);
}

void LCD_nibble(uint8_t nibble) {
    PTD->PDOR = (PTD->PDOR & 0x0F) | (nibble << 4);
    PTA->PSOR = (1 << 2);
    delayMs(1);
    PTA->PCOR = (1 << 2);
    delayMs(1);
}

void LCD_command(uint8_t cmd) {
    PTA->PCOR = (1 << 4);
    PTA->PCOR = (1 << 5);
    LCD_nibble(cmd >> 4);
    LCD_nibble(cmd & 0x0F);
}

void LCD_command_noWait(uint8_t cmd) {
    PTA->PCOR = (1 << 4);
    PTA->PCOR = (1 << 5);
    LCD_nibble(cmd >> 4);
    LCD_nibble(cmd & 0x0F);
}

void LCD_data(uint8_t data) {
    PTA->PSOR = (1 << 4);
    PTA->PCOR = (1 << 5);
    LCD_nibble(data >> 4);
    LCD_nibble(data & 0x0F);
}

void LCD_updateCounts(void) {
    LCD_command(0x01);
    delayMs(2);

    LCD_command(0x80);
    char linea1[17];
    snprintf(linea1, sizeof(linea1), "A:%d B:%d", c1, c2);
    for (char *p = linea1; *p; p++) LCD_data(*p);

    LCD_command(0xC0);
    char linea2[17];
    snprintf(linea2, sizeof(linea2), "C:%d D:%d", c3, c4);
    for (char *p = linea2; *p; p++) LCD_data(*p);
}

// --- Keypad ---
void keypad_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    for (int i = 4; i <= 7; i++) {
        PORTC->PCR[i] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    }

    for (int i = 0; i <= 3; i++) {
        PORTC->PCR[i] = PORT_PCR_MUX(1);
    }

    PTC->PDDR = 0x0F;
    PTC->PSOR = 0x0F;
}

char keypad_getkey(void) {
    int row, col;
    const uint8_t row_masks[] = {0x01, 0x02, 0x04, 0x08};

    PTC->PDDR |= 0x0F;
    PTC->PCOR = 0x0F;
    delayMs(2);
    uint8_t col_read = PTC->PDIR & 0xF0;
    if (col_read == 0xF0) return 0;

    for (row = 0; row < 4; row++) {
        PTC->PDDR = 0x00;
        PTC->PDDR |= row_masks[row];
        PTC->PCOR = row_masks[row];
        delayMs(2);
        col_read = PTC->PDIR & 0xF0;
        if (col_read != 0xF0) break;
    }

    PTC->PDDR = 0x00;
    if (row == 4) return 0;

    switch (col_read) {
        case 0xE0: return row * 4 + 1;
        case 0xD0: return row * 4 + 2;
        case 0xB0: return row * 4 + 3;
        case 0x70: return row * 4 + 4;
        default: return 0;
    }
}

const char keypad_chars[17] = {
    0, '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

int decoder_teclado(char input){
    if (input < 1 || input > 16) return -3;
    switch (input) {
        case 13: return -1; // *
        case 15: return -2; // #
        default:
            if (input == 4) return 10;
            if (input == 8) return 11;
            if (input == 12) return 12;
            if (input == 16) return 13;
            return (keypad_chars[input] - '0');
    }
}

// --- Motor ---
void Motor_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    PORTA->PCR[MOTOR_PWM_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[MOTOR_PWM_PIN] |= PORT_PCR_MUX(3);

    PORTA->PCR[MOTOR_DIR1_PIN] = PORT_PCR_MUX(1);
    PORTA->PCR[MOTOR_DIR2_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR |= (1 << MOTOR_DIR1_PIN) | (1 << MOTOR_DIR2_PIN);

    TPM1->SC = 0;
    TPM1->MOD = 20969;
    TPM1->CONTROLS[0].CnSC = 0x28;
    TPM1->CONTROLS[0].CnV = 0;
    TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
}

void Motor_setSpeed(uint8_t duty) {
    if (duty > 100) duty = 100;
    uint16_t pwmCounts = (TPM1->MOD + 1) * duty / 100;
    TPM1->CONTROLS[0].CnV = pwmCounts;
}

void Motor_forward(void) {
    PTA->PSOR = (1 << MOTOR_DIR1_PIN);
    PTA->PCOR = (1 << MOTOR_DIR2_PIN);
}

void Motor_reverse(void) {
    PTA->PCOR = (1 << MOTOR_DIR1_PIN);
    PTA->PSOR = (1 << MOTOR_DIR2_PIN);
}

void Motor_stop(void) {
    PTA->PCOR = (1 << MOTOR_DIR1_PIN);
    PTA->PCOR = (1 << MOTOR_DIR2_PIN);
    Motor_setSpeed(0);
}

// --- Botones ---
void EmergencyButton_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA->PCR[EMERGENCY_BUTTON_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTA->PCR[MANUAL_BUTTON_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);

    PTA->PDDR &= ~(1 << EMERGENCY_BUTTON_PIN);
    PTA->PDDR &= ~(1 << MANUAL_BUTTON_PIN);
}

void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1 << EMERGENCY_BUTTON_PIN)) {
        PTB->PCOR = (1 << LED_PIN);
        delayMs(100);
        PTB->PSOR = (1 << LED_PIN);
        Motor_stop();
        while(1){}
        PORTA->ISFR |= (1 << EMERGENCY_BUTTON_PIN);
    }
    else if (PORTA->ISFR & (1 << MANUAL_BUTTON_PIN)) {
        PTB->PCOR = (1 << LED_PIN);
        delayMs(100);
        PTB->PSOR = (1 << LED_PIN);
        modo_manual = 1;
        PORTA->ISFR |= (1 << MANUAL_BUTTON_PIN);
    }
}

