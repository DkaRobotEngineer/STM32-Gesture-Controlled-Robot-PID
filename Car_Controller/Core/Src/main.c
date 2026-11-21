#include "stm32f411xe.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// 1. BIEN TOAN CUC VA CAI DAT THONG SO
#define ENCODER_PPR 859.0
#define MAX_RPM 220

volatile int16_t Target_Pitch = 0;   // Goc toi lui tu tay cam
volatile int16_t Target_Roll = 0;    // Goc trai phai tu tay cam
volatile uint8_t New_Data_Flag = 0;  // Co bao co du lieu song moi

volatile int16_t RPM_M1 = 0;         // Toc do banh 1 (Phai - Sau)
volatile int16_t RPM_M2 = 0;         // Toc do banh 2 (Trai - Sau)
volatile int16_t RPM_M3 = 0;         // Toc do banh 3 (Trai - Truoc)
volatile int16_t RPM_M4 = 0;         // Toc do banh 4 (Phai - Truoc)

// 2. HAM NEN TANG DA NHIEM (SYSTICK & TIME)
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}

uint32_t millis(void) {
    return msTicks;
}

void delay_ms(uint32_t ms) {
    uint32_t start_time = millis();
    while ((millis() - start_time) < ms) {
        __NOP();
    }
}

void SystemClock_Config(void) {
    SysTick_Config(16000000 / 1000); // 1ms Systick
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Cap xung Timer 5 (PWM)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN; // SPI1 va ngat ngoai
}

// 3. THU VIEN DIEU KHIEN PID DOC LAP
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    int16_t Setpoint;
    float Integral;
    int16_t Prev_Error;
    int16_t Out_Max;
    int16_t Out_Min;
    int16_t Prev_PWM;
} PID_Controller;

PID_Controller PID1, PID2, PID3, PID4;

void PID_Init(void) {
    float p = 11.3618;
    float i = 0.28;
    float d = 0.536;

    PID1.Kp = p; PID1.Ki = i; PID1.Kd = d;
    PID1.Integral = 0; PID1.Prev_Error = 0; PID1.Prev_PWM = 0;
    PID1.Out_Max = 1000; PID1.Out_Min = -1000;

    PID2 = PID1;
    PID3 = PID1;
    PID4 = PID1;
}

int16_t PID_Compute(PID_Controller *pid, int16_t current_rpm) {
    if (pid->Setpoint == 0) {
        pid->Integral = 0;
        pid->Prev_Error = 0;
        pid->Prev_PWM = 0;
        return 0;
    }

    int16_t error = pid->Setpoint - current_rpm;
    pid->Integral += error;

    if (pid->Integral > 2000) pid->Integral = 2000;
    if (pid->Integral < -2000) pid->Integral = -2000;

    int16_t derivative = error - pid->Prev_Error;
    pid->Prev_Error = error;

    float output = (pid->Kp * error) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);

    if (output > pid->Out_Max) output = pid->Out_Max;
    if (output < pid->Out_Min) output = pid->Out_Min;

    output = (0.3 * output) + (0.7 * pid->Prev_PWM);
    pid->Prev_PWM = output;

    return (int16_t)output;
}

// 4. KHOI DIEU KHIEN DONG CO (PWM TIMER 5)
void Motor_Init(void) {
    GPIOB->MODER &= ~((3U << 24) | (3U << 26) | (3U << 28) | (3U << 30));
    GPIOB->MODER |=  ((1U << 24) | (1U << 26) | (1U << 28) | (1U << 30)); // Chan huong banh 1, 2

    GPIOA->MODER &= ~((3U << 20) | (3U << 22) | (3U << 24));
    GPIOA->MODER |=  ((1U << 20) | (1U << 22) | (1U << 24)); // Chan huong banh 3, 4

    GPIOB->MODER &= ~(3U << 16);
    GPIOB->MODER |=  (1U << 16); // Chan huong phu banh 4 (PB8)

    GPIOA->MODER &= ~((3U << 0) | (3U << 2) | (3U << 4) | (3U << 6));
    GPIOA->MODER |=  ((2U << 0) | (2U << 2) | (2U << 4) | (2U << 6)); // AF PA0..PA3

    GPIOA->AFR[0] &= ~0xFFFF;
    GPIOA->AFR[0] |=  0x2222; // AF2 (TIM5)

    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->PSC = 16 - 1;
    TIM5->ARR = 1000 - 1;

    TIM5->CCMR1 |= (6U << 4) | (6U << 12) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM5->CCMR2 |= (6U << 4) | (6U << 12) | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    TIM5->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM5->CCR1 = 0; TIM5->CCR2 = 0; TIM5->CCR3 = 0; TIM5->CCR4 = 0;

    TIM5->CR1 |= TIM_CR1_CEN;

    GPIOC->MODER &= ~(3U << 26);
    GPIOC->MODER |=  (1U << 26); // LED PC13 Output
    GPIOC->BSRR = (1U << 13);
}

void Set_Motor(uint8_t id, int16_t speed) {
    uint8_t dir = (speed >= 0) ? 1 : 0;
    uint32_t pwm_val = abs(speed);
    if (pwm_val > 1000) pwm_val = 1000;

    switch(id) {
        case 1:
            TIM5->CCR1 = pwm_val;
            if (dir) GPIOB->BSRR = (1U << 12) | (1U << (13 + 16));
            else     GPIOB->BSRR = (1U << (12 + 16)) | (1U << 13);
            break;
        case 2:
            TIM5->CCR2 = pwm_val;
            if (dir) GPIOB->BSRR = (1U << 14) | (1U << (15 + 16));
            else     GPIOB->BSRR = (1U << (14 + 16)) | (1U << 15);
            break;
        case 3:
            TIM5->CCR3 = pwm_val;
            if (dir) GPIOA->BSRR = (1U << (10 + 16)) | (1U << 11);
            else     GPIOA->BSRR = (1U << 10) | (1U << (11 + 16));
            break;
        case 4:
            TIM5->CCR4 = pwm_val;
            if (dir) { GPIOA->BSRR = (1U << 12); GPIOB->BSRR = (1U << (8 + 16)); }
            else     { GPIOA->BSRR = (1U << (12 + 16)); GPIOB->BSRR = (1U << 8); }
            break;
    }
}

// 5. KHOI DOC ENCODER PHAN CUNG (TIM1, TIM2, TIM3, TIM4)
void Encoder_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    // TIM1: PA8, PA9 (AF1)
    GPIOA->MODER &= ~((3U << 16) | (3U << 18));
    GPIOA->MODER |=  ((2U << 16) | (2U << 18));
    GPIOA->PUPDR |=  ((1U << 16) | (1U << 18));
    GPIOA->AFR[1] &= ~((15U << 0) | (15U << 4));
    GPIOA->AFR[1] |=  ((1U << 0)  | (1U << 4));

    // TIM2: PA15, PB3 (AF1)
    GPIOA->MODER &= ~(3U << 30); GPIOA->MODER |= (2U << 30);
    GPIOB->MODER &= ~(3U << 6);  GPIOB->MODER |= (2U << 6);
    GPIOA->PUPDR |= (1U << 30);  GPIOB->PUPDR |= (1U << 6);
    GPIOA->AFR[1] &= ~(15U << 28); GPIOA->AFR[1] |= (1U << 28);
    GPIOB->AFR[0] &= ~(15U << 12); GPIOB->AFR[0] |= (1U << 12);

    // TIM3: PB4, PB5 (AF2)
    GPIOB->MODER &= ~((3U << 8) | (3U << 10));
    GPIOB->MODER |=  ((2U << 8) | (2U << 10));
    GPIOB->PUPDR |=  ((1U << 8) | (1U << 10));
    GPIOB->AFR[0] &= ~((15U << 16) | (15U << 20));
    GPIOB->AFR[0] |=  ((2U << 16)  | (2U << 20));

    // TIM4: PB6, PB7 (AF2)
    GPIOB->MODER &= ~((3U << 12) | (3U << 14));
    GPIOB->MODER |=  ((2U << 12) | (2U << 14));
    GPIOB->PUPDR |=  ((1U << 12) | (1U << 14));
    GPIOB->AFR[0] &= ~((15U << 24) | (15U << 28));
    GPIOB->AFR[0] |=  ((2U << 24)  | (2U << 28));

    // Che do Encoder Mode 3
    TIM1->SMCR |= (3U << 0); TIM2->SMCR |= (3U << 0);
    TIM3->SMCR |= (3U << 0); TIM4->SMCR |= (3U << 0);

    TIM1->CCMR1 |= (1U << 0) | (1U << 8); TIM2->CCMR1 |= (1U << 0) | (1U << 8);
    TIM3->CCMR1 |= (1U << 0) | (1U << 8); TIM4->CCMR1 |= (1U << 0) | (1U << 8);

    TIM1->CR1 |= TIM_CR1_CEN; TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN; TIM4->CR1 |= TIM_CR1_CEN;
}

// 6. KHOI TRUYEN THONG NRF24L01 & NGAT EXTI10
void SPI1_nRF24_RX_Init(void) {
    GPIOA->MODER &= ~((3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->MODER |=  ((2U << 10) | (2U << 12) | (2U << 14));
    GPIOA->OSPEEDR |= ((3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->AFR[0] &= ~((15U << 20) | (15U << 24) | (15U << 28));
    GPIOA->AFR[0] |=  ((5U << 20)  | (5U << 24)  | (5U << 28)); // PA5, PA6, PA7 (AF5)

    GPIOB->MODER &= ~((3U << 0) | (3U << 2));
    GPIOB->MODER |=  ((1U << 0) | (1U << 2)); // PB0 (CE), PB1 (CSN)
    GPIOB->BSRR = (1U << 1);
    GPIOB->BSRR = (1U << (0 + 16));

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_MSTR | (3U << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;

    GPIOB->MODER &= ~(3U << 20); // PB10 làm Input (IRQ)
    GPIOB->PUPDR &= ~(3U << 20);
    GPIOB->PUPDR |=  (1U << 20);

    SYSCFG->EXTICR[2] &= ~(15U << 8);
    SYSCFG->EXTICR[2] |=  (1U << 8); // PB10 map EXTI10
    EXTI->FTSR |= (1U << 10);        // Suon am
    EXTI->IMR  |= (1U << 10);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t SPI1_TransmitReceive(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)) __NOP();
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE)) __NOP();
    return SPI1->DR;
}

void nRF24_WriteReg(uint8_t reg, uint8_t value) {
    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0x20 | reg);
    SPI1_TransmitReceive(value);
    GPIOB->BSRR = (1U << 1);
}

void nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0x20 | reg);
    for (uint8_t i = 0; i < len; i++) SPI1_TransmitReceive(pBuf[i]);
    GPIOB->BSRR = (1U << 1);
}

void nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(reg);
    for (uint8_t i = 0; i < len; i++) pBuf[i] = SPI1_TransmitReceive(0xFF);
    GPIOB->BSRR = (1U << 1);
}

void nRF24_Init_RX(void) {
    GPIOB->BSRR = (1U << (0 + 16));
    delay_ms(5);

    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0xE2);
    GPIOB->BSRR = (1U << 1);

    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0xE1);
    GPIOB->BSRR = (1U << 1);

    nRF24_WriteReg(0x07, 0x70);
    nRF24_WriteReg(0x01, 0x01);
    nRF24_WriteReg(0x02, 0x01);
    nRF24_WriteReg(0x03, 0x03);
    nRF24_WriteReg(0x05, 40);
    nRF24_WriteReg(0x06, 0x06);
    nRF24_WriteReg(0x11, 5);
    nRF24_WriteReg(0x1D, 0x06);
    nRF24_WriteReg(0x1C, 0x01);

    uint8_t RX_Address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nRF24_WriteBuf(0x0A, RX_Address, 5);

    nRF24_WriteReg(0x00, 0x3F);
    GPIOB->BSRR = (1U << 0);
    delay_ms(2);
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1U << 10)) {
        uint8_t rx_payload[5];
        nRF24_ReadBuf(0x61, rx_payload, 5);
        nRF24_WriteReg(0x07, 0x70);

        if (rx_payload[0] == 0xAA) {
            Target_Pitch = (int16_t)((rx_payload[1] << 8) | rx_payload[2]);
            Target_Roll  = (int16_t)((rx_payload[3] << 8) | rx_payload[4]);
            New_Data_Flag = 1;
        }
        EXTI->PR |= (1U << 10);
    }
}

void nRF24_Send_Telemetry(void) {
    int16_t avg_speed = (abs(RPM_M1) + abs(RPM_M2) + abs(RPM_M3) + abs(RPM_M4)) / 4;
    int16_t avg_setpoint = (abs(PID1.Setpoint) + abs(PID2.Setpoint)) / 2;
    int16_t avg_pwm = (abs(PID1.Prev_PWM) + abs(PID2.Prev_PWM) + abs(PID3.Prev_PWM) + abs(PID4.Prev_PWM)) / 4;

    uint8_t ack_data[6];
    ack_data[0] = (avg_speed >> 8) & 0xFF;    ack_data[1] = avg_speed & 0xFF;
    ack_data[2] = (avg_setpoint >> 8) & 0xFF; ack_data[3] = avg_setpoint & 0xFF;
    ack_data[4] = (avg_pwm >> 8) & 0xFF;      ack_data[5] = avg_pwm & 0xFF;

    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0xE1); // Flush TX
    GPIOB->BSRR = (1U << 1);

    GPIOB->BSRR = (1U << (1 + 16));
    SPI1_TransmitReceive(0xA8); // Nap vao ACK Payload
    for (uint8_t i = 0; i < 6; i++) {
        SPI1_TransmitReceive(ack_data[i]);
    }
    GPIOB->BSRR = (1U << 1);
}

// 7. CHUONG TRINH CHINH (MAIN LOOP)
int main(void) {
    SystemClock_Config();
    Motor_Init();
    Encoder_Init();
    PID_Init();

    SPI1_nRF24_RX_Init();
    nRF24_Init_RX();

    uint32_t last_pid_time = 0;
    uint16_t timeout_counter = 0;
    uint8_t stall_counter = 0;
    uint8_t error_system_halt = 0;

    while (1) {
        // LUONG 1: GIAI MA LENH VA HOP SO SKID-STEER
        if (New_Data_Flag == 1) {
            New_Data_Flag = 0;
            timeout_counter = 0;

            int16_t base_rpm = 0;
            int16_t turn_rpm = 0;

            int16_t abs_pitch = abs(Target_Pitch);
            if (abs_pitch >= 55)      base_rpm = MAX_RPM;
            else if (abs_pitch >= 40) base_rpm = 170;
            else if (abs_pitch >= 20) base_rpm = 110;
            else                      base_rpm = 0;

            if (Target_Pitch < 0) base_rpm = -base_rpm;

            int16_t abs_roll = abs(Target_Roll);
            if (abs_roll >= 18) turn_rpm = (Target_Roll * 200) / 100;
            else                turn_rpm = 0;

            int16_t left_sp  = base_rpm + turn_rpm;
            int16_t right_sp = base_rpm - turn_rpm;

            int16_t max_sp = abs(left_sp) > abs(right_sp) ? abs(left_sp) : abs(right_sp);
            if (max_sp > MAX_RPM) {
                float scale_factor = (float)MAX_RPM / max_sp;
                left_sp  = (int16_t)(left_sp * scale_factor);
                right_sp = (int16_t)(right_sp * scale_factor);
            }

            PID1.Setpoint = right_sp; // Banh 1: Phai - Sau
            PID4.Setpoint = right_sp; // Banh 4: Phai - Truoc
            PID2.Setpoint = left_sp;  // Banh 2: Trai - Sau
            PID3.Setpoint = left_sp;  // Banh 3: Trai - Truoc

            nRF24_Send_Telemetry();
        }

        // LUONG 2: VONG DIEU KHIEN PID (10MS) VA AN TOAN
        if (millis() - last_pid_time >= 10) {
            last_pid_time = millis();
            timeout_counter++;

            // TU DONG HOI SINH CHIP SONG NRF24 KHI BI NHIEU TREO
            if (timeout_counter > 50) {
                PID1.Setpoint = 0; PID2.Setpoint = 0;
                PID3.Setpoint = 0; PID4.Setpoint = 0;

                // Reset toan dien bo dem va khoi dong lai chip NRF24 ngay lap tuc
                nRF24_Init_RX();
                timeout_counter = 0; // Xoa dem de tiep tuc bat song
            }

            // Doc va reset thanh ghi Encoder
            int16_t raw_M1 = (int16_t)TIM2->CNT; TIM2->CNT = 0;
            int16_t raw_M2 = (int16_t)TIM1->CNT; TIM1->CNT = 0;
            int16_t raw_M3 = (int16_t)TIM3->CNT; TIM3->CNT = 0;
            int16_t raw_M4 = (int16_t)TIM4->CNT; TIM4->CNT = 0;

            int16_t Ticks_M1 = (PID1.Prev_PWM >= 0) ? abs(raw_M1) : -abs(raw_M1);
            int16_t Ticks_M2 = (PID2.Prev_PWM >= 0) ? abs(raw_M2) : -abs(raw_M2);
            int16_t Ticks_M3 = (PID3.Prev_PWM >= 0) ? abs(raw_M3) : -abs(raw_M3);
            int16_t Ticks_M4 = (PID4.Prev_PWM >= 0) ? abs(raw_M4) : -abs(raw_M4);

            RPM_M1 = (Ticks_M1 * 6000) / ENCODER_PPR;
            RPM_M2 = (Ticks_M2 * 6000) / ENCODER_PPR;
            RPM_M3 = (Ticks_M3 * 6000) / ENCODER_PPR;
            RPM_M4 = (Ticks_M4 * 6000) / ENCODER_PPR;

            // Kiem tra kẹt bánh (Stall Protection)
            uint8_t is_stalled = 0;
            if (PID1.Setpoint != 0 && (abs(PID1.Prev_PWM) > 900) && (abs(RPM_M1) < 10)) is_stalled = 1;
            if (PID2.Setpoint != 0 && (abs(PID2.Prev_PWM) > 900) && (abs(RPM_M2) < 10)) is_stalled = 1;
            if (PID3.Setpoint != 0 && (abs(PID3.Prev_PWM) > 900) && (abs(RPM_M3) < 10)) is_stalled = 1;
            if (PID4.Setpoint != 0 && (abs(PID4.Prev_PWM) > 900) && (abs(RPM_M4) < 10)) is_stalled = 1;

            if (is_stalled) stall_counter++;
            else stall_counter = 0;

//            if (stall_counter > 50) error_system_halt = 1;

            if (error_system_halt == 1) {
                Set_Motor(1, 0); Set_Motor(2, 0); Set_Motor(3, 0); Set_Motor(4, 0);
                if (millis() % 100 < 50) GPIOC->BSRR = (1U << (13 + 16));
                else GPIOC->BSRR = (1U << 13);
            } else {
                int16_t PWM_M1 = PID_Compute(&PID1, RPM_M1);
                int16_t PWM_M2 = PID_Compute(&PID2, RPM_M2);
                int16_t PWM_M3 = PID_Compute(&PID3, RPM_M3);
                int16_t PWM_M4 = PID_Compute(&PID4, RPM_M4);

                Set_Motor(1, PWM_M1);
//                Set_Motor(1, 500); // Ép bánh 1 quay với 50% công suất
                Set_Motor(2, PWM_M2);
                Set_Motor(3, PWM_M3);
                Set_Motor(4, PWM_M4);

                if (timeout_counter < 50) {
                    if (millis() % 1000 < 500) GPIOC->BSRR = (1U << (13 + 16));
                    else GPIOC->BSRR = (1U << 13);
                } else {
                    GPIOC->BSRR = (1U << 13);
                }
            }
        }
    }
}
