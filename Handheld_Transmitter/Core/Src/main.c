#include "stm32f411xe.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MPU6050_ADDR          0xD0
#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_WR_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define MAX_RPM 220

const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x2f, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, {0x24, 0x2a, 0x7f, 0x2a, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00}, {0x00, 0x1c, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1c, 0x00}, {0x14, 0x08, 0x3e, 0x08, 0x14}, {0x08, 0x08, 0x3e, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02}, {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31}, {0x18, 0x14, 0x12, 0x7f, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e}, {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00}, {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06}, {0x32, 0x49, 0x79, 0x41, 0x3e},
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 0x09, 0x09, 0x09, 0x01},
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40},
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
    {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

void delay_ms(volatile uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1600; i++);
}

void SystemClock_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_I2C1EN;
}

void GPIO_Init(void) {
    GPIOA->MODER &= ~(3U << (0 * 2)); // PA0 làm Input (Nút bấm)
    GPIOA->PUPDR &= ~(3U << (0 * 2));
    GPIOA->PUPDR |=  (1U << (0 * 2)); // Pull-up cho PA0

    GPIOA->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    GPIOA->MODER |=  ((1U << (3 * 2)) | (1U << (4 * 2))); // PA3 (CE), PA4 (CSN) làm Output
    GPIOA->BSRR = (1U << 4);
    GPIOA->BSRR = (1U << (3 + 16));

    GPIOA->MODER &= ~(3U << (8 * 2));
    GPIOA->MODER |=  (1U << (8 * 2)); // PA8 (CS OLED)
    GPIOA->BSRR = (1U << 8);

    GPIOB->MODER &= ~((3U << (12 * 2)) | (3U << (14 * 2)));
    GPIOB->MODER |=  ((1U << (12 * 2)) | (1U << (14 * 2))); // PB12 (RES), PB14 (DC)

    GPIOB->MODER &= ~((3U << (13 * 2)) | (3U << (15 * 2)));
    GPIOB->MODER |=  ((2U << (13 * 2)) | (2U << (15 * 2))); // PB13, PB15 Alternate Function (SPI2)
    GPIOB->OSPEEDR |= ((2U << (13 * 2)) | (2U << (15 * 2)));
    GPIOB->AFR[1] &= ~((15U << 20) | (15U << 28));
    GPIOB->AFR[1] |=  ((5U << 20)  | (5U << 28));

    GPIOC->MODER &= ~(3U << (13 * 2));
    GPIOC->MODER |=  (1U << (13 * 2)); // LED PC13
}

void SPI2_OLED_Init(void) {
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR1 |= SPI_CR1_SPE;
}

void SPI2_Transmit(uint8_t data) {
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;
    while (SPI2->SR & SPI_SR_BSY);
}

void OLED_WriteCommand(uint8_t cmd) {
    GPIOA->BSRR = (1U << (8 + 16));
    GPIOB->BSRR = (1U << (14 + 16));
    SPI2_Transmit(cmd);
    GPIOA->BSRR = (1U << 8);
}

void OLED_WriteData(uint8_t data) {
    GPIOA->BSRR = (1U << (8 + 16));
    GPIOB->BSRR = (1U << 14);
    SPI2_Transmit(data);
    GPIOA->BSRR = (1U << 8);
}

void OLED_Init(void) {
    GPIOB->BSRR = (1U << (12 + 16));
    delay_ms(100);
    GPIOB->BSRR = (1U << 12);
    delay_ms(100);

    const uint8_t Init_Commands[] = {
        0xAE, 0x20, 0x00, 0xC8, 0xA1, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x81, 0xFF, 0xA4, 0xA6, 0xD5, 0xF0, 0xD9, 0x22,
        0xDA, 0x12, 0xDB, 0x20, 0xAF
    };
    for (int i = 0; i < sizeof(Init_Commands); i++) {
        OLED_WriteCommand(Init_Commands[i]);
    }
}

void OLED_SetCursor(uint8_t x, uint8_t y) {
    OLED_WriteCommand(0xB0 + y);
    OLED_WriteCommand(0x00 + (x & 0x0F));
    OLED_WriteCommand(0x10 + ((x >> 4) & 0x0F));
}

void OLED_Clear(void) {
    for (uint8_t y = 0; y < 8; y++) {
        OLED_SetCursor(0, y);
        for (uint8_t x = 0; x < 128; x++) {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_PrintChar(char c) {
    if (c >= 32 && c <= 90) {
        uint8_t index = c - 32;
        for (uint8_t i = 0; i < 5; i++) {
            OLED_WriteData(Font5x7[index][i]);
        }
        OLED_WriteData(0x00);
    }
}

void OLED_PrintString(char* str) {
    while (*str) {
        OLED_PrintChar(*str);
        str++;
    }
}

void OLED_PrintNum(int16_t num) {
    char buffer[20];
    sprintf(buffer, "%d    ", num);
    OLED_PrintString(buffer);
}

void SPI1_nRF24_Init(void) {
    GPIOA->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->MODER |=  ((2U << (5 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOA->OSPEEDR |= ((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->AFR[0] &= ~((15U << 20) | (15U << 24) | (15U << 28));
    GPIOA->AFR[0] |=  ((5U << 20)  | (5U << 24)  | (5U << 28));

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_MSTR | (3U << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t SPI1_TransmitReceive(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void nRF24_WriteReg(uint8_t reg, uint8_t value) {
    GPIOA->BSRR = (1U << (4 + 16));
    SPI1_TransmitReceive(NRF_CMD_W_REGISTER | reg);
    SPI1_TransmitReceive(value);
    GPIOA->BSRR = (1U << 4);
}

void nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    GPIOA->BSRR = (1U << (4 + 16));
    SPI1_TransmitReceive(reg);
    for (uint8_t i = 0; i < len; i++) {
        SPI1_TransmitReceive(pBuf[i]);
    }
    GPIOA->BSRR = (1U << 4);
}

void nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    GPIOA->BSRR = (1U << (4 + 16));
    SPI1_TransmitReceive(reg);
    for (uint8_t i = 0; i < len; i++) {
        pBuf[i] = SPI1_TransmitReceive(0xFF);
    }
    GPIOA->BSRR = (1U << 4);
}

uint8_t nRF24_ReadReg(uint8_t reg) {
    uint8_t value;
    GPIOA->BSRR = (1U << (4 + 16));
    SPI1_TransmitReceive(NRF_CMD_R_REGISTER | reg);
    value = SPI1_TransmitReceive(0xFF);
    GPIOA->BSRR = (1U << 4);
    return value;
}

void nRF24_Init_TX(void) {
    GPIOA->BSRR = (1U << (3 + 16));
    nRF24_WriteReg(0x01, 0x01);
    nRF24_WriteReg(0x02, 0x01);
    nRF24_WriteReg(0x03, 0x03);
    nRF24_WriteReg(0x04, 0x2F);
    nRF24_WriteReg(0x05, 40);
    nRF24_WriteReg(0x06, 0x06);
    nRF24_WriteReg(0x11, 5);
    nRF24_WriteReg(0x1D, 0x06);
    nRF24_WriteReg(0x1C, 0x01);

    uint8_t TX_Address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    nRF24_WriteBuf(0x0A, TX_Address, 5);
    nRF24_WriteBuf(0x10, TX_Address, 5);

    nRF24_WriteReg(0x00, 0x0E);
    delay_ms(5);
}

void nRF24_Transmit_Packet(uint8_t *packet, uint8_t len) {
    GPIOA->BSRR = (1U << (4 + 16));
    SPI1_TransmitReceive(NRF_CMD_FLUSH_TX);
    GPIOA->BSRR = (1U << 4);

    nRF24_WriteBuf(NRF_CMD_WR_TX_PAYLOAD, packet, len);

    GPIOA->BSRR = (1U << 3);
    delay_ms(1);
    GPIOA->BSRR = (1U << (3 + 16));
}

void I2C1_Init(void) {
    GPIOB->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOB->OTYPER |= (1U << 6) | (1U << 7);
    GPIOB->OSPEEDR |= ((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->PUPDR |=  ((1U << (6 * 2)) | (1U << (7 * 2)));
    GPIOB->AFR[0] &= ~((15U << 24) | (15U << 28));
    GPIOB->AFR[0] |=  ((4U << 24)  | (4U << 28));

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_WriteReg(uint8_t devAddress, uint8_t regAddress, uint8_t data) {
    while (I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = devAddress;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = regAddress;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void MPU6050_Init(void) {
    I2C_WriteReg(MPU6050_ADDR, 0x6B, 0x00);
    I2C_WriteReg(MPU6050_ADDR, 0x19, 0x07);
    I2C_WriteReg(MPU6050_ADDR, 0x1C, 0x00);
}

void MPU6050_ReadAccel(int16_t *Ax, int16_t *Ay, int16_t *Az) {
    uint8_t data[6];
    while (I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = MPU6050_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = 0x3B;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = MPU6050_ADDR | 0x01;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_ACK;
    for (int i = 0; i < 6; i++) {
        if (i == 5) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        data[i] = I2C1->DR;
    }

    *Ax = (int16_t)((data[0] << 8) | data[1]);
    *Ay = (int16_t)((data[2] << 8) | data[3]);
    *Az = (int16_t)((data[4] << 8) | data[5]);
}

int main(void) {
    SystemClock_Config();
    GPIO_Init();

    SPI2_OLED_Init();
    OLED_Init();
    OLED_Clear();

    SPI1_nRF24_Init();
    nRF24_Init_TX();

    I2C1_Init();
    MPU6050_Init();

    uint8_t system_state = 0;
    uint8_t last_button_state = 1;
    uint8_t blink_counter = 0;

    int16_t Accel_X, Accel_Y, Accel_Z;
    int16_t Pitch, Roll;
    int16_t target_display_L = 0, target_display_R = 0;

    while (1) {
        uint8_t current_button_state = (GPIOA->IDR & (1U << 0)) ? 1 : 0;

        if (last_button_state == 1 && current_button_state == 0) {
            system_state = 1 - system_state;
            OLED_Clear();

            if (system_state == 0) {
                uint8_t stop_payload[5] = {0xAA, 0x00, 0x00, 0x00, 0x00};
                nRF24_Transmit_Packet(stop_payload, 5);
            }
        }
        last_button_state = current_button_state;

        if (system_state == 0) {
            blink_counter++;
            if (blink_counter >= 5) {
                GPIOC->ODR ^= (1U << 13);
                blink_counter = 0;
            }
            OLED_SetCursor(10, 3);
            OLED_PrintString("PRESS PA0 TO START");
            delay_ms(100);
        } else {
            GPIOC->BSRR = (1U << (13 + 16));

            MPU6050_ReadAccel(&Accel_X, &Accel_Y, &Accel_Z);
            Pitch = (int16_t)(atan2(Accel_X, sqrt(Accel_Y * Accel_Y + Accel_Z * Accel_Z)) * 180.0 / 3.14159);
            Roll  = (int16_t)(atan2(Accel_Y, sqrt(Accel_X * Accel_X + Accel_Z * Accel_Z)) * 180.0 / 3.14159);

            int16_t base_rpm = 0, turn_rpm = 0;
            int16_t abs_pitch = abs(Pitch);

            if (abs_pitch >= 55)      base_rpm = MAX_RPM;
            else if (abs_pitch >= 40) base_rpm = 170;
            else if (abs_pitch >= 20) base_rpm = 110;
            else                      base_rpm = 0;

            if (Pitch < 0) base_rpm = -base_rpm;

            int16_t abs_roll = abs(Roll);
            if (abs_roll >= 25) turn_rpm = 150;
            else                turn_rpm = 0;

            if (Roll < 0) turn_rpm = -turn_rpm;

            target_display_L = base_rpm + turn_rpm;
            target_display_R = base_rpm - turn_rpm;

            if (target_display_L > MAX_RPM) { target_display_R -= (target_display_L - MAX_RPM); target_display_L = MAX_RPM; }
            else if (target_display_L < -MAX_RPM) { target_display_R -= (target_display_L + MAX_RPM); target_display_L = -MAX_RPM; }
            if (target_display_R > MAX_RPM) { target_display_L -= (target_display_R - MAX_RPM); target_display_R = MAX_RPM; }
            else if (target_display_R < -MAX_RPM) { target_display_L -= (target_display_R + MAX_RPM); target_display_R = -MAX_RPM; }

            OLED_SetCursor(0, 0); OLED_PrintString("PTCH: "); OLED_PrintNum(Pitch);
            OLED_SetCursor(0, 1); OLED_PrintString("ROLL: "); OLED_PrintNum(Roll);

            char setpoint_buf[30];
            sprintf(setpoint_buf, "SETP L:%-3d R:%-3d ", target_display_L, target_display_R);
            OLED_SetCursor(0, 3); OLED_PrintString(setpoint_buf);

            uint8_t payload[5];
            payload[0] = 0xAA;
            payload[1] = (Pitch >> 8) & 0xFF;
            payload[2] = Pitch & 0xFF;
            payload[3] = (Roll >> 8) & 0xFF;
            payload[4] = Roll & 0xFF;

            nRF24_Transmit_Packet(payload, 5);

            uint8_t status = 0;
            while (1) {
                status = nRF24_ReadReg(0x07);
                if (status & ((1U << 5) | (1U << 4))) break;
            }
            nRF24_WriteReg(0x07, status);

            if (status & (1U << 4)) {
                GPIOA->BSRR = (1U << (4 + 16));
                SPI1_TransmitReceive(NRF_CMD_FLUSH_TX);
                GPIOA->BSRR = (1U << 4);

                OLED_SetCursor(0, 5); OLED_PrintString("STATUS: NO CONNECT!");
                nRF24_Init_TX(); // <--- THEM DONG NAY DE RESET LAI BO PHAT
            } else if (status & (1U << 5)) {
                OLED_SetCursor(0, 5); OLED_PrintString("STATUS: TX SUCCESS!");

                uint8_t fifo_status = nRF24_ReadReg(0x17);
                if (!(fifo_status & 0x01)) {
                    uint8_t ack_payload[6];
                    nRF24_ReadBuf(0x61, ack_payload, 6);

                    int16_t live_speed = (int16_t)((ack_payload[0] << 8) | ack_payload[1]);
                    int16_t live_pwm   = (int16_t)((ack_payload[4] << 8) | ack_payload[5]);

                    char speed_buf[30], pwm_buf[30];
                    sprintf(speed_buf, "SPEED : %-3d RPM    ", live_speed);
                    sprintf(pwm_buf,   "PWM   : %-4d /1000  ", live_pwm);

                    OLED_SetCursor(0, 6); OLED_PrintString(speed_buf);
                    OLED_SetCursor(0, 7); OLED_PrintString(pwm_buf);
                }
            }
            delay_ms(100);
        }
    }
}
