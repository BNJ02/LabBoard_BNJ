#include "i2c.h"
#include "stm32f3xx.h"

// use I2C:
// SCL : PB6
// SDA : PB7

void setupI2C() {
    /***** STEPS FOLLOWED *****
    1/ Program the I2C_CR1 Register to disable the peripheral
    1/ Enable the I2C Clock and GPIO Clock
    2/ Configure the I2C PINSs for Alternate functions
        a) Select Alternate Function (AF) in MODER Register
        b) Select Open Drain Output
        c) Select High SPEED for the PINs
        d) Select Pull-up for the both PINs
        e) Configure the Alternate Function (AF) in AFR Register
    3/ Reset the I2C
    4/ Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
    5/ Program the I2C_CR1 Register to enable the peripheral
    ***************************/

    // 1 : Program the I2C_CR1 Register to enable the peripheral
    I2C1->CR1 &= ~(1 << I2C_CR1_PE_Pos); // Peripheral Disabled

    // 2 : ANF (Analog Noise Filter) & DNF (Digital Noise Filter) in CR1 register
    I2C1->CR1 &= ~(1 << I2C_CR1_ANFOFF_Pos); // ANF disabled

    // 3 : Configure NOSTRETCH in CR1 register
    I2C1->CR1 &= ~(1 << I2C_CR1_NOSTRETCH_Pos); // NOSTRETCH disabled in master mode

    // 1
    // Enable the I2C Clock and GPIO Clock
    RCC->APB1ENR |= (1<<21); // enable I2C clock (p.152 STM32F303RM)
    __asm__("nop");          // wait until I2C clock is OK
    // reset peripheral (mandatory!)
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    // Enable the GPIO Clock
    RCC->AHBENR |=  RCC_AHBENR_GPIOAEN | // clock for GPIOA
                    RCC_AHBENR_GPIOBEN;  // clock for GPIOB
    __asm__("nop");                      // wait until GPIOx clock is OK

    // 2
    // a) Select Alternate Function (AF) in MODER Register for SDA (PB7) & SCL (PB6)
    GPIOB->MODER |= 2 << GPIO_MODER_MODER6_Pos |
                    2 << GPIO_MODER_MODER7_Pos;

    // b) Select Open Drain Output on SDA & SCL
    GPIOB->OTYPER |= 1 << 6 | 1 << 7;

    // c) Select High SPEED for the PINs on SDA & SCL
    GPIOB->OSPEEDR |=   3 << GPIO_OSPEEDER_OSPEEDR6_Pos |
                        3 << GPIO_OSPEEDER_OSPEEDR7_Pos;

    // d) Select Pull-up for the both PINs (I2C obligation)
    GPIOB->PUPDR |= 1 << GPIO_PUPDR_PUPDR6_Pos |
                    1 << GPIO_PUPDR_PUPDR7_Pos;

    // e) Configure the Alternate Function (AF) in AFR Register
    GPIOB->AFR[0] |= 4 << GPIO_AFRL_AFRL6_Pos |   // alternate func AF4 for I2C
                     4 << GPIO_AFRL_AFRL7_Pos |   // in datasheet Physical

    // 3 : Reset the I2C
    I2C1->CR1 |= 1 << I2C_CR1_SWRST_Pos;
    I2C1->CR1 &= ~(1 << I2C_CR1_SWRST_Pos);

    // 4 : Program the peripheral input clock to generate correct timings
    I2C1->TIMINGR |=    1 << I2C_TIMINGR_PRESC_Pos |    // configuration Standard mode (Sm) : 100kHz
                        0x13 << I2C_TIMINGR_SCLL_Pos |  // for f(I2CCLK) = 8 MHz
                        0xF << I2C_TIMINGR_SCLH_Pos |   // p.849
                        0x2 << I2C_TIMINGR_SDADEL_Pos |
                        0x4 << I2C_TIMINGR_SCLDEL_Pos;

    // 5 : Program the I2C_CR1 Register to enable the peripheral
    I2C1->CR1 |= 1 << I2C_CR1_PE_Pos; // Peripheral Enabled

    // 6 : Enable I2C interrupts
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void startI2C(uint8_t slave_address, bool RW, uint8_t nb_bytes)
{
    /***** STEP FOLLOWED *****
    1/ Send the START condition
    ***************************/
    // Addressing mode
    I2C1->CR2 &= ~(1 << I2C_CR2_ADD10_Pos); // slave address on 7 bits not 10 bits

    // Storage slave address
    I2C1->CR2 |= slave_address << I2C_CR2_SADD_Pos;
    I2C1->CR2 &= ~(slave_address << I2C_CR2_SADD_Pos);

    // Read or Write
    RW ? I2C1->CR2 |= 1 << I2C_CR2_RD_WRN_Pos : I2C1->CR2 &= ~(1 << I2C_CR2_RD_WRN_Pos);

    // Number of bytes to be transmitted or to be received
    I2C1->CR2 |= nb_bytes << I2C_CR2_NBYTES_Pos;
    I2C1->CR2 &= ~(nb_bytes << I2C_CR2_NBYTES_Pos);

    // Autoend mode
    I2C1->CR2 |= 1 << I2C_CR2_AUTOEND_Pos; // enabled

    // Start
    I2C->CR2 |= 1 << I2C_CR2_START_Pos;
}

void writeI2C(uint8_t slave_address, uint8_t data, uint8_t nb_bytes = 1)
{
    startI2C(slave_address, false, nb_bytes);

    for(nb_bytes; nb_bytes != 0; nb_bytes--)
    {
        while((I2C1->ISR | I2C_ISR_TXIS_Pos) == 0);

        I2C1->TXDR |= data << I2C_TXDR_TXDATA_Pos;
        I2C1->TXDR &= ~(data << I2C_TXDR_TXDATA_Pos);
    }
}

uint8_t readI2C(uint8_t slave_address, uint8_t nb_bytes = 1)
{
    uint8_t data = 0;

    startI2C(slave_address, false, nb_bytes);

    for(nb_bytes; nb_bytes != 0; nb_bytes--)
    {
        while((I2C1->ISR | I2C_ISR_RXNE_Pos) == 0);

        I2C1->RXDR |= data << I2C_TXDR_TXDATA_Pos;
        data |= (I2C1->RXDR | (0xFF << I2C_TXDR_TXDATA_Pos)) >> I2C_TXDR_TXDATA_Pos;
    }

    return data;
}