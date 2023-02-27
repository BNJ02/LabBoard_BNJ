#include "i2c.h"
#include "stm32f3xx.h"

#define SCL_PIN (6)
#define SDA_PIN (7)
#define LED_PIN (3)

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
    /*
    // 1 : Program the I2C_CR1 Register to enable the peripheral
    I2C1->CR1 &= ~(1 << I2C_CR1_PE_Pos); // Peripheral Disabled

    // 2 : ANF (Analog Noise Filter) & DNF (Digital Noise Filter) in CR1 register
    I2C1->CR1 &= ~(1 << I2C_CR1_ANFOFF_Pos); // ANF disabled

    // 3 : Configure NOSTRETCH in CR1 register
    I2C1->CR1 &= ~(1 << I2C_CR1_NOSTRETCH_Pos); // NOSTRETCH disabled in master mode
    */

    // 1
    // Enable the I2C1 peripheral in 'RCC_APB1ENR' and GPIO Clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable I2C clock (p.152 STM32F303RM)
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
                     4 << GPIO_AFRL_AFRL7_Pos;    // in datasheet Physical

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

void startI2C(uint8_t slave_address, uint8_t RW, uint8_t nb_bytes)
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
    RW ? (I2C1->CR2 |= 1 << I2C_CR2_RD_WRN_Pos) : (I2C1->CR2 &= ~(1 << I2C_CR2_RD_WRN_Pos));

    // Number of bytes to be transmitted or to be received
    I2C1->CR2 |= nb_bytes << I2C_CR2_NBYTES_Pos;
    I2C1->CR2 &= ~(nb_bytes << I2C_CR2_NBYTES_Pos);

    // Autoend mode
    I2C1->CR2 |= 1 << I2C_CR2_AUTOEND_Pos; // enabled

    // Start
    I2C1->CR2 |= 1 << I2C_CR2_START_Pos;
}

void writeI2C(uint8_t slave_address, uint8_t data, uint8_t nb_bytes)
{
    startI2C(slave_address, 0, nb_bytes);

    //while((I2C1->ISR & (1 << I2C_ISR_TXIS_Pos)) == 0);

    for(nb_bytes; nb_bytes > 0; nb_bytes--)
    {
        I2C1->TXDR |= data << I2C_TXDR_TXDATA_Pos;
        I2C1->TXDR &= ~(data << I2C_TXDR_TXDATA_Pos);
    }
}

uint8_t readI2C(uint8_t slave_address, uint8_t nb_bytes)
{
    uint8_t data = 0;

    startI2C(slave_address, 0, nb_bytes);

    for(uint8_t i = nb_bytes; i > 0; i--)
    {
        while((I2C1->ISR & (1 << I2C_ISR_TXIS_Pos)) == 0);

        I2C1->RXDR |= data << I2C_TXDR_TXDATA_Pos;
        data |= (I2C1->RXDR | (0xFF << I2C_TXDR_TXDATA_Pos)) >> I2C_TXDR_TXDATA_Pos;
    }

    return data;
}

void initI2C() {
    // Enable the I2C1 peripheral in 'RCC_APB1ENR'.
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  #ifdef VVC_F0
    // Enable the GPIOB peripheral in 'RCC_AHBENR'.
    RCC->AHBENR   |= RCC_AHBENR_GPIOBEN;
  #elif  VVC_L0
    RCC->IOPENR   |= RCC_IOPENR_IOPBEN;
  #endif
  // Initialize the GPIOB pins.
  // For I2C1, use AF1.
  GPIOB->AFR[SCL_PIN/8] &= ~(0xF << (SCL_PIN*4));
  GPIOB->AFR[SCL_PIN/8] |=  (0x1 << (SCL_PIN*4));
  GPIOB->AFR[SDA_PIN/8] &= ~(0xF << (SDA_PIN*4));
  GPIOB->AFR[SDA_PIN/8] |=  (0x1 << (SDA_PIN*4));
  // B6/7 should be set to 'alt' mode, open-drain, with pull-up.
  GPIOB->MODER  &= ~(0x3 << (SCL_PIN*2));
  GPIOB->MODER  |=  (0x2 << (SCL_PIN*2));
  GPIOB->PUPDR  &= ~(0x3 << (SCL_PIN*2));
  GPIOB->PUPDR  |=  (0x1 << (SCL_PIN*2));
  GPIOB->OTYPER |=  (0x1 << SCL_PIN);
  GPIOB->MODER  &= ~(0x3 << (SDA_PIN*2));
  GPIOB->MODER  |=  (0x2 << (SDA_PIN*2));
  GPIOB->PUPDR  &= ~(0x3 << (SDA_PIN*2));
  GPIOB->PUPDR  |=  (0x1 << (SDA_PIN*2));
  GPIOB->OTYPER |=  (0x1 << SDA_PIN);
  // B3 is connected to an LED on the 'Nucleo' board.
  //    It should be set to push-pull low-speed output.
  GPIOB->MODER  &= ~(0x3 << (LED_PIN*2));
  GPIOB->MODER  |=  (0x1 << (LED_PIN*2));
  GPIOB->OTYPER &= ~(1 << LED_PIN);
  GPIOB->PUPDR  &= ~(0x3 << (LED_PIN*2));

  // Initialize the I2C1 peripheral.
  // First, disable the peripheral.
  I2C1->CR1     &= ~(I2C_CR1_PE);
  // Clear some 'CR1' bits.
  I2C1->CR1     &= ~( I2C_CR1_DNF    |
                      I2C_CR1_ANFOFF |
                      I2C_CR1_SMBHEN |
                      I2C_CR1_SMBDEN );
  // Clear some 'CR2' bits.
  I2C1->CR2     &= ~( I2C_CR2_RD_WRN  |
                      I2C_CR2_NACK    |
                      I2C_CR2_RELOAD  |
                      I2C_CR2_AUTOEND );
  // Clear all 'ICR' flags.
  I2C1->ICR     |=  ( I2C_ICR_ADDRCF   |
                      I2C_ICR_NACKCF   |
                      I2C_ICR_STOPCF   |
                      I2C_ICR_BERRCF   |
                      I2C_ICR_ARLOCF   |
                      I2C_ICR_OVRCF    |
                      I2C_ICR_PECCF    |
                      I2C_ICR_TIMOUTCF |
                      I2C_ICR_ALERTCF  );
  // Configure I2C timing.
  // Reset all but the reserved bits.
  I2C1->TIMINGR &=  (0x0F000000);
  // (100KHz @48MHz core clock, according to an application note)
  I2C1->TIMINGR |=  (0x10420F13);
  // Enable the peripheral. (PE = 'Peripheral Enable')
  I2C1->CR1     |=  (0x000000FF);
  // Master mode
  I2C1->CR2 |= I2C_CR2_AUTOEND;
}

inline void i2c_start(void) {
  // Send 'Start' condition, and wait for acknowledge.
  I2C1->CR2 |=  (I2C_CR2_START);
  while ((I2C1->CR2 & I2C_CR2_START)) {}
}

void i2c_stop(void) {
  // Send 'Stop' condition, and wait for acknowledge.
  I2C1->CR2 |=  (I2C_CR2_STOP);
  //while ((I2C1->CR2 & I2C_CR2_STOP)) {}
  // Reset the ICR ('Interrupt Clear Register') event flag.
  I2C1->ICR |=  (I2C_ICR_STOPCF);
  while ((I2C1->ICR & I2C_ICR_STOPCF)) {}
}

void i2c_write_byte(uint8_t dat) {
  I2C1->TXDR = (I2C1->TXDR & 0xFFFFFF00) | dat;
  // Wait for one of these ISR bits:
  // 'TXIS' ("ready for next byte")
  // 'TC'   ("transfer complete")
  while (!(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_TC))) {}
  // (Also of interest: 'TXE' ("TXDR register is empty") and
  //  'TCR' ("transfer complete, and 'RELOAD' is set."))
}

uint8_t i2c_read_byte(void) {
  // Wait for a byte of data to be available, then read it.
  while (!(I2C1->ISR & I2C_ISR_RXNE)) {}
  return (I2C1->RXDR & 0xFF);
}

uint8_t i2c_read_register(uint8_t reg_addr) {
  // Set '1 byte to send.'
  I2C1->CR2 &= ~(I2C_CR2_NBYTES);
  I2C1->CR2 |=  (0x01 << I2C_CR2_NBYTES_Pos);
  // Start the I2C write transmission.
  i2c_start();
  // Send the register address.
  i2c_write_byte(reg_addr);
  // Stop the I2C write transmission.
  i2c_stop();
  // Set '1 byte to receive.'
  I2C1->CR2 &= ~(I2C_CR2_NBYTES);
  I2C1->CR2 |=  (0x01 << I2C_CR2_NBYTES_Pos);
  // Set 'read' I2C direction.
  I2C1->CR2 |=  (I2C_CR2_RD_WRN);
  // Start the I2C read transmission.
  i2c_start();
  // Read the transmitted data.
  uint8_t read_result = i2c_read_byte();
  // Stop the I2C read transmission.
  i2c_stop();
  // Set 'write' I2C direction again.
  I2C1->CR2 &= ~(I2C_CR2_RD_WRN);

  // Return the read value.
  return read_result;
}