/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v12.0
processor: MKE06Z128xxx4
package_id: MKE06Z128VLK4
mcu_data: ksdk2_0
processor_version: 12.0.0
board: FRDM-KE06Z
pin_labels:
- {pin_num: '61', pin_signal: PTA1/KBI0_P1/FTM0_CH1/I2C0_4WSDAOUT/ACMP0_IN1/ADC0_SE1, label: 'J1[10]/PTA1_D4_T1/PTA1_IRRX', identifier: IRRX;SPI1_HOLD}
- {pin_num: '62', pin_signal: PTA0/KBI0_P0/FTM0_CH0/I2C0_4WSCLOUT/ACMP0_IN0/ADC0_SE0, label: 'J2[2]/PTA0_D8_T0', identifier: SPI_WP;SPI1_WP}
- {pin_num: '19', pin_signal: PTH0/KBI1_P24/FTM2_CH0, label: 'J2[4]/PTH0_D9_PWM0', identifier: SYS_ALARM_LED}
- {pin_num: '60', pin_signal: PTA2/KBI0_P2/UART0_RX/I2C0_SDA, label: 'J2[18]/PTA2_ACCEL_SDA', identifier: ACCEL_SDA;I2C_SDA_HUMI}
- {pin_num: '59', pin_signal: PTA3/KBI0_P3/UART0_TX/I2C0_SCL, label: 'J2[20]/PTA3_ACCEL_SCL', identifier: ACCEL_SCL;I2C_SCL_HUMI}
- {pin_num: '77', pin_signal: PTC5/KBI0_P21/FTM1_CH1/RTCO, label: 'J1[11]/PTC5_T2', identifier: RUN_LED}
- {pin_num: '20', pin_signal: PTE6/KBI1_P6, label: 'J1[13]/PTE6_LED2', identifier: SYS_NOMAL;SYS_NORMAL}
- {pin_num: '76', pin_signal: PTE0/KBI1_P0/SPI0_SCK/TCLK1/I2C1_SDA, label: 'J4[1]/PTE0_SPI0_SCK', identifier: I2C_SDA_RTC}
- {pin_num: '75', pin_signal: PTE1/KBI1_P1/SPI0_MOSI/I2C1_SCL, label: 'J4[3]/PTE1_SPI0_MOSI', identifier: I2C_SCL_RTC}
- {pin_num: '67', pin_signal: PTE3/KBI1_P3/SPI0_PCS, label: 'J4[7]/PTE3_SPI0_SS', identifier: DIR_RS485}
- {pin_num: '18', pin_signal: PTH1/KBI1_P25/FTM2_CH1, label: 'J5[7]', identifier: SYS_FAULT_LED}
- {pin_num: '53', pin_signal: PTG4/KBI1_P20/FTM2_CH2/SPI1_SCK, label: 'J5[2]', identifier: SPI1_CLK}
- {pin_num: '17', pin_signal: PTI0/IRQ/UART2_RX, label: 'J5[8]', identifier: UART_RX}
- {pin_num: '16', pin_signal: PTI1/IRQ/UART2_TX, label: 'J5[12]', identifier: UART_TX}
- {pin_num: '65', pin_signal: PTI3/IRQ, label: 'J5[16]', identifier: SPI1_CS}
- {pin_num: '52', pin_signal: PTG5/KBI1_P21/FTM2_CH3/SPI1_MOSI, label: 'D4[1]/PTG5_RED', identifier: LED_RED;SPI1_MOSI}
- {pin_num: '51', pin_signal: PTG6/KBI1_P22/FTM2_CH4/SPI1_MISO, label: 'D4[4]/PTG6_GREEN', identifier: LED_GREEN;SPI1_MISO}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_gpio_led();
    BOARD_rs485();
    BOARD_flash_spi();
    BOARD_i2c();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_gpio_led:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '77', peripheral: GPIOA, signal: 'GPIO, 21', pin_signal: PTC5/KBI0_P21/FTM1_CH1/RTCO, direction: OUTPUT}
  - {pin_num: '18', peripheral: GPIOB, signal: 'GPIO, 25', pin_signal: PTH1/KBI1_P25/FTM2_CH1, direction: OUTPUT}
  - {pin_num: '19', peripheral: GPIOB, signal: 'GPIO, 24', pin_signal: PTH0/KBI1_P24/FTM2_CH0, direction: OUTPUT}
  - {pin_num: '20', peripheral: GPIOB, signal: 'GPIO, 6', pin_signal: PTE6/KBI1_P6, identifier: SYS_NORMAL, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_gpio_led
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_gpio_led(void)
{

    gpio_pin_config_t RUN_LED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA21 (pin 77) */
    GPIO_PinInit(BOARD_RUN_LED_GPIO_PORT, BOARD_RUN_LED_PIN, &RUN_LED_config);

    gpio_pin_config_t SYS_NORMAL_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB6 (pin 20) */
    GPIO_PinInit(BOARD_SYS_NORMAL_GPIO_PORT, BOARD_SYS_NORMAL_PIN, &SYS_NORMAL_config);

    gpio_pin_config_t SYS_ALARM_LED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB24 (pin 19) */
    GPIO_PinInit(BOARD_SYS_ALARM_LED_GPIO_PORT, BOARD_SYS_ALARM_LED_PIN, &SYS_ALARM_LED_config);

    gpio_pin_config_t SYS_FAULT_LED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB25 (pin 18) */
    GPIO_PinInit(BOARD_SYS_FAULT_LED_GPIO_PORT, BOARD_SYS_FAULT_LED_PIN, &SYS_FAULT_LED_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_rs485:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '16', peripheral: UART2, signal: TX, pin_signal: PTI1/IRQ/UART2_TX}
  - {pin_num: '17', peripheral: UART2, signal: RX, pin_signal: PTI0/IRQ/UART2_RX}
  - {pin_num: '67', peripheral: GPIOB, signal: 'GPIO, 3', pin_signal: PTE3/KBI1_P3/SPI0_PCS, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_rs485
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_rs485(void)
{

    gpio_pin_config_t DIR_RS485_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB3 (pin 67) */
    GPIO_PinInit(BOARD_RS485_DIR_RS485_GPIO_PORT, BOARD_RS485_DIR_RS485_PIN, &DIR_RS485_config);
    /* pin 16,17 is configured as UART2_TX, UART2_RX */
    PORT_SetPinSelect(kPORT_UART2, kPORT_UART2_RXPTI1_TXPTI0);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_flash_spi:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '53', peripheral: SPI1, signal: SCK, pin_signal: PTG4/KBI1_P20/FTM2_CH2/SPI1_SCK}
  - {pin_num: '52', peripheral: SPI1, signal: MOSI, pin_signal: PTG5/KBI1_P21/FTM2_CH3/SPI1_MOSI, identifier: SPI1_MOSI}
  - {pin_num: '51', peripheral: SPI1, signal: MISO, pin_signal: PTG6/KBI1_P22/FTM2_CH4/SPI1_MISO, identifier: SPI1_MISO}
  - {pin_num: '65', peripheral: GPIOC, signal: 'GPIO, 3', pin_signal: PTI3/IRQ, direction: OUTPUT, gpio_init_state: 'false'}
  - {pin_num: '62', peripheral: GPIOA, signal: 'GPIO, 0', pin_signal: PTA0/KBI0_P0/FTM0_CH0/I2C0_4WSCLOUT/ACMP0_IN0/ADC0_SE0, identifier: SPI1_WP, direction: OUTPUT,
    gpio_init_state: 'false'}
  - {pin_num: '61', peripheral: GPIOA, signal: 'GPIO, 1', pin_signal: PTA1/KBI0_P1/FTM0_CH1/I2C0_4WSDAOUT/ACMP0_IN1/ADC0_SE1, identifier: SPI1_HOLD, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_flash_spi
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_flash_spi(void)
{

    gpio_pin_config_t SPI1_WP_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA0 (pin 62) */
    GPIO_PinInit(BOARD_FLASH_SPI_SPI1_WP_GPIO_PORT, BOARD_FLASH_SPI_SPI1_WP_PIN, &SPI1_WP_config);

    gpio_pin_config_t SPI1_HOLD_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA1 (pin 61) */
    GPIO_PinInit(BOARD_FLASH_SPI_SPI1_HOLD_GPIO_PORT, BOARD_FLASH_SPI_SPI1_HOLD_PIN, &SPI1_HOLD_config);

    gpio_pin_config_t SPI1_CS_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC3 (pin 65) */
    GPIO_PinInit(BOARD_FLASH_SPI_SPI1_CS_GPIO_PORT, BOARD_FLASH_SPI_SPI1_CS_PIN, &SPI1_CS_config);
    /* pin 53,52,51 is configured as SPI1_SCK, SPI1_MOSI, SPI1_MISO */
    PORT_SetPinSelect(kPORT_SPI1, kPORT_SPI1_SCKPTG4_MOSIPTG5_MISOPTG6_PCSPTG7);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_i2c:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '76', peripheral: GPIOB, signal: 'GPIO, 0', pin_signal: PTE0/KBI1_P0/SPI0_SCK/TCLK1/I2C1_SDA, direction: OUTPUT}
  - {pin_num: '75', peripheral: GPIOB, signal: 'GPIO, 1', pin_signal: PTE1/KBI1_P1/SPI0_MOSI/I2C1_SCL, direction: OUTPUT}
  - {pin_num: '60', peripheral: GPIOA, signal: 'GPIO, 2', pin_signal: PTA2/KBI0_P2/UART0_RX/I2C0_SDA, identifier: I2C_SDA_HUMI, direction: OUTPUT}
  - {pin_num: '59', peripheral: GPIOA, signal: 'GPIO, 3', pin_signal: PTA3/KBI0_P3/UART0_TX/I2C0_SCL, identifier: I2C_SCL_HUMI, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_i2c
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_i2c(void)
{

    gpio_pin_config_t I2C_SDA_HUMI_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA2 (pin 60) */
    GPIO_PinInit(BOARD_I2C_I2C_SDA_HUMI_GPIO_PORT, BOARD_I2C_I2C_SDA_HUMI_PIN, &I2C_SDA_HUMI_config);

    gpio_pin_config_t I2C_SCL_HUMI_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA3 (pin 59) */
    GPIO_PinInit(BOARD_I2C_I2C_SCL_HUMI_GPIO_PORT, BOARD_I2C_I2C_SCL_HUMI_PIN, &I2C_SCL_HUMI_config);

    gpio_pin_config_t I2C_SDA_RTC_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB0 (pin 76) */
    GPIO_PinInit(BOARD_I2C_I2C_SDA_RTC_GPIO_PORT, BOARD_I2C_I2C_SDA_RTC_PIN, &I2C_SDA_RTC_config);

    gpio_pin_config_t I2C_SCL_RTC_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB1 (pin 75) */
    GPIO_PinInit(BOARD_I2C_I2C_SCL_RTC_GPIO_PORT, BOARD_I2C_I2C_SCL_RTC_PIN, &I2C_SCL_RTC_config);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
