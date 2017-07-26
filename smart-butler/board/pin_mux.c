/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MKW41Z512xxx4
package_id: MKW41Z512VHT4
mcu_data: ksdk2_0
processor_version: 2.0.1
board: FRDM-KW41Z
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*FUNCTION**********************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

#define PCR_PE_DISABLED               0x00u   /*!< Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
#define PCR_SRE_SLOW                  0x01u   /*!< Slew Rate Enable: Slow slew rate is configured on the corresponding pin, if the pin is configured as a digital output. */
#define PIN0_IDX                         0u
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 4 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN18_IDX                       18u
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port */
#define SOPT5_LPUART0RXSRC_LPUART_RX  0x00u   /*!< LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART0TXSRC_LPUART_TX  0x00u   /*!< LPUART0 Transmit Data Source Select: LPUART0_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '37', peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: PTC1/ANT_B/I2C0_SDA/UART0_RTS_b/TPM0_CH2/BLE_RF_ACTIVE}
  - {pin_num: '42', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: TSI0_CH2/PTC6/LLWU_P14/XTAL_OUT_EN/I2C1_SCL/UART0_RX/TPM2_CH0/BSM_FRAME, pull_enable: disable}
  - {pin_num: '43', peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: TSI0_CH3/PTC7/LLWU_P15/SPI0_PCS2/I2C1_SDA/UART0_TX/TPM2_CH1/BSM_DATA}
  - {pin_num: '45', peripheral: GPIOC, signal: 'GPIO, 16', pin_signal: TSI0_CH4/PTC16/LLWU_P0/SPI0_SCK/I2C0_SDA/UART0_RTS_b/TPM0_CH3}
  - {pin_num: '48', peripheral: GPIOC, signal: 'GPIO, 19', pin_signal: TSI0_CH7/PTC19/LLWU_P3/SPI0_PCS0/I2C0_SCL/UART0_CTS_b/BSM_CLK/BLE_RF_ACTIVE}
  - {pin_num: '40', peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: TSI0_CH0/PTC4/LLWU_P12/ANT_A/EXTRG_IN/UART0_CTS_b/TPM1_CH0/BSM_DATA}
  - {pin_num: '38', peripheral: LPUART0, signal: RX, pin_signal: TSI0_CH14/PTC2/LLWU_P10/TX_SWITCH/I2C1_SCL/UART0_RX/CMT_IRO/DTM_RX, slew_rate: slow}
  - {pin_num: '39', peripheral: LPUART0, signal: TX, pin_signal: TSI0_CH15/PTC3/LLWU_P11/RX_SWITCH/I2C1_SDA/UART0_TX/TPM0_CH1/DTM_TX, identifier: ''}
  - {pin_num: '46', peripheral: GPIOC, signal: 'GPIO, 17', pin_signal: TSI0_CH5/PTC17/LLWU_P1/SPI0_SOUT/I2C1_SCL/UART0_RX/BSM_FRAME/DTM_RX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTC, PIN1_IDX, kPORT_MuxAsGpio);          /* PORTC1 (pin 37) is configured as PTC1 */
  PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAsGpio);         /* PORTC16 (pin 45) is configured as PTC16 */
  PORT_SetPinMux(PORTC, PIN17_IDX, kPORT_MuxAsGpio);         /* PORTC17 (pin 46) is configured as PTC17 */
	PORT_SetPinMux(PORTC, PIN18_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTC, PIN19_IDX, kPORT_MuxAsGpio);         /* PORTC19 (pin 48) is configured as PTC19 */
	
 const port_pin_config_t portc5_pin40_config = {
    kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
    kPORT_SlowSlewRate,                                      /* Slow slew rate is configured */
    kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
    kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
    kPORT_MuxAsGpio,                                         /* Pin is configured as PTC4 */
  };
  PORT_SetPinConfig(PORTC, PIN5_IDX, &portc5_pin40_config);
	
  PORT_SetPinMux(PORTC, PIN2_IDX, kPORT_MuxAlt4);            /* PORTC2 (pin 38) is configured as UART0_RX */
  PORTC->PCR[2] = ((PORTC->PCR[2] &
    (~(PORT_PCR_SRE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_SRE(PCR_SRE_SLOW)                           /* Slew Rate Enable: Slow slew rate is configured on the corresponding pin, if the pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTC, PIN3_IDX, kPORT_MuxAlt4);            /* PORTC3 (pin 39) is configured as UART0_TX */
  
  PORT_SetPinMux(PORTA, PIN0_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTA, PIN16_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTA, PIN17_IDX, kPORT_MuxAsGpio);
  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_MuxAsGpio);
  
  PORT_SetPinMux(PORTC, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTC4 (pin 40) is configured as PTC4 */
  PORT_SetPinMux(PORTC, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTC4 (pin 40) is configured as PTC4 */
  PORT_SetPinMux(PORTC, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTC6 (pin 42) is configured as PTC6 */
  PORTC->PCR[6] = ((PORTC->PCR[6] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_DISABLED)                         /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
    );
  PORT_SetPinMux(PORTC, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTC7 (pin 43) is configured as PTC7 */
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK))) /* Mask bits to zero which are setting */
      | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_LPUART_TX) /* LPUART0 Transmit Data Source Select: LPUART0_TX pin */
      | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX) /* LPUART0 Receive Data Source Select: LPUART_RX pin */
    );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
