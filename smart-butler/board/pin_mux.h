#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* PORTC6 (number 42), J1[1]/D0/UART0_RX_TGTMCU */
#define BOARD_INITPINS_DEBUG_UART_RX_GPIO                                  GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_DEBUG_UART_RX_PORT                                  PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_DEBUG_UART_RX_GPIO_PIN                                 6U   /*!< PORTC pin index: 6 */
#define BOARD_INITPINS_DEBUG_UART_RX_PIN_NAME                               PTC6   /*!< Pin name */
#define BOARD_INITPINS_DEBUG_UART_RX_LABEL            "J1[1]/D0/UART0_RX_TGTMCU"   /*!< Label */
#define BOARD_INITPINS_DEBUG_UART_RX_NAME                        "DEBUG_UART_RX"   /*!< Identifier name */

/* PORTC7 (number 43), J1[2]/D1/UART0_TX_TGTMCU */
#define BOARD_INITPINS_DEBUG_UART_TX_GPIO                                  GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_DEBUG_UART_TX_PORT                                  PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_DEBUG_UART_TX_GPIO_PIN                                 7U   /*!< PORTC pin index: 7 */
#define BOARD_INITPINS_DEBUG_UART_TX_PIN_NAME                               PTC7   /*!< Pin name */
#define BOARD_INITPINS_DEBUG_UART_TX_LABEL            "J1[2]/D1/UART0_TX_TGTMCU"   /*!< Label */
#define BOARD_INITPINS_DEBUG_UART_TX_NAME                        "DEBUG_UART_TX"   /*!< Identifier name */

/* PORTC4 (number 40), J1[5]/D4/SW3 */
#define BOARD_INITPINS_SW3_GPIO                                            GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_SW3_PORT                                            PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_SW3_GPIO_PIN                                           4U   /*!< PORTC pin index: 4 */
#define BOARD_INITPINS_SW3_PIN_NAME                                         PTC4   /*!< Pin name */
#define BOARD_INITPINS_SW3_LABEL                                  "J1[5]/D4/SW3"   /*!< Label */
#define BOARD_INITPINS_SW3_NAME                                            "SW3"   /*!< Identifier name */

/* PORTC2 (number 38), J2[10]/U9[4]/D15/I2C1_SCL */
#define BOARD_INITPINS_ACCEL_SCL_PERIPHERAL                              LPUART0   /*!< Device name: LPUART0 */
#define BOARD_INITPINS_ACCEL_SCL_SIGNAL                                       RX   /*!< LPUART0 signal: RX */
#define BOARD_INITPINS_ACCEL_SCL_PIN_NAME                               UART0_RX   /*!< Pin name */
#define BOARD_INITPINS_ACCEL_SCL_LABEL               "J2[10]/U9[4]/D15/I2C1_SCL"   /*!< Label */
#define BOARD_INITPINS_ACCEL_SCL_NAME                                "ACCEL_SCL"   /*!< Identifier name */

/* PORTC17 (number 46), J1[6]/U4[1]/D5 */
#define BOARD_INITPINS_FLASH_SI_GPIO                                       GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_FLASH_SI_PORT                                       PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_FLASH_SI_GPIO_PIN                                     17U   /*!< PORTC pin index: 17 */
#define BOARD_INITPINS_FLASH_SI_PIN_NAME                                   PTC17   /*!< Pin name */
#define BOARD_INITPINS_FLASH_SI_LABEL                           "J1[6]/U4[1]/D5"   /*!< Label */
#define BOARD_INITPINS_FLASH_SI_NAME                                  "FLASH_SI"   /*!< Identifier name */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
