#ifndef FLAGS_H_
#define FLAGS_H_

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif

#include <stdlib.h>
#include <stdint.h>

struct TimerFlags
{
  /*
  BIT 7: UNUSED
  BIT 6: UNUSED
  BIT 5: UNUSED
  BIT 4: Sub-channel 4 Timeout Function Scheduled
  BIT 3: Sub-channel 3 Timeout Function Scheduled
  BIT 2: Sub-channel 2 Timeout Function Scheduled
  BIT 1: Sub-channel 1 Timeout Function Scheduled
  BIT 0: UNUSED
  */
  volatile uint8_t uart_timeout_func = 0x00;
};
extern TimerFlags TIMER2_USER_FLAGS;

#define UART_TIMEOUT_FUNC1_Pos ( 1u )
#define UART_TIMEOUT_FUNC1_Msk ( 0x1u << UART_TIMEOUT_FUNC1_Pos )
#define UART_TIMEOUT_FUNC2_Pos ( 2u )
#define UART_TIMEOUT_FUNC2_Msk ( 0x1u << UART_TIMEOUT_FUNC2_Pos )
#define UART_TIMEOUT_FUNC3_Pos ( 3u )
#define UART_TIMEOUT_FUNC3_Msk ( 0x1u << UART_TIMEOUT_FUNC3_Pos )
#define UART_TIMEOUT_FUNC4_Pos ( 4u )
#define UART_TIMEOUT_FUNC4_Msk ( 0x1u << UART_TIMEOUT_FUNC4_Pos )

#endif /* !FLAGS_H_ */