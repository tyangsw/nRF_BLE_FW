/*
 * UART Support for Spore
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 
#include <includes.h>

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

static void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init()
{
	uint32_t err_code;
	const app_uart_comm_params_t comm_params =
	{

			SER_APP_RX_PIN,
			SER_APP_TX_PIN,
			SER_APP_RTS_PIN,
			SER_APP_CTS_PIN,
			APP_UART_FLOW_CONTROL_ENABLED,
			false,
			UART_BAUDRATE_BAUDRATE_Baud38400
	  };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}
