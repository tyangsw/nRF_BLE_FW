/*
 * Spore nRF test cli
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 * 
 */

#include "includes.h"

/**
 * @brief Function for main application entry.
 */
int main(void)
{

	char c;
	
	/*configure debug uart*/
	uart_init();
	/*configure i2c port**/
	i2c_init ();
	printf("\n\rStart: \n\r");

	while (true)
  {
			c = getchar();
			if(c == ESCAPE)
				test_menu();
	}
#if 0
    while (true)
    {
        uint8_t cr;
        while(app_uart_get(&cr) != NRF_SUCCESS);
        while(app_uart_put(cr) != NRF_SUCCESS);

        if (cr == 'q' || cr == 'Q')
        {
            printf(" \n\rExit!\n\r");
						break;

        }
    }

		while (true)
		{
				// Do nothing.
		}
#endif
}


/** @} */
