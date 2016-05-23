/*
 * Master include file.
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 
/* Standard headers */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <inttypes.h>


/* nRF headers */
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_twi.h"
#include "nrf.h"
#include "app_uart.h"
#include "app_error.h"
#include "bsp.h"

/* Local headers */
/*#include "common.h"*/
#include "uart.h"
/*#include "i2c.h"*/

#if SERIAL_DEBUG
#include "menu.h"
#endif

