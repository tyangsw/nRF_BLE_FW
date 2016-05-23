/*
 * I2C support functions
 *
 * @TODO: added support for second port
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 
#include "includes.h"

/* Global variables */
/* TWI instance. */
static const nrf_drv_twi_t twiInstance = NRF_DRV_TWI_INSTANCE(0);

/* local function */
static ret_code_t i2c_read(uint8_t, uint8_t, uint8_t *, uint32_t);

/**
 * @brief I2C initialization.
 */
ret_code_t i2c_init (void)
{
    ret_code_t rc;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = I2C_SCL_PIN,
       .sda                = I2C_SDA_PIN,
       .frequency          = I2C_SCLK_RATE,
       .interrupt_priority = I2C_APP_IRQ_PRIORITY
    };
    
    rc = nrf_drv_twi_init(&twiInstance, &twi_config, NULL, NULL);
    
    if(rc)
		return rc;
	
	nrf_drv_twi_enable(&twiInstance);
    
    return rc;
}

static ret_code_t i2c_read(uint8_t chip, uint8_t addr, uint8_t *buf, uint32_t len)
{
    ret_code_t rc;

    /* Send address */
    rc = nrf_drv_twi_tx(&twiInstance, chip, &addr, len, false);
    
    if (rc)
        return rc;

    /* Receive data */
    rc = nrf_drv_twi_rx(&twiInstance, chip, buf, len, false);
    
    if (rc)
        return rc;
		
    return rc;
}

static ret_code_t i2c_write(uint8_t chip, uint8_t addr, uint8_t *buf, uint32_t len)
{
	uint8_t			lbuf[I2C_MAX_WRITE_LEN + 1];
	ret_code_t	rc;

  /* Make sure length doesn't exceed max */
	if (len > I2C_MAX_WRITE_LEN)
	{
		printf("Error in i2c_write: len > max (len=%d, max=%d)\n", len, I2C_MAX_WRITE_LEN);
		return NRF_ERROR_DATA_SIZE;
	}

	/* Copy address and data into local buffer - they must be sent in one transfer */
	lbuf[0] = addr;
	memcpy(&lbuf[1], &buf[0], len);
	
	/* Send address + data */
	rc = nrf_drv_twi_tx(&twiInstance, chip, lbuf, len, false);

	return rc;
}



ret_code_t i2c_dev_read(uint8_t chip, uint8_t addr, uint8_t *buf, uint32_t len, bool verbose)
{
    ret_code_t rc;

    memset(buf, 0, len);
    rc = i2c_read(chip, addr, buf, len);
    if (rc && verbose)
        printf("Error reading I2C chip %02X addr %02X\n", chip, addr);

    return rc;
}



ret_code_t i2c_dev_write(uint8_t chip, uint8_t addr, uint8_t *buf, uint32_t len, bool verbose)
{
    ret_code_t rc;

    rc = i2c_write(chip, addr, buf, len);
    
	if (rc && verbose)
        printf("Error writing I2C chip %02X addr %02X\n", chip, addr);

    /* Wait for write operation to complete on actual memory */
    nrf_delay_us(10000);

    return rc;
}

ret_code_t i2c_probe(uint8_t chip)
{
    ret_code_t rc;
    uint8_t      buf;

    rc = i2c_read(chip, 0, &buf, 1);

    return rc;
}




