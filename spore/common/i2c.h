/*
 * I2C support definitions.
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */

#ifndef __I2C_H__
#define __I2C_H__

/*
 * Definitions
 */
#define I2C_SCLK_RATE       	NRF_TWI_FREQ_100K
#define I2C_SCL_PIN       		ARDUINO_SCL_PIN
#define I2C_SDA_PIN       		ARDUINO_SDA_PIN
#define I2C_APP_IRQ_PRIORITY	APP_IRQ_PRIORITY_HIGH
/* Worst case timeout for 1 byte is kept as 2ms */
#define I2C_BYTE_TO         2
#define I2C_BYTE_TO_BB      (I2C_BYTE_TO * 16)
#define I2C_MAX_WRITE_LEN   32

/*
 * Function prototypes
 */
ret_code_t i2c_init (void);
ret_code_t i2c_probe(uint8_t);

ret_code_t i2c_dev_read(uint8_t, uint8_t, uint8_t *, uint32_t, bool);
ret_code_t i2c_dev_write(uint8_t, uint8_t, uint8_t *, uint32_t, bool);
 
#endif /* __I2C_H__ */

