/*
 * CLI Menu
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 

#if SERIAL_DEBUG

#include "includes.h"

/* Local functions */
static void i2c_bus_probe(void);
static void i2c_read_ops(void);
static void i2c_write_ops(void);
//static void test_led_on(void);
//static void test_led_off(void);
static void test_led_reset(void);
static void i2c_menu(void);
static char pend_rx_char(void);
static char *pend_rx_line(void);
static bool ishex(char*, int);
static void pause(void);

static void i2c_bus_probe(void)
{
	uint8_t chip;

	puts("\nProbing I2C bus...\n");

	printf("Valid chip addresses:");
	for (chip = 0; chip < 128; chip++)
	{
			if (i2c_probe(chip) == NRF_SUCCESS)
					printf(" %02X", chip);
	}
	putchar('\n');

	pause();
}

static void test_led_reset(void)
{
	uint8_t  data[2];
	
	/*set mode*/
	data[0] = 0xA5;
	data[1] = 0x5A;
			
	i2c_dev_write(0x68, 0x00, data, 2, true);
		
	pause();
}

void test_led_on(void)
{
	int i = 0;
	uint8_t  data[24];
	
	/*set mode*/
	data[0] = 0;
	data[1] = 0x20;
	
	for(i = 2; i < 18; i++)
		data[i] = 0x2f;
	
	data[18] = 0xFF;
	data[19] = 0x0;
	data[20] = 0x55;
	data[21] = 0x00;
	data[22] = 0x00;
	data[23] = 0x00;
		
	i2c_dev_write(0x60, 0x80, data, 24, true);
		
	//pause();
}

void test_led_off(void)
{
	int i = 0;
	uint8_t  data[24];
	
	/*set mode*/
	data[0] = 0;
	data[1] = 0;
	
	for(i = 2; i < 18; i++)
		data[i] = 0x00;
	
	data[18] = 0x00;
	data[19] = 0x0;
	data[20] = 0x00;
	data[21] = 0x00;
	data[22] = 0x00;
	data[23] = 0x00;
		
	i2c_dev_write(0x60, 0x80, data, 24, true);
		
	//pause();
	
}

static void i2c_write_ops(void)
{
	uint32_t				val;
	char						*inbuf;
	static uint8_t	chip = 0;
	static uint8_t	reg = 0;
	static uint8_t  data = 0;
	
	puts("I2C Write OPS");
	/* Get I2C device addr */
	printf("I2C Address [%X]: ", chip);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
		pause();
		return;
	}
		
	if (strlen(inbuf))
	{
		if (ishex(inbuf, sizeof(uint8_t)) 
				&& (sscanf(inbuf, "%x", &val) == 1) 
					&& ((val > 0x0) && (val <= MAX_I2C_ADDR)))
		{
				chip = val;
		}
		else
		{
				printf("Valid Address are 0x0 to 0x%02X",MAX_I2C_ADDR);
				pause();
				return;
		}
	}
	
	/*get reg addr*/
	printf("Device Reg [%X]: ", reg);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
		pause();
		return;
	}

	if (strlen(inbuf))
	{
		if (ishex(inbuf, sizeof(uint8_t)) 
			&& (sscanf(inbuf, "%x", &val) == 1))
		{
			reg = val;
		}
		else
		{
			puts("Invalid register");
			pause();
			return;
		}
	}
	
	/* Get value */
	printf("Value [%X]: ", data);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
		pause();
		return;
	}
	if (strlen(inbuf))
	{
		if (ishex(inbuf, sizeof(uint8_t)) && (sscanf(inbuf, "%x", &val) == 1) && (val < 0x100))
		{
			data = val;
		}
		else
		{
			puts("Valid values are 0x0 to 0xFF");
			pause();
			return;
		}
	}
	putchar('\n');
	
	printf("chip [%X] reg [%X] data [%X]: \n",chip, reg, data);
	/* Write it */
	i2c_dev_write(chip, reg, (uint8_t *)&data, 1, true);	
	
	pause();
	
}

static void i2c_read_ops(void)
{
	uint32_t				i;
	uint32_t				val;
	ret_code_t			rc;
	char						*inbuf;
	static uint8_t	chip = 0;
	static uint8_t	reg = 0;
	static uint32_t len = 1;
	static uint8_t	buf[MAX_I2C_BUF_LEN];
	
	puts("I2C Read OPS");
	/* Get I2C device addr */
	printf("I2C Address [%X]: ", chip);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
			pause();
			return;
	}
		
	if (strlen(inbuf))
	{
			if (ishex(inbuf, sizeof(uint8_t)) 
					&& (sscanf(inbuf, "%x", &val) == 1) 
						&& ((val > 0x0) && (val <= MAX_I2C_ADDR)))
			{
					chip = val;
			}
			else
			{
					printf("Valid Address are 0x0 to 0x%02X",MAX_I2C_ADDR);
					pause();
					return;
			}
	}


	/*get reg addr*/
	printf("Device Reg [%X]: ", reg);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
		pause();
		return;
	}

	if (strlen(inbuf))
	{
		if (ishex(inbuf, sizeof(uint8_t)) 
			&& (sscanf(inbuf, "%x", &val) == 1))
		{
			reg = val;
		}
		else
		{
			puts("Invalid register");
			pause();
			return;
		}
	}
	
	/*get length*/
	printf("Length [%X]: ", len);
	inbuf = pend_rx_line();
	if (!inbuf)
	{
		pause();
		return;
	}

	if (strlen(inbuf))
	{
		if (ishex(inbuf, sizeof(uint8_t)) 
			&& (sscanf(inbuf, "%x", &val) == 1))
		{
			len = val;
		}
		else
		{
			puts("Invalid Length");
			pause();
			return;
		}
	}
	
	rc = i2c_dev_read(chip, reg, buf, len, true);
	if (rc == NRF_SUCCESS)
	{
		printf("Data at 0x%02X ", chip);
		for (i = 0; i < len; i++)
		{
			printf("%02X", buf[i]);
		}
		putchar('\n');
	}
	 
	pause();
}
	
static void i2c_menu(void)
{

	bool    exit_menu = false;
	char    *inbuf;

	/* Print menu choices */
	while (exit_menu == false)
	{
			puts("\n\n>>>  I2C Menu  <<<\n");
			puts("1. Read");
			puts("2. Write");
			puts("---------------------");
			puts("M. Main menu");
			printf("\nEnter Selection: ");
			inbuf = pend_rx_line();
			if (!inbuf || (strlen(inbuf) > 1))
					continue;

			switch (tolower((int)*inbuf))
			{
					case '1':
							i2c_read_ops();
							break;
					case '2':
							i2c_write_ops();
							break;
					case 'm':
							exit_menu = true;
							break;
			}
	}
}

static char pend_rx_char(void)
{
	int c;

	while ((c = getchar()) <= 0)
			;

	return c;
}


static char *pend_rx_line(void)
{
	static char line_buf[LINE_BUF_SIZE];
	char    *ptr;
	char    c;
	int     i;

	for (i = 1, ptr = line_buf; i < LINE_BUF_SIZE; i++)
	{
			c = pend_rx_char();
			switch (c)
			{
					case ENTER:
							*ptr++ = 0;
							putchar('\n');
							return line_buf;

					case ESCAPE:
							line_buf[0] = 0;
							putchar('\n');
							return 0;

					case BACKSPACE:
							/* Remove previous char, if there */
							if (ptr > line_buf)
							{
									i   -= 2;
									ptr -= 1;
									putchar(BACKSPACE);
									putchar(SPACE);
									putchar(BACKSPACE);
							}
							else
									--i;
							break;

					default:
							*ptr++ = c;
							putchar(c);
			}
	}

	/* Only get here if entered line is longer than line buffer */
	line_buf[LINE_BUF_SIZE - 1] = 0;
	putchar('\n');

	return line_buf;
}


static void pause(void)
{
    printf("\nPress any key to continue...");
    pend_rx_char();
    putchar('\n');
}


static bool ishex(char *str, int hex_bytes)
{
    int     len;
    int     i;
	bool 		bRet = false;
    char    *p = str;

    /* Strip "0x", if prepended */
    if ((str[0] == '0') && (tolower((int)str[1]) == 'x'))
        p += 2;

    len = strlen(str);

    /* We're looking for a 1-X char hex value */
    if ((len > 0) && (len <= (hex_bytes << 1)))
    {
        for (i = 0; i < len; ++i)
        {
            if (isxdigit((int)p[i]) == 0)
                break;
        }
    }
		
		if(i == len)
			bRet = true;
		
    return bRet;
}


void test_menu(void)
{
    bool    exit_menu = false;
    char    *inbuf;
		//ret_code_t rc;
	
    /* Print menu choices */
    while (exit_menu == false)
    {
        puts("\n>>>  Main Menu  <<<\n");
        //puts("1. Mem/Reg Dump");
        puts("1. LED OFF");
				puts("2. LED ON");
				puts("3. LED RESET");
				puts("4. I2C Probe");
				puts("5. I2C R/W");
				puts("E. Exit");
        printf("\nEnter Selection: ");
        inbuf = pend_rx_line();
        if (!inbuf || (strlen(inbuf) > 1))
            continue;

        switch (tolower((int)*inbuf))
        {
            case '1':
                //mem_dump();
								test_led_off();
                break;
            case '2':
                //reg_write();
								test_led_on();
                break;
            case '3':
								test_led_reset();
                break;
            case '4':
								i2c_bus_probe();
                break;
            case '5':
                i2c_menu();
                break;
            case '6':
                //flash_menu();
                break;
            case '7':
                //temp_regs(I2C_TEMP_FUNC_MOD_ADDR);
                break;
            case '8':
                //volt_temp_mon();
                break;
            case '9':
                //int_test();
                break;
            case '0':
                //l2c_menu();
                break;
            case 't':
                //module_menu();
                break;
            case 'e':
                exit_menu = true;
                puts("\nRunning 'main' loop...");
                break;
            case 'r':
                //cpu_reset();
                break;
        }
    }
}

#endif // #if SERIAL_DEBUG

