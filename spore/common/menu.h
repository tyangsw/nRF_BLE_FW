/*
 * CLI Menu definitions
 *
 * Copyright (C) 2016-2017 Karma Mobility, Inc.
 */
 

#ifndef __MENU_H__
#define __MENU_H__

/*
 * Definitions
 */
#define LINE_BUF_SIZE   32

#define BACKSPACE       0x08
#define ENTER           0x0D
#define ESCAPE          0x1B
#define SPACE           0x20

#define STR_TEMP_1      "  %-8s: 0x%02X\n"
#define STR_TEMP_2      "  %-8s: 0x%02X%02X\n"
#define STR_TEMP        "  %-8s: 0x%02X%02X (%d.%02dC"

#define MAX_I2C_ADDR	0x7F
#define MAX_I2C_BUF_LEN	16
typedef enum
{
    I2C_OPCODE_READ = 0,
    I2C_OPCODE_WRITE,
} I2C_OPS;


/*
 * Function prototypes
 */
void test_led_on(void);
void test_led_off(void);
void test_menu(void);

#endif /* __MENU_H__ */
