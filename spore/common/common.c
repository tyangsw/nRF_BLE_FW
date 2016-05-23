/*
 * Common code for Maximillion modules.
 *
 * Copyright (C) 2014-2015 North Atlantic Industries, Inc.
 */
 
#include <includes.h>


/* Exported global variables */
COMMON_AREA *pCommon = (COMMON_AREA *)COMMON_AREA_BASE;

/* Global variables */
static u8 pc_buf[PC_FILE_SIZE];
static CPLD_REGS *pCpld = (CPLD_REGS *)MODULE_CPLD_BASE;


void init_common(void)
{
    u32     version, patchlevel, sublevel;
    char    ts[BUILD_TIME_LEN] = { 0 };
    char    *pStr;

    /* Read in EEPROMs */
    i2c_dev_read(I2C_EEPROM_IF_MOD_ADDR, 0, (u8 *)pCommon->if_mod_eeprom, EEPROM_SIZE, TRUE);
    i2c_dev_read(I2C_EEPROM_FUNC_MOD_ADDR, 0, (u8 *)pCommon->func_mod_eeprom, EEPROM_SIZE, TRUE);

    /* Get serial numbers from EEPROMs */
    i2c_dev_read(I2C_EEPROM_IF_MOD_ADDR | 0x08, EEPROM_SN_ADDR, (u8 *)pCommon->if_mod_sn, EEPROM_SN_SIZE, TRUE);
    i2c_dev_read(I2C_EEPROM_FUNC_MOD_ADDR | 0x08, EEPROM_SN_ADDR, (u8 *)pCommon->func_mod_sn, EEPROM_SN_SIZE, TRUE);

    /* Write our version and build time */
    pCommon->fw_ver = (FW_VER_MAJOR << 8) | FW_VER_MINOR;
    strncpy((char *)pCommon->fw_build_time, FW_DATE " at " FW_TIME, BUILD_TIME_LEN);

    /* Parse and write FSBL version and build time */
    pStr = (char *)FSBL_VERSION_STRING;
    if (sscanf(pStr, "%04lu.%02lu.%02lu.%[A-Za-z0-9: ]", &version, &patchlevel, &sublevel, ts) == 4)
    {
        pCommon->fsbl_ver = (version << 16) | (patchlevel << 8) | sublevel;
        strncpy((char *)pCommon->fsbl_build_time, ts, BUILD_TIME_LEN);
    }

    /* Parse and write U-Boot version and build time */
    pStr = (char *)UBOOT_VERSION_STRING;
    if (sscanf(pStr, "%04lu.%02lu.%02lu.%[A-Za-z0-9: ]", &version, &patchlevel, &sublevel, ts) == 4)
    {
        pCommon->uboot_ver = (version << 16) | (patchlevel << 8) | sublevel;
        strncpy((char *)pCommon->uboot_build_time, ts, BUILD_TIME_LEN);
    }

    /* Write memory map revisions */
    pCommon->mem_map_rev = (MODULE_MEM_MAP_REV << 16) | COMMON_MEM_MAP_REV;

    /* Common area is now valid */
    pCommon->mod_ready |= MOD_READY_COMMON_VALID;
}


void load_param_file(void)
{
    PC_FILE *pParam = (PC_FILE *)pc_buf;
    u32     i;

    /* Load parameter file from flash */
    if (flash_read(PARAM_FILE_FLASH_ADDR, PC_FILE_SIZE, pc_buf))
        return;

    /* Verify signature */
    if (pParam->sig != Xil_In32((u32)PARAM_FILE_SIG))
        return;

    /* Verify module ID */
    if (pParam->mod_id != Xil_In32((u32)MODULE_NAME))
        return;

    /* Verify and write the data */
    for (i = 0; i < PC_FILE_NUM_ENTRIES; ++i)
    {
        /* End of file? */
        if (pParam->data[i].addr == 0xFFFFFFFF)
            break;

        /* Valid address? */
        if (pParam->data[i].addr < MODULE_MEM_BASE || pParam->data[i].addr >= (MODULE_MEM_BASE + MODULE_MEM_MAX_SIZE))
            return;

        /* Write it */
        Xil_Out32(pParam->data[i].addr, pParam->data[i].value);
    }

    /* Parameter file has been successfully loaded */
    pCommon->mod_ready |= MOD_READY_PARAM_LOADED;
}


void load_cal_file(void)
{
    PC_FILE *pCal = (PC_FILE *)pc_buf;
    u32     i;

    /* Load calibration file from flash */
    if (flash_read(CAL_FILE_FLASH_ADDR, PC_FILE_SIZE, pc_buf))
        return;

    /* Verify signature */
    if (pCal->sig != Xil_In32((u32)CAL_FILE_SIG))
        return;

    /* Verify module ID */
    if (pCal->mod_id != Xil_In32((u32)MODULE_NAME))
        return;

    /* Verify module S/N; allow James Bond (Agent 007) to bypass it */
    if (Xil_In16((u32)&pCal->mod_sn) != Xil_In16((u32)CAL_FILE_SN_BYPASS))
    {
        for (i = 0; i < EEPROM_SN_SIZE; ++i)
        {
            if (pCal->mod_sn[i] != pCommon->func_mod_sn[i])
                return;
        }
    }

    /* Verify and write the data */
    for (i = 0; i < PC_FILE_NUM_ENTRIES; ++i)
    {
        /* End of file? */
        if (pCal->data[i].addr == 0xFFFFFFFF)
            break;

        /* Valid address? */
        if (pCal->data[i].addr < MODULE_CAL_BASE || pCal->data[i].addr >= (MODULE_CAL_BASE + MODULE_CAL_MAX_SIZE))
            return;

        /* Write it */
        Xil_Out32(pCal->data[i].addr, pCal->data[i].value);
    }

    /* Calibration file has been successfully loaded */
    pCommon->mod_ready |= MOD_READY_CAL_LOADED;

    /* Save calibration file in flash
    if (flash_erase(CAL_FILE_FLASH_ADDR, PC_FILE_SIZE))
        return;
    if (flash_write(CAL_FILE_FLASH_ADDR, PC_FILE_SIZE, pc_buf))
        return;
    */
}


void caldata_rw(void)
{
    /* Calibration enabled? */
    if (pCommon->cal_unlock == CAL_UNLOCK_CODE)
    {
        if (pCommon->cal_op == CAL_OP_WRITE)
        {
            /* Write calibration data */
            Xil_Out32(pCommon->cal_addr, pCommon->cal_data);

            /* Signal completion */
            pCommon->cal_op = 0;
        }
        else if (pCommon->cal_op == CAL_OP_READ)
        {
            /* Read calibration data */
            pCommon->cal_data = Xil_In32(pCommon->cal_addr);

            /* Signal completion */
            pCommon->cal_op = 0;
        }
    }
}


void program_cpld(u8 cpld)
{
    u32     *pBuf = (u32 *)SCRATCH_MEM_BASE;
    u32     addr = (cpld == CPLD_1) ? CPLD1_FLASH_ADDR : CPLD2_FLASH_ADDR;
    char    *part;
    u32     size;
    XTime   time;
    static bool prog_en = FALSE;

    /* Load CPLD image header from flash */
    if (flash_read(addr, CPLD_IMAGE_HDR_SIZE, (u8 *)pBuf))
        return;

    /* Verify signature */
    if (Xil_In16((u32)pBuf) == CPLD_IMAGE_SIG)
        return;

    /* Find CPLD part string */
    part = memmem(pBuf, CPLD_IMAGE_HDR_SIZE, CPLD_PART_STRING, CPLD_PART_STRING_LEN);
    if (!part)
        return;

    /* Get CPLD image size */
    part += CPLD_PART_STRING_LEN;
    if (memcmp(part, "LM2", 3) == 0)
        size = CPLD_IMAGE_SIZE_LM2;
    else if (memcmp(part, "HX8", 3) == 0)
        size = CPLD_IMAGE_SIZE_HX8;
    else
        return;

    /* Load entire CPLD image from flash */
    if (flash_read(addr, size, (u8 *)pBuf))
        return;

    /* Enable CPLD programming */
    if (!prog_en)
    {
        pCpld->prog_en = 1;
        prog_en = TRUE;
    }

    /* Verify CPLD is ready for programming */
    time = get_timer(0);
    while (pCpld->ready != CPLD_READY_STATUS)
    {
        /* Evaluate timeout */
        if (get_timer(time) > CPLD_READY_TIMEOUT)
            return;
    }

    /* Program it */
    while (size)
    {
        if (cpld == CPLD_1)
            pCpld->fifo1 = __builtin_bswap32(*pBuf++);
        else
            pCpld->fifo2 = __builtin_bswap32(*pBuf++);

        size -= 4;
    }

    /* CPLD has been successfully programmed */
    pCommon->mod_ready |= (cpld == CPLD_1) ? MOD_READY_CPLD1_PROGRAMMED : MOD_READY_CPLD2_PROGRAMMED;
}


XTime get_timer(XTime base)
{
    XTime tCur;

    XTime_GetTime(&tCur);

    return ((tCur / COUNTS_PER_MSECOND) - base);
}


XTime get_timer_count(XTime base)
{
    XTime tCur;

    XTime_GetTime(&tCur);

    return (tCur - base);
}


void cpu_reset(void)
{
    puts("\nResetting...");

    /* Wait 50 ms */
    usleep(50000);

    /* Disable interrupts */
    disable_interrupts();

    /* Unlock SLCR register access */
    Xil_Out32(XSLCR_UNLOCK_ADDR, XSLCR_UNLOCK_CODE);

    /* Reset PS */
    Xil_Out32(PS_RST_CTRL_REG, PS_RST_MASK);

    /* Wait for reset to occur... */
    while (1)
        ;
}


void assert_print(const char *filname, int line)
{
    printf("ASSERT in %s, line %d\n", filname, line);
}


#if SERIAL_DEBUG
int print_buffer(u32 addr, void *data, u8 width, u32 count)
{
    /* linebuf as a union causes proper alignment */
    union linebuf {
        u32 ui[PRINT_LINE_LENGTH/sizeof(u32) + 1];
        u16 us[PRINT_LINE_LENGTH/sizeof(u16) + 1];
        u8  uc[PRINT_LINE_LENGTH/sizeof(u8) + 1];
    } lb;
    u32 i;
    u32 linelen = PRINT_LINE_LENGTH / width;

    while (count)
    {
        printf("%08lX:", addr);

        /* Check for overflow condition */
        if (count < linelen)
            linelen = count;

        /* Copy from memory into linebuf and print HEX values */
        for (i = 0; i < linelen; i++)
        {
            u32 x;
            if (width == 4)
                x = lb.ui[i] = Xil_In32((u32)data);
            else if (width == 2)
                x = lb.us[i] = Xil_In16((u32)data);
            else
                x = lb.uc[i] = Xil_In8((u32)data);
            printf(" %0*lX", width * 2, x);
            data += width;
        }

        /* Print data in ASCII characters */
        for (i = 0; i < linelen * width; i++)
        {
            if (!isprint(lb.uc[i]) || lb.uc[i] >= 0x7F)
                lb.uc[i] = '.';
        }
        lb.uc[i] = '\0';
        printf("    %s\n", lb.uc);

        /* Update references */
        addr += linelen * width;
        count -= linelen;

        /* Check for abort */
        if (getchar() > 0)
            return -1;
    }

    return 0;
}
#endif // #if SERIAL_DEBUG

