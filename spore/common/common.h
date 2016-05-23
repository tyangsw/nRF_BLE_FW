/*
 * Common definitions for the project.
 *
 * Copyright (C) 2014-2015 North Atlantic Industries, Inc.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

/*
 * Module memory layout
 */
#define COMMON_MEM_MAP_REV          1
#define MODULE_MEM_BASE             0x40000000UL                            // M_AXI_GP0 base
#define COMMON_AREA_BASE            MODULE_MEM_BASE
#define COMMON_AREA_SIZE            0x1000UL
#define MODULE_REGS_BASE            (COMMON_AREA_BASE + COMMON_AREA_SIZE)
#define MODULE_PRIV_REGS_BASE       0x60000000UL                            // XPAR_APB_M_SSAREA_BASEADDR
#define MODULE_CAL_BASE             (MODULE_PRIV_REGS_BASE + 0x8000000UL)
#define MODULE_CAL_MAX_SIZE         0x8000000UL
#define MODULE_MEM_MAX_SIZE         0x40000000UL                            // M_AXI_GP0 size
#define MODULE_CPLD_BASE            0x6FFFFFE0UL
#ifdef XPAR_EMC_0_S_AXI_MEM0_BASEADDR
#define SCRATCH_MEM_BASE            0x80000000UL                            // M_AXI_GP1 base
#else
#define SCRATCH_MEM_BASE            0x80000UL                               // Second 512KB of DDR
#endif

/*
 * Module common area
 */
#define MOD_READY_FW_ENTRY          (1UL << 16)
#define MOD_READY_COMMON_VALID      (1UL << 17)
#define MOD_READY_PARAM_LOADED      (1UL << 18)
#define MOD_READY_CAL_LOADED        (1UL << 19)
#define MOD_READY_CPLD1_PROGRAMMED  (1UL << 20)
#define MOD_READY_CPLD2_PROGRAMMED  (1UL << 21)
#define MOD_READY_INIT_DONE         (1UL << 31)

#define IF_MOD_CORE_TEMP            0
#define IF_MOD_PCB_TEMP             1
#define FUNC_MOD_PCB_TEMP           0

#define CAL_UNLOCK_CODE             0x41303037  // "A007"
#define CAL_OP_WRITE                1
#define CAL_OP_READ                 2

typedef volatile struct {
    u8      if_mod_sn[16];              /* 0x000: Interface module S/N */
    u8      func_mod_sn[16];            /* 0x010: Functional module S/N */
    u8      func2_mod_sn[16];           /* 0x020: Functional module #2 (top) S/N */
    u32     fpga_build_cnt;             /* 0x030: FPGA compile count */
    u32     fpga_int_blk_rev[2];        /* 0x034: FPGA internal block revisions */
    u32     fpga_rev;                   /* 0x03C: FPGA revision */
    u32     arm_rev;                    /* 0x040: ARM revision */
    u8      if_mod_cpld_rev[16];        /* 0x044: Interface module CPLD revisions */
    u8      func_mod_cpld_rev[16];      /* 0x054: Functional module CPLD revisions */
    u8      func2_mod_cpld_rev[16];     /* 0x064: Functional module #2 (top) CPLD revisions */
    u32     fw_ver;                     /* 0x074: Firmware version */
    u32     uboot_ver;                  /* 0x078: U-Boot version */
    u32     fsbl_ver;                   /* 0x07C: FSBL version */
    char    fw_build_time[24];          /* 0x080: Firmware build time */
    char    uboot_build_time[24];       /* 0x098: U-Boot build time */
    char    fsbl_build_time[24];        /* 0x0B0: FSBL build time */
    u32     _reserved1[77];             /* 0x0C8 - 0x1F8: Reserved */
    u32     mem_map_rev;                /* 0x1FC: Module Common / Specific Memory Map revision */
    s8      if_mod_temp[8];             /* 0x200: Interface module ambient temperature */
    s8      func_mod_temp[8];           /* 0x208: Functional module ambient temperature */
    s8      func2_mod_temp[8];          /* 0x210: Functional module #2 (top) ambient temperature */
    s8      if_mod_temp_max[8];         /* 0x218: Interface module maximum temperature */
    s8      if_mod_temp_min[8];         /* 0x220: Interface module minimum temperature */
    s8      func_mod_temp_max[8];       /* 0x228: Functional module maximum temperature */
    s8      func_mod_temp_min[8];       /* 0x230: Functional module minimum temperature */
    s8      func2_mod_temp_max[8];      /* 0x238: Functional module #2 (top) maximum temperature */
    s8      func2_mod_temp_min[8];      /* 0x240: Functional module #2 (top) minimum temperature */
    u32     test_enable;                /* 0x248: Test enable */
    u32     test_verify;                /* 0x24C: D2 test verify */
    u32     ps_enable;                  /* 0x250: Power supply enable */
    u32     output_enable;              /* 0x254: Output enable */
    u32     in_calibration;             /* 0x258: Channel currently calibrating */
    u32     mod_ready;                  /* 0x25C: Module ready */
    u32     master_ready;               /* 0x260: Master ready */
    u32     _reserved2[6];              /* 0x264 - 0x278: Reserved */
    u32     cal_disable;                /* 0x27C: Calibration disable */
    u32     _reserved3;                 /* 0x280: Reserved */
    u32     cal_unlock;                 /* 0x284: Calibration Enable - Write 0x41303037 ("A007") to enable calibration mode */
    u32     cal_addr;                   /* 0x288: Calibration Address - Address that will map to RAMs and/or registers */
    u32     cal_data;                   /* 0x28C: Data that will be written to or read from the address set in "Calibration Address" register */
    u32     cal_op;                     /* 0x290: Calibration Operation - Write '1'  to write "Calibration Data" to "Calibration Address"; write '2' to read it */
    u32     test_value;                 /* 0x294: D0 Test Value (Angle, voltage, position, etc…) */
    u32     test_range;                 /* 0x298: D0 Test Range (Example AD module has a volt range) */
    u32     zynq_core_volt;             /* 0x29C: Zynq Core voltage */
    u32     zynq_aux_volt;              /* 0x2A0: Zynq Aux voltage */
    u32     zynq_ddr_volt;              /* 0x2A4: Zynq DDR voltage */
    u32     _reserved4[2];              /* 0x2A8 - 0x2AC: Reserved */
    u32     ch_status_en;               /* 0x2B0: Channel Status Enable - 0=disable, 1=enable (bitmapped) */
    u32     _reserved5[339];            /* 0x2B4 - 0x7FC: Reserved */
    struct {                            /* 0x800 - 0xC0C: Interrupt registers */
        u32     raw_status;             /*         0x000: Dynamic status */
        u32     status;                 /*         0x004: Latched status */
        u32     mask;                   /*         0x008: Mask */
        u32     edge_level;             /*         0x00C: Edge/Level */
    } irq[65];
    u32     _reserved6[60];             /* 0xC10 - 0xCFC: Reserved */
    u8      func2_mod_eeprom[256];      /* 0xD00: Functional module #2 (top) EEPROM */
    u8      func_mod_eeprom[256];       /* 0xE00: Functional module EEPROM */
    u8      if_mod_eeprom[256];         /* 0xF00: Interface module EEPROM */
} COMMON_AREA;

/*
 * Parameter/calibration file layout
 */
#define PC_FILE_SIZE            0x1000
#define PC_FILE_HDR_SIZE        32
#define PC_FILE_ENTRY_SIZE      8
#define PC_FILE_NUM_ENTRIES     ((PC_FILE_SIZE - PC_FILE_HDR_SIZE) / PC_FILE_ENTRY_SIZE)
#define PARAM_FILE_SIG          "PAR1"
#define CAL_FILE_SIG            "CAL1"
#define CAL_FILE_SN_BYPASS      "\xA0\x07"
#define PARAM_FILE_FLASH_ADDR   0x560000
#define CAL_FILE_FLASH_ADDR     0x570000

typedef volatile struct {
    u32     sig;                /* 0x000: Signature */
    u32     mod_id;             /* 0x004: Module ID */
    u8      mod_sn[16];         /* 0x008: Module S/N */
    u32     _reserved[2];       /* 0x018 - 0x01C: Reserved */
    struct {                    /* 0x020 - XXX: Address/Value pairs */
        u32     addr;
        u32     value;
    } data[];
} PC_FILE;

/*
 * CPLD image
 */
#define CPLD_IMAGE_HDR_SIZE     80
#define CPLD_IMAGE_SIG          0xFF00
#define CPLD_IMAGE_SIZE_LM2     68224
#define CPLD_IMAGE_SIZE_HX8     (CPLD_IMAGE_SIZE_LM2 * 2)
#define CPLD_PART_STRING        "Part: iCE40"
#define CPLD_PART_STRING_LEN    11
#define CPLD_READY_STATUS       0xAA55
#define CPLD_READY_TIMEOUT      150 // 150 ms
#define CPLD1_FLASH_ADDR        0x580000
#define CPLD2_FLASH_ADDR        0x600000

typedef enum
{
    CPLD_1 = 0,
    CPLD_2,
    NUM_CPLDS
} CPLD;

typedef volatile struct {
    u32     _reserved[2];       /* 0x00 - 0x04: Reserved */
    u32     prog_en;            /* 0x08: Enable Programming */
    u32     fifo4;              /* 0x0C: CPLD4 FIFO */
    u32     fifo3;              /* 0x10: CPLD3 FIFO */
    u32     fifo2;              /* 0x14: CPLD2 FIFO */
    u32     fifo1;              /* 0x18: CPLD1 FIFO */
    u32     ready;              /* 0x1C: Ready status */
} CPLD_REGS;

/*
 * Type definitions
 */
typedef u8 bool;

/*
 * Support stdout for serial debug/test
 */
#if SERIAL_DEBUG
    #define PRINT_LINE_LENGTH   16
#else
    #define printf(fmt, args...) \
        (void)0
    #define puts(s)              \
        (void)0
#endif
#define debug(fmt, args...)      \
    do {                         \
        if (verbose)             \
            printf(fmt, ##args); \
    } while(0)

/*
 * Stringify MODULE_ID
 */
#define MODULE_STR(s)   __STRINGIFY(s)
#define MODULE_NAME     MODULE_STR(MODULE_ID)

/*
 * Length of build/version strings
 */
#define VERSION_STRING_LEN  35
#define BUILD_TIME_LEN      24

/*
 * OCM locations of FSBL and U-Boot version strings
 */
#define FSBL_VERSION_STRING         0xFFFFFD00
#define UBOOT_VERSION_STRING        (FSBL_VERSION_STRING + VERSION_STRING_LEN)

/*
 * EEPROM definitions
 */
#define I2C_EEPROM_IF_MOD_ADDR      0x50    // Bottom (interface) module board
#define I2C_EEPROM_FUNC_MOD_ADDR    0x51    // Top (functional) module board
// AT24CS04: A0=N/C, A1=VCC, A2=GND
//#define I2C_EEPROM_FUNC_MOD_ADDR    0x52    // Top (functional) module board

#define EEPROM_SIZE                 256     // 256 x 8 (2-Kbit)
#define EEPROM_PAGE_SIZE            16
#define EEPROM_PAGE_WRITE_SIZE      8
#define EEPROM_SN_ADDR              0x80
#define EEPROM_SN_SIZE              16

/*
 * Global Timer is always clocked at half of the CPU frequency
 */
#define COUNTS_PER_MSECOND      (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2 * 1000))

/*
 * SLCR Registers
 */
#define PS_RST_CTRL_REG         (XPS_SYS_CTRL_BASEADDR + 0x200)
#define PS_RST_MASK             0x1

/*
 * Global variables
 */
extern COMMON_AREA *pCommon;

/*
 * Function prototypes
 */
void init_common(void);
void load_param_file(void);
void load_cal_file(void);
void caldata_rw(void);
void program_cpld(u8 cpld);
XTime get_timer(XTime base);
XTime get_timer_count(XTime base);
void assert_print(const char *filname, int line);
void cpu_reset(void);
int print_buffer(u32 addr, void *data, u8 width, u32 count);

void module_init(void);
void module_main(void);

/* From string.h */
_PTR _EXFUN(memmem, (const _PTR, size_t, const _PTR, size_t));

#endif /* __COMMON_H__ */

