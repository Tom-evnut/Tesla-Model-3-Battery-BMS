/**
*  @file   ltc831_regs.h
*  @brief
*/

#ifndef LTC831_REGS_H
#define LTC831_REGS_H

/* This file uses little-endian bit fields and little-endian byte order. */

/******************************************************************************
*                              I N C L U D E S                               *
******************************************************************************/

/******************************************************************************
*                               D E F I N E S                                *
******************************************************************************/

#define LTC831_ITEMS_IN_REG     3
#define LTC831_CELLS_IN_REG     3
#define LTC831_GPIOS_IN_REG     3
#define LTC831_BYTES_PER_ITEM   2
#define LTC831_BYTES_PER_CELL_V 2
#define LTC831_BYTES_PER_GPIO_V 2
#define LTC831_MOD_MV_PER_BIT   2U
#define LTC831_NUM_CC_BITS      2U

#define LTC831_TAGGED_CMD_MASK  0x40

#define LTC831_RDCVA_NO_TAG_CMD 0x01
#define LTC831_RDCVA_TAG_CMD    0x41
#define LTC831_RDCVB_NO_TAG_CMD 0x02
#define LTC831_RDCVB_TAG_CMD    0x42
#define LTC831_RDCVC_NO_TAG_CMD 0x03
#define LTC831_RDCVC_TAG_CMD    0x43
#define LTC831_RDCVD_NO_TAG_CMD 0x04
#define LTC831_RDCVD_TAG_CMD    0x44
#define LTC831_RDCVE_NO_TAG_CMD 0x05
#define LTC831_RDCVE_TAG_CMD    0x45
#define LTC831_RDCVF_NO_TAG_CMD 0x06
#define LTC831_RDCVF_TAG_CMD    0x46

#define LTC831_RDSNA_TAG_CMD    0x47
#define LTC831_RDSNA_NO_TAG_CMD 0x07
#define LTC831_RDSNB_TAG_CMD    0x48
#define LTC831_RDSNB_NO_TAG_CMD 0x08
#define LTC831_RDSNC_TAG_CMD    0x49
#define LTC831_RDSNC_NO_TAG_CMD 0x09
#define LTC831_RDSND_TAG_CMD    0x4a
#define LTC831_RDSND_NO_TAG_CMD 0x0a
#define LTC831_RDSNE_TAG_CMD    0x4b
#define LTC831_RDSNE_NO_TAG_CMD 0x0b
#define LTC831_RDSNF_TAG_CMD    0x4c
#define LTC831_RDSNF_NO_TAG_CMD 0x0c

#define LTC831_RDAUXA_TAG_CMD    0x4d
#define LTC831_RDAUXA_NO_TAG_CMD 0x0d
#define LTC831_RDAUXB_TAG_CMD    0x4e
#define LTC831_RDAUXB_NO_TAG_CMD 0x0e
#define LTC831_RDSTAT_TAG_CMD    0x4f
#define LTC831_RDSTAT_NO_TAG_CMD 0x0f
#define LTC831_RDCFG_TAG_CMD     0x50
#define LTC831_RDCFG_NO_TAG_CMD  0x10
#define LTC831_WRCFG_CMD         0x11
#define LTC831_RDPWMA_NO_TAG_CMD 0x12
#define LTC831_RDPWMA_TAG_CMD    0x52
#define LTC831_WRPWMA_CMD        0x14
#define LTC831_RDPWMB_NO_TAG_CMD 0x13
#define LTC831_RDPWMB_TAG_CMD    0x53
#define LTC831_WRPWMB_CMD        0x15
#define LTC831_MUTE_CMD          0x20
#define LTC831_UNMUTE_CMD        0x21
#define LTC831_PRIMEZ_CMD        0x22
#define LTC831_PRIMEN_CMD        0x23
#define LTC831_CLRDIAG_CMD       0x24
#define LTC831_TEST5_CMD         0x25
#define LTC831_TESTA_CMD         0x26
#define LTC831_TESTOFF_CMD       0x27
#define LTC831_ARMSOLO_CMD       0x28
#define LTC831_DISSOLO_CMD       0x29
#define LTC831_CLRCNT_CMD        0x2a
#define LTC831_SNAP_CMD          0x2b
#define LTC831_RDFS_NO_TAG_CMD   0x30   // 3 lsb's define fuse row in range 0 to 5
#define LTC831_RDFS_TAG_CMD      0x70   // 3 lsb's define fuse row in range 0 to 5
#define LTC831_RDFSLM_NO_TAG_CMD 0x80   // 3 lsb's define fuse row in range 0 to 5
#define LTC831_RDFSLM_TAG_CMD    0xC0   // 3 lsb's define fuse row in range 0 to 5
#define LTC831_RDFSHM_NO_TAG_CMD 0x90   // 3 lsb's define fuse row in range 0 to 5
#define LTC831_RDFSHM_TAG_CMD    0xd0   // 3 lsb's define fuse row in range 0 to 5

#define LTC831_WRFS_CMD          0xa0   // 3 lsb's define fuse row in range 0 to 3
#define LTC831_BLFS_CMD          0xb0   // 3 lsb's define fuse row in range 0 to 3
#define LTC831_EOR               0xff

#define LTC831_FS_ROW_START      0x0    // First fuse row address
#define LTC831_FS_ROW_END        0x5    // Last fuse row address

#ifdef LTC831_USE_TAGS
#define LTC831_RDCVA_CMD     LTC831_RDCVA_TAG_CMD
#define LTC831_RDCVB_CMD     LTC831_RDCVB_TAG_CMD
#define LTC831_RDCVC_CMD     LTC831_RDCVC_TAG_CMD
#define LTC831_RDCVD_CMD     LTC831_RDCVD_TAG_CMD
#define LTC831_RDCVE_CMD     LTC831_RDCVE_TAG_CMD
#define LTC831_RDCVF_CMD     LTC831_RDCVF_TAG_CMD
#define LTC831_RDSNA_CMD     LTC831_RDSNA_TAG_CMD
#define LTC831_RDSNB_CMD     LTC831_RDSNB_TAG_CMD
#define LTC831_RDSNC_CMD     LTC831_RDSNC_TAG_CMD
#define LTC831_RDSND_CMD     LTC831_RDSND_TAG_CMD
#define LTC831_RDSNE_CMD     LTC831_RDSNE_TAG_CMD
#define LTC831_RDSNF_CMD     LTC831_RDSNF_TAG_CMD
#define LTC831_RDAUXA_CMD    LTC831_RDAUXA_TAG_CMD
#define LTC831_RDAUXB_CMD    LTC831_RDAUXB_TAG_CMD
#define LTC831_RDSTAT_CMD    LTC831_RDSTAT_TAG_CMD
#define LTC831_RDCFG_CMD     LTC831_RDCFG_TAG_CMD
#define LTC831_RDPWMA_CMD    LTC831_RDPWMA_TAG_CMD
#define LTC831_RDPWMB_CMD    LTC831_RDPWMB_TAG_CMD
#define LTC831_RDFS_CMD      LTC831_RDFS_TAG_CMD
#define LTC831_RDFSLM_CMD    LTC831_RDFSLM_TAG_CMD
#define LTC831_RDFSHM_CMD    LTC831_RDFSHM_TAG_CMD

#else
#define LTC831_RDCVA_CMD     LTC831_RDCVA_NO_TAG_CMD
#define LTC831_RDCVB_CMD     LTC831_RDCVB_NO_TAG_CMD
#define LTC831_RDCVC_CMD     LTC831_RDCVC_NO_TAG_CMD
#define LTC831_RDCVD_CMD     LTC831_RDCVD_NO_TAG_CMD
#define LTC831_RDCVE_CMD     LTC831_RDCVE_NO_TAG_CMD
#define LTC831_RDCVF_CMD     LTC831_RDCVF_NO_TAG_CMD
#define LTC831_RDSNA_CMD     LTC831_RDSNA_NO_TAG_CMD
#define LTC831_RDSNB_CMD     LTC831_RDSNB_NO_TAG_CMD
#define LTC831_RDSNC_CMD     LTC831_RDSNC_NO_TAG_CMD
#define LTC831_RDSND_CMD     LTC831_RDSND_NO_TAG_CMD
#define LTC831_RDSNE_CMD     LTC831_RDSNE_NO_TAG_CMD
#define LTC831_RDSNF_CMD     LTC831_RDSNF_NO_TAG_CMD
#define LTC831_RDAUXA_CMD    LTC831_RDAUXA_NO_TAG_CMD
#define LTC831_RDAUXB_CMD    LTC831_RDAUXB_NO_TAG_CMD
#define LTC831_RDSTAT_CMD    LTC831_RDSTAT_NO_TAG_CMD
#define LTC831_RDCFG_CMD     LTC831_RDCFG_NO_TAG_CMD
#define LTC831_RDPWMA_CMD    LTC831_RDPWMA_NO_TAG_CMD
#define LTC831_RDPWMB_CMD    LTC831_RDPWMB_NO_TAG_CMD
#define LTC831_RDFS_CMD      LTC831_RDFS_NO_TAG_CMD
#define LTC831_RDFSLM_CMD    LTC831_RDFSLM_NO_TAG_CMD
#define LTC831_RDFSHM_CMD    LTC831_RDFSHM_NO_TAG_CMD
#endif


/******************************************************************************
*                              T Y P E D E F S                               *
******************************************************************************/

typedef struct
{
    UVAR16 C1;
    UVAR16 C2;
    UVAR16 C3;
} __attribute__((packed)) LTC831_CVA_REG_T;

typedef struct
{
    UVAR16 C4;
    UVAR16 C5;
    UVAR16 C6;
} __attribute__((packed)) LTC831_CVB_REG_T;

typedef struct
{
    UVAR16 C7;
    UVAR16 C8;
    UVAR16 C9;
} __attribute__((packed)) LTC831_CVC_REG_T;

typedef struct
{
    UVAR16 C10;
    UVAR16 C11;
    UVAR16 C12;
} __attribute__((packed)) LTC831_CVD_REG_T;

typedef struct
{
    UVAR16 C13;
    UVAR16 C14;
    UVAR16 C15;
} __attribute__((packed)) LTC831_CVE_REG_T;

typedef struct
{
    UVAR16 C16;
    UVAR16 STACK;
} __attribute__((packed)) LTC831_CVF_REG_T;

typedef struct
{
    UVAR16 TEMP1;
    UVAR16 V5;
    UVAR16 TEMP2;
} __attribute__((packed)) LTC831_AUXA_REG_T;

typedef struct
{
    UVAR16 V3;
    UVAR16 VDIODE;
    UVAR16 REF2;
} __attribute__((packed)) LTC831_AUXB_REG_T;

typedef struct
{
    UVAR16 PWM1 : 4;
    UVAR16 PWM2 : 4;
    UVAR16 PWM3 : 4;
    UVAR16 PWM4 : 4;
    UVAR16 PWM5 : 4;
    UVAR16 PWM6 : 4;
    UVAR16 PWM7 : 4;
    UVAR16 PWM8 : 4;
} __attribute__((packed)) LTC831_PWMA_REG_T;

typedef struct
{
    UVAR16 PWM9  : 4;
    UVAR16 PWM10 : 4;
    UVAR16 PWM11 : 4;
    UVAR16 PWM12 : 4;
    UVAR16 PWM13 : 4;
    UVAR16 PWM14 : 4;
    UVAR16 PWM15 : 4;
    UVAR16 PWM16 : 4;
} __attribute__((packed)) LTC831_PWMB_REG_T;

typedef enum { LTC831_FILT_OFF = 0, LTC831_FILT_23, LTC831_FILT_11, LTC831_FILT_6, LTC831_FILT_3, LTC831_FILT_2, LTC831_FILT_1, LTC831_FILT_04 }LTC831_FILT_E;

#define LTC831_FILTER_INIT_VALUE ( LTC831_FILT_6 )

#if BITS_LITTLE_ENDIAN
typedef union
{
    UVAR16 reg[2];
    struct
    {
        UVAR16 FILT    : 3;  // LTC831_FILT_E
        UVAR16 RAND    : 1;
        UVAR16 TSOLO   : 4;
        UVAR16 MOD_DIS : 1;
        UVAR16 TRY     : 1;
        UVAR16 SPARE   : 1;
        UVAR16 TEMP_OW : 1;
        UVAR16 DCT0    : 4;  // 0 = off, 0.5,1
        UVAR8  DCC8_1;
        UVAR8  DCC16_9;
    };
} __attribute__((packed)) LTC831_CFG_REG_T;
#else
typedef union
{
    UVAR16 reg[2];
    struct
    {
        UVAR16 TSOLO   : 4;
        UVAR16 RAND    : 1;
        UVAR16 FILT    : 3;  // LTC831_FILT_E
        UVAR16 DCT0    : 4;  // 0 = off, 0.5,1
        UVAR16 TEMP_OW : 1;
        UVAR16 SPARE   : 1;
        UVAR16 TRY     : 1;
        UVAR16 MOD_DIS : 1;
        UVAR8  DCC8_1;
        UVAR8  DCC16_9;
    };
} __attribute__((packed)) LTC831_CFG_REG_T;
#endif

typedef struct
{
    UVAR16 POR_FLAG   : 1;
    UVAR16 SOLO_FLAG  : 1;
    UVAR16 UNUSED     : 2;
    UVAR16 MUX_FAIL   : 1;
    UVAR16 THSD_FLAG  : 1;
    UVAR16 CEC_FAIL   : 1;
    UVAR16 CMD_FAIL   : 1;
    UVAR16 TEST5      : 1;
    UVAR16 TESTA      : 1;
    UVAR16 ADC_MIN    : 1;
    UVAR16 ADC_MAX    : 1;
    UVAR16 REV        : 4;
    UVAR16 MUTE       : 1;
    UVAR16 SENSE      : 1;
    UVAR16 SOLO_ARMED : 1;
    UVAR16 THSD       : 1;
    UVAR16 LOTP_ED    : 1;
    UVAR16 LOTP_MED   : 1;
    UVAR16 TOTP_ED    : 1;
    UVAR16 TOTP_MED   : 1;
    UVAR8  FILT_CNT;
} __attribute__((packed)) LTC831_STATUS_REG_T;

typedef struct
{
    UVAR16 TSLA_LCK   : 1;
    UVAR16 ARRAY_SEL  : 1;
    UVAR16 SOLO_TO    : 4;
    UVAR16 REF_TRIM   : 5;
    UVAR16 OTP_CRC    : 5;
} __attribute__((packed)) LTC831_OTP_FSR0_REG_T;

typedef struct
{
    UVAR16 OT_EXT     : 6;
    UVAR16 OT_INT     : 5;
    UVAR16 OTP_CRC    : 5;
} __attribute__((packed)) LTC831_OTP_FSR1_REG_T;

typedef struct
{
    UVAR16 SOLO_UV    : 5;
    UVAR16 NCELLS     : 4;
    UVAR16 UNUSED     : 2;
    UVAR16 OTP_CRC    : 5;
} __attribute__((packed)) LTC831_OTP_FSR2_REG_T;

typedef struct
{
    UVAR16 MOD_ID     : 11;
    UVAR16 OTP_CRC    : 5;
} __attribute__((packed)) LTC831_OTP_FSR3_REG_T;

typedef struct
{
    UVAR16 DIE_ID_0_10 : 11;
    UVAR16 OTP_CRC     : 5;
} __attribute__((packed)) LTC831_OTP_FSR4_REG_T;

typedef struct
{
    UVAR16 DIE_ID_11_21 : 11;
    UVAR16 OTP_CRC      : 5;
} __attribute__((packed)) LTC831_OTP_FSR5_REG_T;

#endif /* LTC831_REGS_H */
