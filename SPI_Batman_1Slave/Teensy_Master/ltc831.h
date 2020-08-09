/**
*  @file   ltc831.h
*  @brief
*/

#ifndef LTC831_H
#define LTC831_H

/* This file uses little-endian bit fields and little-endian byte order */

/******************************************************************************
*                              I N C L U D E S                               *
******************************************************************************/

// Additional integration headers
#include "bmb_system.h"
#include "pack.h"

/******************************************************************************
*                               D E F I N E S                                *
******************************************************************************/

#define LTC831_CMD_SIZE_B      1U
#define LTC831_TAG_SIZE_B      1U
#define LTC831_CMD_PEC_SIZE_B  1U
#define LTC831_PEC_SIZE_B      2U
#define LTC831_CMD_PKT_SIZE_B (LTC831_CMD_SIZE_B + LTC831_CMD_PEC_SIZE_B)
#define LTC831_CMD_PKT_TAGGED_SIZE_B (LTC831_CMD_SIZE_B + LTC831_TAG_SIZE_B + LTC831_CMD_PEC_SIZE_B)
#define LTC831_NUM_CC_BITS     2U

#define LTC831_CELL_UV_b  80   // Also applies to Ref2, V5 and V3
#define LTC831_STACK_UV_b 1280
#define LTC831_DIE_TEMP_GAIN_10K 2178U
#define LTC831_DIE_TEMP_OFFSET_X10 (2732U)
#define LTC831_PEC_SEED_VALUE      (0x0010U) // PEC calculation seed value, as per datasheet

/******************************************************************************
*                              T Y P E D E F S                               *
******************************************************************************/

/* An LTC831 packet consists of a command and a data portion,
   There are 2 command types, one with a tag byte and other without. Tag types are only for read packets
   There are 3 different data lengths : 0, 2, 4 or 6 bytes. Read packets can not be 0 length
 */

typedef struct {
    UVAR8  addr;
    UVAR8  cmdPec;
    UVAR16 data;
} __attribute__((packed)) LTC831_PKT_CMD_T;

typedef struct {
    UVAR8  addr;
    UVAR8  cmdTag;
    UVAR8  cmdPec;
} __attribute__((packed)) LTC831_PKT_CMD_TAG_T;

typedef union {
    UVAR16 word;
    struct {
        #if (BITS_LITTLE_ENDIAN)
            UVAR16 pec : 14;
            UVAR16 cc  : 2;    //  Command Count
        #else
            UVAR16 cc  : 2;
            UVAR16 pec : 14;
        #endif
    } bits;
} __attribute__((packed)) LTC831_DATA_PEC_T;

typedef union {
//  UVAR8 cmdBytes[sizeof(LTC831_PKT_CMD_TAG_T) + ((MAX_NUM_LTC831_DEVS)* ((3 * sizeof(UVAR16)) + LTC_PEC_SIZE_B))];
    LTC831_PKT_CMD_T      ltcPkt;
    LTC831_PKT_CMD_TAG_T  ltcPktTagged;
} __attribute__((packed)) LTC831_CMD_PKT_T;

typedef union {
    UVAR16 word;
    struct {
        UVAR16 ch : 3;
        UVAR16 opc1 : 1;    //
        UVAR16 dcp : 1;     // Discharge enable
        UVAR16 st : 2;      // Self test mode
        UVAR16 md : 2;
        UVAR16 opc2 : 2;    // Fixed at 10b
        UVAR16 unused : 5;
    } __attribute__((packed)) bits;
} LTC831_ADC_CMD_T;

typedef enum ltc831InitMode { LTC831_POR_RESET = 0x00, LTC831_BMS_RESET } LTC831_INIT_MODE_T;


/******************************************************************************
*          P U B L I C   F U N C T I O N   D E C L A R A T I O N S           *
******************************************************************************/

void ltc831Init( LTC831_INIT_MODE_T mode, CL_DIR_E chainHead );
void ltc831Task( void );
void ltc831GetVersion( APP_CL_CHIP_Version_s *ver );

CL_APP_respStatus_e ltc831ClCmdStart( void );
CL_APP_respStatus_e ltc831GetCell( UVAR8 devIdx, UVAR8 cellIdx, PACK_DEV_REG_CELL_V_T *devReg );
CL_APP_respStatus_e ltc831GetTemp( UVAR8 devIdx, UVAR8 tempIdx, PACK_DEV_REG_TEMP_T *devReg );
CL_APP_respStatus_e ltc831GetDieTemp( UVAR8 devIdx, PACK_DEV_REG_TEMP_T *devReg );
CL_APP_respStatus_e ltc831GetStack( UVAR8 devIdx, PACK_DEV_REG_PACK_V_T *devReg );
CL_APP_respStatus_e ltc831SetWriteCfg( UVAR8 devIdx, PACK_DEV_REG_CFG_T *devReg );

// Auxiliary reading access functions
CL_APP_respStatus_e ltc831GetV5( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg );
CL_APP_respStatus_e ltc831GetV3( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg );
CL_APP_respStatus_e ltc831GetRef2( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg );
CL_APP_respStatus_e ltc831GetReadCfg( UVAR8 devIdx, PACK_DEV_REG_CFG_T *devReg );
CL_APP_respStatus_e ltc831GetStatus( UVAR8 devIdx, PACK_DEV_REG_STATUS_T *devReg );
CL_APP_respStatus_e ltc831GetFuseModuleID( UVAR8 devIdx, PACK_DEV_REG_FUSEROW_T *devReg );

void ltc831GetErrorStats( UVAR8 i, UVAR16 * hostPecErrorCount, UVAR16 * devPecErrorCount );
void ltc831SetActiveBalance( bool activeBalance );
bool ltc831GetActiveBalance( void );

#endif /* LTC831_H */
