/**
 *  @file   ltc831.c
 *  @brief  Driver files for LTC831 BMB chip
 *  @author Charlie Mellone
 */

/******************************************************************************
 *                              I N C L U D E S                               *
 ******************************************************************************/

#include "bmb_system.h"
#include <stddef.h>

// Additional integration includes
#include <string.h> // For memset
#include "lib_assert.h"
#include "bmb_timer.h"
#include "bmb_therm.h"
#include "bmb_driver.h"
#include "ltc831.h"
#include "ltc831_regs.h"
#include "pack.h"
#include "util.h" // For CRC14()

#include "dma.h" // For non-cached DMA rx buffer source
#include "gpio.h"

/******************************************************************************
 *                               D E F I N E S                                *
 ******************************************************************************/

#define LTC831_MAJOR_VER 1U
#define LTC831_MINOR_VER 0U
#define LTC831_BUILD_VER 1U

#define LTC831_USE_ACK             1

#define LTC831_TEST_CRC_RX         0
#define LTC831_DEBUG_CRC_DEV_IDX   4  // Inject receive PEC error on this received index
#define LTC831_DEBUG_CRC_DEV_COUNT 3U

#define LTC831_TEST_CRC_TX         0
#define LTC831_TX_CRC_ERROR_CNT    5  // Inject PEC error on this command in a new conversion cycle

#define LTC_DO_SCALE_CELLS         1  // Scale cell reading to 100uV/b rather then the native 80uV/b

#define LTC831_T_IDLE_MS           3  // ISO SPI times out to a powered-down state. Core still up
#define LTC831_T_WAKE_US         300  // ISO SPI wakeup time
#define LTC831_T_DIR_US           50  // Blankout time after direction change
#define LTC831_T_READY_US         10  // Startup time after ISO SPI wakeup
#define LTC831_SLEEP_TIMEOUT_MS 1700  // Core powers down (Vref, ADC, 3V power) if no valid command in this timeout
#define LTC831_T_REFUP_MS          5  // Time it takes for ADC and reference to wake up from core standby

#define LTC831_NOMINAL_POLLING_RATE_MS 100

#define LTC831_FUSE_MODULE_ID_REG ( LTC831_RDFS_NO_TAG_CMD + 0x3 ) // Third fuse row contains the module ID

static const UVAR8 ltc831_pollAuxRegs[] = { LTC831_RDAUXA_NO_TAG_CMD, LTC831_RDAUXB_NO_TAG_CMD,
                                            LTC831_EOR };

static const UVAR8 ltc831_pollConfigRegs[] = { LTC831_RDCFG_NO_TAG_CMD, LTC831_EOR };

// RDSTAT is first to allow some time between SNAP and result retrieval
// End with a module ID fuse row read
static const UVAR8 ltc831_pollRegs[] = { LTC831_RDSTAT_NO_TAG_CMD, LTC831_RDSNA_NO_TAG_CMD, LTC831_RDSNB_NO_TAG_CMD, LTC831_RDSNC_NO_TAG_CMD,
                                         LTC831_RDSND_NO_TAG_CMD, LTC831_RDSNE_NO_TAG_CMD, LTC831_RDSNF_NO_TAG_CMD,
                                         LTC831_EOR };

/******************************************************************************
 *                              T Y P E D E F S                               *
 ******************************************************************************/

typedef enum ltc831_state { LTC831_IDLE_STATE = 0, LTC831_ERROR_STATE, LTC831_CL_PASSTHROUGH_STATE, LTC831_COMMAND_COMPLETE_STATE, LTC831_BER_STATE,
                            LTC831_WAKEUP_STATE, LTC831_REG_READ_STATE, LTC831_REG_WRITE_STATE, LTC831_DEV_POLL_STATE, LTC831_NULL_STATE = 0xff } LTC831_STATE_E;

typedef struct {
    UVAR8 numWords; // There are 1, 2 and 3 word commands
    bool allowRead;
    bool allowWrite;
} LTC_REG_CONFIG_T;

typedef struct {
    LTC831_CVA_REG_T      CVA;
    LTC831_CVB_REG_T      CVB;
    LTC831_CVC_REG_T      CVC;
    LTC831_CVD_REG_T      CVD;
    LTC831_CVE_REG_T      CVE;
    LTC831_CVF_REG_T      CVF;
    LTC831_CVA_REG_T      SNA;
    LTC831_CVB_REG_T      SNB;
    LTC831_CVC_REG_T      SNC;
    LTC831_CVD_REG_T      SND;
    LTC831_CVE_REG_T      SNE;
    LTC831_CVF_REG_T      SNF;
    LTC831_AUXA_REG_T     AUXA;
    LTC831_AUXB_REG_T     AUXB;
    LTC831_STATUS_REG_T   STATUS;
    LTC831_CFG_REG_T      CFG_R;
    LTC831_CFG_REG_T      CFG_W;
    LTC831_PWMA_REG_T     PWMA_R;
    LTC831_PWMA_REG_T     PWMA_W;
    LTC831_PWMB_REG_T     PWMB_R;
    LTC831_PWMB_REG_T     PWMB_W;
    LTC831_OTP_FSR0_REG_T FSR0_R;
    LTC831_OTP_FSR1_REG_T FSR1_R;
    LTC831_OTP_FSR2_REG_T FSR2_R;
    LTC831_OTP_FSR3_REG_T FSR3_R;
    LTC831_OTP_FSR4_REG_T FSR4_R;
    LTC831_OTP_FSR5_REG_T FSR5_R;
    LTC831_OTP_FSR0_REG_T FSR0_W;
    LTC831_OTP_FSR1_REG_T FSR1_W;
    LTC831_OTP_FSR2_REG_T FSR2_W;
    LTC831_OTP_FSR3_REG_T FSR3_W;
} __attribute__((packed)) LTC831_REGS_T;

// This macro references the first element of the first register to satisfy cast alignment requirements
#define LTC831_GET_REG_BASE( _devIdx )  ( &ltcDevData[_devIdx].devRegs.CVA.C1 )
COMPILE_TIME_ASSERT(ltc831, 0U == offsetof(LTC831_REGS_T, CVA));   // Assert that this is the first element of the struct
COMPILE_TIME_ASSERT(ltc831, 0U == offsetof(LTC831_CVA_REG_T, C1)); // Assert that this is the first element of the struct

#if 0
typedef union {
    UVAR16 words[sizeof(LTC831_REGS_G_T)/2U];
    LTC831_REGS_G_T regs;
} __attribute__((packed)) LTC831_REGS_T;
#endif

typedef enum { LTC_CL_PARAM_SOURCE = 0, LTC_PASSED_PARAM_SOURCE, LTC_PREDEFINED_PARAM_SOURCE } LTC_TX_SRC_SEL_E;

typedef enum { LTC_REG_READ_CMD = 0, LTC_REG_READ_TAG_CMD, LTC_REG_WRITE_CMD, LTC_CMD_NULL=0xff } LTC_SPI_CMD_ID_E;

typedef enum ltc_status { LTC_SUCCESS = 0, LTC_BAD_PARAMETER, LTC_TIMEOUT, LTC_UNSUPPORTED_FEATURE, LTC_DEFERRED,
                          LTC_BUSY, LTC_DRIVER_ERROR, LTC_INVALID_COMMAND, LTC_PACK_FAILURE, LTC_ACK_FAILED, LTC_UNEXPECTED_COMMAND } LTC_STATUS_E;

typedef struct {
    bool running;
    bool newState;
    bool stateReEnter;
    bool por;
    bool wrCfgTime;
    bool waitDMA;
    bool dir;     // LTC_DIR_E
    bool prevDir; // LTC_DIR_E
    bool dirChange;
    bool autoPoll;
    bool newEntry;
    bool newConvert;
    bool didReturn;
    bool didFirst;
    bool doStore;
    bool activeBalance;
    CL_DIR_E chainHead; // Port direction first BMB is connected to
} LTC831_FLAGS_T;

typedef struct {
    bool otpPort;
    bool synchronous;
    bool useTag;
    bool useAck;
    bool doBreakDetect;
} LTC_PARAM_FLAGS_T;

typedef struct {
    LTC_PARAM_FLAGS_T  flags;
    LTC_SPI_CMD_ID_E   spiCmd;
    UVAR8              addr;
    UVAR8              tag;
    UVAR8              numDevs;
    UVAR16             *data_p;
} LTC_PARAM_T;

typedef struct {
    bool ltcResetMain;
    bool ltcResetMainParams;
} LTC_SM_FLAGS_T;

typedef enum ltc_frame_state { LTC_FRAME_EMPTY_STATE = 0, LTC_FRAME_READY_STATE, LTC_FRAME_COMPLETE_STATE, LTC_FRAME_NULL_STATE = 0xff } LTC_FRAME_STATE_E;

typedef struct {
    bool crcError;
    bool tagError;
} LTC_FRAME_FLAGS_T;

typedef struct {
    // The RX buffer needs to be 16-bit aligned for DMA to function correctly
    UVAR8              cmdDataRx[sizeof(LTC831_PKT_CMD_TAG_T) + ((CL_MAX_NUM_DEVS)* ((3 * sizeof(UVAR16)) + LTC831_PEC_SIZE_B + LTC831_TAG_SIZE_B))];
    UVAR8              alignPadding;   // Dummy for alignment padding
    UVAR8              cmdDataTx[sizeof(LTC831_PKT_CMD_TAG_T) + ((CL_MAX_NUM_DEVS)* ((3 * sizeof(UVAR16)) + LTC831_PEC_SIZE_B + LTC831_TAG_SIZE_B))];
    LTC_FRAME_FLAGS_T  frameFlags;
    LTC_FRAME_STATE_E  state;
    UVAR8              lengthB;        // Includes command and any FIFO frame length
    UVAR8              txCmdLengthB;   // Includes SPI command and crc
    UVAR8              txDataLengthW;  // Includes TX data plus PEC
    UVAR8              rxDataLengthB;  // Includes RX data plus PEC and optional TAG byte
    UVAR8              regLengthW;     // Does not include the read tag (ltc831RegMapConfig[addr].bits.lenB)
    UVAR8              respParamLengthB;
    UVAR8              rxNumDevs;
} LTC_FRAME_T;
// These buffers need to be 16-byte aligned. This coupled with the DMA_BUFF type declaration helps
// assert this requirement for now.
COMPILE_TIME_ASSERT(ltc831, 0U == (offsetof(LTC_FRAME_T, cmdDataRx) % 2));
COMPILE_TIME_ASSERT(ltc831, 0U == (offsetof(LTC_FRAME_T, cmdDataTx) % 2));

typedef enum { LTC831_REG_U = 0, LTC831_REG_R, LTC831_REG_W, LTC831_REG_B } LTC831_REG_TYPE_E;

typedef struct {
    LTC831_REG_TYPE_E type;   // LTC831_REG_TYPE_E
    bool cc_inc;
    bool ack;
    bool byte_swap;     // This register requires byte swapping as the contents are in words
    UVAR8 lenB;         // Length in bytes. Only 0,2,4 and 6 are used for LTC831
    UVAR8 regFlagIdx;   // Index into ltcDevRegFlags[]
    UVAR8 regOffset;
} LTC831_REG_FLAGS_T;

typedef struct {
    bool firstBreak;
    bool crcError;
    bool breakString;
    bool ackError;
} LTC831_BREAK_FLAGS_T;

typedef struct
{
    LTC831_BREAK_FLAGS_T flags;
    UVAR8                breakIdx;
    UVAR8                breakIdxPrev;
} LTC831_BREAK_DATA_T;


/******************************************************************************
 *         P R I V A T E   F U N C T I O N   D E C L A R A T I O N S          *
 ******************************************************************************/

CL_APP_respStatus_e ltc831ConvertStatus( LTC_STATUS_E status );
static void ltc831WakeupIdle( CL_SPI_PORTS_E port, CL_DIR_E dir, UVAR8 numDevs );
#if LTC831_USE_ACK
static LTC_STATUS_E LTC831_checkAck( void );
#endif // #if LTC831_USE_ACK
static void LTC831_clearDevReg( UVAR8 addr, UVAR8 numDevs );
static LTC_STATUS_E LTC831_RxParse( void );
static LTC_STATUS_E ltc831ReadCmd( LTC_PARAM_FLAGS_T flags, UVAR8 tag, UVAR8 numDevs, UVAR8 addr, UVAR16 *data_p );
static LTC_STATUS_E ltc831WriteCmd( LTC_PARAM_FLAGS_T flags, UVAR8 numDevs, UVAR8 addr, UVAR16 *data_p );
static void ltcResetStateMachine( LTC_SM_FLAGS_T smFlags );
static LTC_STATUS_E ltc831BuildCommand( void );
static LTC_STATUS_E ltcSendCommandFrame( void );
static LTC_STATUS_E ltc831SendClCmd( bool synchronous, CL_SPI_PORTS_E port, CL_DIR_E dir, UVAR8 *txBuff_p, UVAR8 *rxBuff_p, UVAR8 lenB, UVAR8 numDevs );
static void ltcPrepPoll( CL_DIR_E dir );
static void LTC831_updateOutData( void );

/******************************************************************************
 *              P R I V A T E   D A T A   D E F I N I T I O N S               *
 ******************************************************************************/
LTC831_STATE_E             ltc831State, ltc831PrevState, ltc831ReturnState;
UVAR8                      ltc831SubState;
LTC831_FLAGS_T             ltc831Flags;
static UVAR8               ltc831RegAddr;

static CL_reqFlags_t       ltc831ReqParamFlags;
static APP_CL_DIAG_FLAGS_T ltc831DiagFlags;
static UVAR16              ltc831BerTime;
static UVAR16              ltc831BerDelta;
static UVAR32              ltc831TxCycleCount;              // Number of convert cycles

#if (LTC831_TEST_CRC_TX)
static UVAR8               ltc831TxCrcErrCount;
#endif
#if (LTC831_TEST_CRC_RX)
static UVAR8               ltc831CrcErrDevIdx;
static UVAR8               ltc831CrcErrDevCount;
#endif

#if DO_BLEED
// LTC discharge timer values times 2 minutes to allow the first value, 0.5 minutes, to be stored as a value of 1.
static const UVAR8 ltcDCTO_x2[16] = { 0x00, 1, 2, 4, 6, 8, 10, 20, 30, 40, 60, 80, 120, 150, 180, 240 };
#endif

static LTC831_CFG_REG_T     ltc831ConfigReg[CL_MAX_NUM_DEVS]; // Used for writing new configuration values

#define LTC_WRITE_OFF          32
#define LTC831_NUM_MAPPED_REGS (LTC_WRITE_OFF+1) // Add an extra slot in case a write is done to the LTC_WRITE_OFF

typedef struct
{
    LTC831_REGS_T           devRegs;
    PACK_DEV_REG_FLAGS_T    devRegFlags[LTC831_NUM_MAPPED_REGS];
    UVAR16                  hostPecErrorCount;     // Host received a response with bad pec
    UVAR16                  devPecErrorCount;      // Device reported a bad pec
} LTC831_DEV_DATA_T;

LTC831_DEV_DATA_T           ltcDevData[CL_MAX_NUM_DEVS];

const APP_CL_CHIP_Version_s ltc831Ver = { LTC831_CHIP, {LTC831_MAJOR_VER, LTC831_MINOR_VER, LTC831_BUILD_VER} };

#define LTC831_NUM_REG_ADDR    58
#define LTC831_ITEMS_IN_REG_A_E 3

// These map one to one to LCR register addresses up to LTC831_RDFS_NO_TAG_CMD
// TYPE                 Command Counter  ACK          Byte Swap        lenB      flag index                 Register offset
const LTC831_REG_FLAGS_T ltc831RegMapConfig[LTC831_NUM_REG_ADDR] =
{ {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx=  0U,          .regOffset= 0U },
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  1U,          .regOffset= offsetof(LTC831_REGS_T, CVA)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  2U,          .regOffset= offsetof(LTC831_REGS_T, CVB)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  3U,          .regOffset= offsetof(LTC831_REGS_T, CVC)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  4U,          .regOffset= offsetof(LTC831_REGS_T, CVD)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  5U,          .regOffset= offsetof(LTC831_REGS_T, CVE)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx=  6U,          .regOffset= offsetof(LTC831_REGS_T, CVF)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  7U,          .regOffset= offsetof(LTC831_REGS_T, SNA)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  8U,          .regOffset= offsetof(LTC831_REGS_T, SNB)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx=  9U,          .regOffset= offsetof(LTC831_REGS_T, SNC)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx= 10U,          .regOffset= offsetof(LTC831_REGS_T, SND)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx= 11U,          .regOffset= offsetof(LTC831_REGS_T, SNE)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx= 12U,          .regOffset= offsetof(LTC831_REGS_T, SNF)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx= 13U,          .regOffset= offsetof(LTC831_REGS_T, AUXA)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 6U, .regFlagIdx= 14U,          .regOffset= offsetof(LTC831_REGS_T, AUXB)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 4U, .regFlagIdx= 15U,          .regOffset= offsetof(LTC831_REGS_T, STATUS)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 4U, .regFlagIdx= 16U,          .regOffset= offsetof(LTC831_REGS_T, CFG_R)/sizeof(UVAR16)},  // 0x10
  {.type= LTC831_REG_W, .cc_inc= true,  .ack= false, .byte_swap= false, .lenB= 4U, .regFlagIdx= 17U,          .regOffset= offsetof(LTC831_REGS_T, CFG_W)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx= 18U,          .regOffset= offsetof(LTC831_REGS_T, PWMA_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_W, .cc_inc= true,  .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx= 19U,          .regOffset= offsetof(LTC831_REGS_T, PWMA_W)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx= 20U,          .regOffset= offsetof(LTC831_REGS_T, PWMB_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_W, .cc_inc= true,  .ack= false, .byte_swap= true,  .lenB= 4U, .regFlagIdx= 21U,          .regOffset= offsetof(LTC831_REGS_T, PWMB_W)/sizeof(UVAR16)},
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },                          // 0x16 through 0x1f are unused
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_MUTE_CMD },                // 0x20
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_UNMUTE_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_PRIMEZ_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_PRIMEN_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_CLRDIAG_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_TEST5_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_TESTA_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_TESTOFF_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_ARMSOLO_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_DISSOLO_CMD },
  {.type= LTC831_REG_B, .cc_inc= false, .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_CLRCNT_CMD },
  {.type= LTC831_REG_B, .cc_inc= true,  .ack= true,  .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= LTC831_SNAP_CMD },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_U, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 0U, .regFlagIdx= LTC_WRITE_OFF, .regOffset= 0U },
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 22U,          .regOffset= offsetof(LTC831_REGS_T, FSR0_R)/sizeof(UVAR16)},  // 0x30
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 23U,          .regOffset= offsetof(LTC831_REGS_T, FSR1_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 24U,          .regOffset= offsetof(LTC831_REGS_T, FSR2_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 25U,          .regOffset= offsetof(LTC831_REGS_T, FSR3_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 26U,          .regOffset= offsetof(LTC831_REGS_T, FSR4_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_R, .cc_inc= false, .ack= false, .byte_swap= true,  .lenB= 2U, .regFlagIdx= 27U,          .regOffset= offsetof(LTC831_REGS_T, FSR5_R)/sizeof(UVAR16)},
  {.type= LTC831_REG_W, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 2U, .regFlagIdx= 28U,          .regOffset= offsetof(LTC831_REGS_T, FSR0_W)/sizeof(UVAR16)},  // 0xA0
  {.type= LTC831_REG_W, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 2U, .regFlagIdx= 29U,          .regOffset= offsetof(LTC831_REGS_T, FSR1_W)/sizeof(UVAR16)},
  {.type= LTC831_REG_W, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 2U, .regFlagIdx= 30U,          .regOffset= offsetof(LTC831_REGS_T, FSR2_W)/sizeof(UVAR16)},
  {.type= LTC831_REG_W, .cc_inc= false, .ack= false, .byte_swap= false, .lenB= 2U, .regFlagIdx= 31U,          .regOffset= offsetof(LTC831_REGS_T, FSR3_W)/sizeof(UVAR16)},
};

LTC_PARAM_T ltcParams;
// The RX buffer needs to be 16-bit aligned for DMA to function correctly
LTC_FRAME_T ltcFrames __attribute__((section(".dma_data"))) __attribute__((aligned(2)));

// Break detection variables
LTC831_BREAK_DATA_T ltcBreakData[CL_NUM_DIR];  // CL_DIR_E

/******************************************************************************
 *                     G L O B A L  V A R S                                   *
 ******************************************************************************/

/******************************************************************************
 *                     P R I V A T E   F U N C T I O N S                      *
 ******************************************************************************/

/*******************************************************
 * @brief  ltc831WakeupIdle
 * @param  port: The LTC831 Port that is intended to wakeup
 * @retval NONE
 ********************************************************/
/*
  The wakeup process is a bit permuted. If the core is in sleep then we must wakeup all the devices. This core wakeup
  takes so long that for long daisy chains some devices would have gone back to idle. Once the cores are all awake we
  need to pull the devices out of idle.
*/
static void ltc831WakeupIdle(CL_SPI_PORTS_E port, CL_DIR_E dir, UVAR8 numDevs)
{
    UVAR8 cnt;
    UVAR8 tempData[2U];

    TIM_OS_ID_T timerId;

    if ( ltc831Flags.dir != ltc831Flags.prevDir )
    {
        ltc831Flags.dirChange = true;
        ltc831Flags.prevDir = ltc831Flags.dir;
    }

    timerId = (dir == CL_DIR_A)? LTC_IDLE_TIMER_A : LTC_IDLE_TIMER_B;

    if ( ltc831Flags.dirChange || timOsCheckExpired( timerId ) )
    {   // If idle timer expired then do a short wakeup of ISO SPI's
        timDelaySync( 3 );

        for ( cnt = 0; cnt < numDevs; cnt++ )
        {
            GPIO_set(PRI_PORT_SEL, (dir==CL_DIR_A) ? false : true);

            // Conditionally send the right command for mute/unmute as
            // wakeup depending on the mode
            if( ltc831Flags.activeBalance )
            {
                tempData[0] = LTC831_MUTE_CMD;
                tempData[1] = 0xdd;
            }
            else
            {
                tempData[0] = LTC831_UNMUTE_CMD;
                tempData[1] = 0xf2;
            }


            spiTxData( port, (UVAR8 *)&tempData, 1 );

            spiWaitComplete( SPI_BM_A );

            if ( ltc831Flags.dirChange )
            {
                timDelaySync( LTC831_T_DIR_US );
                ltc831Flags.dirChange = false;
            }

            timDelaySync( LTC831_T_READY_US );
        }
        timOsSetAndStartTimer( timerId, LTC831_T_IDLE_MS );
    }
}


/******************************************************************************
 *                      P U B L I C   F U N C T I O N S                       *
 ******************************************************************************/

/*******************************************************
 * @brief  ltc831Init
 *         Initialize the LTC 831 Chip Driver
 * @param  mode: chip instance that needs to be initialized
 * @retval NONE
 ********************************************************/
void ltc831Init( LTC831_INIT_MODE_T mode, CL_DIR_E chainHead )
{
    GPIO_set(BMB_PRI_MEASURE, false);

#if (LTC831_TEST_CRC_RX)
    ltc831CrcErrDevIdx = LTC831_DEBUG_CRC_DEV_IDX;
    ltc831CrcErrDevCount = 0U;
#endif
#if (LTC831_TEST_CRC_TX)
    ltc831TxCrcErrCount  = 0U;
#endif

    ltc831State = LTC831_IDLE_STATE;
    ltc831PrevState = LTC831_NULL_STATE;
    //ltc831Flags.word = 0U;
    //ltc831DiagFlags.byte = 0U;
    ltc831TxCycleCount = 0U;
    ltc831BerTime = 0U;
    ltc831BerDelta = 0U;

    memset( (void *)&ltc831Flags, 0U, sizeof(ltc831Flags) );
    memset( (void *)&ltc831DiagFlags, 0U , sizeof(ltc831DiagFlags) );
    ltc831Flags.por = ( mode == LTC831_POR_RESET )?true:false;
    ltc831Flags.chainHead = chainHead; // Set the chain head from configuration

    memset( (void *)&ltcDevData, 0U, (sizeof(LTC831_DEV_DATA_T) * CL_MAX_NUM_DEVS) );
    memset( (void *)&ltcBreakData, 0U, (sizeof(LTC831_BREAK_DATA_T) * CL_NUM_DIR) );

    // Clear out the internal configuration register array
    memset( (void *)ltc831ConfigReg, 0U, sizeof(ltc831ConfigReg) );

    // The OS timers are based off the 1ms system tick
    timOsSetAndStartTimer( LTC_OS_TIMER_1, LTC831_NOMINAL_POLLING_RATE_MS );
    timOsSetExpired(LTC_OS_TIMER_1);
    timOsSetExpired(LTC_IDLE_TIMER_A);
    timOsSetExpired(LTC_IDLE_TIMER_B);
    timOsSetExpired(LTC_SLEEP_TIMER_A);
    timOsSetExpired(LTC_SLEEP_TIMER_B);



    // This delay causes problems when called early in the init
    // process because the watchdog will trip. Disabled for now
    // to fix SW-96532, but we might need to migrate this later
    // in the sequence if needed for the host chip init
    //timDelaySync( LTC831_T_WAKE_US );

    GPIO_set(BMB_PRI_MEASURE, true);
}


/*******************************************************
 * @brief  ltc831Task
 * @param  NONE
 * @retval NONE
 ********************************************************/
void ltc831Task( void )
{
    LTC_STATUS_E status = LTC_SUCCESS;
    CL_APP_respStatus_e clStatus = CL_APP_SUCCESS;
    CL_SPI_PORTS_E     port;


    if ( (!timOsCheckExpired(LTC_OS_TIMER_1) && !ltc831Flags.newEntry && !ltc831Flags.waitDMA)
         || ( !clTransaction.flags.reqValid ) )
    {
        return;
    }

    port = (ltc831ReqParamFlags.dir == CL_DIR_B )? SPI_BM_B : SPI_BM_A;

    ltc831Flags.newEntry = false;
    ltc831Flags.running = true;

    while ( ltc831Flags.running )
    {
        ltc831Flags.newState = false;
        if ( (ltc831State != ltc831PrevState) || ltc831Flags.stateReEnter )
        {
            ltc831Flags.newState = true;
        }
        ltc831PrevState = ltc831State;
        ltc831Flags.stateReEnter = false;

        ltc831Flags.running = false;

        switch ( ltc831State )
        {
            case ( LTC831_IDLE_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    timOsStopTimer( LTC_OS_TIMER_1 );
                }
                if ( clTransaction.chip != LTC831_CHIP )
                {
                    // Stay in idle
                }
                else
                {
                    if ( clTransaction.state == CL_RUNNING )
                    {   // The init command has completed.
                        clTransaction.state = CL_COMPLETE;
                    }
                    if ( clTransaction.state == CL_COMPLETE )
                    {   // Wait here until chiplayer responds
                        ltc831Flags.stateReEnter = true;
                        timOsSetAndStartTimer( LTC_OS_TIMER_1, LTC831_NOMINAL_POLLING_RATE_MS );
                    }
                }
                break;
            }

            case ( LTC831_WAKEUP_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    ltc831Flags.didFirst = false;
                    ltc831SubState = 0;
                }
                if ( ltc831SubState < (ltc831ReqParamFlags.numDevs + 1) )
                {
                    // Send a valid command as wakeup
                    ltcFrames.cmdDataTx[0] = LTC831_SNAP_CMD;
                    ltcFrames.cmdDataTx[1] = 0xfb;

                    spiTxData( SPI_BM_A, (UVAR8 *)&ltcFrames.cmdDataTx, 3 );
                    GPIO_set(PRI_PORT_SEL, (ltc831Flags.dir==CL_DIR_A) ? false : true);

                    timDelaySync( LTC831_T_DIR_US );

                    timOsSetAndStartTimer( LTC_OS_TIMER_1, 2 );  // Delay at least LTC_T_WAKE_US per interval
                    // Increment until we have woken up all of the LTC831 devices
                    ltc831SubState++;
                }
                else
                {
                    ltc831Flags.running = true;
                    if ( ltc831ReqParamFlags.needLoopback && !ltc831Flags.didFirst )  // needLoopback is the doBreak flag
                    {
                        ltc831Flags.didFirst = true;
                        ltc831Flags.dir = !ltc831Flags.dir;   // Wake other direction as well
                        ltc831SubState = 0x00U;
                    }
                    else
                    {
                        ltc831Flags.didFirst = false;
                        ltc831State = ltc831ReturnState;
                        ltc831ReturnState = LTC831_NULL_STATE;
                    }
                }
                break;
            }

            case ( LTC831_CL_PASSTHROUGH_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    ltc831SubState = 0U;
                    ltc831Flags.dir = (CL_DIR_E)ltc831ReqParamFlags.dir;
                }
#if 0
                if (  clTransaction.state != CL_RUNNING )
                {   // aborted usually due to timeout at chipDriver
                    clTransaction.state = CL_COMPLETE;
                    ltc831State = LTC831_IDLE_STATE;
                    ltc831Flags.running = true;
                }
#endif
                switch(ltc831SubState)
                {
                    case( 0U ):
                    {
                        ltc831WakeupIdle(port, (CL_DIR_E)ltc831Flags.dir, ltc831ReqParamFlags.numDevs );
                        ltc831SubState++;
                        ltc831Flags.running = true;
                        break;
                    }
                    case( 1U ):
                    {
                        status = ltc831SendClCmd( ltc831ReqParamFlags.synchronous,
                                                  port,
                                                  (CL_DIR_E)ltc831Flags.dir,
                                                  (UVAR8 *)&clTransaction.req_p->param.sPassthrough.data,
                                                  (UVAR8 *)&clTransaction.resp_p->param.sPassthrough.data,
                                                  clTransaction.req_p->paramLength,
                                                  ltc831ReqParamFlags.numDevs );

                        if ( status != LTC_DEFERRED )
                        {
                            ltcFrames.respParamLengthB = clTransaction.req_p->paramLength;
                            ltcFrames.rxNumDevs = (UVAR8)(clTransaction.req_p->reqFlags.numDevs & 0xFF);
                            ltc831State = LTC831_COMMAND_COMPLETE_STATE;
                            ltc831Flags.running = true;
                        }
                        break;
                    }
                }
                break;
            }

            case ( LTC831_REG_READ_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    ltc831Flags.dir = (CL_DIR_E)ltc831ReqParamFlags.dir;
                }

                status = ltc831ReadCmd( (LTC_PARAM_FLAGS_T){
                            .useTag=ltc831ReqParamFlags.useTag,
                            .doBreakDetect=ltc831ReqParamFlags.needLoopback,
                            .synchronous=ltc831ReqParamFlags.synchronous},
                    clTransaction.req_p->tag,
                    ltc831ReqParamFlags.numDevs & 0xFFU,
                    clTransaction.req_p->param.sRegRead.regAddr,
                    (UVAR16 *)&clTransaction.resp_p->param.sRegRead.data[0] );

                if ( status != LTC_DEFERRED )
                {
                    ltc831State = LTC831_COMMAND_COMPLETE_STATE;
                    ltc831Flags.running = true;
                }
                break;
            }

            case ( LTC831_REG_WRITE_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    ltc831Flags.dir = (CL_DIR_E)ltc831ReqParamFlags.dir;
                }

                // Update the internal config registers if this was a LTC831_WRCFG_CMD
                if( LTC831_WRCFG_CMD == clTransaction.req_p->param.sRegWrite.regAddr )
                {
                    memcpy( (void *)&ltc831ConfigReg,
                            (void*)clTransaction.req_p->param.sRegWrite.regData,
                            sizeof(ltc831ConfigReg) );
                }

                status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                            .useAck=ltc831ReqParamFlags.needResponse,
                            .doBreakDetect=ltc831ReqParamFlags.needLoopback,
                            .synchronous=ltc831ReqParamFlags.synchronous },
                    ltc831ReqParamFlags.numDevs & 0xFFU,
                    clTransaction.req_p->param.sRegWrite.regAddr,
                    (UVAR16 *)&clTransaction.req_p->param.sRegWrite.regData[0] );

                if ( status != LTC_DEFERRED )
                {
                    ltc831State = LTC831_COMMAND_COMPLETE_STATE;
                    ltc831Flags.running = true;
                }
                break;
            }

            case ( LTC831_BER_STATE ):
            {
                if ( !ltc831DiagFlags.bits.running )
                {
                    ltc831State = LTC831_IDLE_STATE;
                    ltc831Flags.running = true;
                }
                else
                {
                    if ( !ltc831Flags.didReturn )
                    {
                        ltc831ReturnState = ltc831State;
                        ltc831Flags.running = true;
                        ltc831State = LTC831_DEV_POLL_STATE;
                    }
                    else
                    {
                        if ( ltc831DiagFlags.bits.berMode != CL_BER_FIXED )
                        {
                            if ( ltc831DiagFlags.bits.dir )
                            {
                                ltc831BerTime = (UVAR16)(ltc831BerTime + ltc831BerDelta);
                            }
                            else
                            {
                                ltc831BerTime = (UVAR16)(ltc831BerTime - ltc831BerDelta);
                            }
                            if ( ltc831BerTime >= DIAG_BER_MAX_MS )
                            {
                                ltc831BerTime = DIAG_BER_MAX_MS;
                                ltc831DiagFlags.bits.dir = !ltc831DiagFlags.bits.dir;
                            }
                            else if ( ltc831BerTime <= DIAG_BER_MIN_MS )
                            {
                                ltc831BerTime = DIAG_BER_MIN_MS;
                                ltc831DiagFlags.bits.dir = !ltc831DiagFlags.bits.dir;
                            }
                        }
                        timOsSetAndStartTimer( LTC_OS_TIMER_1, ltc831BerTime );
                        ltc831Flags.didReturn = false;
                    }
                }
                break;
            }

/***************************************************************************
 * In Poll state, the poll sequence is as follows:
 *
 *  | <----------------------------------------- One cycle ----------------->|
 *  |                                                                        |
 *  | |MUTE#| |AUX RD| |PRIMEN#| |CFG RD| |SNAP| |STATUS + SNAP RD| |UMUTE#| |
 *
 * The # operations only run when doing an active balance
 * This sequence allows SNAP sufficient hold time before any MUTE/UNMUTE is
 * requested,(SNAP needs at least 220us after issued to cycle through its
 * mux for all the voltages). Reading the AUX groups gets this required delay
 *
 * Additionally, PRIMEN needs worse-case 380us to load the values, so the
 * CFG read transactions are used to statisfy this hold time as well
 ***************************************************************************/
            case ( LTC831_DEV_POLL_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    ltc831TxCycleCount++;
                    ltc831SubState = 0U;
                    ltc831RegAddr = 0U;
                    ltc831Flags.didFirst = false;
                }
                switch ( ltc831SubState )
                {
                    case ( 0 ):
                    {   // Set a MUTE command if in active balancing

                        // Send out in both directions
                        if ( !ltc831Flags.didFirst )
                        {
                            ltc831Flags.newConvert = true;
                        }
                        else
                        {
                            ltc831Flags.dir = !ltc831Flags.dir;
                        }

                        if ( ltc831Flags.activeBalance )
                        {
                            // When active balance is required, issue a MUTE
                            status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                                    .otpPort=ltc831ReqParamFlags.otpPort,
                                    .synchronous=ltc831ReqParamFlags.synchronous },
                                ltc831ReqParamFlags.numDevs & 0xFFU,
                                LTC831_MUTE_CMD,
                                0U );
                        }

                        if ( status != LTC_DEFERRED )
                        {
                            if ( !ltc831Flags.didFirst )
                            {
                                ltc831Flags.didFirst = true;
                            }
                            else
                            {
                                ltc831SubState++;
                                ltc831Flags.didFirst = false;
                                ltc831Flags.dir = ltc831ReqParamFlags.dir;
                            }

                            ltc831Flags.newEntry = true;
                        }
                        break;
                    }

                    case ( 1 ):
                    {   // Retrieve the auxiliary registers group
                        status = ltc831ReadCmd( (LTC_PARAM_FLAGS_T){
                                    .otpPort=ltc831ReqParamFlags.otpPort,
                                    .useTag=ltc831ReqParamFlags.useTag,
                                    .synchronous=ltc831ReqParamFlags.synchronous,
                                    .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                    },
                            clTransaction.req_p->tag,
                            ltc831ReqParamFlags.numDevs & 0xFFU,
                            ltc831_pollAuxRegs[ltc831RegAddr],
                            0U );

                        if ( status != LTC_DEFERRED )
                        {
                            ltc831RegAddr++;
                            ltc831Flags.newEntry = true;
                            if ( (status == LTC_SUCCESS) && (ltc831_pollAuxRegs[ltc831RegAddr] == LTC831_EOR) )
                            {
                                // Done with all auxiliary group reads
                                if ( ltc831Flags.activeBalance )
                                {
                                    // Issue a PRIMEN to set filters before reading
                                    status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                                            .otpPort=ltc831ReqParamFlags.otpPort,
                                            .synchronous=ltc831ReqParamFlags.synchronous },
                                        ltc831ReqParamFlags.numDevs & 0xFFU,
                                        LTC831_PRIMEN_CMD,
                                        0U );

                                    if ( status != LTC_DEFERRED )
                                    {
                                        // Proceed to next state
                                        ltc831SubState++;
                                    }
                                }
                                else
                                {
                                    // No need to do a PRIMEN if not balancing
                                    ltc831SubState++;
                                }

                                // Reset RegAddr for next substate
                                ltc831RegAddr = 0U;
                            }
                        }
                        break;
                    }

                    case ( 2 ):
                    {   // Retrieve the configuration registers group
                        status = ltc831ReadCmd( (LTC_PARAM_FLAGS_T){
                                    .otpPort=ltc831ReqParamFlags.otpPort,
                                    .useTag=ltc831ReqParamFlags.useTag,
                                    .synchronous=ltc831ReqParamFlags.synchronous,
                                    .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                    },
                            clTransaction.req_p->tag,
                            ltc831ReqParamFlags.numDevs & 0xFFU,
                            ltc831_pollConfigRegs[ltc831RegAddr],
                            0U );

                        if ( status != LTC_DEFERRED )
                        {
                            ltc831RegAddr++;
                            ltc831Flags.newEntry = true;
                            if ( (status == LTC_SUCCESS) && (ltc831_pollConfigRegs[ltc831RegAddr] == LTC831_EOR) )
                            {
                                // Switch to next state
                                ltc831SubState++;
                                // Reset RegAddr for next substate
                                ltc831RegAddr = 0U;
                            }
                        }
                        break;
                    }

                    case ( 3 ):
                    {   // Take a snapshot
                        status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                                .otpPort=ltc831ReqParamFlags.otpPort,
                                .useAck=ltc831ReqParamFlags.needResponse,
                                .synchronous=ltc831ReqParamFlags.synchronous },
                            ltc831ReqParamFlags.numDevs & 0xFFU,
                            LTC831_SNAP_CMD,
                            0U );

                        if ( status != LTC_DEFERRED )
                        {
                            ltc831SubState++;

                            ltc831Flags.newEntry = true;
                        }
                        break;
                    }

                    case ( 4 ):
                    {   // Retrieve the snapshot voltages from the devices
                        status = ltc831ReadCmd( (LTC_PARAM_FLAGS_T){
                                    .otpPort=ltc831ReqParamFlags.otpPort,
                                    .useTag=ltc831ReqParamFlags.useTag,
                                    .synchronous=ltc831ReqParamFlags.synchronous,
                                    .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                    },
                            clTransaction.req_p->tag,
                            ltc831ReqParamFlags.numDevs & 0xFFU,
                            ltc831_pollRegs[ltc831RegAddr],
                            0U );

                        if ( status != LTC_DEFERRED )
                        {
                            ltc831RegAddr++;
                            ltc831Flags.newEntry = true;
                            if ( (status == LTC_SUCCESS) && (ltc831_pollRegs[ltc831RegAddr] == LTC831_EOR) )
                            {
                                // Done with all snapshot voltages
                                if ( ltc831Flags.activeBalance )
                                {
                                    // Issue an UMUTE to bleed until next cycle
                                    status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                                            .otpPort=ltc831ReqParamFlags.otpPort,
                                            .useTag=false, // Tag not supported
                                            .synchronous=ltc831ReqParamFlags.synchronous,
                                            .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                            },
                                        ltc831ReqParamFlags.numDevs & 0xFFU,
                                        LTC831_UNMUTE_CMD,
                                        0U );

                                    if ( status != LTC_DEFERRED )
                                    {
                                        // Proceed to next state
                                        ltc831SubState++;
                                        ltc831RegAddr = 0U;
                                    }
                                }
                                else
                                {
                                    // No need to do an UNMUTE if not in active balance
                                    // proceed to next state
                                    ltc831SubState++;
                                    ltc831RegAddr = 0U;
                                }
                            }
                        }
                        break;
                    }

                    case ( 5 ):
                    {   // Read fuse row for module ID
                        status = ltc831ReadCmd( (LTC_PARAM_FLAGS_T){
                                     .otpPort=ltc831ReqParamFlags.otpPort,
                                     .useTag=false, // Fuse read doesn't support command tagging
                                     .synchronous=ltc831ReqParamFlags.synchronous,
                                     .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                     },
                             clTransaction.req_p->tag,
                             ltc831ReqParamFlags.numDevs & 0xFFU,
                             LTC831_FUSE_MODULE_ID_REG,
                             0U );

                        if ( status != LTC_DEFERRED )
                        {
                            ltc831SubState++;
                            ltc831Flags.newEntry = true;
                        }
                        break;
                    }

                    case ( 6 ):
                    {  // Populate pack.cell data
                        clStatus = packUpdate();

                        if ( clStatus != CL_APP_SUCCESS )
                        {
                            status = LTC_PACK_FAILURE;
                        }

                        ltc831SubState++;
                        ltc831Flags.newEntry = true;
                        break;
                    }
                    case ( 7 ):
                    {
                        // Now write the configuration register command
                        status = ltc831WriteCmd( (LTC_PARAM_FLAGS_T){
                                                .otpPort=ltc831ReqParamFlags.otpPort,
                                                .useTag=ltc831ReqParamFlags.useTag,
                                                .useAck=ltc831ReqParamFlags.useAck,
                                                .synchronous=ltc831ReqParamFlags.synchronous,
                                                .doBreakDetect=ltc831ReqParamFlags.needLoopback
                                            },
                                            ltc831ReqParamFlags.numDevs & 0xFFU,
                                            LTC831_WRCFG_CMD,
                                            (UVAR16 *)&ltc831ConfigReg[0].reg);

                        if ( status != LTC_DEFERRED )
                        {
                            // Done with poll
                            ltc831State = LTC831_COMMAND_COMPLETE_STATE;
                            ltc831Flags.newEntry = true;
                        }
                        break;
                    }
                }
                break;
            }

            case ( LTC831_COMMAND_COMPLETE_STATE ):
            {   // gcc
                if ( ltc831ReturnState != LTC831_NULL_STATE )
                {
                    ltc831State = ltc831ReturnState;
                    ltc831Flags.didReturn = true;
                    ltc831ReturnState = LTC831_NULL_STATE;
                }
                else
                {
                    ltc831Flags.didReturn = false;
                    clTransaction.state = CL_COMPLETE;
                    clTransaction.resp_p->status = ltc831ConvertStatus( status );
                    clTransaction.resp_p->respFlags.dir = ltc831ReqParamFlags.dir;
                    clTransaction.resp_p->respFlags.gotLoopback = ltc831ReqParamFlags.needLoopback;
                    clTransaction.resp_p->respFlags.gotResponse = ltc831ReqParamFlags.needResponse;
                    clTransaction.resp_p->respFlags.synchronous = ltc831ReqParamFlags.synchronous;
                    clTransaction.resp_p->respFlags.numDevs = ltcFrames.rxNumDevs;
                    clTransaction.resp_p->paramLength = ltcFrames.respParamLengthB;
                    clTransaction.resp_p->respFlags.tagFail = ltcFrames.frameFlags.tagError;

                    ltc831State = LTC831_IDLE_STATE;
                }
                ltc831Flags.running = true;
                break;
            }

            case ( LTC831_ERROR_STATE ):
            {
                if ( ltc831Flags.newState )
                {
                    timOsStopTimer( LTC_OS_TIMER_1 );
                    timOsClearExpired( LTC_OS_TIMER_1 );
                }
                else
                {
                    timOsSetAndStartTimer( LTC_OS_TIMER_1, 500 );
                }
                ltc831Flags.running = true;
                break;
            }

            case ( LTC831_NULL_STATE ):
            default:
            {
                break;
            }
        }
    }
}

CL_APP_respStatus_e ltc831ConvertStatus( LTC_STATUS_E  status )
{
    CL_APP_respStatus_e clStatus;

    switch (status)
    {
        case ( LTC_SUCCESS ):
        {
            clStatus = CL_APP_SUCCESS;
            break;
        }
        case ( LTC_INVALID_COMMAND ):
        {
            clStatus = CL_APP_INVALID_COMMAND;
            break;
        }
        case ( LTC_TIMEOUT ):
        {
            clStatus = CL_APP_TIMEOUT;
            break;
        }
        case ( LTC_UNSUPPORTED_FEATURE ):
        {
            clStatus = CL_APP_UNSUPPORTED_FEATURE;
            break;
        }
        case ( LTC_ACK_FAILED ):
        {
            clStatus = CL_APP_TOO_FEW_RESPONSES;
            break;
        }
        case ( LTC_DRIVER_ERROR ):
        {
            clStatus = CL_APP_DRIVER_ERROR;
            break;
        }
        case ( LTC_DEFERRED ):
        {
            clStatus = CL_APP_DEFERRED;
            break;
        }
        case ( LTC_PACK_FAILURE ):
        {
            clStatus = CL_APP_PACK_FAILURE;
            break;
        }
        default:
        {
            clStatus = CL_APP_UNKNOWN_ERROR;
            break;
        }
    }
    return( clStatus );
}

/*******************************************************
 * @brief  ltc831ClCmdStart
 *         Kick off a LTC831 command
 * @param  NONE
 * @retval NONE
 ********************************************************/
CL_APP_respStatus_e ltc831ClCmdStart( void )
{    // gcs
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    REQ_RESP_ID_E reqId;
    UVAR8 devIdx;
    APP_CL_errorCountSelect_T errCountSelect;
    TIM_OS_ID_T timerId;

    reqId = (REQ_RESP_ID_E)clTransaction.req_p->reqId.id_l;

    if ( (reqId != APP_CL_DIAG) && (ltc831State != LTC831_IDLE_STATE) )
    {   // APP_CL_DIAG is special because it runs in the background
        status = CL_APP_BUSY;
    }
    else
    {
        ltc831Flags.didReturn = false;
        ltc831ReqParamFlags = clTransaction.req_p->reqFlags;
        clTransaction.resp_p->respFlags.dir = clTransaction.req_p->reqFlags.dir;
        clTransaction.resp_p->respFlags.numDevs = 0U; // clTransaction.req_p->reqFlags.bits.numDevs;

        switch ( reqId )
        {
            // LTC831 initialization request
            case ( APP_CL_REQ_INIT ):
            {
                break;
            }

            // LTC831 passthrough request
            case (APP_CL_PASSTHROUGH):
            {
                // Set the size of the response data buffer, will be needed later
                if ( clTransaction.req_p->paramLength == 0U )
                {
                    status = CL_APP_BAD_PARAMETER;
                }
                else
                {
                    status = CL_APP_DEFERRED;
                    ltc831State = LTC831_CL_PASSTHROUGH_STATE;
                    ltc831Flags.newEntry = true;
                }
                break;
            }
            case ( APP_CL_REG_READ ):
            {
                status = CL_APP_DEFERRED;
                ltc831State = LTC831_REG_READ_STATE;
                ltc831Flags.newEntry = true;
                break;
            }
            case ( APP_CL_REG_WRITE ):
            {
                status = CL_APP_DEFERRED;
                ltc831State = LTC831_REG_WRITE_STATE;
                ltc831Flags.newEntry = true;
                break;
            }
            case ( APP_CL_PACK_CONVERT ):
            {
                status = CL_APP_DEFERRED;
                ltc831State = LTC831_DEV_POLL_STATE;
                ltc831Flags.newEntry = true;
                break;
            }

            case ( APP_CL_DIAG ):
            {
                switch ( clTransaction.req_p->param.sDiag.diagCmd )
                {
                    case ( CL_DIAG_BER ):
                    {
                        if ( ltc831State == LTC831_IDLE_STATE )
                        {   // If not already running then kick off a new BER. Otherwise just update the parameters
                            ltc831ReturnState = LTC831_BER_STATE;
                            ltc831State =  LTC831_WAKEUP_STATE;
                            ltc831Flags.newEntry = true;
                        }
                        ltc831DiagFlags.byte = clTransaction.req_p->param.sDiag.diagParams[0];
                        ltc831BerTime = (UVAR16)clTransaction.req_p->param.sDiag.diagParams[1];
                        ltc831BerDelta = (UVAR16)clTransaction.req_p->param.sDiag.diagParams[2];
                        break;
                    }
                    case ( CL_DIAG_READ ):
                    {
                        clTransaction.resp_p->param.sDiag.cycleCount = ltc831TxCycleCount;
                        clTransaction.resp_p->paramLength = sizeof(UVAR32);

                        for ( devIdx = 0U; devIdx < ltc831ReqParamFlags.numDevs; devIdx++ )
                        {
                            clTransaction.resp_p->param.sDiag.errorCounts[devIdx].hostRespErrorCount = 0U;
                            clTransaction.resp_p->param.sDiag.errorCounts[devIdx].hostCrcErrorCount = ltcDevData[devIdx].hostPecErrorCount;
                            clTransaction.resp_p->param.sDiag.errorCounts[devIdx].devErrorCount = ltcDevData[devIdx].devPecErrorCount;
                            clTransaction.resp_p->paramLength = (UVAR8)(clTransaction.resp_p->paramLength  + (UVAR8) sizeof(APP_CL_devErrorCount_s));
                        }
                        break;
                    }
                    case ( CL_DIAG_CLEAR ):
                    {
                        errCountSelect.byte = clTransaction.req_p->param.sDiag.diagParams[0];
                        if ( errCountSelect.bits.cycleCount )
                        {
                            ltc831TxCycleCount = 0U;
                        }
                        for ( devIdx = 0U; devIdx < ltc831ReqParamFlags.numDevs; devIdx++ )
                        {
                            if ( errCountSelect.bits.hostCrcErrorCount )
                            {
                                ltcDevData[devIdx].hostPecErrorCount = 0U;
                            }
                            if ( errCountSelect.bits.devErrorCount )
                            {
                                ltcDevData[devIdx].devPecErrorCount = 0U;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        status = CL_APP_BAD_PARAMETER;
                        break;
                    }
                }
                break;
            }
            default:
            {
                status = CL_APP_INVALID_COMMAND;
            }
        }
        if ( status == CL_APP_DEFERRED )
        {
            ltc831Flags.dir = (CL_DIR_E)ltc831ReqParamFlags.dir;
            timerId = (ltc831Flags.dir == CL_DIR_A) ? LTC_SLEEP_TIMER_A : LTC_SLEEP_TIMER_B;

            if ( timOsCheckExpired( timerId ) )
            {
                ltc831ReturnState = ltc831State;
                ltc831State = LTC831_WAKEUP_STATE;
            }
        }
    }
    return ( status );
}

static LTC_STATUS_E ltc831ReadCmd( LTC_PARAM_FLAGS_T flags, UVAR8 tag, UVAR8 numDevs, UVAR8 addr, UVAR16 *data_p )
{   // grc
    LTC_STATUS_E status = LTC_SUCCESS;

    if( !ltc831Flags.waitDMA )
    {
        if ( addr > ( LTC831_RDFS_NO_TAG_CMD + LTC831_FS_ROW_END ) )
        {
            status = LTC_UNSUPPORTED_FEATURE;
        }
        else if ( ltc831RegMapConfig[addr].type != LTC831_REG_R )
        {
            status = LTC_INVALID_COMMAND;
        }
        else if ( numDevs == 0U )
        {
            status = LTC_BAD_PARAMETER;
        }
        else
        {
            if ( flags.doBreakDetect )
            {
                // Only clear if break detection is requested
                if ( !ltc831Flags.didFirst )
                {
                    LTC831_clearDevReg( addr, numDevs );
                    (void) memset( (void *)&packApi.flags, 0U, sizeof(PACK_STATUS_FLAGS_T) );
                    packApi.breakIdx = 0x00U;
                }

                ltcPrepPoll( (CL_DIR_E) ltc831Flags.dir );   // Initialize breakDetect register in this direction
            }

            ltcResetStateMachine( (LTC_SM_FLAGS_T){.ltcResetMainParams=1} );

            ltcParams.flags = flags;
            ltcParams.tag = tag;
            ltcParams.numDevs = numDevs;
            ltcParams.data_p = data_p;

            if ( flags.useTag )
            {
                ltcParams.spiCmd = LTC_REG_READ_TAG_CMD;
            }
            else
            {
                ltcParams.spiCmd = LTC_REG_READ_CMD;
            }

            ltcParams.addr = addr;

            if ( status == LTC_SUCCESS )
            {
                status = ltc831BuildCommand();
            }
        }
    }

    if ( status == LTC_SUCCESS )
    {
        status = ltcSendCommandFrame();
    }

    if ( status == LTC_SUCCESS )
    {
        status = LTC831_RxParse();   // Update devRegs

        if ( ltcBreakData[ltc831Flags.dir].flags.crcError &&
             !ltc831Flags.didFirst &&
             ltcParams.flags.doBreakDetect )
        {   // There was a CRC error in our direction. Go poll data the other way. This may also indicate a break in comms
            ltc831Flags.didFirst = true;
            ltc831Flags.dir = !ltc831Flags.dir;
            status = LTC_DEFERRED;             // This will cause a re-entry into ltc831ReadCmd()
            ltc831Flags.newEntry = true;
        }
        else
        {
            if ( ltcParams.flags.doBreakDetect && (ltcBreakData[CL_DIR_A].flags.breakString || ltcBreakData[CL_DIR_B].flags.breakString) )
            {
                ltcFrames.rxNumDevs = (UVAR8)(ltcBreakData[CL_DIR_A].breakIdx + ltcBreakData[CL_DIR_B].breakIdx);

                if ( ltcFrames.rxNumDevs < ltc831ReqParamFlags.numDevs )
                {
                    packApi.flags.breakDetected = true;
                    packApi.flags.breakDetectDouble = true;
                }
                else if ( ltcFrames.rxNumDevs == ltc831ReqParamFlags.numDevs )
                {
                    packApi.flags.breakDetected = true;
                }
                else
                {
                    packApi.flags.breakDetectError = true;
                }
                // Adjust rxNumDevs to represent the sum of both directions
                packApi.breakIdx = ltcBreakData[CL_DIR_A].breakIdx;
            }
            if ( ltc831Flags.didFirst )
            {
                ltc831Flags.dir = !ltc831Flags.dir;  // Swap back so updateOutData writes in the correct order
            }
            LTC831_updateOutData();      // Update output data to chiplayer
            ltc831Flags.didFirst = false;
        }
    }

    if ( ltcBreakData[ltc831Flags.dir].flags.breakString )
    {
        ltcBreakData[ltc831Flags.dir].breakIdxPrev = ltcBreakData[ltc831Flags.dir].breakIdx;
    }

    return (status );
}


static LTC_STATUS_E ltc831WriteCmd( LTC_PARAM_FLAGS_T flags, UVAR8 numDevs, UVAR8 addr, UVAR16 *data_p )
{   // gwc
    LTC_STATUS_E status = LTC_SUCCESS;

    if( !ltc831Flags.waitDMA )
    {
        if ( addr >= LTC831_RDFS_NO_TAG_CMD )
        {
            status = LTC_UNSUPPORTED_FEATURE;
        }
        else if ( (ltc831RegMapConfig[addr].type != LTC831_REG_W) && (ltc831RegMapConfig[addr].type != LTC831_REG_B) )
        {
            status = LTC_BAD_PARAMETER;
        }
        else
        {
            if ( flags.doBreakDetect && ltc831RegMapConfig[addr].ack )
            {
                // Only clear if break detection is requested, and this command supports ACK
                if ( !ltc831Flags.didFirst )
                {
                    (void) memset( (void *)&packApi.flags, 0U, sizeof(PACK_STATUS_FLAGS_T) );
                    packApi.breakIdx = 0x00U;
                }

                ltcPrepPoll( (CL_DIR_E)ltc831Flags.dir );   // Initialize breakDetect register
            }

            ltcResetStateMachine( (LTC_SM_FLAGS_T){.ltcResetMainParams=1} );
            ltcParams.flags = flags;
            ltcParams.numDevs = numDevs;
            ltcParams.spiCmd = LTC_REG_WRITE_CMD;
        }

        if ( status == LTC_SUCCESS )
        {
            ltcParams.addr = addr;
            ltcParams.data_p = data_p;

            status = ltc831BuildCommand();
        }
    }

    if ( status == LTC_SUCCESS )
    {
        status = ltcSendCommandFrame();
    }

    if ( status == LTC_SUCCESS )
    {
        ltcFrames.rxNumDevs = numDevs;
#if LTC831_USE_ACK
        if ( ltc831RegMapConfig[ltcParams.addr].ack && ltcParams.flags.useAck )
        {
            status = LTC831_checkAck();

            if ( ltcBreakData[ltc831Flags.dir].flags.ackError &&
                 !ltc831Flags.didFirst &&
                 ltcParams.flags.doBreakDetect )
            {   // There was a ACK error in our direction. Go try data the other way. This may also indicate a break in comms
                ltc831Flags.didFirst = true;
                ltc831Flags.dir = !ltc831Flags.dir;
                status = LTC_DEFERRED;             // This will cause a re-entry into ltc831ReadCmd()
                ltc831Flags.newEntry = true;
            }
            else
            {
                ltc831Flags.didFirst = false;
                if ( ltcParams.flags.doBreakDetect && (ltcBreakData[CL_DIR_A].flags.breakString || ltcBreakData[CL_DIR_B].flags.breakString) )
                {
                    ltcFrames.rxNumDevs = (UVAR8)(ltcBreakData[CL_DIR_A].breakIdx + ltcBreakData[CL_DIR_B].breakIdx);
                    if ( ltcFrames.rxNumDevs < ltc831ReqParamFlags.numDevs )
                    {
                        packApi.flags.breakDetected = true;
                        packApi.flags.breakDetectDouble = true;
                    }
                    else if ( ltcFrames.rxNumDevs == ltc831ReqParamFlags.numDevs )
                    {
                        packApi.flags.breakDetected = true;
                        status = LTC_SUCCESS;   // The write would be successful in this case
                    }
                    else
                    {
                        packApi.flags.breakDetectError = true;
                    }
                    packApi.breakIdx = ltcBreakData[CL_DIR_A].breakIdx;
                }
                ltc831Flags.dir = !ltc831Flags.dir;  // Swap back
            }
        }
#endif
        if ( data_p == NULL )
        {
            clTransaction.resp_p->paramLength = 0U;
        }
    }
    return (status );
}

// Build up a command for transmit from transaction buffer
static LTC_STATUS_E ltc831BuildCommand( void )
{   // gbc
    LTC_STATUS_E       status = LTC_SUCCESS;
    UVAR8             *cmdPec_p;
    UVAR16             payPec;
    LTC_SPI_CMD_ID_E   spiCmd = LTC_CMD_NULL;
    UVAR8              addr;
    UVAR8              regIdx;
    UVAR16            *cmdData_p;
    UVAR16            *inData_p;
    UVAR8              regLengthW;
    UVAR8              devIdx, devIdxDest, itemIdx;
    UVAR8              numDevs;
    UVAR16            *devReg_p;
    LTC831_CMD_PKT_T  *cmd_p;

    cmdPec_p = NULL;

    ltcResetStateMachine( (LTC_SM_FLAGS_T){.ltcResetMain=1} );

    spiCmd = ltcParams.spiCmd;
    addr = ltcParams.addr;

    cmd_p = (LTC831_CMD_PKT_T *)((void *)&ltcFrames.cmdDataTx);

    cmd_p->ltcPkt.addr = addr;  // This is first byte of all command packets
    ltcFrames.txCmdLengthB = LTC831_CMD_PKT_SIZE_B;
    regLengthW = ltcFrames.regLengthW = ltc831RegMapConfig[addr].lenB/2U;

    if ( (packApi.numDevs != 0U) && (packApi.numDevs < ltcParams.numDevs) )
    {
        numDevs = packApi.numDevs;
    }
    else
    {
        numDevs = ltcParams.numDevs;
    }

    switch( spiCmd )
    {
        case( LTC_REG_WRITE_CMD ):
        {
            if ( (ltc831RegMapConfig[addr].type == LTC831_REG_W) || (ltc831RegMapConfig[addr].type == LTC831_REG_B) )
            {   // We need to first copy user data into the write registers and then populate the ltcFrames.cmdDataTx
                regIdx = ltc831RegMapConfig[addr].regFlagIdx;

                cmdPec_p = (UVAR8 *)&(cmd_p->ltcPkt.cmdPec);

                if( regLengthW )
                {
                    cmdData_p = (UVAR16 *)&(cmd_p->ltcPkt.data);

                    // Daisy write data must be written last device first
                    for (devIdx = 0; devIdx < numDevs; devIdx++)
                    {    // executes for each LTC device
                        devIdxDest = devIdx;

                        if ( (ltc831Flags.dir == ltc831Flags.chainHead) && ( numDevs != 0U ) )
                        {
                            devIdxDest = (UVAR8)(numDevs - devIdx - 1);
                        }
                        devReg_p = (UVAR16 *)&ltcDevData[devIdxDest].devRegs.CVA.C1 + ltc831RegMapConfig[addr].regOffset;

                        inData_p = ltcParams.data_p + (regLengthW * devIdxDest);

                        for (itemIdx = 0; itemIdx < regLengthW; itemIdx++)
                        {   // executes for each of items in the register
                            *(devReg_p + itemIdx) = *(inData_p + itemIdx);
                            *cmdData_p++ = *(inData_p + itemIdx);
                        }

                        payPec = LTC831_PEC_SEED_VALUE;
                        crc14_bytes( (UVAR8)(regLengthW * 2U), (UVAR8 *)inData_p, (UVAR16 *)&payPec ); // calculating the PEC for the data of each dev
                        itemIdx = 0U;
                        crc14_bits( LTC831_NUM_CC_BITS, itemIdx, (UVAR16 *)&payPec );   // Continue PEC over 2 zero value CC bits

#if (LTC831_TEST_CRC_TX)
                        if ( ltc831Flags.bits.newConvert )
                        {  // Only inject 1 TX CRC error per conver cycle
                            if ( ++ltc831TxCrcErrCount > LTC831_TX_CRC_ERROR_CNT )
                            {
                                ltc831Flags.bits.newConvert = false;
                                ltc831TxCrcErrCount = 0U;
                                payPec.word++;  // Force crc error in tx
                            }
                        }
#endif

#if (BITS_LITTLE_ENDIAN)
                        *cmdData_p = byteSwap16(payPec);
#else
                        *cmdData_p = payPec;
#endif
                        cmdData_p += 1U;

                        ltcDevData[devIdxDest].devRegFlags[regIdx].cmdCount = 0U;
                        ltcDevData[devIdxDest].devRegFlags[regIdx].dir = ltc831Flags.dir;
                        ltcDevData[devIdxDest].devRegFlags[regIdx].tag = ltcParams.tag;

                        ltcFrames.txDataLengthW = (UVAR8)(ltcFrames.txDataLengthW + regLengthW + (UVAR8)(LTC831_PEC_SIZE_B / 2U));
                    }
                }
            }
            else
            {
                status = LTC_BAD_PARAMETER;
            }
            break;
        }

        case( LTC_REG_READ_CMD ):
        case( LTC_REG_READ_TAG_CMD ):
        {
            if ( ltc831RegMapConfig[addr].type == LTC831_REG_R )
            {  //
                ltcFrames.rxDataLengthB = (UVAR8)((regLengthW * 2U) + LTC831_PEC_SIZE_B);

                if ( spiCmd == LTC_REG_READ_TAG_CMD )
                {
                    cmd_p->ltcPktTagged.addr |= LTC831_TAGGED_CMD_MASK;
                    cmd_p->ltcPktTagged.cmdTag = ltcParams.tag;
                    ltcFrames.txCmdLengthB =  LTC831_CMD_PKT_TAGGED_SIZE_B;
                    ltcFrames.rxDataLengthB++; // Add in RX Tag
                    cmdPec_p = (UVAR8 *)&(cmd_p->ltcPktTagged.cmdPec);
                }
                else
                {
                    ltcFrames.txCmdLengthB =  LTC831_CMD_PKT_SIZE_B;
                    cmdPec_p = (UVAR8 *)&(cmd_p->ltcPkt.cmdPec);
                }
                break;
            }
            else
            {
                status = LTC_BAD_PARAMETER;
            }
            break;
        }
        default :
        {
            if ( status == LTC_SUCCESS )
            {
                status = LTC_UNEXPECTED_COMMAND;
            }
        }
    }
    if ( cmdPec_p != NULL)
    {
        *cmdPec_p = 0x10;
        calcCRC( UTIL_CRC8_2F_POLY,
                 (UVAR8 *)&(cmd_p->ltcPkt.addr),
                 (UVAR8)(ltcFrames.txCmdLengthB - LTC831_CMD_PEC_SIZE_B),
                 cmdPec_p );

        ltcFrames.lengthB = (UVAR8)(ltcFrames.txCmdLengthB + (ltcFrames.txDataLengthW * 2U) + (UVAR8)(ltcParams.numDevs * ltcFrames.rxDataLengthB));
#if LTC831_USE_ACK
        if ( ltc831RegMapConfig[ltcParams.addr].ack && numDevs )
        {   // Read back 2 bits per device extra
            ltcFrames.lengthB = (UVAR8)(ltcFrames.lengthB + (UVAR8)(1U + ((numDevs-1U)/4U)));   // 2 bits per dev
        }
#endif
    }

    if ( status == LTC_SUCCESS )
    {
        ltcFrames.state = LTC_FRAME_READY_STATE;
//        status = ltcSendCommandFrame();
    }
    return( status );
}

static LTC_STATUS_E ltcSendCommandFrame( void )
{   // gsc
    LTC_STATUS_E status = LTC_INVALID_COMMAND;
    CL_SPI_PORTS_E port = SPI_BM_A;
//
//#if( HW_PLATFORM != PLATFORM_B2_APPLICATION ) // No OTP support yet
//        if ( ltcParams.flags.bits.otpPort )
//        {
//            port = SPI_BM_OTP;
//        }
//        else
//#endif // #if( HW_PLATFORM != PLATFORM_B2_APPLICATION )
//        {
//            if ( ltc831Flags.dir == CL_DIR_A )
//            {
//                port = SPI_BM_A;
//            }
//            else
//            {
//                port = SPI_BM_B;
//            }
//        }

//    if( !ltc831Flags.waitDMA )
//    {
//        if ( ltcFrames.state != LTC_FRAME_READY_STATE )
//        {
//            return ( LTC_BUSY );
//        }
//    }

    status = ltc831SendClCmd( ltcParams.flags.synchronous,
                              port,
                              (CL_DIR_E)ltc831Flags.dir,
                              (UVAR8 *)&ltcFrames.cmdDataTx,
                              (UVAR8 *)&ltcFrames.cmdDataRx,
                              ltcFrames.lengthB,
                              ltcParams.numDevs );

    if ( status == LTC_SUCCESS )
    {
        ltcFrames.state = LTC_FRAME_COMPLETE_STATE;
    }
    return( status );
}

/*******************************************************
 * @brief  ltc831SendClCmd
 *         Handle an LTC831 SPI-type command
 * @param  NONE
 * @retval status: The result of the SPI Xfer
 ********************************************************/
static LTC_STATUS_E ltc831SendClCmd( bool synchronous, CL_SPI_PORTS_E port, CL_DIR_E dir, UVAR8 *txBuff_p, UVAR8 *rxBuff_p, UVAR8 lenB, UVAR8 numDevs )
{
    LTC_STATUS_E status = LTC_SUCCESS;
    UVAR16 lenW;
    TIM_OS_ID_T timerId;

    if( !ltc831Flags.waitDMA )
    {
        ltc831WakeupIdle(port, dir, numDevs);

        GPIO_set(PRI_PORT_SEL, (dir==CL_DIR_A) ? false : true);

        lenW = lenB/2U;
        if ( lenB & BIT0_MSK )
        {   // Round up to handle odd number of bytes
            lenW++;
        }

        spiTxRxData( IO_SPI_DEVICE_BMB_PRIMARY,
                     txBuff_p,
                     rxBuff_p,
                     lenW );

        if ( synchronous )
        {
            if ( ltc831Flags.dirChange )
            {
                timDelaySync( LTC831_T_DIR_US );
            }

            while( !spiCheckComplete(IO_SPI_DEVICE_BMB_PRIMARY) );
        }
        else
        {
            if ( ltc831Flags.dirChange )
            {
                timDelaySync( LTC831_T_DIR_US );
            }

            ltc831Flags.waitDMA = true;
            status = LTC_DEFERRED;
        }
    }

    if ( spiCheckComplete(IO_SPI_DEVICE_BMB_PRIMARY) )
    {
        ltc831Flags.waitDMA = false;
        status = LTC_SUCCESS;

        timerId = (dir == CL_DIR_A) ? LTC_SLEEP_TIMER_A : LTC_SLEEP_TIMER_B;
        timOsSetAndStartTimer( timerId, LTC831_SLEEP_TIMEOUT_MS );
        timerId = (dir == CL_DIR_A) ? LTC_IDLE_TIMER_A : LTC_IDLE_TIMER_B;
        timOsSetAndStartTimer( timerId, LTC831_T_IDLE_MS );
    }
    return( status );
}

#if( LTC831_USE_ACK )
static LTC_STATUS_E LTC831_checkAck( void )
{
    LTC_STATUS_E status = LTC_SUCCESS;
    UVAR8   devIdx, devIdxDest;
    UVAR8   regFlagIdx;
    UVAR8  *inData_p;
    UVAR8   numDevs;
    UVAR8   bitMask;

    inData_p = &ltcFrames.cmdDataRx[LTC831_CMD_PKT_SIZE_B];
    bitMask = 0xc0;
    ltcFrames.rxNumDevs = 0U;

    if ( (packApi.numDevs != 0U) && (packApi.numDevs < ltcParams.numDevs) )
    {
        numDevs = packApi.numDevs;
    }
    else
    {
        numDevs = ltcParams.numDevs;
    }

    for ( devIdx = 0; devIdx < ltcParams.numDevs; devIdx++)      // executes for every device in the daisy chain
    {   // Check 2 bits per dev
        devIdxDest = devIdx;

        if ( (ltc831Flags.dir != ltc831Flags.chainHead) && ( numDevs != 0U ) )
        {
            devIdxDest = (UVAR8)(numDevs - devIdx - 1);
        }

        regFlagIdx = ltc831RegMapConfig[ltcParams.addr].regFlagIdx;

        if ( (*inData_p & bitMask) != 0U )
        {
            // Ack Failed
            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].ackFail = true;
            status = LTC_ACK_FAILED;

            ltcBreakData[ltc831Flags.dir].flags.ackError = true;

            if ( ltcBreakData[ltc831Flags.dir].flags.firstBreak == false )
            {
                ltcBreakData[ltc831Flags.dir].flags.firstBreak = true;
                ltcBreakData[ltc831Flags.dir].flags.breakString = true;
            }
            else
            {
            }
        }
        else
        {
            // Got ACK
            ltcBreakData[ltc831Flags.dir].breakIdx = (UVAR8)(devIdx + 0x01U);         // Track last good device
            if ( ltcBreakData[ltc831Flags.dir].flags.firstBreak == true )
            {
                ltcBreakData[ltc831Flags.dir].flags.breakString = false;
            }

            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].ackFail = false;
            ltcFrames.rxNumDevs++;
        }

        if ( bitMask == 0x03 )
        {
            bitMask = 0xc0;
            inData_p++;
        }
        else
        {
            bitMask = bitMask >> 2U;
        }
    }
    return(status);
}
#endif // #if LTC831_USE_ACK

static void LTC831_clearDevReg( UVAR8 addr, UVAR8 numDevs )
{
    UVAR8 devIdx;

    for ( devIdx = 0; devIdx < numDevs; devIdx++ )
    {
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].dir         = CL_DIR_A;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].crcError    = false;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].newData     = false;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].cmdCount    = 0U;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].sna         = false;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].discharge   = false;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].ackFail     = false;
        ltcDevData[devIdx].devRegFlags[ltc831RegMapConfig[addr].regFlagIdx].tag         = 0U;
    }
}

static void LTC831_updateOutData( void )
{   //
    UVAR8   devIdx, devIdxDest;
    UVAR8   numDevs;
    UVAR8   regFlagIdx, regAddr;
    UVAR16 *outData_p = NULL;
    UVAR16 *devReg_p;
    CL_REG_DESC_T regDesc;

    if ( ltcParams.data_p )
    {
        outData_p = ltcParams.data_p;

        if ( (packApi.numDevs != 0U) && (packApi.numDevs < ltcParams.numDevs) )
        {
            numDevs = packApi.numDevs;
        }
        else
        {
            numDevs = ltcParams.numDevs;
        }

        ltcFrames.frameFlags.tagError = false;

        for ( devIdx = 0; devIdx < ltcParams.numDevs; devIdx++)      // executes for every device in the daisy chain
        {   // Receive data is first device first. If we are reading from B dir we must reverse the order
            devIdxDest = devIdx;

            if ( (ltc831Flags.dir != CL_DIR_A) && numDevs )
            {
                devIdxDest = (UVAR8)(numDevs - devIdx - 1);
            }

            devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdxDest) + ltc831RegMapConfig[ltcParams.addr].regOffset;

            regFlagIdx = ltc831RegMapConfig[ltcParams.addr].regFlagIdx;

            // Write data descriptor if we have a good CRC or bad CRC and no newData
            (void) memset( (void *)&regDesc, 0U, sizeof(CL_REG_DESC_T) );
            regDesc.dir = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].dir;
            regDesc.crcError = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].crcError;
            regDesc.sna = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].sna;
            regDesc.newData = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].newData;
            regDesc.cmdCount = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].cmdCount;
            regDesc.ackFail = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].ackFail;
            regDesc.tagFail = ltcDevData[devIdxDest].devRegFlags[regFlagIdx].tagFail;

            if ( regDesc.tagFail )
            {
                ltcFrames.frameFlags.tagError = true;
            }

            regDesc.numWords = (UVAR8)(ltcFrames.regLengthW & 0x0F);

            //*outData_p++ = regDesc;
            memcpy( (void *)outData_p, (void*)&regDesc, sizeof(CL_REG_DESC_T) );
            outData_p += sizeof(CL_REG_DESC_T)/sizeof(UVAR16);

            ltcFrames.respParamLengthB = (UVAR8)(ltcFrames.respParamLengthB + 2U);

            for ( regAddr = 0;
                  regAddr < ltcFrames.regLengthW;
                  regAddr++ )
            {   // executes for each of items in the register
                *outData_p++ = *devReg_p++;
                ltcFrames.respParamLengthB = (UVAR8)(ltcFrames.respParamLengthB + 2U);
            }
        }
    }
}

static LTC_STATUS_E LTC831_RxParse( void )
{
    LTC_STATUS_E status = LTC_SUCCESS;
    UVAR8         devIdx, devIdxDest;
    UVAR16 parsed_item = 0U;
    bool    crcError;
    UVAR16 *devReg_p;
    UVAR8   regFlagIdx, regAddr;
    UVAR8  *inData_p;
    UVAR8  *pec_p = NULL;
    UVAR8   numDevs;
    UVAR8   tempTag = 0U;

    UVAR16 payload_pec = 0U;
    UVAR16 payloadRx_pec = 0U;
    UVAR8 payloadRx_commandCounter = 0U;

    regFlagIdx = 0U;

    if ( ltcParams.spiCmd == LTC_REG_READ_CMD )
    {
        inData_p = &ltcFrames.cmdDataRx[LTC831_CMD_PKT_SIZE_B];
    }
    else
    {
        inData_p = &ltcFrames.cmdDataRx[LTC831_CMD_PKT_TAGGED_SIZE_B];
    }

    if ( (packApi.numDevs != 0U) && (packApi.numDevs < ltcParams.numDevs) )
    {
        numDevs = packApi.numDevs;
    }
    else
    {
        numDevs = ltcParams.numDevs;
    }

    regFlagIdx = ltc831RegMapConfig[ltcParams.addr].regFlagIdx;

    (void) memset( (void *)&ltcFrames.frameFlags, 0U, sizeof(LTC_FRAME_FLAGS_T) );

    for ( devIdx = 0; devIdx < ltcParams.numDevs; devIdx++)      // executes for every device in the daisy chain
    {   // Receive data is first device first. If we are reading from B dir we must reverse the order

        if ( (ltc831Flags.dir != ltc831Flags.chainHead) && ( numDevs != 0U ) )
        {
            devIdxDest = (UVAR8)(numDevs - devIdx - 1);
        }
        else
        {
            devIdxDest = devIdx;
        }

        payload_pec = LTC831_PEC_SEED_VALUE;

        if( ltcFrames.rxDataLengthB < LTC831_PEC_SIZE_B )
        {
            // Set the payload RX values to zero in this case
            payloadRx_commandCounter = 0U;
            payloadRx_pec = 0U;
        }
        else
        {
            UVAR16 length_bits = (UVAR16)(((ltcFrames.rxDataLengthB - LTC831_PEC_SIZE_B) * 8U) + LTC831_NUM_CC_BITS);
            crc14( length_bits, inData_p, &payload_pec );    // calculating the PEC for each device
            pec_p = inData_p + ltcFrames.rxDataLengthB - LTC831_PEC_SIZE_B;

            // Command counter bits are in the top two bits of the first byte
            payloadRx_commandCounter = (pec_p[0] & 0xC0U) >> 6U;

            // PEC value is 14 bits of the remainder
            payloadRx_pec =  (UVAR16)( ((UVAR16)pec_p[0] << 8U ) | (UVAR16)(pec_p[1]) ) & 0x3FFFU;
        }

        if ( payload_pec == payloadRx_pec )
        {
            crcError = false;
        }
        else
        {
            crcError = true;
        }

        devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdxDest) + ltc831RegMapConfig[ltcParams.addr].regOffset;

        if ( ltcParams.spiCmd == LTC_REG_READ_TAG_CMD )
        {
            tempTag = *inData_p++;
        }

        ltc831Flags.doStore = false;
        if ( !crcError || !ltc831Flags.didFirst )
        {
            ltc831Flags.doStore = true;
        }

        if ( ltc831Flags.doStore )
        {
            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].cmdCount = payloadRx_commandCounter;
            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].dir = ltc831Flags.dir;
            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].crcError = crcError;
            ltcDevData[devIdxDest].devRegFlags[regFlagIdx].newData= true;

            if ( ltcParams.spiCmd == LTC_REG_READ_TAG_CMD )
            {
                if ( ltcParams.flags.useTag )
                {
                    ltcDevData[devIdxDest].devRegFlags[regFlagIdx].tagFail = false;
                    if ( tempTag != ltcParams.tag )
                    {
                        ltcDevData[devIdxDest].devRegFlags[regFlagIdx].tagFail = true;
                    }
                }
                ltcDevData[devIdxDest].devRegFlags[regFlagIdx].tag = tempTag;
            }

        }

        for ( regAddr = 0;
              regAddr < ltcFrames.regLengthW;
              regAddr++ )
        { // executes for each of items in the register

            if( ltc831RegMapConfig[ltcParams.addr].byte_swap )
            {
                UVAR8 temp_byte = 0U;
                // Each cell code is received as two bytes and is combined into a word
                // which may require swapping to get the bytes in the right endian format
                temp_byte = *(inData_p++);
                parsed_item= (UVAR16)( (*(inData_p++) << 8U ) | ( temp_byte ) );
            }
            else
            {
                // No byte swapping needed
                UVAR8 temp_byte = 0U;

                temp_byte =  *(inData_p++);
                parsed_item = (UVAR16)(( temp_byte << 8U ) | (*inData_p++));
            }

            if ( ltc831Flags.doStore )
            {
                *devReg_p++ = parsed_item;
            }
        }

        if ( crcError )
        {
            // Track the direction of the PEC error
            ltcBreakData[ltc831Flags.dir].flags.crcError = true;

            if (parsed_item == 0xFFFFU )
            {  // Write data descriptor sna when data is 0xffff and there is a PEC error
                if ( ltc831Flags.doStore )
                {
                    ltcDevData[devIdxDest].devRegFlags[regFlagIdx].sna = true;
                    ltcDevData[devIdxDest].devPecErrorCount++;
                }
                if ( !ltcBreakData[ltc831Flags.dir].flags.firstBreak )
                {
                    ltcBreakData[ltc831Flags.dir].flags.firstBreak = true;
                    ltcBreakData[ltc831Flags.dir].flags.breakString = true;
                }
            }
            else
            {
                if ( ltcBreakData[ltc831Flags.dir].flags.firstBreak )
                {
                    if ( devIdx > ltcBreakData[ltc831Flags.dir].breakIdx )
                    {   // SNA followed by good data clears breakString flag
                        ltcBreakData[ltc831Flags.dir].flags.breakString = false;
                    }
                }
                ltcDevData[devIdxDest].hostPecErrorCount++;
            }
        }
        else
        {
            ltcBreakData[ltc831Flags.dir].breakIdx = (UVAR8)(devIdx + 1U);         // Track last good device
            if ( ltcBreakData[ltc831Flags.dir].flags.firstBreak )
            {
                ltcBreakData[ltc831Flags.dir].flags.breakString = false;
            }
            ltcFrames.rxNumDevs++;
        }

        inData_p += LTC831_PEC_SIZE_B;
    }

    return(status);
}

static void ltcResetStateMachine( LTC_SM_FLAGS_T smFlags )
{
    if ( smFlags.ltcResetMainParams )
    {
        memset( &ltcParams, 0U, sizeof(LTC_PARAM_T));
    }
    if ( smFlags.ltcResetMain )
    {
        memset( &ltcFrames, 0U,sizeof(LTC_FRAME_T));
    }
}

// Sets the internal configuration register value to the one provided. The new values
// will be written as part of the poll state after the pack update is done
CL_APP_respStatus_e ltc831SetWriteCfg( UVAR8 devIdx, PACK_DEV_REG_CFG_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;

    if( ( devIdx > CL_MAX_NUM_DEVS ) || ( devReg == NULL ) )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        memcpy( &ltc831ConfigReg[devIdx], &(devReg->regValue), sizeof(LTC831_CFG_REG_T) );
    }

    return status;
}

// Returns a single cell voltage scaled to 100uV/b
CL_APP_respStatus_e ltc831GetCell( UVAR8 devIdx, UVAR8 cellIdx, PACK_DEV_REG_CELL_V_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR8  itemIdx;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = (UVAR8)(cellIdx/LTC831_ITEMS_IN_REG_A_E + LTC831_RDSNA_NO_TAG_CMD);
    itemIdx = cellIdx%LTC831_ITEMS_IN_REG_A_E;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( (devIdx > CL_MAX_NUM_DEVS) || (cellIdx >= LTC831_MAX_CELLS_PER_DEV) )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + itemIdx);
#if (LTC_DO_SCALE_CELLS)
        devReg->regValue = (UVAR16)(((UVAR32)LTC831_CELL_UV_b * itemValue)/PACK_BRICK_UV_b);
#else
        devReg->regValue = itemValue;
#endif
    }
    return( status );
}

CL_APP_respStatus_e ltc831GetDieTemp( UVAR8 devIdx, PACK_DEV_REG_TEMP_T *devReg )
{
    CL_APP_respStatus_e clStatus = CL_APP_SUCCESS;
    UVAR8    regAddr;
    UVAR16   itemValue;
    UVAR16  *devReg_p;
    UVAR8    regIdx;

    regAddr = LTC831_RDAUXB_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if ( devIdx > CL_MAX_NUM_DEVS )
    {
        clStatus = CL_APP_PARAM_OUT_OF_RANGE;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + 1U);

        devReg->regValue = (VAR16)(((LTC831_DIE_TEMP_GAIN_10K * itemValue)/1000U) - LTC831_DIE_TEMP_OFFSET_X10);
    }
    return( clStatus );
}

// Returns a single temperature value scaled to 0.1C/b
CL_APP_respStatus_e ltc831GetTemp( UVAR8 devIdx, UVAR8 tempIdx, PACK_DEV_REG_TEMP_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR8  itemIdx;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDAUXA_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( (devIdx > CL_MAX_NUM_DEVS) || ( tempIdx >= LTC831_MAX_TEMPS_PER_DEV) )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        itemIdx = (tempIdx == 0U)?0U:2U;

        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + itemIdx);

        devReg->regValue = (VAR16)(utilXyLookup_U16( itemValue, ltcToTemp, LTC_NUM_TEMP_ENTRIES ) - (LTC831_THERM_OFF * LTC831_THERM_MUL));
    }
    return( status );
}

CL_APP_respStatus_e ltc831GetStack( UVAR8 devIdx, PACK_DEV_REG_PACK_V_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR8  itemIdx;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDSNF_NO_TAG_CMD;
    itemIdx = 1U;   // Second item in reg

    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + itemIdx);

        devReg->regValue = (UVAR32)(LTC831_STACK_UV_b * itemValue)/PACK_STACK_UV_b;
    }
    return( status );
}

void ltc831GetVersion( APP_CL_CHIP_Version_s *ver )
{
    memcpy( ver, &ltc831Ver, sizeof(APP_CL_CHIP_Version_s));
}

// Returns the 5 volt regulator voltage for the given device, scaled to 100uV/b
CL_APP_respStatus_e ltc831GetV5( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDAUXA_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + 1U);   // V5 is the middle item in the AUX Group A register

#if (LTC_DO_SCALE_CELLS)
        // Same resolution as cells, so we can do the conversion similarly
        devReg->regValue = (UVAR16)(((UVAR32)LTC831_CELL_UV_b * itemValue)/PACK_BRICK_UV_b);
#else
        devReg->regValue = itemValue;
#endif
    }
    return( status );
}

void ltcPrepPoll( CL_DIR_E dir )
{
    (void) memset( &ltcBreakData[dir].flags, 0U, sizeof(LTC831_BREAK_FLAGS_T) );
    ltcBreakData[dir].breakIdx     = 0x00U;
    ltcBreakData[dir].breakIdxPrev = 0xffU;
}

// Returns the 3 volt regulator voltage for the given device, scaled to 100uV/b
CL_APP_respStatus_e ltc831GetV3( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDAUXB_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p);   // V3 is the first item in the AUX Group B register

#if (LTC_DO_SCALE_CELLS)
        // Same resolution as cells, so we can do the conversion similarly
        devReg->regValue = (UVAR16)(((UVAR32)LTC831_CELL_UV_b * itemValue)/PACK_BRICK_UV_b);
#else
        devReg->regValue = itemValue;
#endif
    }
    return( status );
}

// Returns the Ref2 measured voltage for the given device, scaled to 100uV/b
CL_APP_respStatus_e ltc831GetRef2( UVAR8 devIdx, PACK_DEV_REG_AUX_V_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 itemValue;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDAUXB_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        itemValue = *(devReg_p + 2U);   // Ref2 is the last item in the AUX Group B register

#if (LTC_DO_SCALE_CELLS)
        // Same resolution as cells, so we can do the conversion similarly
        devReg->regValue = (UVAR16)(((UVAR32)LTC831_CELL_UV_b * itemValue)/PACK_BRICK_UV_b);
#else
        devReg->regValue = itemValue;
#endif
    }

    return( status );
}

// Returns the read configuration register values
CL_APP_respStatus_e ltc831GetReadCfg( UVAR8 devIdx, PACK_DEV_REG_CFG_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDCFG_NO_TAG_CMD;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        (void) memcpy( (void *)&(devReg->regValue), (void *)devReg_p, sizeof(devReg->regValue) );
    }

    return( status );
}

// Returns the module ID, read from the 3rd fuse row
CL_APP_respStatus_e ltc831GetFuseModuleID( UVAR8 devIdx, PACK_DEV_REG_FUSEROW_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_FUSE_MODULE_ID_REG;
    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        devReg->regValue = *(devReg_p);
    }
    return( status );
}

// Returns the read status register values
CL_APP_respStatus_e ltc831GetStatus( UVAR8 devIdx, PACK_DEV_REG_STATUS_T *devReg )
{
    CL_APP_respStatus_e status = CL_APP_SUCCESS;
    UVAR8  regAddr;
    UVAR16 *devReg_p;
    UVAR8  regIdx;

    regAddr = LTC831_RDSTAT_NO_TAG_CMD;

    devReg_p = (UVAR16 *)LTC831_GET_REG_BASE(devIdx) + ltc831RegMapConfig[regAddr].regOffset;
    regIdx = ltc831RegMapConfig[regAddr].regFlagIdx;

    if( devIdx > CL_MAX_NUM_DEVS )
    {
        status = CL_APP_BAD_PARAMETER;
    }
    else
    {
        devReg->regFlags = ltcDevData[devIdx].devRegFlags[regIdx];

        (void) memcpy( (void *)&(devReg->regValue), (void *)devReg_p, sizeof(devReg->regValue) );
    }

    return( status );
}

void ltc831GetErrorStats( UVAR8 i, UVAR16 * hostPecErrorCount, UVAR16 * devPecErrorCount )
{
    if( i < CL_MAX_NUM_DEVS )
    {
        *hostPecErrorCount = ltcDevData[i].hostPecErrorCount;
        *devPecErrorCount = ltcDevData[i].devPecErrorCount;
    }
}

void ltc831SetActiveBalance( bool activeBalance )
{
    ltc831Flags.activeBalance = activeBalance;
}

bool ltc831GetActiveBalance( void )
{
    return ltc831Flags.activeBalance;
}
