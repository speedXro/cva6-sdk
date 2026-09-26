#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>
#include <stddef.h>

#define AXI_BRAM_BASE   0x50000000UL
#define MAP_SIZE        0x1000          /* 4 KB register window          */

#ifndef DMA_BUF_BASE
#define DMA_BUF_BASE    0xBFF00000UL    /* physical base of reserved RAM */
#endif
#ifndef DMA_BUF_SIZE
#define DMA_BUF_SIZE    0x00100000UL    /* 1 MiB                         */
#endif

#define ALIGN           0x40u           /* 64-byte AXI bus alignment     */
#define ALIGN_MASK      (ALIGN - 1u)    /* 0x3F                          */
#define PAYLOAD_BYTES   0x40u           /* 8 x 64-bit words              */

/* ---- Channels ---- */
#define CH_SRC_MM2S 0
#define CH_DST_S2MM 1

/* ---- Register bit positions ---- */
#define CTRL_BIT_RS        0u
#define CTRL_BIT_RESET     2u
#define CTRL_BIT_IOCIRQEN 12u
#define STS_BIT_HALTED     0u
#define STS_BIT_IDLE       1u

#define STS_BIT_ERR_IRQ  14u
#define STS_BIT_DLY_IRQ  13u
#define STS_BIT_IOC_IRQ  12u

#define STS_BIT_SGD_ERR  10u
#define STS_BIT_SGS_ERR   9u
#define STS_BIT_SGI_ERR   8u
#define STS_BIT_DMD_ERR   6u
#define STS_BIT_DMS_ERR   5u
#define STS_BIT_DMI_ERR   4u

#define REG_SRC_CTRL      0x00u
#define REG_SRC_STS       0x04u
#define REG_SRC_ADDR_LSB  0x18u
#define REG_SRC_ADDR_MSB  0x1Cu
#define REG_SRC_LEN       0x28u

#define REG_DST_CTRL     0x30u
#define REG_DST_STS      0x34u
#define REG_DST_ADDR_LSB 0x48u
#define REG_DST_ADDR_MSB 0x4Cu
#define REG_DST_LEN      0x58u

#define F7_ERR_HANDLING  0x7Fu
#define F3_GET_ERR_DATA  0x7u



#define OPCODE 0x7B

#define F7_SET_QM_W_0 0x40
#define F7_GET_QM_W_0 0x50


#define F7_GET_AXIS_TMRS 0x60
#define F7_GET_AXIS_CNTS 0x61

#define F7_PRAM_OPERATION 0x68
#define F3_SELECT_PRAM    0x5
#define F3_WRITE_PRAM     0x6
#define F3_READ_PRAM      0x7


#define F7_AD_OPERATION   0x69
#define F3_AD_START       0x1
#define F3_AD_CLEAR       0x2
#define F3_AD_GET_STATS   0x4
#define F3_AD_GET_WR_CNT  0x5
#define F3_AD_SET_LIM     0x7

#define F7_UA_OPERATION   0x6A
#define F3_UA_START       0x1
#define F3_UA_CLEAR       0x2
#define F3_UA_GET_STATS   0x4
#define F3_UA_GET_RD_CNT  0x6
#define F3_UA_SET_LIM     0x7

#define F7_DA_OPERATION   0x6B
#define F3_DA_START       0x1
#define F3_DA_CLEAR       0x2
#define F3_DA_GET_STATS   0x4
#define F3_DA_GET_RD_CNT  0x6
#define F3_DA_SET_LIM     0x7

#define F7_UR_OPERATION   0x6C
#define F3_UR_GET_STOP    0x1
#define F3_UR_GET_READY   0x2
#define F3_UR_CLR_STOP    0x3
#define F3_UR_CLR_READY   0x4
#define F3_UR_GET_QFACT   0x5
#define F3_UR_GET_PIXFLR  0x6
#define F3_UR_GET_WSEL    0x7


#define F7_DMA_OPERATION  0x70
#define F3_DMA_SELECTION  0x03
#define F3_DMA_RESET      0x04
#define F3_DMA_INIT       0x05
#define F3_DMA_TRANSFER   0x06
#define F3_DMA_GET_STATUS 0x07

#define F7_STFT_OPERATION 0x78
#define F3_STFT_CFG       0x04
#define F3_STFT_START     0x05
#define F3_STFT_STATUS    0x06
#define F3_STFT_RESET     0x07

#define FMT_CPX   0x0
#define FMT_PWR32 0x1
#define FMT_PIX8  0x2

#define WSEL_RECT 0x0
#define WSEL_HANN 0x1

#define BLOCK_AD 0x00
#define BLOCK_UA 0x01
#define BLOCK_DA 0x02

#define STR2(x) #x
#define STR(x) STR2(x)

#define RVEXP_INSN(result, rs1, rs2, opcode, funct3, funct7) \
    asm volatile (                                               \
        ".insn r " STR(opcode) ", " STR(funct3) ", " STR(funct7) ", %0, %1, %2" \
        : "=r"(result)                                           \
        : "r"(rs1), "r"(rs2)                                     \
    )

#define SELECT_DMA(selection_result,dma_device)                                   \
    RVEXP_INSN(selection_result, dma_device, dma_device, OPCODE, F3_DMA_SELECTION, F7_DMA_OPERATION);

#define DO_SET_QM(op_h,op_l,dr)                                   \
    RVEXP_INSN(dr, op_h[0], op_l[0], OPCODE, 0x4, F7_SET_QM_W_0); \
    RVEXP_INSN(dr, op_h[1], op_l[1], OPCODE, 0x5, F7_SET_QM_W_0); \
    RVEXP_INSN(dr, op_h[2], op_l[2], OPCODE, 0x6, F7_SET_QM_W_0); \
    RVEXP_INSN(dr, op_h[3], op_l[3], OPCODE, 0x7, F7_SET_QM_W_0);

#define DO_GET_QM(results,dr)                                       \
    RVEXP_INSN(results[0], dr, dr, OPCODE, 0x0, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[1], dr, dr, OPCODE, 0x1, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[2], dr, dr, OPCODE, 0x2, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[3], dr, dr, OPCODE, 0x3, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[4], dr, dr, OPCODE, 0x4, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[5], dr, dr, OPCODE, 0x5, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[6], dr, dr, OPCODE, 0x6, F7_GET_QM_W_0);     \
    RVEXP_INSN(results[7], dr, dr, OPCODE, 0x7, F7_GET_QM_W_0);     

#define DO_GET_AXIS_TMRS(results,dr)                                   \
    RVEXP_INSN(results[0], dr, dr, OPCODE, 0x0, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[1], dr, dr, OPCODE, 0x1, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[2], dr, dr, OPCODE, 0x2, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[3], dr, dr, OPCODE, 0x3, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[4], dr, dr, OPCODE, 0x4, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[5], dr, dr, OPCODE, 0x5, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[6], dr, dr, OPCODE, 0x6, F7_GET_AXIS_TMRS);     \
    RVEXP_INSN(results[7], dr, dr, OPCODE, 0x7, F7_GET_AXIS_TMRS);  
    
#define DO_GET_AXIS_CNTS(results,dr)                                   \
    RVEXP_INSN(results[0], dr, dr, OPCODE, 0x0, F7_GET_AXIS_CNTS);     \
    RVEXP_INSN(results[1], dr, dr, OPCODE, 0x1, F7_GET_AXIS_CNTS);     

void Pack_QM_Inputs(uint8_t inputs[64], uint64_t p_ops_h[4], uint64_t p_ops_l[4]);
void Unpack_QM_Outputs(uint64_t results[8], uint8_t outputs[64]);

void Set_QM(uint8_t input[64]);
void Get_QM(uint8_t outputs[64]);

uint32_t DMA_Length(uint32_t addr, uint32_t payload);

typedef struct _STFT_Config {
    uint8_t window_sel;
    uint8_t enable;
    uint8_t out_format;
    uint8_t pix_flor;
} STFT_Config;

typedef struct _STFT_Status{
    uint8_t ctrl_busy;
    uint8_t frame_done;
    uint8_t frame_full;
    uint8_t frame_count;
} STFT_Status;


uint8_t STFT_Configure(STFT_Config new_config, STFT_Config* p_old);
uint8_t STFT_Start(void);
uint8_t STFT_Reset(void);
STFT_Status STFT_GetStatus(void); 


/* ---- Uncached DMA buffer pool ---- */
int       DMA_Buf_Init(void);                                
void      DMA_Buf_DeInit(void);
void     *DMA_Buf_Alloc(size_t size, uintptr_t *phys_out);   
void      DMA_Buf_Reset(void);                               
uintptr_t DMA_Buf_VirtToPhys(const void *vaddr);            


uint8_t Write_Bram_NoSel(uint64_t address, uint64_t data);
uint64_t Read_Bram_NoSel(uint64_t address);

uint8_t Select_PRAM(uint8_t pram_selection);
uint8_t Write_Bram(uint8_t block, uint64_t address, uint64_t data);
uint64_t Read_Bram(uint8_t block, uint64_t address);

uint8_t  AD_Write_Bram(uint64_t address, uint64_t data);
uint64_t AD_Read_Bram(uint64_t address);
uint8_t  UA_Write_Bram(uint64_t address, uint64_t data);
uint64_t UA_Read_Bram(uint64_t address);
uint8_t  DA_Write_Bram(uint64_t address, uint64_t data);
uint64_t DA_Read_Bram(uint64_t address);

void Write_AD_Array(uint64_t* array, int len);
void Read_AD_Array(uint64_t* array, int len);
void Write_UA_Array(uint64_t* array, int len);
void Read_UA_Array(uint64_t* array, int len);
void Write_DA_Array(uint64_t* array, int len);
void Read_DA_Array(uint64_t* array, int len);

uint8_t  Block_Start(uint8_t block);
uint8_t  Block_Clear(uint8_t block);
uint64_t Block_Get_Write_Counter(void);
uint64_t Block_Get_Read_Counter(uint8_t block);
uint64_t Block_Set_Lim(uint8_t block, uint64_t limit);

uint8_t  AD_Aq_Start(void);
uint8_t  AD_Aq_Clear(void);
void     AD_Get_Statuses(uint8_t* p_ad_full);
uint64_t AD_Get_Write_Counter(void);
uint64_t AD_Set_Lim(uint64_t limit);

uint8_t  UA_Start(void);
uint8_t  UA_Clear(void);
void     UA_Get_Statuses(uint8_t* p_sf_empty);
uint64_t UA_Get_Read_Counter(void);
uint64_t UA_Set_Lim(uint64_t limit);

uint8_t  DA_Start(void);
uint8_t  DA_Clear(void);
void     DA_Get_Statuses(uint8_t* p_sf_empty);
uint64_t DA_Get_Read_Counter(void);
uint64_t DA_Set_Lim(uint64_t limit);

uint8_t  UR_GetStop(void);
uint8_t  UR_GetReady(void);
uint8_t  UR_ClearStop(void);
uint8_t  UR_ClearReady(void);
uint8_t  UR_GetQFact(void);
uint8_t  UR_GetPixFloor(void);
uint8_t  UR_GetWindowSel(void);

void RVEXP_DMA_SelectDevice(uint8_t device);
void RVEXP_DMA_Reset(void);
uint8_t RVEXP_DMA_Init(void);
uint8_t RVEXP_DMA_Init_NoAlloc(void);
uint8_t RVEXP_DMA_Transaction(uint64_t* source, uint64_t* destination, uint64_t source_length, uint64_t destination_length);
void RVEXP_DMA_GetStatus_Opt(uint8_t* p_complete, uint8_t* p_irc, uint8_t* p_error);
void RVEXP_DMA_DeInit(void);

uint8_t RVEXP_Get_CustomInstructionError(uint8_t* p_code, uint8_t* p_state);
#endif /* DMA_H_ */
