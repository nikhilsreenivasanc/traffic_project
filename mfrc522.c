#include "mfrc522.h"

// ---------- Register map ----------
#define CommandReg        0x01
#define CommIEnReg        0x02
#define CommIrqReg        0x04
#define ErrorReg          0x06
#define FIFODataReg       0x09
#define FIFOLevelReg      0x0A
#define ControlReg        0x0C
#define BitFramingReg     0x0D
#define ModeReg           0x11
#define TxControlReg      0x14
#define TxASKReg          0x15
#define CRCResultRegH     0x21
#define CRCResultRegL     0x22
#define TModeReg          0x2A
#define TPrescalerReg     0x2B
#define TReloadRegH       0x2C
#define TReloadRegL       0x2D
#define VersionReg        0x37

// ---------- SS control ----------
static inline void SS_LOW(uint8_t reader) {
    if (reader==0) PORTH &= ~(1<<READER1_SS);
    else if(reader==1) PORTH &= ~(1<<READER2_SS);
    else if(reader==2) PORTH &= ~(1<<READER3_SS);
    else if(reader==3) PORTH &= ~(1<<READER4_SS);
}
static inline void SS_HIGH(uint8_t reader) {
    if (reader==0) PORTH |= (1<<READER1_SS);
    else if(reader==1) PORTH |= (1<<READER2_SS);
    else if(reader==2) PORTH |= (1<<READER3_SS);
    else if(reader==3) PORTH |= (1<<READER4_SS);

    
}

void SPI_init(void) {
    // MOSI, SCK, SS, RESET output; MISO input
    DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS);
    DDR_SPI &= ~(1<<DD_MISO);

    // Reader SS pins as output
    DDR_SS |= (1<<READER1_SS) | (1<<READER2_SS) | (1<<READER3_SS) | (1<<READER4_SS);
    DDR_SS |= (1<<RESET_PIN); // RESET output

    // Idle high
    SS_HIGH(0); SS_HIGH(1); SS_HIGH(2); SS_HIGH(3);
    PORTH |= (1<<RESET_PIN); // release reset

    // SPI enable, master, F_CPU/16
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

static uint8_t SPI_xfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

// ---------- R/W ----------
static void PCD_WriteRegister(uint8_t reader, uint8_t reg, uint8_t val) {
    SS_LOW(reader);
    SPI_xfer((reg<<1)&0x7E);
    SPI_xfer(val);
    SS_HIGH(reader);
}
static uint8_t PCD_ReadRegister(uint8_t reader, uint8_t reg) {
    uint8_t val;
    SS_LOW(reader);
    SPI_xfer(((reg<<1)&0x7E)|0x80);
    val = SPI_xfer(0);
    SS_HIGH(reader);
    return val;
}
static void PCD_SetBitMask(uint8_t r, uint8_t reg, uint8_t mask) {
    uint8_t tmp = PCD_ReadRegister(r,reg);
    PCD_WriteRegister(r,reg,tmp|mask);
}
static void PCD_ClearBitMask(uint8_t r, uint8_t reg, uint8_t mask) {
    uint8_t tmp = PCD_ReadRegister(r,reg);
    PCD_WriteRegister(r,reg,tmp&(~mask));
}
static void PCD_Reset(uint8_t r) {
    PCD_WriteRegister(r, CommandReg, PCD_RESETPHASE);
    _delay_ms(50);
}

uint8_t MFRC522_ReadVersion(uint8_t r) {
    return PCD_ReadRegister(r, VersionReg);
}

void MFRC522_Init(uint8_t r) {
    PCD_Reset(r);
    PCD_WriteRegister(r, TModeReg, 0x8D);
    PCD_WriteRegister(r, TPrescalerReg, 0x3E);
    PCD_WriteRegister(r, TReloadRegL, 30);
    PCD_WriteRegister(r, TReloadRegH, 0);
    PCD_WriteRegister(r, TxASKReg, 0x40);
    PCD_WriteRegister(r, ModeReg, 0x3D);

    uint8_t tx = PCD_ReadRegister(r, TxControlReg);
    if (!(tx & 0x03)) {
        PCD_WriteRegister(r, TxControlReg, tx | 0x03);
    }
}

void MFRC522_AllInit(void) {
    for(uint8_t i=0;i<NUM_READERS;i++) {    //change i to 0 to init all 4 readers, i = pin6, pin7, pin8, pin9 on mega
        MFRC522_Init(i);
    }
}

// ---------- ToCard ----------
static uint8_t PCD_ToCard(uint8_t r, uint8_t cmd,
                          uint8_t *sendData, uint8_t sendLen,
                          uint8_t *backData, uint8_t *backBits) {
    uint8_t status=MI_ERR,n,lastBits;
    uint16_t i;

    PCD_WriteRegister(r, CommIEnReg, 0x77|0x80);
    PCD_ClearBitMask(r, CommIrqReg, 0x80);
    PCD_SetBitMask(r, FIFOLevelReg, 0x80);

    for (i=0;i<sendLen;i++) PCD_WriteRegister(r,FIFODataReg,sendData[i]);
    PCD_WriteRegister(r,CommandReg,cmd);
    if (cmd==PCD_TRANSCEIVE) PCD_SetBitMask(r,BitFramingReg,0x80);

    i=2000;
    do {
        n=PCD_ReadRegister(r,CommIrqReg);
        i--;
    } while(i && !(n&0x01) && !(n&0x30));

    PCD_ClearBitMask(r,BitFramingReg,0x80);
    if(i) {
        uint8_t err=PCD_ReadRegister(r,ErrorReg);
        if(!(err&0x1B)) {
            status=MI_OK;
            if(n&0x01) status=MI_NOTAGERR;
            if(cmd==PCD_TRANSCEIVE) {
                n=PCD_ReadRegister(r,FIFOLevelReg);
                lastBits=PCD_ReadRegister(r,ControlReg)&0x07;
                if(lastBits) *backBits=(n-1)*8+lastBits;
                else *backBits=n*8;
                if(n==0) n=1;
                if(n>MAX_LEN) n=MAX_LEN;
                for(i=0;i<n;i++) backData[i]=PCD_ReadRegister(r,FIFODataReg);
            }
        }
    }
    return status;
}

// ---------- High-level ----------
uint8_t MFRC522_Request(uint8_t r,uint8_t reqMode,uint8_t *TagType) {
    uint8_t status,backBits;
    PCD_WriteRegister(r,BitFramingReg,0x07);
    TagType[0]=reqMode;
    status=PCD_ToCard(r,PCD_TRANSCEIVE,TagType,1,TagType,&backBits);
    if(status!=MI_OK||backBits!=0x10) status=MI_ERR;
    return status;
}
uint8_t MFRC522_Anticoll(uint8_t r,uint8_t *serNum) {
    uint8_t status,unLen;
    PCD_WriteRegister(r,BitFramingReg,0x00);
    serNum[0]=PICC_ANTICOLL;
    serNum[1]=0x20;
    status=PCD_ToCard(r,PCD_TRANSCEIVE,serNum,2,serNum,&unLen);
    if(status!=MI_OK) return status;
    uint8_t check=0;
    for(uint8_t i=0;i<4;i++) check^=serNum[i];
    if(check!=serNum[4]) status=MI_ERR;
    return status;
}
void MFRC522_Halt(uint8_t r) {
    uint8_t buf[4],unLen;
    buf[0]=PICC_HALT; buf[1]=0;
    PCD_WriteRegister(r,CommandReg,PCD_IDLE);
    PCD_SetBitMask(r,FIFOLevelReg,0x80);
    PCD_WriteRegister(r,FIFODataReg,buf[0]);
    PCD_WriteRegister(r,FIFODataReg,buf[1]);
    PCD_WriteRegister(r,CommandReg,PCD_TRANSCEIVE);
    PCD_ToCard(r,PCD_TRANSCEIVE,buf,2,buf,&unLen);
}
