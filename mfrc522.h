#ifndef MFRC522_H
#define MFRC522_H

#include <avr/io.h>
#include <util/delay.h>

// ---------- Commands ----------
#define PCD_IDLE              0x00
#define PCD_AUTHENT           0x0E
#define PCD_RECEIVE           0x08
#define PCD_TRANSMIT          0x04
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESETPHASE        0x0F
#define PCD_CALCCRC           0x03

// ---------- PICC (tag) commands ----------
#define PICC_REQIDL           0x26
#define PICC_REQALL           0x52
#define PICC_ANTICOLL         0x93
#define PICC_SELECTTAG        0x93
#define PICC_HALT             0x50

// ---------- Status ----------
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

#define MAX_LEN               16

// ---------- SPI pin mapping (Arduino Nano/Uno) ----------
#define DDR_SPI   DDRB
#define PORT_SPI  PORTB
#define PIN_SPI   PINB

#define DDR_SS    DDRH
#define PORT_SS   PORTH
#define PIN_SS    PINH

#define DD_MISO   PB3    // on mega pin 50
#define DD_MOSI   PB2    // on mega pin 51
#define DD_SCK    PB1    // on mega pin 52


#define DD_SS     PB0    // unused (we use custom SS pins), on mega pin 53

#define RESET_PIN PH1    // on mega pin 16

// ---------- Multiple readers SS pins ----------
#define NUM_READERS 4      // we have 4 readers, change to 1 if you have only 1 reader i = 0 to 3, pin6, pin7, pin8, pin9 on mega
#define READER1_SS  PH3   //on mega pin 6
#define READER2_SS  PH4   //on mega pin 7
#define READER3_SS  PH5   //0n mega pin 8
#define READER4_SS  PH6   //on mega pin 9


// API
void    SPI_init(void);
void    MFRC522_AllInit(void);
uint8_t MFRC522_Request(uint8_t reader, uint8_t reqMode, uint8_t *TagType);
uint8_t MFRC522_Anticoll(uint8_t reader, uint8_t *serNum);
void    MFRC522_Halt(uint8_t reader);
uint8_t MFRC522_ReadVersion(uint8_t reader);

#endif
