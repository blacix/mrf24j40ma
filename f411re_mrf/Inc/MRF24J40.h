
#ifndef MRF24J40_H
#define MRF24J40_H

#include "stm32f4xx_hal.h"
#include <vector>
#define DEFAULT_CHANNEL 11
#define MRF_DEFAULT_PAN 1

/* INTSTAT */
#define SLPIF		(1<<7)
#define WAKEIF		(1<<6)
#define HSYMTMRIF	(1<<5)
#define SECIF		(1<<4)
#define RXIF		(1<<3)
#define TXG2IF		(1<<2)
#define TXG1IF		(1<<1)
#define TXNIF		(1)

/* INTCON */
#define SLPIE		(1<<7)
#define WAKEIE		(1<<6)
#define HSYMTMRIE	(1<<5)
#define SECIE		(1<<4)
#define RXIE		(1<<3)
#define TXG2IE		(1<<2)
#define TXG1IE		(1<<1)
#define TXNIE		(1)

/* SLPACK */
#define _SLPACK		(1<<7)
#define WAKECNT_L(x)	(x & 0x03F)

/* RFCTL */
#define WAKECNT_H(x)	((x & 0x03) << 3)
#define RFRST		(1<<2)
#define RFTXMODE	(1<<1)
#define RFRXMODE	(1)

/* REGISTERS */
#define MRF_RXMCR    0x00
#define MRF_PANIDL   0x01
#define MRF_PANIDH   0x02
#define MRF_SADRL    0x03
#define MRF_SADRH    0x04
#define MRF_RXFLUSH  0x0D
#define MRF_TXNCON   0x1B
#define MRF_PACON2   0x18
#define MRF_WAKECON  0x22
#define MRF_TXSTAT   0x24
#define MRF_TXBCON1  0x25
#define MRF_SOFTRST  0x2A
#define MRF_TXSTBL   0x2E
#define MRF_INTSTAT  0x31
#define MRF_INTCON   0x32
#define MRF_GPIO     0x33
#define MRF_TRISGPIO 0x34
#define MRF_SLPACK   0x35
#define MRF_RFCTL    0x36
#define MRF_BBREG1   0x39
#define MRF_BBREG2   0x3A
#define MRF_BBREG6   0x3E
#define MRF_CCAEDTH  0x3F
#define MRF_RFCON0   0x200
#define MRF_RFCON1   0x201
#define MRF_RFCON2   0x202
#define MRF_RFCON3   0x203
#define MRF_RFCON6   0x206
#define MRF_RFCON7   0x207
#define MRF_RFCON8   0x208
#define MRF_RFSTATE  0x20F
#define MRF_RSSI     0x210
#define MRF_SLPCON0  0x211
#define MRF_SLPCON1  0x220
#define MRF_TESTMODE 0x22F

#define MRF_I_RXIF  0b00001000
#define MRF_I_TXNIF 0b00000001

#define TXNRETRY1       7
#define TXNRETRY0       6
#define CCAFAIL         5
#define TXG2FNT         4
#define TXG1FNT         3
#define TXG2STAT        2
#define TXG1STAT        1
#define TXNSTAT         0

//#define RSSISAMPLES  50
#define MRF_DEFAULT_CHANNEL 11
#define MRF_LQI_START 110
#define MRF_LQI_THRESHOLD 90

#define MRF_MIN_CHANNEL 11
#define MRF_MAX_CHANNEL 26
#define MRF_PAN_ID_SIZE 2
#define MRF_DEST_ADDR_SIZE 2
#define MRF_SRC_ADDR_SIZE 2
#define MRF_ADDR_FIELDS_SIZE (MRF_PAN_ID_SIZE + MRF_DEST_ADDR_SIZE + MRF_SRC_ADDR_SIZE)
#define MRF_MHR_SIZE (2 + 1 + MRF_ADDR_FIELDS_SIZE)
#define MRF_MFR_SIZE 2
#define MRF_MAX_PSDU_SIZE 127
#define MRF_MAX_PAYLOAD_SIZE (MRF_MAX_PSDU_SIZE - MRF_MHR_SIZE - MRF_MFR_SIZE)
#define MRF_MAX_TX_FIFO_SIZE (1 + 1 + MRF_MHR_SIZE + MRF_MAX_PAYLOAD_SIZE)

#define MRF_MAX_RX_FIFO_SIZE (1 + MRF_MHR_SIZE + MRF_MAX_PAYLOAD_SIZE + 2 + 1 + 1)
#ifdef __cplusplus
class MRF
{
  public:

    //MRF(PinName mosi, PinName miso, PinName sck, PinName cs, PinName reset, PinName irq=NC, PinName wake=NC);
    MRF(SPI_HandleTypeDef* spiHandle);
    MRF();
    void begin(int channel, uint16_t panId, uint16_t address);
    void reset();
    void setPanId(uint16_t panId);
    // TODO read from the corresponding register?
    inline uint16_t getPanId() const {return panId;}
    void setAddress(uint16_t addr);
    void setCoordinator(bool coord);
    void setChannel(int channel);
    float measureSignalStrength();
    float getSignalStrength();
    int getLastLinkQuality();
    int getAverageLinkQuality() const;
    void sleep();
    void wakeup();
    void setPower(uint8_t percent);
    int getCleanChannel();
    bool isOnSpi(uint16_t panId);
    void startPacket();
    int bytesLeftToWrite();
    bool sendPacketToPan(uint16_t dstPan, uint16_t addr, bool ack = false);
    bool sendPacket(uint16_t addr, bool ack = false);
    bool transmissionDone();
    bool transmissionSuccess();
    void RXcopy(std::vector<uint8_t>& target);
    void clearISR();
    void enableRX(bool yes);
    void flushRX();
    bool receivePacketOnNoInt(std::vector<uint8_t>& target);
    int bytesLeftToRead();
    void clear();

    int readBytes(uint8_t* buf, int size);
    uint8_t readShort (uint8_t address);
    uint8_t read();
    bool write(uint8_t b);
    bool writeInt(int16_t i);

  private:

//    SPI mSpi;
//    DigitalOut mCs;
//    DigitalOut mReset;
//    DigitalIn mIrq;
//    DigitalOut mWake;
    SPI_HandleTypeDef* spiHandle;

    uint16_t panId;
    uint16_t srcAddr;
    int channel;
    uint8_t seqNumber;
    int txCount;
    uint8_t txBuf[MRF_MAX_PAYLOAD_SIZE];
    int rxCount;
    int rxSize;
    //uint8_t rxBuf[MRF_MAX_RX_FIFO_SIZE];
    uint8_t rssi;
    uint8_t lqi;
    int averageLqi;

    void init();
    void writeShort (uint8_t address, uint8_t data);
    uint8_t readLong (uint16_t address);
    void writeLong (uint16_t address, uint8_t data);
    bool writeToBuffer(void* data, int size);
    bool readFromBuffer(void* data, int size);

    //mbed
    void wait_ms(uint32_t s);
    //void wait_us(uint32_t s);


};

extern "C"
{
#endif // #ifdef __cplusplus

#ifdef __cplusplus
}
#endif

#endif
