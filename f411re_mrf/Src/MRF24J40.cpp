
#include "MRF24J40.h"
#include <cstring>
#include "cmsis_os.h"
#include "logger.h"
#include "stm32f4xx_hal.h"


//extern SPI_HandleTypeDef hspi2;
MRF::MRF():
  spiHandle(NULL)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, GPIO_PIN_SET);
}

MRF::MRF(SPI_HandleTypeDef* hspi):
  spiHandle(hspi)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, GPIO_PIN_SET);
}


//// PRIVATE ////////

uint8_t MRF::readShort (uint8_t address)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

  uint8_t outBytes[1];// = {address};
  uint8_t rxdata[1];
  outBytes[0] = (address<<1) & 0x7E;
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spiHandle, outBytes, sizeof(outBytes), HAL_TIMEOUT);

  outBytes[0] = 0;//0xFF;
  status = HAL_SPI_TransmitReceive(spiHandle, outBytes, rxdata, sizeof(rxdata), HAL_TIMEOUT);

  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

  if( status == HAL_OK )
  {
    return rxdata[0];
  }
  return 0u;
}

void MRF::writeShort (uint8_t address, uint8_t data)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

  uint8_t outBytes[1];// = {address};
  outBytes[0] = ((address<<1) & 0x7E) | 0x01;
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spiHandle, outBytes, sizeof(outBytes), HAL_TIMEOUT);

  outBytes[0] = data;
  status = HAL_SPI_Transmit(spiHandle, outBytes, sizeof(outBytes), HAL_TIMEOUT);

  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

  if( status == HAL_OK )
  {

  }
}

uint8_t MRF::readLong (uint16_t address)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

  uint16_t outShorts[1];
  outShorts[0] = (address>>3) | 0x80;
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spiHandle, (uint8_t*)outShorts, sizeof(outShorts), HAL_TIMEOUT);

  outShorts[0] = (address<<5) & 0xE0;
  status = HAL_SPI_Transmit(spiHandle, (uint8_t*)outShorts, sizeof(outShorts), HAL_TIMEOUT);

  uint8_t dataBytes[] =  {0};//{0xFF};
  uint8_t rxdata[1];
  status = HAL_SPI_TransmitReceive(spiHandle, dataBytes, rxdata, sizeof(rxdata), HAL_TIMEOUT);
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

  if( status == HAL_OK )
  {
    return rxdata[0];
  }
  return 0u;
}

void MRF::writeLong (uint16_t address, uint8_t data)
{
  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

  uint16_t outShort[1];// = {address};
  outShort[0] = (address>>3) | 0x80;
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spiHandle, (uint8_t*)outShort, sizeof(outShort), HAL_TIMEOUT);

  outShort[0] = ((address<<5) & 0xE0) | 0x10;
  status = HAL_SPI_Transmit(spiHandle, (uint8_t*)outShort, sizeof(outShort), HAL_TIMEOUT);

  uint8_t outBytes[] = {data};
  status = HAL_SPI_Transmit(spiHandle, outBytes, sizeof(outBytes), HAL_TIMEOUT);

  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

  if( status == HAL_OK )
  {

  }
}


bool MRF::writeToBuffer(void* data, int size) {
  if (size > bytesLeftToWrite()) {
    return false;
  }

  memcpy(&txBuf[txCount], data, size);
  txCount += size;
  return true;
}

bool MRF::readFromBuffer(void* data, int size) {
  if (size > bytesLeftToRead()) {
    return false;
  }

  memcpy(data, &rxBuf[rxCount], size);
  rxCount += size;
  return true;
}

//// PUBLIC //////////

void MRF::reset() {
  writeShort(MRF_SOFTRST, 0b00000111); // Perform full software reset (RSTPWR = 1, RSTBB = 1, RSTMAC = 1)
}

void MRF::setPanId(uint16_t panId) {
  this->panId = panId;
  writeShort(MRF_PANIDH, panId >> 8);
  writeShort(MRF_PANIDL, panId);
}

void MRF::setAddress(uint16_t addr) {
  this->srcAddr = addr;
  writeShort(MRF_SADRH, addr >> 8);
  writeShort(MRF_SADRL, addr);
}

void MRF::setChannel(int newChannel) {
  if (newChannel < 11 || newChannel > 26) {
    return;
  }

  this->channel = newChannel;
  // Change channel
  writeLong(MRF_RFCON0, (this->channel - 11) << 4 | 0x03);

  // Perform RF state machine reset and wait for RF circuitry to calibrate
  writeShort(MRF_RFCTL, 0b00000100);
  writeShort(MRF_RFCTL, 0);

  // reset link quality measurement
  averageLqi = MRF_LQI_START;

  //delayMicroseconds(200); // arduino
  //wait_us(200); // mbed
  // TODO:
  wait_ms(1); // wait 1 millisec; good for now...
}

void MRF::init() {
	reset();
  // MRF24J40 module configuration
  writeShort(MRF_PACON2, 0b10011000);    // Setup recommended PA/LNA control timing (TXONTS = 0x6)
  writeShort(MRF_TXSTBL, 0b10010101);    // Setup recommended PA/LNA control timing (RFSTBL = 0x9)
  writeLong(MRF_RFCON0, 0b00000011);     // Set recommended value for RF Optimize Control (RFOPT = 0x3)
  writeLong(MRF_RFCON1, 0b00000010);     // Set recommended value for VCO Optimize Control (VCOOPT = 0x2)
  writeLong(MRF_RFCON2, 0b10000000);     // Enable PLL (PLLEN = 1)
  writeLong(MRF_RFCON6, 0b10010000);     // Set recommended value for TX Filter Control and 20MHz Clock Recovery Control
                                         //  (TXFIL = 1, 20MRECVR = 1)
  writeLong(MRF_RFCON7, 0b10000000);     // Use 100kHz internal oscillator for Sleep Clock (SLPCLKSEL = 0x2)
  writeLong(MRF_RFCON8, 0b00010000);     // Set recommended value for VCO Control (RFVCO = 1)
  writeLong(MRF_SLPCON0, 0b00000001);    // Disable the sleep clock to save power (/SLPCLKEN = 1)
  writeLong(MRF_SLPCON1, 0b00100001);    // Disable CLKOUT pin and set Sleep Clock Divisor to minimum of 0x01 for the
                                         //  100kHz internal oscillator (/CLOUKTEN = 1, SLPCLKDIV = 0x01)
  writeShort(MRF_BBREG2, 0b10111000);    // Use CCA Mode 1 - Energy above threshold - and set CCA Carrier Sense
                                         //  Threshold to recommended value (CCAMODE = 0x2, CCACSTH = 0xE)
  writeShort(MRF_CCAEDTH, 0b01100000);   // Set energy detection threshold to recommended value (CCAEDTH = 0x60)
  writeShort(MRF_BBREG6, 0b01000000);    // Calculate RSSI for each packet received (RSSIMODE2 = 1)

  writeShort(MRF_INTCON, 0b11110111);    // Enable RX FIFO reception interrupt (RXIE = 0)
  //writeShort(MRF_INTCON, 0b11110110);     // enabel rx/tx irq

  setPanId(panId);
  setAddress(this->srcAddr);
  setChannel(this->channel);                        // Set default channel (must keep 11) and reset state machine

  // wait 2 millis
  //delay(2); // arduino
  wait_ms(2);
}

void MRF::begin(int channel, uint16_t panId, uint16_t address) {
  this->channel = channel;
  this->panId = panId;
  this->srcAddr = address;
  init();
}

void MRF::setPower(uint8_t percent){
  if ( percent > 100 ) percent = 100;
  if ( percent <= 0 ) percent = 0;
  uint8_t power = 31 - (31/100)*percent;
	writeLong(MRF_RFCON3, power << 3);
}

float MRF::getSignalStrength() {
  return rssi * 0.197948 - 87.646977;
}

int MRF::getLastLinkQuality() {
  return lqi;
}

int MRF::getAverageLinkQuality() const{
  return this->averageLqi;
}

float MRF::measureSignalStrength() {

  writeShort(MRF_BBREG6, 0b10000000);
  /*
  doc quote:
  If RSSIMODE1 = 1, then
  1 = RSSI calculation has finished and the RSSI value is ready
  0 = RSSI calculation in progress

  so we might want to check If RSSIMODE1 = 1 as well
  (readShort(MRF_BBREG6) & 0b10000000)
  */

  // Wait until conversion is ready and read RSSI
  while (!(readShort(MRF_BBREG6) & 0b00000001));
  rssi = readLong(MRF_RSSI);

  return getSignalStrength();
}

int MRF::getCleanChannel(){
	int clean = 0;
    float currentSignal = 0.0f;
    float prevSignal = 1.0f;

	for (int i = 0; i < 16; i++) {
    setChannel(i + 11);
    // this is not really needed and this might be the cause of dongle "freeze"
    // the mrf chip performs measurements based on symbol numbers configured in MRF_TXBCON1
    // for (int j = 0; j < RSSISAMPLES; j++) {
    //     currentSignal += measureSignalStrength();
    // }
    // currentSignal /= RSSISAMPLES;
    currentSignal = measureSignalStrength();

	  if (currentSignal < prevSignal) {
		  clean = i;
	  }

	  prevSignal = currentSignal;
  }

	return clean+11;
}

void MRF::sleep() {

	/*  RXFLUSH 0x0D
		------------
		bit 7 - Reserved: Maintain as '0'

		bit 6 - WAKEPOL: Wake Signal Polarity bit
			1 = Wake signal polarity is active-high
			0 = Wake signal polarity is active-low (default)

		bit 5 - WAKEPAD: Wake I/O Pin Enable bit 1 = Enable wake I/O pin
			0 = Disable wake I/O pin (default)

		bit 4 - Reserved: Maintain as '0'

		bit 3 - CMDONLY: Command Frame Receive bit
			1 = Only command frames are received, all other frames are filtered out
			0 = All valid frames are received (default)

		bit 2 - DATAONLY: Data Frame Receive bit
			1 = Only data frames are received, all other frames are filtered out
			0 = All valid frames are received (default)

		bit 1 - BCNONLY: Beacon Frame Receive bit
			1 = Only beacon frames are received, all other frames are filtered out
			0 = All valid frames are received (default)

		bit 0 - RXFLUSH: Reset Receive FIFO Address Pointer bit
			1 = Resets the RXFIFO Address Pointer to zero. RXFIFO data is not modified. Bit is automatically cleared to '0' by hardware.
	*/

  writeShort(MRF_SOFTRST, 0b00000100); // Perform Power Management Reset (RSTPWR = 1)
  writeShort(MRF_WAKECON, 0b10000000); // Enable Immediate Wake-up Mode (IMMWAKE = 1)
  writeShort(MRF_SLPACK, 0b10000000);  // Put the module to sleep (SLPACK = 1)
}

void MRF::wakeup() {
	reset(); // Perform software reset
	init();  // Reinitialize all registers
}

bool MRF::isOnSpi(uint16_t panId){
  this->panId = panId;
  uint8_t panidl = readShort(MRF_PANIDL);
  uint8_t panidh = readShort(MRF_PANIDH);
  uint16_t check = panidl | panidh << 8;
  if (panId == check){ return true; }
  else { return false; }
}

void MRF::startPacket() {
  txCount = 0;
}

int MRF::bytesLeftToWrite() {
  return MRF_MAX_PAYLOAD_SIZE - txCount;
}

bool MRF::sendPacketToPan(uint16_t dstPan, uint16_t addr, bool ack) {
  if (txCount == 0) {
    return false;
  }

  // Send header size, frame size and header
  int i = 0;
  writeLong(i++, MRF_MHR_SIZE);                  // Header size
  writeLong(i++, MRF_MHR_SIZE + txCount);        // Frame size
  writeLong(i++, ack ? 0b01100001 : 0b01000001); // Frame control bits 0-7 (data frame, no security, no pending data, ack disabled, intra-PAN)
  writeLong(i++, 0b10001000);                    // Frame control bits 8-15 (16-bit addresses with PAN)
  writeLong(i++, seqNumber++);                   // Sequence number
  writeLong(i++, dstPan);                         // PAN ID
  writeLong(i++, dstPan >> 8);
  writeLong(i++, addr);                          // Destination address
  writeLong(i++, addr >> 8);
  writeLong(i++, srcAddr);                       // Source address
  writeLong(i++, srcAddr >> 8);

  // Send data payload
  for (int j = 0; j < txCount; j++) {
    writeLong(i++, txBuf[j]);
  }

  // Start transmission (0x37 0x01)
  writeShort(MRF_TXNCON, ack ? 0b00000101 : 0b00000001);

  return true;
}

bool MRF::sendPacket(uint16_t addr, bool ack) {
  return sendPacketToPan(panId, addr, ack);
}

bool MRF::transmissionDone() {
  return readShort(MRF_INTSTAT) & 0b00000001;
}

bool MRF::transmissionSuccess() {
  return !(readShort(MRF_TXSTAT) & 0b00000001);
}

void MRF::RXcopy(std::vector<uint8_t>& target){

  // Packet received, get the number of bytes
    int frameSize = readLong(0x300);
    // Copy the message bytes into the user buffer
    rxSize = 0;
    rxCount = 0;
    while (rxSize < (frameSize - MRF_MHR_SIZE - MRF_MFR_SIZE)) {
      target.push_back(readLong(0x301 + MRF_MHR_SIZE + rxSize));
      rxSize++;
    }

    // Read RSSI and LQI
    lqi = readLong(0x301 + frameSize);
    averageLqi = (averageLqi + lqi) / 2;
    rssi = readLong(0x301 + frameSize + 1);
}

void MRF::clearISR(){
	readShort(MRF_INTSTAT); // clear isr
}

void MRF::enableRX(bool yes){
	if (!yes) writeShort(MRF_BBREG1, 0b00000100);	// disable receiver
	else	  writeShort(MRF_BBREG1, 0b00000000);	// enable receiver
}

void MRF::flushRX(){
	writeShort(MRF_RXFLUSH, 0b00000001); // flush rx hw fifo
}

bool MRF::receivePacketOnNoInt(std::vector<uint8_t>& target) {
  int frameSize;

  // Check RXIF in INTSTAT
  if (readShort(MRF_INTSTAT) & 0b00001000) {
    // Disable receiver
    enableRX(false);

    // Packet received, get the number of bytes
    frameSize = readLong(0x300);

    // Copy the message bytes into the user buffer
    rxSize = 0;
    rxCount = 0;
    while (rxSize < (frameSize - MRF_MHR_SIZE - MRF_MFR_SIZE)) {
      target.push_back(readLong(0x301 + MRF_MHR_SIZE + rxSize));
      rxSize++;
    }

    // Read RSSI and LQI
    lqi = readLong(0x301 + frameSize);
    averageLqi = (averageLqi + lqi) / 2;
    rssi = readLong(0x301 + frameSize + 1);

    // Flush the reception buffer, re-enable receiver
    flushRX();
    enableRX(true);

    // Wait until RXIF is cleared (takes a while)
    while(readShort(MRF_INTSTAT) & 0b00001000);

    return true;
  }

  return false;
}

int MRF::bytesLeftToRead() {
  return rxSize - rxCount;
}

int MRF::readBytes(uint8_t* buf, int size) {
  if (size > bytesLeftToRead()) {
    size = bytesLeftToRead();
  }

  readFromBuffer(buf, size);
  return size;
}

void MRF::clear(){
	uint8_t old = readShort(MRF_RFCTL);
	writeShort(MRF_RFCTL, old | RFRST);
	writeShort(MRF_RFCTL, old & ~RFRST);
	//delay(2); // arduino, millisecs
	wait_ms(2);
}

uint8_t MRF::read() {
  uint8_t result;
  return readFromBuffer(&result, 1) ? result : 0;
}

bool MRF::write(uint8_t b) {
  return writeToBuffer(&b, 1);
}

bool MRF::writeInt(int16_t i) {
  return writeToBuffer(&i, 2);
}

void MRF::wait_ms(uint32_t s)
{
  HAL_Delay(s);
}

// TODO:
//void MRF::wait_us(uint32_t s)
//{
//  /*HAL_Delay(s);*/
//}
