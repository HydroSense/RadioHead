// RH_RF95.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF95.cpp,v 1.11 2016/04/04 01:40:12 mikem Exp $

#include <RH_RF95.h>



// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF95, allowing you to have
// 2 or more LORAs per Arduino
RH_RF95* RH_RF95::_deviceForInterrupt[RH_RF95_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF95::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const RH_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
{
  //  1d,     1e,      26
  { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
  { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
  { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
  { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096

  { 0x99,   0xc0,    0x04}, // Bw500 Sf4096 Implicit Headers, NO CRC, autoAGC
  { 0x98,   0xc4,    0x04}, // Bw500 Sf4096 Explicit Headers, YES CRC, autoAGC,
  { 0x98,   0xc4,    0x0c}, // Bw500 Sf4096 Explicit Headers, YES CRC, autoAGC, LOW DR OPTIMIZE
  { 0x98,   0xc0,    0x04}, // Bw500 Sf4096 Explicit Headers, NO CRC, autoAGC,
};

/* MHz
902.2746582 903.3239746 904.373291 905.4226074 906.4719238
907.5212402 908.5705566 909.619873 910.6691895 911.7185059
912.7678223 913.8171387 914.8664551 915.9157715 916.9650879
918.0144043 919.0637207 920.1130371 921.1623535 922.2116699
923.2609863 924.3103027 925.3596191 926.4089355 927.458252
*/
PROGMEM static const uint32_t FHSS_CHANNEL_TABLE[] =
{
  14782868, 14800060, 14817252, 14834444, 14851636,
  14868828, 14886020, 14903212, 14920404, 14937596,
  14954788, 14971980, 14989172, 15006364, 15023556,
  15040748, 15057940, 15075132, 15092324, 15109516,
  15126708, 15143900, 15161092, 15178284, 15195476
};
RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin,
  uint8_t fhssInterruptPin,
  RHGenericSPI& spi, void (*rxCallback)(void))
:
RHSPIDriver(slaveSelectPin, spi),
_rxBufValid(0)
{
  _rxCallback = rxCallback;
  _interruptPin = interruptPin;
  _fhssInterruptPin = fhssInterruptPin,
  _myInterruptIndex = 0xff; // Not allocated yet
  _useFhss = 0;
}

bool RH_RF95::init()
{
  if (!RHSPIDriver::init()){
    printf("spi driver failed to init.\n");
    return false;
  }

  // Determine the interrupt number that corresponds to the interruptPin
  int interruptNumber = digitalPinToInterrupt(_interruptPin);

  //printf("digitalPinToInterrupt(%d)==%d\n", _interruptPin, interruptNumber);
  if (interruptNumber == NOT_AN_INTERRUPT){
    printf("could not attach interrupt.\n");
    return false;
  }
  #ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
  interruptNumber = _interruptPin;
  #endif

  // No way to check the device type :-(

  // Set sleep mode, so we can also set LORA mode:
  spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
  delay(10); // Wait for sleep mode to take over from say, CAD
  // Check we are in sleep mode, with LORA set
  if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
  {

    printf("bad response to set OP_MODE [0x%02x], got 0x%02x, expected 0x%02x. Check the chip select is correct (using %d).\n",
    RH_RF95_REG_01_OP_MODE,
    spiRead(RH_RF95_REG_01_OP_MODE),
    RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE,
    _slaveSelectPin);
    //erial.println(spiRead(RH_RF95_REG_01_OP_MODE), HEX);
    return false; // No device present?
  }

  _chipver = spiRead(RH_RF95_REG_42_VERSION);

  // Serial.print("CHIPVER: ");
  // Serial.println(chipver);

  _perf.interrupt_count = 0;
  _perf.rx_timeout = 0;
  _perf.rx_crc_err = 0;
  _perf.cad_cnt = 0;
  // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
  // ARM M4 requires the below. else pin interrupt doesn't work properly.
  // On all other platforms, its innocuous, belt and braces
  pinMode(_interruptPin, INPUT);
  pinMode(_fhssInterruptPin, INPUT);

  // Set up interrupt handler
  // Since there are a limited number of interrupt glue functions isr*() available,
  // we can only support a limited number of devices simultaneously
  // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the
  // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
  // yourself based on knwledge of what Arduino board you are running on.
  if (_myInterruptIndex == 0xff)
  {
    // First run, no interrupt allocated yet
    if (_interruptCount <= RH_RF95_NUM_INTERRUPTS)
    _myInterruptIndex = _interruptCount++;
    else{
      printf("No more interrupts available in driver.\n");
      return false; // Too many devices, not enough interrupt vectors
    }
  }
  _deviceForInterrupt[_myInterruptIndex] = this;
  if (_myInterruptIndex == 0){
    attachInterrupt(interruptNumber, isr0, RISING);
    attachInterrupt(digitalPinToInterrupt(_fhssInterruptPin), fhss_isr0, RISING);
  } else if (_myInterruptIndex == 1) {
    attachInterrupt(interruptNumber, isr1, RISING);
    attachInterrupt(digitalPinToInterrupt(_fhssInterruptPin), fhss_isr1, RISING);
  } else if (_myInterruptIndex == 2) {
    attachInterrupt(interruptNumber, isr2, RISING);
    attachInterrupt(digitalPinToInterrupt(_fhssInterruptPin), fhss_isr2, RISING);
  } else
    return false; // Too many devices, not enough interrupt vectors

  // added by AMM, if the radio has a pending interrupt, we must clear it now
  uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
  // if (irq_flags > 0){
  //   printf("irq_flags: 0x%02x\n", irq_flags);
  // }
  spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

  // Set up FIFO
  // We configure so that we can use the entire 256 byte FIFO for either receive
  // or transmit, but not both at the same time
  spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
  spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

  // Packet format is preamble + explicit-header + payload + crc
  // Explicit Header Mode
  // payload is TO + FROM + ID + FLAGS + message data
  // RX mode is implmented with RXCONTINUOUS
  // max message data length is 255 - 4 = 251 octets

  // turn LNA gain up and LNA boost
  spiWrite(RH_RF95_REG_0C_LNA, 0x23); // G1 = maximum gain, LNA Boost 150% current

  // Set up default configuration
  // No Sync Words in LORA mode.
  setModemConfig(Bw125Cr45Sf128); // Radio default
  //    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
  setPreambleLength(8); // Default is 8
  // An innocuous ISM frequency, same as RF22's
  // leave radio at default frequency
  // setFrequency(434.0);
  // Lowish power
  // leave radio at default power
  // setTxPower(13);

  setModeIdle();

  return true;
}

// for FHSS, see 4.1.1.8 in the manual.
// read RhssPresentChannel to get the requested channel
// program the new channel and clear the ChangeChanelFhss by writing a 1
void RH_RF95::handleFhssInterrupt()
{


  setFhssChannel();

  // clear the RH_RF95_FHSS_CHANGE_CHANNEL interrupt
  spiWrite(RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_FHSS_CHANGE_CHANNEL);

}
// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void RH_RF95::handleInterrupt()
{
  _perf.interrupt = millis();
  _perf.interrupt_count ++;

  // Read the interrupt register
  uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
  if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
  {
    if (irq_flags & RH_RF95_RX_TIMEOUT){
      _perf.rx_timeout ++;
    }
    if (irq_flags & RH_RF95_PAYLOAD_CRC_ERROR){
      _perf.rx_crc_err ++;
    }
    _rxBad++;
  }
  else if (_mode == RHModeRx && irq_flags & RH_RF95_CAD_DONE){
    // CAD detected, just get the time and wait for RxDone
    _perf.cad_done = millis();
    _perf.cad_cnt++;
    spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
  }
  else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
  {
    _perf.rx_done= millis();



    // Have received a packet
    uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

    // Reset the fifo read ptr to the beginning of the packet
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
    spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
    _bufLen = len;
    _perf.recv_bytes = len;
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

    // Remember the RSSI of this packet
    // this is according to the doc, but is it really correct?
    // weakest receiveable signals are reported RSSI at about -66
    //_lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

    // the semtech datasheet pg 87 has a different value.
    _lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 157;


    // We have received a message.
    validateRxBuf();

    // stay in RX mode to get the next packet.
    //if (_rxBufValid)
    //  setModeIdle(); // Got one

    // make callback if requested, BEFORE actually reading the packet
    // this is to allow the user to set the next mode immediately
    // if another packet is expected.
    if (_rxCallback != NULL){
      _rxCallback();
    }
  }
  else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
  {
    _txGood++;
    setModeIdle();
  }
  else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
  {
    _perf.cad_done = millis();
    _perf.cad_cnt++;
    _cad = irq_flags & RH_RF95_CAD_DETECTED;
    if (_cad){
      //get the packet
      setModeRx();
    }else{
      setModeIdle();
      //keep checking.
      //setModeCAD();
    }
  }

  // don't clear FhssChangeChannel
  //spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

  // Clear all IRQ flags, except RH_RF95_FHSS_CHANGE_CHANNEL
  spiWrite(RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_RX_TIMEOUT |
      RH_RF95_RX_DONE | RH_RF95_PAYLOAD_CRC_ERROR | RH_RF95_VALID_HEADER |
      RH_RF95_TX_DONE | RH_RF95_CAD_DONE | RH_RF95_CAD_DETECTED);

}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF95.
// 3 interrupts allows us to have 3 different devices
void RH_RF95::isr0()
{
  if (_deviceForInterrupt[0])
  _deviceForInterrupt[0]->handleInterrupt();
}
void RH_RF95::isr1()
{
  if (_deviceForInterrupt[1])
  _deviceForInterrupt[1]->handleInterrupt();
}
void RH_RF95::isr2()
{
  if (_deviceForInterrupt[2])
  _deviceForInterrupt[2]->handleInterrupt();
}
void RH_RF95::fhss_isr0()
{
  if (_deviceForInterrupt[0])
  _deviceForInterrupt[0]->handleFhssInterrupt();
}
void RH_RF95::fhss_isr1()
{
  if (_deviceForInterrupt[1])
  _deviceForInterrupt[1]->handleFhssInterrupt();
}
void RH_RF95::fhss_isr2()
{
  if (_deviceForInterrupt[2])
  _deviceForInterrupt[2]->handleFhssInterrupt();
}
// Check whether the latest received message is complete and uncorrupted
void RH_RF95::validateRxBuf()
{

#ifdef RH_RF95_SEND_RH_HEADER
  if (_bufLen < 4)
    return; // Too short to be a real message

  // Extract the 4 headers
  _rxHeaderTo    = _buf[0];
  _rxHeaderFrom  = _buf[1];
  _rxHeaderId    = _buf[2];
  _rxHeaderFlags = _buf[3];
  if (_promiscuous ||
    _rxHeaderTo == _thisAddress ||
    _rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
      _rxGood++;
      _rxBufValid = true;
    }
#else
  _rxGood++;
  _rxBufValid = true;
#endif
  }

  bool RH_RF95::available()
  {
    if (_mode == RHModeTx){
      return false;
    }
    setModeRx();

    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
  }

  void RH_RF95::clearRxBuf()
  {
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
  }

  bool RH_RF95::recv(uint8_t* buf, uint8_t* len)
  {
    if (!available())
      return false;
    if (buf && len)
    {
      ATOMIC_BLOCK_START;

#ifdef RH_RF95_SEND_RH_HEADER
      // Skip the 4 headers that are at the beginning of the rxBuf
      if (*len > _bufLen-RH_RF95_HEADER_LEN)
        *len = _bufLen-RH_RF95_HEADER_LEN;
      memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
#else
      if (*len > _bufLen)
        *len = _bufLen;
      memcpy(buf, _buf, *len);
#endif

      ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
  }

  bool    RH_RF95::setFhssHoppingPeriod(uint8_t i){
    _useFhss = i > 0;
    spiWrite(RH_RF95_REG_24_HOP_PERIOD, i);
    return true;
  }

  uint16_t RH_RF95::configureFhss(uint16_t dwell){
    double bw = 0.0;
     // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
     switch( _bw )
     {
     case 7: // 125 kHz
         bw = 125;
         break;
     case 8: // 250 kHz
         bw = 250;
         break;
     case 9: // 500 kHz
         bw = 500;
         break;
     }
     // Symbol rate : time for one symbol (secs)
     double rs = bw / ( 1 << _sf );
     double ts = 1 / rs;

     uint32_t hp = floor(dwell / ts);

     // at high rates (small ts), we cannot get long dwell times due to the
     // 8 bit reg, so saturate.
     if (hp > 0xff)
      hp = 0xff;
     setFhssHoppingPeriod(hp);
    //  Serial.println("-------------------");
    //  Serial.println(ts);
    //  Serial.print("Fhss actual dwell time: ");
    //  Serial.print((int)(hp * ts));
    //  Serial.println("-------------------");
     return (uint16_t)(hp * ts);
  }

  uint32_t RH_RF95::getTimeOnAir(uint8_t pktLen){
    //from SEMTECH Firmware Driver and LoRaWAN Stack V4.1.0

    uint32_t airTime = 0;
    double bw = 0.0;
     // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
     switch( _bw )
     {
     case 7: // 125 kHz
         bw = 125;
         break;
     case 8: // 250 kHz
         bw = 250;
         break;
     case 9: // 500 kHz
         bw = 500;
         break;
     }

          //
          // Serial.println("-------------------");
          // Serial.print("pktLen: ");
          // Serial.println(pktLen);
          //
          // Serial.print("bw: ");
          // Serial.println(bw);


     // Symbol rate : time for one symbol (secs)
     double rs = bw / ( 1 << _sf );
     double ts = 1 / rs;
    //  Serial.println(_sf);
    //  Serial.println(rs);
    //  Serial.println(ts);
     //

     // time of preamble
     double tPreamble = ( _preamblelen + 4.25 ) * ts;
    //  Serial.print("Tpre: ");
    //  Serial.println(tPreamble);
    //  Serial.print("lowDR: ");
    //  Serial.println(_lowDR);
    //  Serial.print("cr: ");
    //  Serial.println(_cr);
    //  Serial.print("paylodCrc: ");
    //  Serial.println(_payloadCrc);
    //  Serial.print("fixedLen: ");
    //  Serial.println(_fixedLen);
     // Symbol length of payload and time
     double tmp = ceil( ( 8 * pktLen - 4 * _sf + 28 + 16 * _payloadCrc -
                          ( _fixedLen ? 20 : 0 ) ) /
                          ( double )( 4 * ( _sf - ( ( _lowDR > 0 ) ? 2 : 0 ) ) )
                        ) * ( _cr + 4 );
     double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
     double tPayload = nPayload * ts;
    //  Serial.print("nPay: ");
    //  Serial.println(nPayload);
    //
    //  Serial.print("Tpay: ");
    //  Serial.println(tPayload);
     // Time on air
     double tOnAir = tPreamble + tPayload;
     // return us secs
     airTime = floor( tOnAir + 0.999 );

          // Serial.println("-------------------");
     return airTime;
  }
  bool RH_RF95::writefifo(const uint8_t* data, uint8_t len){
    _perf.send_call = millis();
    if (len > RH_RF95_MAX_MESSAGE_LEN)
      return false;

    // must be in standby mode to write to fifo
    setModeIdle();
    // Position at the beginning of the FIFO
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);

#ifdef RH_RF95_SEND_RH_HEADER
    // The headers
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);
    _pref.sent_bytes = len+ RH_RF95_HEADER_LEN;
#else
    // The message data
    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
    _perf.sent_bytes = len;
#endif

    // from RH_RF95_MODE_FSTX, turnaround to TX is faster.
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_FSTX);
    return true;
  }

  bool RH_RF95::send(const uint8_t* data, uint8_t len)
  {
    _perf.send_call = millis();

    waitPacketSent(); // Make sure we dont interrupt an outgoing message

    if (writefifo(data, len)){
      setModeTx(); // Start the transmitter
      // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
      return true;
    }else{
      return false;
    }
  }

  bool RH_RF95::printRegisters()
  {
    #ifdef RH_HAVE_SERIAL
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
      Serial.print(registers[i], HEX);
      Serial.print(": ");
      Serial.println(spiRead(registers[i]), HEX);
    }
    #endif
    return true;
  }

  uint8_t RH_RF95::maxMessageLength()
  {
    return RH_RF95_MAX_MESSAGE_LEN;
  }

  bool RH_RF95::setFhssChannel()
  {
    // this is the channel the radio is requesting
    uint8_t i = spiRead(RH_RF95_REG_1C_HOP_CHANNEL) & RH_RF95_FHSS_PRESENT_CHANNEL;

    uint32_t frf = FHSS_CHANNEL_TABLE[i % RH_RF95_FHSS_CHANNELS];
    _freq = frf;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    // Serial.print("====FHSS ch is now ");
    // Serial.print(i % RH_RF95_FHSS_CHANNELS);
    // Serial.println("====");
  }
  bool RH_RF95::setFrequency(float centre)
  {
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    _freq = frf;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return true;
  }
  void RH_RF95::setModeIdle()
  {
    if (_mode != RHModeIdle)
    {
      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_STDBY);
      _mode = RHModeIdle;
    }
  }

  bool RH_RF95::sleep()
  {
    if (_mode != RHModeSleep)
    {
      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_SLEEP);
      _mode = RHModeSleep;
    }
    return true;
  }
  void RH_RF95::setModeCAD(){
    if (_mode != RHModeCad)
    {
      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_CAD);
      spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone (same as CadDetected)
      _mode = RHModeCad;
    }
  }
  void RH_RF95::setModeRx()
  {
    if (_mode != RHModeRx)
    {
      if (_useFhss)
        setFhssChannel();

      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_RXCONTINUOUS);
      //spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXSINGLE);
      spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone

      //spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Preamble detect
      _mode = RHModeRx;
    }
  }

  void RH_RF95::setModeTx()
  {
    if (_mode != RHModeTx)
    {
      _perf.tx_mode = millis();

      if (_useFhss)
        setFhssChannel();

      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_TX);
      spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
      _mode = RHModeTx;
    }
  }

  void RH_RF95::setTxPower(int8_t power)
  {
    // rewritten to always use PA_BOOST since RFO pin doesn't seem to be connected.

    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    // if (useRFO)
    // {
    //   if (power > 14)
    //     power = 14;
    //   if (power < -1)
    //     power = -1;
    //   spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    // }
    // else
    {
      if (power > 23)
        power = 23;
      if (power < 5)
        power = 5;

      // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
      // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
      // for 21, 22 and 23dBm
      // the Semtech drivers turn this on above 17 db
      // http://www.semtech.com/apps/filedown/down.php?file=LoRaMac-node-master.zip
      if (power > 17)
      {
        spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_RESERVED | RH_RF95_PA_DAC_ENABLE);
        power -= 3;

        //turn up OCP to 130mA (semtech data sheet says current could be up to 125mA)
        spiWrite(RH_RF95_REG_0B_OCP, RH_RF95_OCP_ON | 16);
      }
      else
      {
        spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_RESERVED | RH_RF95_PA_DAC_DISABLE);

        //turn up OCP to default seeting of 100 mA
        spiWrite(RH_RF95_REG_0B_OCP, RH_RF95_OCP_ON | 11);
      }

      // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
      // pin is connected, so must use PA_BOOST
      // Pout = 2 + OutputPower.
      // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
      // but OutputPower claims it would be 17dBm.
      // My measurements show 20dBm is correct

      // power is in the range [5, ..., 20]
      // output power is 17 - (15-power), so 7 .. 23 dBm
      spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    }
  }

  // Sets registers from a canned modem configuration structure
  void RH_RF95::setModemRegisters(const ModemConfig* config)
  {
    setModeIdle(); // standby radio mode before reconfiguring radio

    _bw = config->reg_1d >> 4;
    _cr = (config->reg_1d >> 1) & 0x7;
    _sf = (config->reg_1e) >> 4;
    _payloadCrc = 0x1 & (config->reg_1e >> 2);
    _fixedLen = 0x1 & config->reg_1d;
    _lowDR = 0x1 & (config->reg_26 >> 3);
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(RH_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);

    if (_chipver == 0x12){
      // errata http://www.semtech.com/images/datasheet/SX1276_77_8_ErrataNote_1_1.pdf

      // 2.1 and 2.3, assume 900 MHz operation.
      if (_bw == 9){ // 500 KHz
        spiWrite(RH_RF95_REG_36_RESERVED, 0x02);
        spiWrite(RH_RF95_REG_3a_RESERVED, 0x64);
        spiWrite(RH_RF95_REG_31_RESERVED, 0x80 | spiRead(RH_RF95_REG_31_RESERVED));

      }else{  // not 500 KHz
        spiWrite(RH_RF95_REG_36_RESERVED, 0x03);
        spiWrite(RH_RF95_REG_3a_RESERVED, 0x65);
        spiWrite(RH_RF95_REG_31_RESERVED, 0x7F & spiRead(RH_RF95_REG_31_RESERVED));
        spiWrite(RH_RF95_REG_2F_RESERVED, 0x40);
        spiWrite(RH_RF95_REG_30_RESERVED, 0x00);
      }
    }

  }

  // Set one of the canned FSK Modem configs
  // Returns true if its a valid choice
  bool RH_RF95::setModemConfig(ModemConfigChoice index)
  {
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
      return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return true;
  }

  void RH_RF95::setEncoding(int sf, int cr){
    RH_RF95::setEncoding(sf, cr, true);
  }

  void RH_RF95::setEncoding(int sf, int cr, bool crc){
    sf &= 0xF;
    cr &= 0x7; // remove all the uncessary bits
    ModemConfig cfg;
    // set the cfg
    cfg.reg_1d = 0x90 | (cr << 1);
    cfg.reg_1e = crc?( (sf << 4) | 0x04) : (sf << 4);
    cfg.reg_26 = 0x04;
    memcpy_P(&cfg, &cfg, sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

  }

  void RH_RF95::setPayloadLength(uint8_t len) {
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
  }
  void RH_RF95::setPreambleLength(uint16_t bytes)
  {
    //the radio adds 4 symbols, the minimum is 6+4 = 10 symbols in LoRa mode
    if (bytes < 6)
      bytes = 6;
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
    _preamblelen = bytes;
  }

  bool RH_RF95::isChannelActive()
  {
    // Set mode RHModeCad
    if (_mode != RHModeCad)
    {
      spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE | RH_RF95_MODE_CAD);
      spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
      _mode = RHModeCad;
    }

    while (_mode == RHModeCad)
    YIELD;

    return _cad;
  }
