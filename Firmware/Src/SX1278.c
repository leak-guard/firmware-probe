#include "SX1278.h"

void Timer_Start(Timer_t *timer, uint32_t delay) {
  timer->start = HAL_GetTick();
  timer->delay = delay;
}

bool Timer_Expired(Timer_t *timer) {
  return (HAL_GetTick() - timer->start) >= timer->delay;
}

void SX1278_hw_init(SX1278_pins_t *hw)
{
  SX1278_SetNSS(hw, 1);
  HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
}

void SX1278_SetNSS(SX1278_pins_t *hw, int value)
{
  HAL_GPIO_WritePin(hw->nss.port, hw->nss.pin, (value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SX1278_Reset(SX1278_pins_t *hw)
{
  SX1278_SetNSS(hw, 1);
  HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_RESET);

  SX1278_DelayMs(1);

  HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);

  SX1278_DelayMs(100);
}

void SX1278_SPICommand(SX1278_pins_t *hw, uint8_t cmd)
{
  SX1278_SetNSS(hw, 0);
  HAL_SPI_Transmit(hw->spi, &cmd, 1, 1000);
  while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
    ;
}

uint8_t SX1278_SPIReadByte(SX1278_pins_t *hw)
{
  uint8_t txByte = 0x00;
  uint8_t rxByte = 0x00;

  SX1278_SetNSS(hw, 0);
  HAL_SPI_TransmitReceive(hw->spi, &txByte, &rxByte, 1, 1000);
  while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
    ;
  return rxByte;
}

void SX1278_DelayMs(uint32_t msec)
{
  HAL_Delay(msec);
}

int SX1278_GetDIO0(SX1278_pins_t *hw)
{
  return (HAL_GPIO_ReadPin(hw->dio0.port, hw->dio0.pin) == GPIO_PIN_SET);
}

uint8_t SX1278_SPIRead(SX1278_t *module, uint8_t addr)
{
  uint8_t tmp;
  SX1278_SPICommand(module->hw, addr);
  tmp = SX1278_SPIReadByte(module->hw);
  SX1278_SetNSS(module->hw, 1);

  return tmp;
}

void SX1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t cmd)
{
  SX1278_SetNSS(module->hw, 0);
  SX1278_SPICommand(module->hw, addr | 0x80);
  SX1278_SPICommand(module->hw, cmd);
  SX1278_SetNSS(module->hw, 1);
}

void SX1278_SPIBurstRead(SX1278_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length)
{
  uint8_t i;
  if (length <= 1)
  {
    return;
  }
  else
  {
    SX1278_SetNSS(module->hw, 0);
    SX1278_SPICommand(module->hw, addr);
    for (i = 0; i < length; i++)
    {
      *(rxBuf + i) = SX1278_SPIReadByte(module->hw);
    }
    SX1278_SetNSS(module->hw, 1);
  }
}

void SX1278_SPIBurstWrite(SX1278_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length)
{
  unsigned char i;
  if (length <= 1)
  {
    return;
  }
  else
  {
    SX1278_SetNSS(module->hw, 0);
    SX1278_SPICommand(module->hw, addr | 0x80);
    for (i = 0; i < length; i++)
    {
      SX1278_SPICommand(module->hw, *(txBuf + i));
    }
    SX1278_SetNSS(module->hw, 1);
  }
}

void SX1278_config(SX1278_t *module)
{
  SX1278_sleep(module); // Change modem mode Must in Sleep mode
  SX1278_DelayMs(15);

  SX1278_SPIWrite(module, LR_RegOpMode, 0x88);
  // SX1278_SPIWrite(module, 0x5904); //?? Change digital regulator from 1.6V to 1.47V: see errata note

  uint64_t freq = (SX1278_Frequency[module->frequency] << 19) / 32000000;
  uint8_t freq_reg[3];
  freq_reg[0] = (uint8_t)(freq >> 16);
  freq_reg[1] = (uint8_t)(freq >> 8);
  freq_reg[2] = (uint8_t)(freq >> 0);
  SX1278_SPIBurstWrite(module, LR_RegFrMsb, (uint8_t *)freq_reg, 3); // setting  frequency parameter

  SX1278_SPIWrite(module, RegSyncWord, module->sync_word);

  // setting base parameter
  SX1278_SPIWrite(module, LR_RegPaConfig, SX1278_Power[module->power]); // Setting output power parameter

  SX1278_SPIWrite(module, LR_RegOcp, 0x0B);        // RegOcp,Close Ocp
  SX1278_SPIWrite(module, LR_RegLna, 0x23);        // RegLNA,High & LNA Enable
  if (SX1278_SpreadFactor[module->LoRa_SF] == 6)
  { // SFactor=6
    uint8_t tmp;
    SX1278_SPIWrite(module, LR_RegModemConfig1,
                    ((SX1278_LoRaBandwidth[module->LoRa_BW] << 4) +
                     (SX1278_CodingRate[module->LoRa_CR] << 1) +
                     0x01)); // Implicit Enable CRC Enable(0x02) & Error Coding
                             // rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

    SX1278_SPIWrite(module, LR_RegModemConfig2,
                    ((SX1278_SpreadFactor[module->LoRa_SF] << 4) +
                     (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

    tmp = SX1278_SPIRead(module, 0x31);
    tmp &= 0xF8;
    tmp |= 0x05;
    SX1278_SPIWrite(module, 0x31, tmp);
    SX1278_SPIWrite(module, 0x37, 0x0C);
  }
  else
  {
    SX1278_SPIWrite(module, LR_RegModemConfig1,
                    ((SX1278_LoRaBandwidth[module->LoRa_BW] << 4) +
                    (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x00));
                    // Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

    SX1278_SPIWrite(module, LR_RegModemConfig2,
                    ((SX1278_SpreadFactor[module->LoRa_SF] << 4) +
                    (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00));
                    // Spreading Factor & LNA gain set by the internal AGC loop
  }

  SX1278_SPIWrite(module, LR_RegModemConfig3, 0x04);
  SX1278_SPIWrite(module, LR_RegSymbTimeoutLsb, 0x08); // RegSymbTimeoutLsb Timeout = 0x3FF(Max)
  SX1278_SPIWrite(module, LR_RegPreambleMsb, 0x00); // RegPreambleMsb
  SX1278_SPIWrite(module, LR_RegPreambleLsb, 8); // RegPreambleLsb 8+4=12byte Preamble
  SX1278_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); // RegDioMapping2 DIO5=00, DIO4=01
  module->readBytes = 0;
  SX1278_standby(module); // Entry standby mode
}

void SX1278_standby(SX1278_t *module)
{
  SX1278_SPIWrite(module, LR_RegOpMode, 0x09);
  module->status = STANDBY;
}

void SX1278_sleep(SX1278_t *module)
{
  SX1278_SPIWrite(module, LR_RegOpMode, 0x08);
  module->status = SLEEP;
}

void SX1278_clearLoRaIrq(SX1278_t *module)
{
  SX1278_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}

int SX1278_LoRaEntryRx(SX1278_t *module, uint8_t length, Timer_t *timeout) {
  static LoraState_t state = LORA_STATE_IDLE;

  switch (state) {
    case LORA_STATE_IDLE:
      module->packetLength = length;
      SX1278_config(module);
      SX1278_SPIWrite(module, REG_LR_PADAC, 0x84); // Normal and RX
      SX1278_SPIWrite(module, LR_RegHopPeriod, 0xFF);
      SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01);
      SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F);
      SX1278_clearLoRaIrq(module);
      SX1278_SPIWrite(module, LR_RegPayloadLength, length);
      uint8_t addr = SX1278_SPIRead(module, LR_RegFifoRxBaseAddr);
      SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr);
      SX1278_SPIWrite(module, LR_RegOpMode, 0x8D);
      Timer_Start(timeout, 2000); // Initialize the timer
      state = LORA_STATE_PREPARE_RX;
      break;

    case LORA_STATE_PREPARE_RX:
      if ((SX1278_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {
        module->status = RX;
        state = LORA_STATE_IDLE;
        return 1; // Success
      }

      if (Timer_Expired(timeout)) {
        SX1278_Reset(module->hw);
        SX1278_config(module);
        state = LORA_STATE_IDLE;
        return 0; // Timeout
      }
      break;

    default:
      state = LORA_STATE_IDLE;
      break;
  }

  return -1; // Still processing
}

uint8_t SX1278_LoRaRxPacket(SX1278_t *module)
{
  unsigned char addr;
  unsigned char packet_size;

  if (SX1278_GetDIO0(module->hw))
  {
    memset(module->rxBuffer, 0x00, SX1278_MAX_PACKET);

    addr = SX1278_SPIRead(module, LR_RegFifoRxCurrentaddr); // last packet addr
    SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); // RxBaseAddr -> FiFoAddrPtr

    if (module->LoRa_SF == SX1278_LORA_SF_6)
    {
      // When SpreadFactor is six, will be used Implicitly
      // Header mode(Excluding internal packet length)
      packet_size = module->packetLength;
    }
    else
    {
      packet_size = SX1278_SPIRead(module, LR_RegRxNbBytes); // Number for received bytes
    }

    SX1278_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size);
    module->readBytes = packet_size;
    SX1278_clearLoRaIrq(module);
  }
  return module->readBytes;
}

int SX1278_LoRaEntryTx(SX1278_t *module, uint8_t length, Timer_t *timeout) {
  uint8_t temp;

  module->packetLength = length;

  SX1278_config(module);
  SX1278_SPIWrite(module, REG_LR_PADAC, 0x87);
  SX1278_SPIWrite(module, LR_RegHopPeriod, 0x00);
  SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41);  // DIO0=01, DIO1=00, DIO2=00, DIO3=01
  SX1278_clearLoRaIrq(module);
  SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7);  // Open TxDone interrupt
  SX1278_SPIWrite(module, LR_RegPayloadLength, length);

  uint8_t addr = SX1278_SPIRead(module, LR_RegFifoTxBaseAddr);
  SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr);

  temp = SX1278_SPIRead(module, LR_RegPayloadLength);
  if (temp == length) {
    module->status = TX;
    return 1;
  }

  if (Timer_Expired(timeout)) {
    SX1278_Reset(module->hw);
    SX1278_config(module);
    return 0;
  }

  return -1;
}

int SX1278_LoRaTxPacket(SX1278_t *module, uint8_t *txBuffer, uint8_t length, Timer_t *timeout) {
  static bool dataWritten = false;

  if (!dataWritten) {
    SX1278_SPIBurstWrite(module, 0x00, txBuffer, length);
    SX1278_SPIWrite(module, LR_RegOpMode, 0x8b);  // Tx Mode
    dataWritten = true;
    return -1;
  }

  if (SX1278_GetDIO0(module->hw)) {
    SX1278_SPIRead(module, LR_RegIrqFlags);
    SX1278_clearLoRaIrq(module);
    SX1278_standby(module);
    dataWritten = false;
    return 1;
  }

  if (Timer_Expired(timeout)) {
    SX1278_Reset(module->hw);
    SX1278_config(module);
    dataWritten = false;
    return 0;
  }

  return -1;
}

void SX1278_init(SX1278_t *module, uint64_t frequency, uint8_t power,
                 uint8_t LoRa_SF, uint8_t LoRa_BW, uint8_t LoRa_CR,
                 uint8_t LoRa_CRC_sum, uint8_t packetLength,
                 uint8_t sync_word)
{
  SX1278_hw_init(module->hw);

  module->frequency = frequency;
  module->power = power;
  module->LoRa_SF = LoRa_SF;
  module->LoRa_BW = LoRa_BW;
  module->LoRa_CR = LoRa_CR;
  module->LoRa_CRC_sum = LoRa_CRC_sum;
  module->packetLength = packetLength;
  module->sync_word = sync_word;

  SX1278_config(module);
}

int SX1278_transmit(SX1278_t *module, uint8_t *txBuf, uint8_t length, Timer_t *timeout) {
  Timer_t txEntryTimeout;
  Timer_t txPacketTimeout;

  Timer_Start(&txEntryTimeout, timeout->delay);

  if (SX1278_LoRaEntryTx(module, length, &txEntryTimeout)) {
    Timer_Start(&txPacketTimeout, timeout->delay);

    return SX1278_LoRaTxPacket(module, txBuf, length, &txPacketTimeout);
  }

  return 0;
}

int SX1278_receive(SX1278_t *module, uint8_t length, uint32_t timeout) {
  Timer_t rxTimeout;

  Timer_Start(&rxTimeout, timeout);

  return SX1278_LoRaEntryRx(module, length, &rxTimeout);
}

uint8_t SX1278_available(SX1278_t *module)
{
  return SX1278_LoRaRxPacket(module);
}

uint8_t SX1278_read(SX1278_t *module, uint8_t *rxBuf, uint8_t length)
{
  if (length != module->readBytes)
  {
    length = module->readBytes;
  }
  memcpy(rxBuf, module->rxBuffer, length);
  rxBuf[length] = '\0';
  module->readBytes = 0;
  return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t *module)
{
  uint32_t temp = 10;
  temp = SX1278_SPIRead(module, LR_RegRssiValue);      // Read RegRssiValue, Rssi value
  temp = temp + 127 - 137;                                  // 127:Max RSSI, 137:RSSI offset
  return (uint8_t)temp;
}

uint8_t SX1278_RSSI(SX1278_t *module)
{
  uint8_t temp;
  temp = SX1278_SPIRead(module, RegRssiValue);
  temp = 127 - (temp >> 1);       // 127:Max RSSI
  return temp;
}
