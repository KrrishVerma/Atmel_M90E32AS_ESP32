#include "EnergyMeteringIC.h"

EnergyMeteringIC::EnergyMeteringIC() {}

EnergyMeteringIC::~EnergyMeteringIC() {}

unsigned short EnergyMeteringIC::readWriteRegister(unsigned char rw, unsigned short address, unsigned short val) {
  unsigned char* data = (unsigned char*)&val;
  unsigned char* adata = (unsigned char*)&address;
  unsigned short output;
  unsigned short address1;
  int retry = 0;
  unsigned char response[2]; // Declared at function scope

#if defined(ESP32)
  SPISettings settings(100000, MSBFIRST, SPI_MODE3); // 100 kHz clock for stability
#endif

  output = (val >> 8) | (val << 8);
  val = output;

  address |= rw << 15;
  address1 = (address >> 8) | (address << 8);
  address = address1;

#if defined(ESP32)
  SPI.beginTransaction(settings);
#endif

  do {
    Serial.println("Attempting SPI read/write for address 0x" + String(address, HEX) + ", retry " + String(retry + 1));
    digitalWrite(_chipSelectPin, LOW);
    delayMicroseconds(200); // CS low delay

    for (byte i = 0; i < 2; i++) {
      byte sent = *adata;
      byte received = SPI.transfer(sent);
      Serial.print("Sent byte "); Serial.print(i); Serial.print(": 0x"); Serial.print(sent, HEX);
      Serial.print(", Received: 0x"); Serial.println(received, HEX);
      adata++;
    }

    delayMicroseconds(20); // Inter-byte delay

    if (rw) {
      for (byte i = 0; i < 2; i++) {
        response[i] = SPI.transfer(0x00);
        *data = response[i];
        data++;
        Serial.print("Read byte "); Serial.print(i); Serial.print(": 0x"); Serial.println(response[i], HEX);
      }
    } else {
      for (byte i = 0; i < 2; i++) {
        SPI.transfer(*data);
        data++;
      }
    }

    digitalWrite(_chipSelectPin, HIGH);
    delayMicroseconds(200); // CS high delay
#if defined(ESP32)
    SPI.endTransaction();
#endif

    output = (val >> 8) | (val << 8);
    Serial.print("Raw response: 0x");
    Serial.print(response[0], HEX);
    Serial.println(response[1], HEX);
    if (rw && (response[0] == 0xFF && response[1] == 0xFF) && retry < 2) {
      Serial.println("SPI read error: 0xFFFF detected, retrying...");
      retry++;
      delay(10); // Retry delay
    } else {
      break;
    }
  } while (retry < 3);

  if (rw && (response[0] == 0xFF && response[1] == 0xFF)) {
    Serial.println("SPI read failed after 3 retries for address 0x" + String(address, HEX));
  }
  return output;
}

int EnergyMeteringIC::read32BitRegister(unsigned short highAddr, unsigned short lowAddr) {
  int val, val_h, val_l;
  val_h = readWriteRegister(READ, highAddr, 0xFFFF);
  val_l = readWriteRegister(READ, lowAddr, 0xFFFF);
  val = val_h << 16;
  val |= val_l;
  if ((val & 0x80000000) != 0) {
    val = (~val) + 1; // Handle signed 32-bit value
  }
  return val;
}

void EnergyMeteringIC::initialize(int pin, unsigned short lineFreq, unsigned short pgaGain,
                                unsigned short voltGainA, unsigned short voltGainB, unsigned short voltGainC,
                                unsigned short currGainA, unsigned short currGainB, unsigned short currGainC) {
  _chipSelectPin = pin;
  _lineFrequency = lineFreq;
  _pgaGain = pgaGain;
  _voltGainA = voltGainA;
  _voltGainB = voltGainB;
  _voltGainC = voltGainC;
  _currGainA = currGainA;
  _currGainB = currGainB;
  _currGainC = currGainC;

  pinMode(_chipSelectPin, OUTPUT);
  SPI.begin();

  Serial.println("Performing soft reset...");
  readWriteRegister(WRITE, SoftReset, 0x789A); // Soft reset
  delayMicroseconds(500);
  Serial.println("Enabling register access...");
  readWriteRegister(WRITE, CfgRegAccEn, 0x55AA); // Enable register access
  delayMicroseconds(500);
  Serial.println("Enabling metering...");
  readWriteRegister(WRITE, MeterEn, 0x0001); // Enable metering
  delayMicroseconds(500);
  Serial.println("Setting frequency config...");
  readWriteRegister(WRITE, MMode0, _lineFrequency); // Frequency config (0x0185 for 3P3W, 50Hz)
  delayMicroseconds(500);
  Serial.println("Setting PGA gain...");
  readWriteRegister(WRITE, MMode1, _pgaGain); // PGA gain
  delayMicroseconds(500);
  Serial.println("Setting voltage gains...");
  readWriteRegister(WRITE, UgainA, _voltGainA); // Voltage A gain
  delayMicroseconds(500);
  readWriteRegister(WRITE, UgainB, _voltGainB); // Voltage B gain
  delayMicroseconds(500);
  readWriteRegister(WRITE, UgainC, _voltGainC); // Voltage C gain
  delayMicroseconds(500);
  Serial.println("Setting current gains...");
  readWriteRegister(WRITE, IgainA, _currGainA); // Current A gain
  delayMicroseconds(500);
  readWriteRegister(WRITE, IgainB, _currGainB); // Current B gain
  delayMicroseconds(500);
  readWriteRegister(WRITE, IgainC, _currGainC); // Current C gain
  delayMicroseconds(500);
  // Temperature sensor configuration
  Serial.println("Configuring temperature sensor...");
  readWriteRegister(WRITE, RegLock, 0x55AA); // Unlock
  delayMicroseconds(500);
  readWriteRegister(WRITE, TempCtrl, 0x5183); // Temp control
  delayMicroseconds(500);
  readWriteRegister(WRITE, TempCal, 0x01C1); // Temp calibration
  delayMicroseconds(500);
  readWriteRegister(WRITE, RegLock, 0x0000); // Lock
  delayMicroseconds(500);
  readWriteRegister(WRITE, CfgRegAccEn, 0x0000); // End configuration
  delayMicroseconds(500);
}

double EnergyMeteringIC::getVoltageA() {
  unsigned short voltage = readWriteRegister(READ, UrmsA, 0xFFFF);
  Serial.println("Raw UrmsA: 0x" + String(voltage, HEX));
  return (double)voltage / 63; // Adjusted scaling factor
}

double EnergyMeteringIC::getVoltageB() {
  unsigned short voltage = readWriteRegister(READ, UrmsB, 0xFFFF);
  Serial.println("Raw UrmsB: 0x" + String(voltage, HEX));
  return (double)voltage / 63;
}

double EnergyMeteringIC::getVoltageC() {
  unsigned short voltage = readWriteRegister(READ, UrmsC, 0xFFFF);
  Serial.println("Raw UrmsC: 0x" + String(voltage, HEX));
  return (double)voltage / 63;
}

double EnergyMeteringIC::getCurrentA() {
  unsigned short current = readWriteRegister(READ, IrmsA, 0xFFFF);
  Serial.println("Raw IrmsA: 0x" + String(current, HEX));
  return (double)current / 1000;
}

double EnergyMeteringIC::getCurrentB() {
  unsigned short current = readWriteRegister(READ, IrmsB, 0xFFFF);
  Serial.println("Raw IrmsB: 0x" + String(current, HEX));
  return (double)current / 1000;
}

double EnergyMeteringIC::getCurrentC() {
  unsigned short current = readWriteRegister(READ, IrmsC, 0xFFFF);
  Serial.println("Raw IrmsC: 0x" + String(current, HEX));
  return (double)current / 1000;
}

double EnergyMeteringIC::getTotalActivePower() {
  int val = read32BitRegister(PmeanT, PmeanTLSB);
  Serial.println("Raw PmeanT: 0x" + String(val, HEX));
  return (double)val * 0.00032;
}

double EnergyMeteringIC::getTotalPowerFactor() {
  signed short pf = (signed short)readWriteRegister(READ, PFmeanT, 0xFFFF);
  Serial.println("Raw PFmeanT: 0x" + String(pf, HEX));
  if (pf & 0x8000) {
    pf = (~pf) + 1;
  }
  return (double)pf / 1000;
}

double EnergyMeteringIC::getFrequency() {
  unsigned short freq = readWriteRegister(READ, Freq, 0xFFFF);
  Serial.println("Raw Freq: 0x" + String(freq, HEX));
  return (double)freq / 100;
}

double EnergyMeteringIC::getTemperature() {
  signed short temp = (signed short)readWriteRegister(READ, Temp, 0xFFFF);
  Serial.println("Raw Temp: 0x" + String(temp, HEX));
  return (double)temp / 1.6; // Adjusted for ~25C
}

unsigned short EnergyMeteringIC::getSysStatus0() {
  unsigned short status = readWriteRegister(READ, EMMState0, 0xFFFF);
  Serial.println("Raw EMMState0: 0x" + String(status, HEX));
  return status;
}

unsigned short EnergyMeteringIC::getSysStatus1() {
  unsigned short status = readWriteRegister(READ, EMMState1, 0xFFFF);
  Serial.println("Raw EMMState1: 0x" + String(status, HEX));
  return status;
}

unsigned short EnergyMeteringIC::getConfigMode0() {
  unsigned short config = readWriteRegister(READ, MMode0, 0xFFFF);
  Serial.println("Raw MMode0: 0x" + String(config, HEX));
  return config;
}

unsigned short EnergyMeteringIC::getConfigMode1() {
  unsigned short config = readWriteRegister(READ, MMode1, 0xFFFF);
  Serial.println("Raw MMode1: 0x" + String(config, HEX));
  return config;
}
