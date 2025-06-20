#ifndef ENERGY_METERING_IC_H
#define ENERGY_METERING_IC_H
#include <Arduino.h>
#include <SPI.h>

#define WRITE 0 // Write SPI
#define READ 1  // Read SPI

// Essential Registers
#define MeterEn 0x00      // Metering Enable
#define MMode0 0x33       // Metering Mode Config
#define MMode1 0x34       // PGA Gain Configuration
#define SoftReset 0x70    // Software Reset
#define EMMState0 0x71    // System Status 0
#define EMMState1 0x72    // System Status 1
#define CfgRegAccEn 0x7F  // Register Access Enable
#define UrmsA 0xD9        // A RMS Voltage
#define UrmsB 0xDA        // B RMS Voltage
#define UrmsC 0xDB        // C RMS Voltage
#define IrmsA 0xDD        // A RMS Current
#define IrmsB 0xDE        // B RMS Current
#define IrmsC 0xDF        // C RMS Current
#define PmeanT 0xB0       // Total (all-phase-sum) Mean Active Power (High Byte)
#define PmeanTLSB 0xC0    // Total Mean Active Power (LSB)
#define PFmeanT 0xBC      // Total Power Factor
#define Freq 0xF8         // Frequency
#define Temp 0xFC         // Temperature
#define TempCtrl 0x216    // Temperature Control
#define TempCal 0x219     // Temperature Calibration
#define RegLock 0x2FF     // Register Lock/Unlock
#define UgainA 0x29       // Voltage Gain A
#define UgainB 0x2A       // Voltage Gain B
#define UgainC 0x2B       // Voltage Gain C
#define IgainA 0x2C       // Current Gain A
#define IgainB 0x2D       // Current Gain B
#define IgainC 0x2E       // Current Gain C

class EnergyMeteringIC {
private:
  unsigned short readWriteRegister(unsigned char rw, unsigned short address, unsigned short val);
  int _chipSelectPin;
  unsigned short _lineFrequency;
  unsigned short _pgaGain;
  unsigned short _voltGainA;
  unsigned short _voltGainB;
  unsigned short _voltGainC;
  unsigned short _currGainA;
  unsigned short _currGainB;
  unsigned short _currGainC;

  int read32BitRegister(unsigned short highAddr, unsigned short lowAddr);

public:
  EnergyMeteringIC();
  ~EnergyMeteringIC();

  void initialize(int pin, unsigned short lineFreq, unsigned short pgaGain,
                  unsigned short voltGainA, unsigned short voltGainB, unsigned short voltGainC,
                  unsigned short currGainA, unsigned short currGainB, unsigned short currGainC);

  double getVoltageA();
  double getVoltageB();
  double getVoltageC();
  double getCurrentA();
  double getCurrentB();
  double getCurrentC();
  double getTotalActivePower();
  double getTotalPowerFactor();
  double getFrequency();
  double getTemperature();
  unsigned short getSysStatus0();
  unsigned short getSysStatus1();
  unsigned short getConfigMode0();
  unsigned short getConfigMode1();
};

#endif