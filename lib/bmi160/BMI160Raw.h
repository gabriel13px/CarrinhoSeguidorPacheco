// BMI160Raw — driver mínimo do BMI160 por I2C (Wire), sem dependência de biblioteca externa.
// Configura giroscópio ±2000 dps e acelerômetro ±4 g a 1600 Hz e lê os 6 eixos num burst de 12 bytes.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class BMI160Raw {
 public:
  // addr: 0x68 (SDO em GND) ou 0x69 (SDO em VDDIO). Retorna false se o CHIP_ID não for 0xD1.
  bool begin(TwoWire& wire, uint8_t addr = 0x69);
  // Lê gx,gy,gz (rad/s) e ax,ay,az (m/s²). Retorna false em erro de I2C.
  bool read(float* gyro_rad_s, float* acc_m_s2);
  bool readRaw(int16_t* gyro, int16_t* acc);
  bool ok() const { return ok_; }

 private:
  bool write8(uint8_t reg, uint8_t val);
  bool read8(uint8_t reg, uint8_t* val);
  TwoWire* w_ = nullptr;
  uint8_t addr_ = 0x69;
  bool ok_ = false;
  static constexpr float GYRO_LSB_PER_DPS = 16.4f;   // ±2000 dps
  static constexpr float ACC_LSB_PER_G = 8192.0f;    // ±4 g
};
