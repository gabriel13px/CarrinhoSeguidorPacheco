#include "BMI160Raw.h"

namespace {
const uint8_t REG_CHIP_ID = 0x00, REG_DATA_GYR = 0x0C, REG_ACC_CONF = 0x40, REG_ACC_RANGE = 0x41,
              REG_GYR_CONF = 0x42, REG_GYR_RANGE = 0x43, REG_CMD = 0x7E;
const uint8_t CHIP_ID = 0xD1;
const uint8_t CMD_SOFT_RESET = 0xB6, CMD_ACC_NORMAL = 0x11, CMD_GYR_NORMAL = 0x15;
const uint8_t ACC_CONF_1600HZ_NORMAL = 0x2C, ACC_RANGE_4G = 0x05;
const uint8_t GYR_CONF_1600HZ_NORMAL = 0x2C, GYR_RANGE_2000DPS = 0x00;
}  // namespace

bool BMI160Raw::write8(uint8_t reg, uint8_t val) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  w_->write(val);
  return w_->endTransmission() == 0;
}

bool BMI160Raw::read8(uint8_t reg, uint8_t* val) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  if (w_->endTransmission(false) != 0) return false;
  if (w_->requestFrom((int)addr_, 1) != 1) return false;
  *val = w_->read();
  return true;
}

bool BMI160Raw::begin(TwoWire& wire, uint8_t addr) {
  w_ = &wire; addr_ = addr; ok_ = false;
  uint8_t id = 0;
  if (!read8(REG_CHIP_ID, &id) || id != CHIP_ID) return false;
  write8(REG_CMD, CMD_SOFT_RESET); delay(20);
  write8(REG_CMD, CMD_ACC_NORMAL); delay(5);
  write8(REG_CMD, CMD_GYR_NORMAL); delay(85);
  if (!write8(REG_ACC_CONF, ACC_CONF_1600HZ_NORMAL)) return false;
  if (!write8(REG_ACC_RANGE, ACC_RANGE_4G)) return false;
  if (!write8(REG_GYR_CONF, GYR_CONF_1600HZ_NORMAL)) return false;
  if (!write8(REG_GYR_RANGE, GYR_RANGE_2000DPS)) return false;
  delay(10);
  ok_ = true;
  return true;
}

bool BMI160Raw::readRaw(int16_t* gyro, int16_t* acc) {
  w_->beginTransmission(addr_);
  w_->write(REG_DATA_GYR);
  if (w_->endTransmission(false) != 0) return false;
  if (w_->requestFrom((int)addr_, 12) != 12) return false;
  uint8_t b[12];
  for (int i = 0; i < 12; i++) b[i] = w_->read();
  for (int i = 0; i < 3; i++) {
    gyro[i] = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    acc[i] = (int16_t)((uint16_t)b[6 + 2 * i] | ((uint16_t)b[7 + 2 * i] << 8));
  }
  return true;
}

bool BMI160Raw::read(float* gyro_rad_s, float* acc_m_s2) {
  int16_t g[3], a[3];
  if (!readRaw(g, a)) return false;
  const float dps2rad = 3.14159265f / 180.0f;
  for (int i = 0; i < 3; i++) {
    gyro_rad_s[i] = (float)g[i] / GYRO_LSB_PER_DPS * dps2rad;
    acc_m_s2[i] = (float)a[i] / ACC_LSB_PER_G * 9.80665f;
  }
  return true;
}
