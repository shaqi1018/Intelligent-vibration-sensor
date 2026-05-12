/**
  ******************************************************************************
  * @file    sensor_bin.c
  * @brief   传感器二进制帧 CRC32 实现
  ******************************************************************************
  */

#include "sensor_bin.h"

#include <stddef.h>

/* 软件 CRC32（IEEE 802.3 / zlib 风格） */
uint32_t SensorBin_CRC32(const void *data, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint32_t b;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)p[i];
    for (b = 0U; b < 8U; b++)
    {
      uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }

  return crc ^ 0xFFFFFFFFUL;
}

void SensorBin_FillCRC(SensorBinFrame_t *frame)
{
  if (frame == NULL)
  {
    return;
  }
  frame->crc32 = SensorBin_CRC32(frame, SENSOR_BIN_FRAME_BYTES - 4U);
}
