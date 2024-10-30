// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/SPI.h"

#include "HALInitializer.h"
#include "mockdata/SPIDataInternal.h"
#include "hal/handles/IndexedHandleResource.h"
#include "HALInternal.h"
#include <fmt/format.h>
using namespace hal;

namespace {
struct SPI {
  HAL_SPIPort port;
};
}  // namespace

static constexpr int32_t kSpiMaxHandles = 5;

typedef IndexedHandleResource<HAL_SPIHandle, SPI, kSpiMaxHandles,
                              HAL_HandleEnum::SPI>
    SPIHandleResource;

static SPIHandleResource* spiHandles;

namespace hal::init {
void InitializeSPI() {
  static SPIHandleResource sH;
  spiHandles = &sH;
}
}  // namespace hal::init

extern "C" {

HAL_SPIHandle HAL_InitializeSPI(HAL_SPIPort port, int32_t* status) {
  hal::init::CheckInit();
  if (port < 0 || port >= kSpiMaxHandles) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(
        status, fmt::format("SPI port must be between 0 and {}. Requested {}",
                            kSpiMaxHandles, static_cast<int>(port)));
    return HAL_kInvalidHandle;
  }

  HAL_SPIHandle hal_handle;
  auto spi =
      spiHandles->Allocate(static_cast<int16_t>(port), &hal_handle, status);

  if (*status != 0) {
    return HAL_kInvalidHandle;
  }

  SimSPIData[port].initialized = true;
  spi->port = port;
  return hal_handle;
}

int32_t HAL_TransactionSPI(HAL_SPIHandle handle, const uint8_t* dataToSend,
                           uint8_t* dataReceived, int32_t size) {
  auto spi = spiHandles->Get(handle);
  if (!spi) {
    return -1;
  }
  return SimSPIData[spi->port].Transaction(dataToSend, dataReceived, size);
}

int32_t HAL_WriteSPI(HAL_SPIHandle handle, const uint8_t* dataToSend,
                     int32_t sendSize) {
  auto spi = spiHandles->Get(handle);
  if (!spi) {
    return -1;
  }
  return SimSPIData[spi->port].Write(dataToSend, sendSize);
}
int32_t HAL_ReadSPI(HAL_SPIHandle handle, uint8_t* buffer, int32_t count) {
  auto spi = spiHandles->Get(handle);
  if (!spi) {
    return -1;
  }
  return SimSPIData[spi->port].Read(buffer, count);
}
void HAL_CloseSPI(HAL_SPIHandle handle) {
  auto spi = spiHandles->Get(handle);
  if (!spi) {
    return;
  }
  SimSPIData[spi->port].initialized = false;
  spiHandles->Free(handle);
}
void HAL_SetSPISpeed(HAL_SPIHandle handle, int32_t speed) {}
void HAL_SetSPIMode(HAL_SPIHandle handle, HAL_SPIMode mode) {}
HAL_SPIMode HAL_GetSPIMode(HAL_SPIHandle port) {
  return HAL_SPI_kMode0;
}
void HAL_SetSPIChipSelectActiveHigh(HAL_SPIHandle handle, int32_t* status) {}
void HAL_SetSPIChipSelectActiveLow(HAL_SPIHandle handle, int32_t* status) {}

void HAL_InitSPIAuto(HAL_SPIHandle handle, int32_t bufferSize,
                     int32_t* status) {}
void HAL_FreeSPIAuto(HAL_SPIHandle handle, int32_t* status) {}
void HAL_StartSPIAutoRate(HAL_SPIHandle handle, double period,
                          int32_t* status) {}
void HAL_StartSPIAutoTrigger(HAL_SPIHandle handle,
                             HAL_Handle digitalSourceHandle,
                             HAL_AnalogTriggerType analogTriggerType,
                             HAL_Bool triggerRising, HAL_Bool triggerFalling,
                             int32_t* status) {}
void HAL_StopSPIAuto(HAL_SPIHandle handle, int32_t* status) {}
void HAL_SetSPIAutoTransmitData(HAL_SPIHandle handle, const uint8_t* dataToSend,
                                int32_t dataSize, int32_t zeroSize,
                                int32_t* status) {}
void HAL_ForceSPIAutoRead(HAL_SPIHandle handle, int32_t* status) {}
int32_t HAL_ReadSPIAutoReceivedData(HAL_SPIHandle handle, uint32_t* buffer,
                                    int32_t numToRead, double timeout,
                                    int32_t* status) {
  auto spi = spiHandles->Get(handle);
  if (!spi) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  return SimSPIData[spi->port].ReadAutoReceivedData(buffer, numToRead, timeout,
                                                    status);
}
int32_t HAL_GetSPIAutoDroppedCount(HAL_SPIHandle handle, int32_t* status) {
  return 0;
}

void HAL_ConfigureSPIAutoStall(HAL_SPIHandle handle, int32_t csToSclkTicks,
                               int32_t stallTicks, int32_t pow2BytesPerRead,
                               int32_t* status) {}

}  // extern "C"
