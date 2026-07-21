#pragma once

// Hardware context: compile-time pins, physical transport objects and ISR-owned
// FIFO interrupt counters. This file intentionally owns only board/hardware
// singletons, not runtime policy or command wiring.

#include <Arduino.h>
#include <SPI.h>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "sensor/qmc6309.hpp"

using namespace tracker;

// Hardware constants. Keep pins compile-time for now.
static constexpr int PIN_LSM_SCK  = cfg::PIN_LSM_SCK;
static constexpr int PIN_LSM_MISO = cfg::PIN_LSM_MISO;
static constexpr int PIN_LSM_MOSI = cfg::PIN_LSM_MOSI;
static constexpr int PIN_LSM_CS   = cfg::PIN_LSM_CS;
static constexpr int PIN_LSM_INT1 = cfg::PIN_LSM_INT1;

static constexpr uint32_t SERIAL_BAUD_DEFAULT = cfg::SERIAL_BAUD;
static constexpr uint32_t SPI_HZ_DEFAULT = cfg::SPI_HZ;
static constexpr uint8_t SPI_MODE_DEFAULT = cfg::SPI_MODE;

static constexpr size_t FIFO_RAW_BUFFER_CAPACITY = cfg::FIFO_RAW_BUFFER_CAPACITY;
static constexpr size_t MAG_RAW_BUFFER_CAPACITY = cfg::MAG_RAW_BUFFER_CAPACITY;
static constexpr size_t FIFO_RUNTIME_RAW_QUEUE_CAPACITY = cfg::FIFO_RUNTIME_RAW_QUEUE_CAPACITY;
static constexpr size_t FIFO_RUNTIME_MAG_QUEUE_CAPACITY = cfg::FIFO_RUNTIME_MAG_QUEUE_CAPACITY;
static constexpr float MAG_HUB_PERIOD_US = cfg::MAG_HUB_PERIOD_US;
static constexpr uint16_t FIFO_MAX_WORDS_PER_DRAIN_DEFAULT = cfg::FIFO_MAX_WORDS_PER_DRAIN;
static constexpr uint8_t MAX_DRAIN_ROUNDS_PER_EVENT_DEFAULT = cfg::FIFO_MAX_DRAIN_ROUNDS_PER_EVENT;
static constexpr uint32_t FIFO_WAIT_TIMEOUT_MS = cfg::FIFO_WAIT_TIMEOUT_MS;
// Non-blocking runtime loop calls consumeFifoInterruptEvent(0) very often.
// Keep the fallback FIFO_STATUS SPI poll as a rare safety net only;
// normal runtime data flow should be driven by INT1.
static constexpr uint32_t FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US = cfg::FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = cfg::HEARTBEAT_PERIOD_MS;

static ArduinoLsm6dsvSpiTransport lsmBus(SPI, PIN_LSM_CS, SPI_HZ_DEFAULT, SPI_MODE_DEFAULT);
static Lsm6dsv lsm(lsmBus);
static Lsm6dsvFifoReader lsmFifo(lsmBus, lsm);
static Lsm6dsvSensorHub lsmHub(lsmBus);
static Qmc6309 qmc(lsmHub);

static Lsm6dsv::RawSample g_fifoRaw[FIFO_RAW_BUFFER_CAPACITY];
static Lsm6dsvFifoReader::MagRawSample g_magRaw[MAG_RAW_BUFFER_CAPACITY];
static Lsm6dsv::RawSample g_fifoRuntimeRawQueue[FIFO_RUNTIME_RAW_QUEUE_CAPACITY];
static uint8_t g_fifoRuntimeRawQueueFlags[FIFO_RUNTIME_RAW_QUEUE_CAPACITY];
static Lsm6dsvFifoReader::MagRawSample g_fifoRuntimeMagQueue[FIFO_RUNTIME_MAG_QUEUE_CAPACITY];

static volatile uint32_t g_fifoIntCount = 0;
static volatile uint32_t g_fifoLastIrqUs = 0;

static void IRAM_ATTR onFifoInt1() {
    g_fifoLastIrqUs = micros();
    g_fifoIntCount++;
}
