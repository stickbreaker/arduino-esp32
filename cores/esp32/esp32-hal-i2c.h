// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// modified Nov 2017 by Chuck Todd <StickBreaker> to support Interrupt Driven I/O

#ifndef _ESP32_HAL_I2C_H_
#define _ESP32_HAL_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// External Wire.h equivalent error Codes
typedef enum {
    I2C_ERROR_OK=0,
    I2C_ERROR_DEV,      // hardware fault, interrupt allocation error, configuration error, pin State error, i2c==NULL
    I2C_ERROR_ACK,      // Slave device did not acknowledge
    I2C_ERROR_TIMEOUT,  // SCL stretching exceeded time out, Lock Acquire time out 
    I2C_ERROR_BUS,      // Arbitration failure
    I2C_ERROR_BUSY,     // bus is inuse by other Master
    I2C_ERROR_MEMORY,   // Heap allocation  error, Semaphore creation error
    I2C_ERROR_CONTINUE, // ReSTART transaction queued, will not execute until STOP issued.
    I2C_ERROR_NO_BEGIN  // Not used by hal-i2c, just TwoWire()
} i2c_err_t;

struct i2c_struct_t;
typedef struct i2c_struct_t i2c_t;

struct i2c_queue_block_t;
typedef struct i2c_queue_block_t i2c_queue_t;

i2c_err_t i2cInit(int8_t sda, int8_t scl, uint32_t clk_speed, i2c_queue_t ** Qb, i2c_t ** I2c);
void i2cRelease(i2c_t *i2c, i2c_queue_t ** inQb); // free ISR, Free DQ, Power off peripheral clock.  Must call i2cInit() to recover
i2c_err_t i2cWrite(i2c_t * i2c, i2c_queue_t * inQb, uint16_t address, uint8_t* buff, uint16_t size, bool sendStop, uint16_t timeOutMillis);
i2c_err_t i2cRead(i2c_t * i2c, i2c_queue_t * inQb, uint16_t address, uint8_t* buff, uint16_t size, bool sendStop, uint16_t timeOutMillis, uint32_t *readCount);
i2c_err_t i2cFlush(i2c_t *i2c, i2c_queue_t * inQb );
i2c_err_t i2cSetFrequency(i2c_queue_t * inQb, uint32_t clk_speed);
uint32_t i2cGetFrequency(i2c_queue_t * inQb);
uint32_t i2cGetStatus(i2c_t * i2c); // Status register of peripheral
i2c_err_t i2cGetLock(i2c_t * i2c, i2c_queue_t * inQb, uint16_t timeOutMillis, uint32_t * count);
i2c_err_t i2cReleaseLock(i2c_t * i2c, i2c_queue_t * inQb, uint32_t * count);

//stickbreaker debug support
uint32_t i2cDebug(i2c_queue_t * inQb, uint32_t setBits, uint32_t resetBits);
/*  Debug actions have 3 currently defined locus 
 0xXX------ : at entry of ProcQueue 
 0x--XX---- : at exit of ProcQueue
 0x------XX : at entry of Flush
 
 bit 0 causes DumpI2c to execute 
 bit 1 causes DumpInts to execute
 bit 2 causes DumpCmdqueue to execute
 bit 3 causes DumpStatus to execute
 bit 4 causes DumpFifo to execute

 To enable debug:
  * at line 45 of esp32-hal-i2c.c, ENABLE_I2C_DEBUG_BUFFER must be defined. 
  * Core Level Debug must be >= INFO

 */
#ifdef __cplusplus
}
#endif

#endif /* _ESP32_HAL_I2C_H_ */