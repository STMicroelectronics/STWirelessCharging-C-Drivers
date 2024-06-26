/**
  ******************************************************************************
  * @file    stwlc99.h
  * @brief   This file contains definitions for:
  *          - STWLC99 wireless charger from STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * This file is part of stwlc99-pid.
  *
  * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
  * Author(s): ACD (Analog Custom Devices) Software Team for STMicroelectronics.
  *
  * License terms: BSD 3-clause "New" or "Revised" License.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STWLC99_H_
#define STWLC99_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/** @addtogroup STWLC99
  * @{
  *
  */

/** @defgroup Driver_Infos
  * @brief    This section provide information related to this driver.
  * @{
  *
  */

/** Driver version **/
#define STWLC99_DRIVER_VER               "v1.0.0"

/**
  * @}
  *
  */

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of interface functions to be assigned
  *              by users.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

struct stwlc99_dev {
	/** Mandatory fields **/
	int32_t (*bus_write)(void *phandle, uint8_t *wbuf, int32_t wlen);
	int32_t (*bus_write_read)(void *phandle, uint8_t *wbuf, int32_t wlen, uint8_t *rbuf, int32_t rlen);
	void (*mdelay)(uint32_t millisec);
	void *(*alloc_mem)(size_t size);
	void (*free_mem)(void *ptr);

	/** Optional fields **/
	void (*log)(void *phandle, int32_t level, const char *msg, int32_t len);
	void *phandle;
	int32_t log_info;
};

/**
  * @}
  *
  */


/** @defgroup Chip_Infos
  * @brief    This section provide definitions related to STWLC99 chip.
  * @{
  *
  */

/** I2C Device Address 7 bit format **/
#define STWLC99_I2C_ADDR                                        0x2B

struct stwlc99_chip_info {
	/** Chip ID **/
	uint16_t chip_id;
	/** Chip Revision **/
	uint8_t chip_rev;
	/** Customer ID **/
	uint8_t cust_id;
	/** ROM ID **/
	uint16_t rom_id;
	/** Patch ID **/
	uint16_t patch_id;
	/** Config ID **/
	uint16_t cfg_id;
	/** PE ID **/
	uint8_t pe_id;
	/** PE ADC ID **/
	uint8_t pe_adc_id;
	/** System Errors **/
	uint32_t sys_err;
	/** HW Cut ID **/
	uint8_t cut_id;
};

enum stwlc99_fw_type {
	/** Patch and Config **/
	STWLC99_FW_PATCH_CFG = 0,
	/** Patch only **/
	STWLC99_FW_PATCH,
	/** Config only **/
	STWLC99_FW_CFG,
};

/**
  * @}
  *
  */

#ifndef u8
#define u8	uint8_t
#endif /* u8 */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */


/** @defgroup Error_Codes
  * @brief    This section provide error information returned by this driver.
  * @{
  *
  */

#define STWLC99_OK							                    0x0
#define STWLC99_ERR_BUS_W					                  0x80000000				
#define STWLC99_ERR_BUS_WR					                0x80000001                                       
#define STWLC99_ERR_ALLOC_MEM				                0x80000002
#define STWLC99_ERR_INVALID_PARAM			              0x80000003
#define STWLC99_ERR_TIMEOUT					                0x80000004
#define STWLC99_ERR_UNEXPECTED_OP_MODE		          0x80000005
#define STWLC99_ERR_UNEXPECTED_CHIP_ID		          0x80000006
#define STWLC99_ERR_NVM_ID_MISMATCH			            0x80000007
#define STWLC99_ERR_NVM_DATA_CORRUPTED		          0x80000008

/**
  * @}
  *
  */

/* FW register access */
int32_t stwlc99_read_fwreg(struct stwlc99_dev *dev, uint16_t reg, uint8_t *data, int32_t len);
int32_t stwlc99_write_fwreg(struct stwlc99_dev *dev, uint16_t reg, const uint8_t *data, int32_t len);

/* Common */
int32_t stwlc99_get_chip_info(struct stwlc99_dev *dev, struct stwlc99_chip_info *info);

/* NVM */
int32_t stwlc99_fw_update(struct stwlc99_dev *dev, enum stwlc99_fw_type fw_type, int32_t force_update);

/* Utility */
void stwlc99_log(struct stwlc99_dev *dev, int32_t level, const char *msg, ...);


/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* STWLC99_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
