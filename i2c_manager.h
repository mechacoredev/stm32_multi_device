/*
 * I2C_manager.h
 *
 *  Created on: Jul 15, 2026
 *      Author: Enes
 */

#ifndef INC_I2C_MANAGER_H_
#define INC_I2C_MANAGER_H_

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_dma.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"

typedef enum{
	i2c_manager_ok = 0,
	i2c_manager_fail,
	i2c_manager_timeout,
	i2c_manager_busy,
	i2c_manager_size_0,
	i2c_manager_device_not_found,
	i2c_manager_dma_busy,
}i2c_manager_status_t;

#define I2C_MANAGER_TIMEOUT_MS 100

i2c_manager_status_t i2c_manager_read_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);
i2c_manager_status_t i2c_manager_write_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* txdata, uint16_t size);
i2c_manager_status_t i2c_manager_find_device(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t device_id);
bool i2c_manager_is_dma_busy(void);
void i2c_manager_init(void);
i2c_manager_status_t i2c_manager_read_dma(DMA_TypeDef* dma_handle, uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);
void i2c_manager_error_handler(I2C_TypeDef* i2c_handle);
void i2c_manager_dma_rx_handler(void);
void i2c_manager_event_handler(I2C_TypeDef* i2c_handle);

#endif /* INC_I2C_MANAGER_H_ */
