/*
 * I2C_manager.c
 *
 *  Created on: Jul 15, 2026
 *      Author: Enes
 */

#include "I2C_manager.h"

extern osMutexId_t I2C_mutexHandle;
extern osSemaphoreId_t DMA_complete_semaphoreHandle;

typedef enum{
	DMA_STATE_IDLE = 0,
	DMA_STATE_START_SENT,
	DMA_STATE_WRITE_ADDR_SENT ,
	DMA_STATE_REG_ADDR_SENT,
	DMA_STATE_RESTART_SENT,
	DMA_STATE_READ_ADDR_SENT,
	DMA_STATE_READING,
	DMA_STATE_TRANSFER_ERROR,
	DMA_STATE_TRANSFER_COMPLETE,
}i2c_dma_state_t;

typedef struct{
	I2C_TypeDef* i2c_handle;
	uint8_t dev_addr;
	uint8_t reg_addr;
	uint8_t* rx_buffer;
	uint16_t size;
	volatile i2c_dma_state_t state;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
}i2c_dma_job_t;

static volatile i2c_dma_job_t current_job;

void i2c_manager_init(void){
	current_job.state = DMA_STATE_IDLE;
	current_job.dma_handle = NULL;
	current_job.dma_stream = 0;
	current_job.i2c_handle = NULL;
	current_job.rx_buffer = NULL;
	current_job.size = 0;
	current_job.dev_addr = 0;
	current_job.reg_addr = 0;
}

static uint32_t get_tick(void){
	return osKernelGetTickCount();
}

static uint8_t flag_busy(I2C_TypeDef* i2c_handle){
	return !LL_I2C_IsActiveFlag_BUSY(i2c_handle);
}

static uint8_t flag_addr(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_ADDR(i2c_handle);
}

static uint8_t flag_txe(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_TXE(i2c_handle);
}

static uint8_t flag_rxne(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_RXNE(i2c_handle);
}

static uint8_t flag_btf(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_BTF(i2c_handle);
}

static uint8_t flag_sb(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_SB(i2c_handle);
}

static uint8_t flag_addr_or_af(I2C_TypeDef* i2c_handle){
	return (LL_I2C_IsActiveFlag_ADDR(i2c_handle) || LL_I2C_IsActiveFlag_AF(i2c_handle));
}

typedef uint8_t (*i2c_condition_t) (I2C_TypeDef* i2c_handle);

static i2c_manager_status_t wait_for_condition(I2C_TypeDef* i2c_handle, i2c_condition_t condition_met){
	uint32_t start_time = get_tick();

	while(condition_met(i2c_handle) == 0){
		if((get_tick() - start_time) >= I2C_MANAGER_TIMEOUT_MS){
			return i2c_manager_timeout;
		}
		osDelay(1);
	}

	return i2c_manager_ok;
}

i2c_manager_status_t i2c_manager_read_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	if(size == 0) return i2c_manager_size_0;

	if(osMutexAcquire(I2C_mutexHandle, osWaitForever) != osOK) return i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(I2C_mutexHandle);
		return i2c_manager_busy;
	}

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr) != i2c_manager_ok) goto i2c_error;
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	LL_I2C_TransmitData8(i2c_handle, reg_addr);
	if(wait_for_condition(i2c_handle, flag_txe) != i2c_manager_ok) goto i2c_error;

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr | 1);
	if(wait_for_condition(i2c_handle, flag_addr) != i2c_manager_ok) goto i2c_error;

	if(size == 1){
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
		LL_I2C_ClearFlag_ADDR(i2c_handle);
	}else{
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_ACK);
		LL_I2C_ClearFlag_ADDR(i2c_handle);
		while(size > 1){
			if(wait_for_condition(i2c_handle, flag_rxne) != i2c_manager_ok) goto i2c_error;
			*rxdata = LL_I2C_ReceiveData8(i2c_handle);
			rxdata++;
			size--;
		}
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
	}

	if(wait_for_condition(i2c_handle, flag_rxne) != i2c_manager_ok) goto i2c_error;
	*rxdata = LL_I2C_ReceiveData8(i2c_handle);

	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(I2C_mutexHandle);
	return i2c_manager_ok;

	i2c_error:
	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(I2C_mutexHandle);
	return i2c_manager_timeout;
}

i2c_manager_status_t i2c_manager_write_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* txdata, uint16_t size){
	if(size == 0) return i2c_manager_size_0;

	if(osMutexAcquire(I2C_mutexHandle, osWaitForever) != osOK) return i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != i2c_manager_ok) return i2c_manager_busy;

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr) != i2c_manager_ok) goto i2c_error;
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	LL_I2C_TransmitData8(i2c_handle, reg_addr);
	if(wait_for_condition(i2c_handle, flag_txe) != i2c_manager_ok) goto i2c_error;

	while(size > 1){
		LL_I2C_TransmitData8(i2c_handle, *txdata);
		if(wait_for_condition(i2c_handle, flag_txe) != i2c_manager_ok) goto i2c_error;
		txdata++;
		size--;
	}

	LL_I2C_TransmitData8(i2c_handle, *txdata);
	if(wait_for_condition(i2c_handle, flag_btf) != i2c_manager_ok) goto i2c_error;

	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(I2C_mutexHandle);
	return i2c_manager_ok;

	i2c_error:
	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(I2C_mutexHandle);
	return i2c_manager_timeout;
}

i2c_manager_status_t i2c_manager_find_device(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t device_id){
	if(osMutexAcquire(I2C_mutexHandle, osWaitForever) != osOK) return i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(I2C_mutexHandle);
		return i2c_manager_busy;
	}

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr_or_af) != i2c_manager_ok) goto i2c_error;

	if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){
		LL_I2C_ClearFlag_ADDR(i2c_handle);
		LL_I2C_TransmitData8(i2c_handle, reg_addr);
		if(wait_for_condition(i2c_handle, flag_txe) != i2c_manager_ok) goto i2c_error;

		LL_I2C_GenerateStartCondition(i2c_handle);
		if(wait_for_condition(i2c_handle, flag_sb) != i2c_manager_ok) goto i2c_error;

		LL_I2C_TransmitData8(i2c_handle, dev_addr | 0x01);
		if(wait_for_condition(i2c_handle, flag_addr_or_af) != i2c_manager_ok) goto i2c_error;

		if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){

			LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
			LL_I2C_ClearFlag_ADDR(i2c_handle);
			LL_I2C_GenerateStopCondition(i2c_handle);

			if(wait_for_condition(i2c_handle, flag_rxne) != i2c_manager_ok) goto i2c_error;
			uint8_t buffer;
			buffer = LL_I2C_ReceiveData8(i2c_handle);

			osMutexRelease(I2C_mutexHandle);

			if(buffer == device_id){
				return i2c_manager_ok;
			}else{
				return i2c_manager_fail;
			}
		}
		else if(LL_I2C_IsActiveFlag_AF(i2c_handle)){
			LL_I2C_ClearFlag_AF(i2c_handle);
			LL_I2C_GenerateStopCondition(i2c_handle);
			osMutexRelease(I2C_mutexHandle);
			return i2c_manager_device_not_found;
		}

	}
	else if(LL_I2C_IsActiveFlag_AF(i2c_handle)){
		LL_I2C_ClearFlag_AF(i2c_handle);
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(I2C_mutexHandle);
		return i2c_manager_device_not_found;
	}

	i2c_error:
	if(LL_I2C_IsActiveFlag_AF(i2c_handle)) LL_I2C_ClearFlag_AF(i2c_handle);
	if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)) LL_I2C_ClearFlag_ADDR(i2c_handle);
	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(I2C_mutexHandle);
	return i2c_manager_timeout;
}

bool i2c_manager_is_dma_busy(void){
	return (current_job.state != DMA_STATE_IDLE);
}

static void i2c_dma_cleanup(void){
	LL_I2C_DisableIT_BUF(current_job.i2c_handle);
	LL_I2C_DisableIT_ERR(current_job.i2c_handle);
	LL_I2C_DisableIT_EVT(current_job.i2c_handle);
	LL_I2C_DisableDMAReq_RX(current_job.i2c_handle);
	LL_I2C_DisableLastDMA(current_job.i2c_handle);

	LL_DMA_DisableIT_TC(current_job.dma_handle, current_job.dma_stream);
	LL_DMA_DisableIT_TE(current_job.dma_handle, current_job.dma_stream);
	LL_DMA_DisableStream(current_job.dma_handle, current_job.dma_stream);

	current_job.state = DMA_STATE_IDLE;
}

i2c_manager_status_t i2c_manager_read_dma(DMA_TypeDef* dma_handle, uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	if(size == 0) return i2c_manager_size_0;
	if(i2c_manager_is_dma_busy()) return i2c_manager_dma_busy;

	current_job.dma_handle = dma_handle;
	current_job.dma_stream = dma_stream;
	current_job.i2c_handle = i2c_handle;
	current_job.dev_addr = dev_addr;
	current_job.reg_addr = reg_addr;
	current_job.rx_buffer = rxdata;
	current_job.size = size;

	current_job.state = DMA_STATE_START_SENT;

	LL_I2C_ClearFlag_AF(i2c_handle);
	LL_I2C_ClearFlag_BERR(i2c_handle);
	LL_DMA_ClearFlag_TC0(dma_handle); // şimdi çok uğraştırmasın diye burayı böyle bırakıyorum ama ileride struct içindeki dma_stream kullanılabilsin diye başka bir fonksiyonda switch-case kuracağım ve onu kullanacağım. çünkü şu an sırf bu satırdan ötürü stream 0 kullanmak zorunda kalıyoruz.
	LL_DMA_ClearFlag_TE0(dma_handle);

	LL_I2C_EnableIT_BUF(i2c_handle);
	LL_I2C_EnableIT_ERR(i2c_handle);
	LL_I2C_EnableIT_EVT(i2c_handle);
	LL_DMA_EnableIT_TC(dma_handle, dma_stream);
	LL_DMA_EnableIT_TE(dma_handle, dma_stream);

	LL_I2C_GenerateStartCondition(i2c_handle);

	return i2c_manager_ok;
}

void i2c_manager_error_handler(I2C_TypeDef* i2c_handle){
	if(current_job.state == DMA_STATE_IDLE) return;

	if (LL_I2C_IsActiveFlag_AF(i2c_handle)) { LL_I2C_ClearFlag_AF(i2c_handle); }
	if (LL_I2C_IsActiveFlag_BERR(i2c_handle)) { LL_I2C_ClearFlag_BERR(i2c_handle); }
	if (LL_I2C_IsActiveFlag_OVR(i2c_handle)) { LL_I2C_ClearFlag_OVR(i2c_handle); }
	if(LL_I2C_IsActiveFlag_ARLO(i2c_handle)) { LL_I2C_ClearFlag_ARLO(i2c_handle); }

	LL_I2C_GenerateStopCondition(i2c_handle);
	i2c_dma_cleanup();

	osSemaphoreRelease(DMA_complete_semaphoreHandle);
}

void i2c_manager_dma_rx_handler(void){
	if(current_job.state == DMA_STATE_IDLE) return;

	if(LL_DMA_IsActiveFlag_TE0(DMA1)){ // Transfer Hatası
		LL_DMA_ClearFlag_TE0(DMA1);
		current_job.state = DMA_STATE_TRANSFER_ERROR;
		LL_I2C_GenerateStopCondition(current_job.i2c_handle);
		i2c_dma_cleanup();
		osSemaphoreRelease(DMA_complete_semaphoreHandle);
	}

	if(LL_DMA_IsActiveFlag_TC0(DMA1)){ // Transfer Başarıyla Tamamlandı!
		LL_DMA_ClearFlag_TC0(DMA1);
		current_job.state = DMA_STATE_TRANSFER_COMPLETE;
		LL_I2C_GenerateStopCondition(current_job.i2c_handle);
		i2c_dma_cleanup();
		osSemaphoreRelease(DMA_complete_semaphoreHandle);
	}

}

void i2c_manager_event_handler(I2C_TypeDef* i2c_handle){
	if(current_job.state == DMA_STATE_IDLE) return;

	switch(current_job.state){

		case DMA_STATE_START_SENT:
			if(LL_I2C_IsActiveFlag_SB(i2c_handle)){
				LL_I2C_TransmitData8(i2c_handle, current_job.dev_addr); // Cihaz Adresi Yaz(0)
				current_job.state = DMA_STATE_WRITE_ADDR_SENT;
			}
			break;

		case DMA_STATE_WRITE_ADDR_SENT:
			if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){
				LL_I2C_ClearFlag_ADDR(i2c_handle);
				LL_I2C_TransmitData8(i2c_handle, current_job.reg_addr); // Okunacak Register Adresi
				current_job.state = DMA_STATE_REG_ADDR_SENT;
			}
			break;

		case DMA_STATE_REG_ADDR_SENT:
			if(LL_I2C_IsActiveFlag_TXE(i2c_handle)){
				LL_I2C_GenerateStartCondition(i2c_handle); // Restart at
				current_job.state = DMA_STATE_RESTART_SENT;
			}
			break;

		case DMA_STATE_RESTART_SENT:
			if(LL_I2C_IsActiveFlag_SB(i2c_handle)){
				LL_I2C_TransmitData8(i2c_handle, current_job.dev_addr | 1); // Cihaz Adresi Oku(1)
				current_job.state = DMA_STATE_READ_ADDR_SENT;
			}
			break;

		case DMA_STATE_READ_ADDR_SENT:
			if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){

				// DMA'yı Dinamik Olarak Hedefe Kur
				LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
				LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_I2C_DMA_GetRegAddr(i2c_handle));
				LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)current_job.rx_buffer);
				LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, current_job.size);

				LL_I2C_EnableLastDMA(i2c_handle);
				LL_I2C_EnableDMAReq_RX(i2c_handle);
				LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

				LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_ACK);

				// Sadece DMA'nın işini bitirmesini bekleyeceğiz, I2C Event kesmelerini kapatıyoruz
				LL_I2C_DisableIT_EVT(i2c_handle);
				LL_I2C_DisableIT_BUF(i2c_handle);

				current_job.state = DMA_STATE_READING;

				LL_I2C_ClearFlag_ADDR(i2c_handle); // Adres bayrağını temizle, DMA akışı başlasın!
			}
			break;

		case DMA_STATE_READING: break;
		case DMA_STATE_IDLE: break;
		default: break;
	}
}
