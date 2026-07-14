#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "i2c_manager.h" // YENİ: I2C Manager'ı buraya dahil ediyoruz
#include "stdint.h"
#include "stddef.h"

typedef enum{
	_bme280_ok = 0,
	_bme280_fail = 1,
	_bme280_timeout = 2,
	_bme280_read_fail = 3,
	_bme280_write_fail = 4,
	_bme280_i2c_busy = 5,
	_bme280_device_not_found = 6
	// DMA_BUSY silindi, çünkü DMA işlerini ileride Manager'a taşıyacağız.
}bme280_return_status;

typedef enum{
	bme280_i2c_addr0 = (0x76 << 1),
	bme280_i2c_addr1 = (0x77 << 1),
}bme280_i2c_address;

// Sensör konfigürasyon yapıları (Değişmedi)
typedef union{
	uint8_t raw;
	struct{
		uint8_t spi_en: 1;
		uint8_t reserved: 1;
		uint8_t filter: 3;
		uint8_t t_sb: 3;
	}bits;
}bme280_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t mode: 2;
		uint8_t osrs_p: 3;
		uint8_t osrs_t: 3;
	}bits;
}bme280_ctrlmeas_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t osrs_h: 3;
		uint8_t reserved: 5;
	}bits;
}bme280_ctrlhum_t;

// Kullanıcı konfigürasyonu
typedef struct{
	I2C_TypeDef* i2c_handle; // Manager'a hangi I2C'yi kullanacağını söylemek için lazım
	uint8_t i2c_addr;
	bme280_config_t config_t;
	bme280_ctrlhum_t ctrlhum_t;
	bme280_ctrlmeas_t ctrlmeas_t;
}bme280_user_configs;

struct bme280_t;
typedef struct bme280_t* bme280_handle_t;

// Kullanılacak Fonksiyon Prototipleri
bme280_handle_t bme280_init(bme280_user_configs* config);
bme280_return_status bme280_get_value(bme280_handle_t dev);
bme280_return_status bme280_read_data_poll(bme280_handle_t dev); // Yeni: Ham verileri polling ile çekmek için eklendi

#endif /* INC_BME280_H_ */
