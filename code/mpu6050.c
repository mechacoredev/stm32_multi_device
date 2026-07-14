#include "mpu6050.h"
#include <stdlib.h>

typedef enum{
	addr_sample_rate_div = 0x19,
	addr_config = 0x1A,
	addr_gyro_config = 0x1B,
	addr_accel_config = 0x1C,
	addr_pwr_mgmt_1 = 0x6B,
	addr_pwr_mgmt_2 = 0x6C,
	addr_who_am_i = 0x75,
	addr_accel_xout_h = 0x3B,
}mpu6050_register_map;

struct mpu6050_t{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint8_t sample_rate;
	mpu6050_reg_config reg_config;
	mpu6050_reg_gyro_config reg_gyro_config;
	mpu6050_reg_accel_config reg_accel_config;
	mpu6050_reg_pwr_mgmt_1 reg_pwr_mgmt_1;
	mpu6050_reg_pwr_mgmt_2 reg_pwr_mgmt_2;

	uint8_t raw_data[14];
	int16_t accel_G[3];
	int16_t gyro_dps[3];
	float temperature;

	float acceling[3];
	float gyroins[3];
};

#define MAX_MPU_INSTANCES 2
static struct mpu6050_t sensor_pool[MAX_MPU_INSTANCES];
static uint8_t next_free_index = 0;

static mpu6050_return_status_t write_register(mpu6050_handle_t dev, uint8_t reg_addr, uint8_t* txdata, uint16_t size){
	if(i2c_manager_write(dev->i2c_handle, dev->i2c_addr, reg_addr, txdata, size) == I2C_MANAGER_OK){
		return _mpu6050_ok;
	}
	return _mpu6050_register_write_unavailable;
}

static mpu6050_return_status_t read_register(mpu6050_handle_t dev, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	if(i2c_manager_write(dev->i2c_handle, dev->i2c_addr, reg_addr, rxdata, size) == I2C_MANAGER_OK){
		return _mpu6050_ok;
	}
	return _mpu6050_register_read_unavailable;
}

mpu6050_return_status_t mpu6050_find_or_check_device(mpu6050_user_config_t* config){
	if(config == NULL) return _mpu6050_fail;
	if(i2c_manager_check_device(config->i2c_handle, config->i2c_addr) == I2C_MANAGER_OK){
		return _mpu6050_ok;
	}
	return _mpu6050_device_not_found;
}

mpu6050_handle_t mpu6050_init(mpu6050_user_config_t* config){
	if(config == NULL) return NULL;
	if(next_free_index >= MAX_MPU_INSTANCES) return NULL;

	mpu6050_handle_t dev = &sensor_pool[next_free_index];
	next_free_index++;

	dev->i2c_addr = config->i2c_addr;
	dev->i2c_handle = config->i2c_handle;
	dev->reg_accel_config.raw = config->reg_accel_config.raw;
	dev->reg_config.raw = config->reg_config.raw;
	dev->reg_gyro_config.raw = config->reg_gyro_config.raw;
	dev->reg_pwr_mgmt_1.raw = config->reg_pwr_mgmt_1.raw;
	dev->reg_pwr_mgmt_2.raw = config->reg_pwr_mgmt_2.raw;
	dev->sample_rate = config->sample_rate;

	uint8_t who_am_i_val = 0;
	if(read_register(dev, addr_who_am_i, &who_am_i_val, 1) != _mpu6050_ok){
		next_free_index--;
		return NULL;
	}
	if(who_am_i_val != 0x68 && who_am_i_val != 0x70){
		next_free_index--;
		return NULL;
	}

	uint8_t wake_up_cmd = 0;
	if(write_register(dev, addr_pwr_mgmt_1, &wake_up_cmd, 1) != _mpu6050_ok){
		next_free_index--;
		return NULL;
	}

	osDelay(100);
	return dev;
}

mpu6050_return_status_t mpu6050_configurate(mpu6050_handle_t dev, mpu6050_user_config_t* config){
	if(dev == NULL || config == NULL) return _mpu6050_fail;

	if(write_register(dev, addr_pwr_mgmt_1, &config->reg_pwr_mgmt_1.raw, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;
	if(write_register(dev, addr_config, &config->reg_config.raw, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;
	if(write_register(dev, addr_gyro_config, &config->reg_gyro_config.raw, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;
	if(write_register(dev, addr_accel_config, &config->reg_accel_config.raw, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;
	if(write_register(dev, addr_sample_rate_div, &config->sample_rate, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;
	if(write_register(dev, addr_pwr_mgmt_2, &config->reg_pwr_mgmt_2.raw, 1) != _mpu6050_ok) return _mpu6050_register_write_unavailable;

	return _mpu6050_ok;
}

mpu6050_return_status_t mpu6050_read_data_poll(mpu6050_handle_t dev){
	if(dev == NULL) return _mpu6050_fail;
	// 0x3B (ACCEL_XOUT_H) adresinden itibaren aralıksız 14 bayt oku
	return read_register(dev, addr_accel_xout_h, dev->raw_data, 14);
}

mpu6050_return_status_t mpu6050_get_values(mpu6050_handle_t dev){
	if(dev == NULL) return _mpu6050_fail;

	int16_t tempData;
	// High ve Low baytları bit kaydırma ile birleştir
	dev->accel_G[0] = (int16_t) ((dev->raw_data[0] << 8) | dev->raw_data[1]);
	dev->accel_G[1] = (int16_t) ((dev->raw_data[2] << 8) | dev->raw_data[3]);
	dev->accel_G[2] = (int16_t) ((dev->raw_data[4] << 8) | dev->raw_data[5]);

	tempData = (int16_t) ((dev->raw_data[6] << 8) | dev->raw_data[7]);

	dev->gyro_dps[0] = (int16_t) ((dev->raw_data[8] << 8) | dev->raw_data[9]);
	dev->gyro_dps[1] = (int16_t) ((dev->raw_data[10] << 8) | dev->raw_data[11]);
	dev->gyro_dps[2] = (int16_t) ((dev->raw_data[12] << 8) | dev->raw_data[13]);

	// Datasheet'te belirtilen sıcaklık denklemi
	dev->temperature = (float) tempData / 340.0f + 36.53f;

	return _mpu6050_ok;
}

mpu6050_return_status_t mpu6050_get_values_ing_ins(mpu6050_handle_t dev, mpu6050_user_config_t config){
	if(dev == NULL) return _mpu6050_fail;

	// Seçilen Full Scale aralığına göre (fs_sel) bölme işlemi yaparak fiziksel birime çevirme
	// MPU6050 Datasheet'e göre ivmeölçer bölenleri: 0=16384, 1=8192, 2=4096, 3=2048
	dev->acceling[0] = (float) ((dev->accel_G[0] / 16384.0f) / (dev->reg_accel_config.bits.fs_sel + 1));
	dev->acceling[1] = (float) ((dev->accel_G[1] / 16384.0f) / (dev->reg_accel_config.bits.fs_sel + 1));
	dev->acceling[2] = (float) ((dev->accel_G[2] / 16384.0f) / (dev->reg_accel_config.bits.fs_sel + 1));

	// Jiroskop bölenleri: 0=131.0, 1=65.5, 2=32.8, 3=16.4
	dev->gyroins[0] = (float) ((dev->gyro_dps[0] / 131.0f) * (1 << dev->reg_gyro_config.bits.fs_sel));
	dev->gyroins[1] = (float) ((dev->gyro_dps[1] / 131.0f) * (1 << dev->reg_gyro_config.bits.fs_sel));
	dev->gyroins[2] = (float) ((dev->gyro_dps[2] / 131.0f) * (1 << dev->reg_gyro_config.bits.fs_sel));

	return _mpu6050_ok;
}
