#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "i2c_manager.h"
#include "stdint.h"

struct mpu6050_t;
typedef struct mpu6050_t* mpu6050_handle_t;

typedef enum{
	_mpu6050_ok=0,
	_mpu6050_fail,
	_mpu6050_device_not_found,
	_mpu6050_register_read_unavailable,
	_mpu6050_register_write_unavailable,
	_mpu6050_uninited_device
}mpu6050_return_status_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t dlpf_cfg: 3;
		uint8_t ext_sync_set: 3;
		uint8_t reserved: 2;
	}bits;
}mpu6050_reg_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t fs_sel: 2;
		uint8_t reserved2: 3;
	}bits;
}mpu6050_reg_gyro_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t fs_sel: 2;
		uint8_t za_st: 1;
		uint8_t ya_st: 1;
		uint8_t xa_st: 1;
	}bits;
}mpu6050_reg_accel_config;

typedef union{
	uint8_t raw;
	struct{
		uint8_t clksel: 3;
		uint8_t temp_dis: 1;
		uint8_t reserved: 1;
		uint8_t cycle: 1;
		uint8_t sleep: 1;
		uint8_t device_reset: 1;
	}bits;
}mpu6050_reg_pwr_mgmt_1;

typedef union{
	uint8_t raw;
	struct{
		uint8_t stby_zg: 1;
		uint8_t stby_yg: 1;
		uint8_t stby_xg: 1;
		uint8_t stby_za: 1;
		uint8_t stby_ya: 1;
		uint8_t stby_xa: 1;
		uint8_t lp_wake_ctrl: 2;
	}bits;
}mpu6050_reg_pwr_mgmt_2;

typedef struct{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint8_t sample_rate;
	mpu6050_reg_config reg_config;
	mpu6050_reg_gyro_config reg_gyro_config;
	mpu6050_reg_accel_config reg_accel_config;
	mpu6050_reg_pwr_mgmt_1 reg_pwr_mgmt_1;
	mpu6050_reg_pwr_mgmt_2 reg_pwr_mgmt_2;
}mpu6050_user_config_t;

mpu6050_return_status_t mpu6050_find_or_check_device(mpu6050_user_config_t* config);
mpu6050_handle_t mpu6050_init(mpu6050_user_config_t* config);
mpu6050_return_status_t mpu6050_configurate(mpu6050_handle_t dev, mpu6050_user_config_t* config);
mpu6050_return_status_t mpu6050_get_values(mpu6050_handle_t dev);
mpu6050_return_status_t mpu6050_get_values_ing_ins(mpu6050_handle_t dev, mpu6050_user_config_t config);
mpu6050_return_status_t mpu6050_read_data_poll(mpu6050_handle_t dev);

#endif /* INC_MPU6050_H_ */
