#ifndef __CHIPS_COMMON_H__
#define __CHIPS_COMMON_H__

struct param {
	unsigned char cmd;
	unsigned short addr;
	unsigned short data;	
};

struct config {
	struct param *p_param;
	int num;
};

enum mode{
	IDLE = 0,
	NORMAL = 1,
	SLEEP = 2,
	DEEP_SLEEP = 6,	
};

int chips_sfr_read(uint16_t addr,uint8_t *data,uint16_t len);
int chips_sfr_write(uint16_t addr,uint8_t *data,uint16_t len);
int chips_sram_read(uint16_t addr,uint8_t *data,uint16_t len);
int chips_sram_write(uint16_t addr,uint8_t *data,uint16_t len);
int chips_spi_send_cmd(uint8_t *cmd,uint16_t len);
int chips_write_configs(struct param *p_param, int num);
int chips_probe_sensorID(uint16_t *sensorid);


//only for debug 
int read_SFR(uint16_t addr,uint8_t *data);
int write_SFR(uint16_t addr,uint8_t data);
int read_SRAM(uint16_t addr,uint16_t *data);
int write_SRAM(uint16_t addr,uint16_t data);
void chips_sensor_config(void);
int chips_scan_one_image(uint16_t addr, uint8_t *buffer, uint16_t len);
int chips_set_sensor_mode(int mode);

#endif