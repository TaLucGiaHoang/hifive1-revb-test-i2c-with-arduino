/*
 * main.c
 *
 *  Created on: Dec 6, 2020
 *      Author: TaLucGiaHoang
 *
 *  IDE: FreedomStudio-2019-08-2-win64 (https://github.com/sifive/freedom-studio/releases/tag/v2019.08.2)
 *  SDK: freedom-e-sdk-v201908
 *
 *  This is an example for using I2C of SiFive HiFive1 Rev B01 board
 *  Connect I2C pins of this board to I2C pins of Arduino UNO board
 *
 *  The program used on Arduino UNO board could be Arduino's Wire
 *  library examples "Wire Slave Sender" and "Wire Slave Receiver"
 *
 *  SiFive HiFive1 Rev B01 board supports I2C Master Interface only
 *
 *  Pin configuration:
 *  GPIO | IOF0    | IOF1
 *  12   | I2C0_SDA| PWM2_PWM2
 *  13   | I2C0_SCL| PWM2_PWM3
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "drv_i2c0.h"

#define ARDUINO_ADDR 8
#define I2C_BAUDRATE 100000
#define METAL_I2C_MASTER 1

/* 1s delay macro */
#define WAIT_1S(timeout)                                                       \
  timeout = time(NULL) + 1;                                                    \
  while (timeout > time(NULL))                                                 \
    ;

static int master_send(void);
static int master_recv(void);

int main(void)
{
	printf("%s %s \n", __DATE__, __TIME__);
	printf("Initialize I2C master %d(Hz)\r\n", I2C_BAUDRATE);
	if(driver_i2c0_init(I2C_BAUDRATE, METAL_I2C_MASTER) != 0)
	{
		printf("I2C initialization failed\r\n");
	}
#if 1 // I2C MASTER SEND
	master_send();
#else // I2C MASTER RECEIVE
	master_recv();
#endif

	return 0;
}

static int master_send(void)
{
	int ret = -1;
	time_t timeout;
	int idx = 0;
	char *s[3] = {"hifive1 ", "hello ", "arduino\r\n"};
	char buf[] = "message from hifive1\r\n";

	driver_i2c0_write(ARDUINO_ADDR, strlen(s[0]), s[0], METAL_I2C_STOP_ENABLE);
	driver_i2c0_write(ARDUINO_ADDR, strlen(s[1]), s[1], METAL_I2C_STOP_ENABLE);
	driver_i2c0_write(ARDUINO_ADDR, strlen(s[2]), s[2], METAL_I2C_STOP_ENABLE);

	while(1)
	{
		sprintf(buf, "message from hifive1 %d\r\n", idx);
		ret = driver_i2c0_write(ARDUINO_ADDR, strlen(buf), buf, METAL_I2C_STOP_ENABLE);
		printf("I2C send: return %d, idx %d\r\n", ret, idx);
		++idx;
		WAIT_1S(timeout)
	}

	printf("master_send done\r\n");
}

static int master_recv(void)
{
	int ret = -1;
	time_t timeout;
	int idx = 0;
	char buf[20];

	while(1)
	{
		memset(buf, 0, sizeof(buf));
		ret = driver_i2c0_read(ARDUINO_ADDR, 6, buf, METAL_I2C_STOP_ENABLE);
		if(ret != 0)
		{
			printf("I2C receive: return %d, idx %d,\r\n", ret, idx);
		} else {
			printf("I2C receive: return %d, idx %d, msg \"%s\"\r\n", ret, idx, buf);
		}
		++idx;
		WAIT_1S(timeout)
	}

	printf("master_recv done\r\n");
	return 0;
}
