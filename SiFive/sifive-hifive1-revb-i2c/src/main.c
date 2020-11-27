/*
 * main.c
 *
 *  Created on: Sep 4, 2020
 *      Author: HoangSHC
 *
 *  IDE: FreedomStudio-2019-08-2-win64
 *  SDK: freedom-e-sdk-v201908
 *
 *  This program is used to test I2C communication between
 *  SiFive HiFive1 Rev B01 board and Arduino UNO board.
 *  The program used for Arduino UNO could be examples from Wire library
 *  Example: "Wire Slave Sender" and "Wire Slave Receiver"
 */

//
//
//  GPIO | IOF0    | IOF1
//  12   | I2C0_SDA| PWM2_PWM2
//  13   | I2C0_SCL| PWM2_PWM3


//#include <metal/cpu.h>
//#include <metal/io.h>
//#include <metal/machine.h>
//#include <metal/uart.h>
#include <stdio.h>
#include <time.h>
#include "drv_i2c0.h"
#include <string.h>

/* PmodAD2, PmodTmp2 sensor modules are connected to I2C0 bus */
#define ARDUINO_ADDR 8
#define I2C_BAUDRATE 100000
#define METAL_I2C_MASTER 1
/* Return values */
#define RET_OK 0
#define RET_NOK 1
/* Buffer length macros */
#define LEN0 0
#define LEN1 1
#define LEN2 2
/* 1s delay macro */
#define WAIT_1S(timeout)                                                       \
  timeout = time(NULL) + 1;                                                    \
  while (timeout > time(NULL))                                                 \
    ;

static void delay_1ms(void)
{
  time_t stop_time = time(NULL) + 1;
  while (stop_time > time(NULL))
  {
	  ;
  }
}

int main(void)
{
  unsigned int temp, volt;
  unsigned char buf[LEN2];
  time_t timeout;
  int ret = 0;
  char recv[12];
//  printf("recv size %d, len %d\n", sizeof(recv), strlen(recv)); // msg size 12, len 0
//  printf("%s %s \n", __DATE__, __TIME__);
  printf("I2C demo test..\n");


  ret = driver_i2c0_init(I2C_BAUDRATE, METAL_I2C_MASTER);
  if(ret != 0)
  {
	  printf("driver_i2c0_init failed , ret %d\r\n", ret);
  }
#if 1
  printf("Test driver_i2c0_write send continuously\n");
//  driver_i2c0_write(8, 6, "123456", METAL_I2C_STOP_ENABLE);
  driver_i2c0_write(8, strlen("string 1\r\n"), "string 1\r\n", METAL_I2C_STOP_ENABLE);
  driver_i2c0_write(8, strlen("string 2\r\n"), "string 2\r\n", METAL_I2C_STOP_ENABLE);
  driver_i2c0_write(8, strlen("string 3\r\n"), "string 3\r\n", METAL_I2C_STOP_ENABLE);
  driver_i2c0_write(8, strlen("string 4\r\n"), "string 4\r\n", METAL_I2C_STOP_ENABLE);

  printf("Test driver_i2c0_write retrun, stop enable\n");
  ret = driver_i2c0_write(8, strlen("I2C send with stop bit\r\n"), "I2C send with stop bit\r\n", METAL_I2C_STOP_ENABLE);
  printf("driver_i2c0_write returns %d\r\n", ret);

  printf("Test driver_i2c0_write retrun, stop enable\n");
  ret = driver_i2c0_write(8, strlen("I2C send with stop bit\r\n"), "I2C send with stop bit\r\n", METAL_I2C_STOP_ENABLE);
  printf("driver_i2c0_write returns %d\r\n", ret);

  printf("Test driver_i2c0_write retrun, stop enable\n");
  ret = driver_i2c0_write(8, strlen("I2C send with stop bit\r\n"), "I2C send with stop bit\r\n", METAL_I2C_STOP_ENABLE);
  printf("driver_i2c0_write returns %d\r\n", ret);
#endif
//  printf("driver_i2c0_write retrun, stop disable\n");
//  if(driver_i2c0_write(8, strlen("I2C send without stop bit\r\n"), "I2C send without stop bit\r\n", METAL_I2C_STOP_DISABLE) != 0)
//  {
//	  printf("driver_i2c0_write failed , ret %d\r\n", ret);
//  }
//  printf("Test1 driver_i2c0_read retrun, stop disable\n");
//  if(driver_i2c0_read(8, 6, recv, METAL_I2C_STOP_DISABLE) != 0)
//  {
//	  printf("driver_i2c0_read failed\r\n");
//  } else {
//	  printf("%s\r\n", recv);
//  }
//
//  printf("Test2 driver_i2c0_read retrun, stop disable\n");
//  if(driver_i2c0_read(8, 6, recv, METAL_I2C_STOP_DISABLE) != 0)
//  {
//	  printf("driver_i2c0_read failed\r\n");
//  } else {
//	  printf("%s\r\n", recv);
//  }

  printf("Test 1 driver_i2c0_read retrun, stop enable\n");
  memset(recv, 0, sizeof(recv));
  if(driver_i2c0_read(8, 6, recv, METAL_I2C_STOP_ENABLE) != 0)
  {
	  printf("driver_i2c0_read failed\r\n");
  } else {
	  printf("\"%s\"\r\n", recv);
  }
//  WAIT_1S(timeout)
  printf("Test 2 driver_i2c0_read retrun, stop enable\n");
  memset(recv, 0, sizeof(recv));
  if(driver_i2c0_read(8, 6, recv, METAL_I2C_STOP_ENABLE) != 0)
  {
	  printf("driver_i2c0_read failed\r\n");
  } else {
	  printf("\"%s\"\r\n", recv);
  }
//  WAIT_1S(timeout)
  printf("Test 3 driver_i2c0_read retrun, stop enable\n");
  memset(recv, 0, sizeof(recv));
  if(driver_i2c0_read(8, 6, recv, METAL_I2C_STOP_ENABLE) != 0)
  {
	  printf("driver_i2c0_read failed\r\n");
  } else {
	  printf("\"%s\"\r\n", recv);
  }
  WAIT_1S(timeout)

  while(1)
  {
	  ;
  }
	return 0;
}
