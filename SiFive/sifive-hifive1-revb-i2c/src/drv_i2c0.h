/*
 * drv_i2c0.h
 *
 *  Created on: Sep 1, 2020
 *      Author: HoangSHC
 */

#ifndef DRV_I2C0_H_
#define DRV_I2C0_H_

/*! @brief Enums to enable/disable stop condition. */
#define METAL_I2C_STOP_DISABLE  (0)
#define METAL_I2C_STOP_ENABLE   (1)

/*
 * Initialize I2C0 peripheral
 * @param baud_rate: 100kHz
 * @param mode: slave (0), master (1)
 * @return 0 if success, otherwise is false
 */
int driver_i2c0_init(unsigned int baud_rate, int mode);

/*
 * I2C transmit to slave
 * @param addr: slave address transfer, current driver supports only 7-bit
 * @param len: length of buffer
 * @param buf: buffer
 * @param stop_bit: stop bit disable (0), stop bit enable (1)
 * @return 0 if success, otherwise is false
 */
int driver_i2c0_write(unsigned int addr, unsigned int len,
             unsigned char buf[], int stop_bit);

/*
 * I2C receive from slave
 * @param addr: slave address transfer, current driver supports only 7-bit
 * @param len: length of buffer
 * @param buf: buffer
 * @param stop_bit: stop bit disable (0), stop bit enable (1)
 * @return 0 if success, otherwise is false
 */
int driver_i2c0_read(unsigned int addr, unsigned int len,
            unsigned char buf[], int stop_bit);

/*
 * I2C Tranfer to slave
 * @param addr: slave address transfer, current driver supports only 7-bit
 * @param txbuf: transmit buffer
 * @param txlen: length of transmit buffer
 * @param rxbuf: received buffer
 * @param rxlen: length of received buffer
 * @return 0 if success, otherwise is false
 */
int driver_i2c0_transfer(unsigned int addr,
                unsigned char txbuf[], unsigned int txlen,
                unsigned char rxbuf[], unsigned int rxlen);

/*
 * Get I2C0 baud rate
 * @param none
 * @return I2C0 baud rate
 */
int driver_i2c0_get_baud_rate(void);

/*
 * Set I2C0 baud rate
 * @param baud_rate: baud rate
 * @return 0 if success, otherwise is false
 */
int driver_i2c0_set_baud_rate(unsigned int baud_rate);

#endif /* DRV_I2C0_H_ */
