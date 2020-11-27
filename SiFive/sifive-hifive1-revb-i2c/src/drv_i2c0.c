/*
 * drv_i2c0.c
 *
 *  Created on: Sep 1, 2020
 *      Author: HoangSHC
 */
#include "drv_i2c0.h"
#include <metal/clock.h>
#include <metal/compiler.h>
#include <metal/drivers/sifive_gpio0.h>
#include <metal/io.h>
#include <metal/machine.h>
#include <metal/time.h>
#include <stdio.h>

/* Driver console logging */
//#include "debug_serial.h" // comment out this line to debug
#if defined(DEBUG_SERIAL_H_)
#define METAL_I2C_LOG(x) debug_puts("drv_i2c0.c: ");debug_puts(x)
#else
#define METAL_I2C_LOG(x)
#endif


/* I2C0 Register Address */
#define SIFIVE_I2C0_BASE_ADDRESS (0x10016000)
#define REG_SIFIVE_I2C0_PRESCALE_LOW       (*(volatile uint32_t*)(0x10016000))
#define REG_SIFIVE_I2C0_PRESCALE_HIGH      (*(volatile uint32_t*)(0x10016004))
#define REG_SIFIVE_I2C0_CONTROL            (*(volatile uint32_t*)(0x10016008))
#define REG_SIFIVE_I2C0_TRANSMIT           (*(volatile uint32_t*)(0x1001600C))
#define REG_SIFIVE_I2C0_RECEIVE            (*(volatile uint32_t*)(0x1001600C))
#define REG_SIFIVE_I2C0_COMMAND            (*(volatile uint32_t*)(0x10016010))
#define REG_SIFIVE_I2C0_STATUS             (*(volatile uint32_t*)(0x10016010))

/* GPIO0 Register Address */
#define SIFIVE_GPIO0_BASE_ADDRESS  (0x10012000)
#define REG_SIFIVE_GPIO0_INPUT_VAL (*(volatile uint32_t*)(0x10012000))
#define REG_SIFIVE_GPIO0_INPUT_EN (*(volatile uint32_t*)(0x10012004))


/* From i2c@10016000 */
#define METAL_SIFIVE_I2C0_10016000_BASE_ADDRESS 268525568UL
#define METAL_SIFIVE_I2C0_0_BASE_ADDRESS 268525568UL
#define METAL_SIFIVE_I2C0_10016000_SIZE 4096UL
#define METAL_SIFIVE_I2C0_0_SIZE 4096UL

#define METAL_SIFIVE_I2C0
#define METAL_SIFIVE_I2C0_PRESCALE_LOW 0UL
#define METAL_SIFIVE_I2C0_PRESCALE_HIGH 4UL
#define METAL_SIFIVE_I2C0_CONTROL 8UL
#define METAL_SIFIVE_I2C0_TRANSMIT 12UL
#define METAL_SIFIVE_I2C0_RECEIVE 12UL
#define METAL_SIFIVE_I2C0_COMMAND 16UL
#define METAL_SIFIVE_I2C0_STATUS 16UL

/* Register fields */
#define METAL_I2C_CONTROL_EN (1UL << 7)
#define METAL_I2C_CONTROL_IE (1UL << 6)
#define METAL_I2C_WRITE (0UL << 0)
#define METAL_I2C_READ (1UL << 0)
#define METAL_I2C_CMD_START (1UL << 7)
#define METAL_I2C_CMD_STOP (1UL << 6)
#define METAL_I2C_CMD_READ (1UL << 5)
#define METAL_I2C_CMD_WRITE (1UL << 4)
#define METAL_I2C_CMD_ACK (1UL << 3)
#define METAL_I2C_CMD_IACK (1UL << 0)
#define METAL_I2C_STATUS_RXACK (1UL << 7)
#define METAL_I2C_STATUS_BUSY (1UL << 6)
#define METAL_I2C_STATUS_AL (1UL << 5)
#define METAL_I2C_STATUS_TIP (1UL << 1)
#define METAL_I2C_STATUS_IP (1UL << 0)

/* Prescaler max value */
#define METAL_I2C_PRESCALE_MAX 0xFFFF
/* Macros to access registers */
#define METAL_I2C_REG(offset) ((base + offset))
#define METAL_I2C_REGB(offset)                                                 \
    (__METAL_ACCESS_ONCE((__metal_io_u8 *)METAL_I2C_REG(offset)))
#define METAL_I2C_REGW(offset)                                                 \
    (__METAL_ACCESS_ONCE((__metal_io_u32 *)METAL_I2C_REG(offset)))

/* Timeout macros for register status checks */
#define METAL_I2C_RXDATA_TIMEOUT 1
#define METAL_I2C_TIMEOUT_RESET(timeout)                                       \
    timeout = metal_time() + METAL_I2C_RXDATA_TIMEOUT
#define METAL_I2C_TIMEOUT_CHECK(timeout)                                       \
    if (metal_time() > timeout) {                                              \
        METAL_I2C_LOG("I2C timeout error.\r\n");                                 \
        return METAL_I2C_RET_ERR;                                              \
    }
#define METAL_I2C_REG_CHECK(exp, timeout)                                      \
    while (exp) {                                                              \
        METAL_I2C_TIMEOUT_CHECK(timeout)                                       \
    }


#define METAL_SIFIVE_I2C_INSERT_STOP(stop_flag) ((stop_flag & 0x01UL) << 6)
#define METAL_SIFIVE_I2C_INSERT_RW_BIT(addr, rw)                               \
    ((addr & 0x7FUL) << 1 | (rw & 0x01UL))
#define METAL_SIFIVE_I2C_GET_PRESCALER(baud)                                   \
    ((clock_rate / (baud_rate * 5)) - 1)
#define METAL_I2C_INIT_OK 1
#define METAL_I2C_RET_OK 0
#define METAL_I2C_RET_ERR -1

static unsigned int _baud_rate = 0;
static int driver_i2c0_write_addr(unsigned long base, unsigned int addr, unsigned char rw_flag) {
    time_t timeout;
    int ret = 0;

    /* Reset timeout */
    METAL_I2C_TIMEOUT_RESET(timeout);

    /* Check if any transfer is in progress */
    METAL_I2C_REG_CHECK(
        (METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) & METAL_I2C_STATUS_TIP),
        timeout);

    /* Set transmit register to given address with read/write flag */
    METAL_I2C_REGB(METAL_SIFIVE_I2C0_TRANSMIT) = METAL_SIFIVE_I2C_INSERT_RW_BIT(addr, rw_flag);
//    REG_SIFIVE_I2C0_TRANSMIT = METAL_SIFIVE_I2C_INSERT_RW_BIT(addr, rw_flag);

    /* Set start flag to trigger the address transfer */
    METAL_I2C_REGB(METAL_SIFIVE_I2C0_COMMAND) =
        METAL_I2C_CMD_WRITE | METAL_I2C_CMD_START;
    /* Reset timeout */
    METAL_I2C_TIMEOUT_RESET(timeout);

    /* Check for transmit completion */
    METAL_I2C_REG_CHECK(
        (METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) & METAL_I2C_STATUS_TIP),
        timeout);

    /* Check for ACK from slave */
    if ((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) & METAL_I2C_STATUS_RXACK)) {
        /* No ACK, return error */
        METAL_I2C_LOG("I2C RX ACK failed.\r\n");
        ret = -1;
    }

    return ret;
}

static int driver_sifive_gpio0_enable_io(long source, long dest)
{
    long base = METAL_SIFIVE_GPIO0_10012000_BASE_ADDRESS;

    __METAL_ACCESS_ONCE((__metal_io_u32 *)(base + METAL_SIFIVE_GPIO0_IOF_SEL)) &= ~source;
    __METAL_ACCESS_ONCE((__metal_io_u32 *)(base + METAL_SIFIVE_GPIO0_IOF_EN))  |= dest;

    return 0;
}

//  GPIO | IOF0    | IOF1
//  12   | I2C0_SDA| PWM2_PWM2
//  13   | I2C0_SCL| PWM2_PWM3
/*
 * baud_rate: 100000 Hz or 400000 Hz
 * mode: (1) Master, (0) Slave
 *
 */
int driver_i2c0_init(unsigned int baud_rate, int mode) {

	int ret = -1;

	/* Initialize baud rate */
	_baud_rate = 0;

	/* configure I2C I/O pins */
	long pinmux_output_selector = 0;  // i2c0 pimux output selector
	long pinmux_source_selector = 12288;  // i2c0 pinmux source selector 0x3000

    driver_sifive_gpio0_enable_io(pinmux_output_selector, pinmux_source_selector);

	/* 1: Master 0: Slave */
	if (mode == 1) {
		METAL_I2C_LOG("Set I2C master mode\r\n");
		/* Set requested baud rate */
		if (driver_i2c0_set_baud_rate(baud_rate) == 0) {
			METAL_I2C_LOG("Set baud rate done\r\n");
			ret = 0;
		}
	} else {
		METAL_I2C_LOG("Set I2C slave mode - not supported\r\n");
		/* Nothing to do. slave mode not supported */
		ret = -1;
	}

	return ret;
}

int driver_i2c0_get_baud_rate(void) {
    return _baud_rate;
}

int driver_i2c0_set_baud_rate(unsigned int baud_rate) {
    struct metal_clock *clock = NULL;
    unsigned long base = METAL_SIFIVE_I2C0_10016000_BASE_ADDRESS;
    int ret = -1;

    /* Select i2c0 clock source */
	clock = (struct metal_clock *)&__metal_dt_clock_4.clock;

    if (clock != NULL) {
        long clock_rate = clock->vtable->get_rate_hz(clock);

        /* Calculate prescaler value */
        long prescaler = ((clock_rate / (baud_rate * 5)) - 1); //METAL_SIFIVE_I2C_GET_PRESCALER(baud_rate);

        if ((prescaler > METAL_I2C_PRESCALE_MAX) || (prescaler < 0)) {
            /* Out of range value, return error */
            METAL_I2C_LOG("I2C Set baud failed - baud rate out of range value\r\n");
        } else {
            /* Set pre-scaler value */
            METAL_I2C_REGB(METAL_SIFIVE_I2C0_CONTROL) &= ~METAL_I2C_CONTROL_EN;
            METAL_I2C_REGB(METAL_SIFIVE_I2C0_PRESCALE_LOW) = prescaler & 0xFF;
            METAL_I2C_REGB(METAL_SIFIVE_I2C0_PRESCALE_HIGH) = (prescaler >> 8) & 0xFF;
            METAL_I2C_REGB(METAL_SIFIVE_I2C0_CONTROL) |= METAL_I2C_CONTROL_EN;

            _baud_rate = baud_rate;
            ret = 0;
        }
    } else {
        METAL_I2C_LOG("I2C Set baud failed.\r\n");
    	ret = -1;
    }

    return ret;
}

int driver_i2c0_write(unsigned int addr, unsigned int len, unsigned char buf[], int stop_bit) {
    __metal_io_u8 command;
    time_t timeout;
    int ret;
    unsigned long base = METAL_SIFIVE_I2C0_10016000_BASE_ADDRESS;
    unsigned int i;

	/* Send address over I2C bus, current driver supports only 7bit
	 * addressing */
	ret = driver_i2c0_write_addr(base, addr, METAL_I2C_WRITE);

	if (ret != 0) {
		/* Write address failed */
		METAL_I2C_LOG("I2C Address Write failed.\r\n");
	} else {
		/* Set command flags */
		command = METAL_I2C_CMD_WRITE;

		for (i = 0; i < len; i++) {
			/* Copy into transmit register */
			METAL_I2C_REGB(METAL_SIFIVE_I2C0_TRANSMIT) = buf[i];

			/* for last byte transfer, check if stop condition is requested
			 */
			if (i == (len - 1)) {
				command |= METAL_SIFIVE_I2C_INSERT_STOP(stop_bit);
			}
			/* Write command register */
			METAL_I2C_REGB(METAL_SIFIVE_I2C0_COMMAND) = command;
			/* Reset timeout */
			METAL_I2C_TIMEOUT_RESET(timeout);

			/* Check for transfer completion */
			METAL_I2C_REG_CHECK((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
								 METAL_I2C_STATUS_TIP),
								timeout);

			/* Check for ACK from slave */
			if ((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
				 METAL_I2C_STATUS_RXACK)) {
				/* No ACK, return error */
				METAL_I2C_LOG("I2C RX ACK failed.\r\n");
				ret = -1;
				break;
			}
		}
	}

    return ret;
}

int driver_i2c0_read(unsigned int addr, unsigned int len, unsigned char buf[], int stop_bit) {
    int ret;
    __metal_io_u8 command;
    time_t timeout;
    unsigned int i;
    unsigned long base = METAL_SIFIVE_I2C0_10016000_BASE_ADDRESS;


	/* Send address over I2C bus, current driver supports only 7bit
	 * addressing */
	ret = driver_i2c0_write_addr(base, addr, METAL_I2C_READ);

	if (ret != 0) {
		/* Write address failed */
		METAL_I2C_LOG("I2C Read failed.\r\n");
	} else {
		/* Set command flags */
		command = METAL_I2C_CMD_READ;

		for (i = 0; i < len; i++) {
			/* check for last transfer */
			if (i == (len - 1)) {
				/* Set NACK to end read, if requested generate STOP
				 * condition */
				command |= (METAL_I2C_CMD_ACK |
							METAL_SIFIVE_I2C_INSERT_STOP(stop_bit));
			}
			/* Write command register */
			METAL_I2C_REGB(METAL_SIFIVE_I2C0_COMMAND) = command;
			/* Reset timeout */
			METAL_I2C_TIMEOUT_RESET(timeout);

			/* Wait for the read to complete */
			METAL_I2C_REG_CHECK((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
								 METAL_I2C_STATUS_TIP),
								timeout);
			/* Store the received byte */
			buf[i] = METAL_I2C_REGB(METAL_SIFIVE_I2C0_TRANSMIT);
		}
	}

    return ret;
}

int driver_i2c0_transfer(unsigned int addr,
                                    unsigned char txbuf[], unsigned int txlen,
                                    unsigned char rxbuf[], unsigned int rxlen) {
    __metal_io_u8 command;
    time_t timeout;
    int ret;
    unsigned int i;
    unsigned long base = METAL_SIFIVE_I2C0_10016000_BASE_ADDRESS;

        if (txlen) {
            /* Set command flags */
            command = METAL_I2C_CMD_WRITE;
            /* Send address over I2C bus, current driver supports only 7bit
             * addressing */
            ret = driver_i2c0_write_addr(base, addr, METAL_I2C_WRITE);

            if (ret != 0) {
                /* Write address failed */
                METAL_I2C_LOG("I2C Write failed.\r\n");
                return ret;
            }
            for (i = 0; i < txlen; i++) {
                /* Copy into transmit register */
                METAL_I2C_REGB(METAL_SIFIVE_I2C0_TRANSMIT) = txbuf[i];

                if (i == (txlen - 1) && (rxlen == 0)) {
                    /* Insert stop condition to end transfer */
                    command |= METAL_I2C_CMD_STOP;
                }
                /* Write command register */
                METAL_I2C_REGB(METAL_SIFIVE_I2C0_COMMAND) = command;
                /* Reset timeout */
                METAL_I2C_TIMEOUT_RESET(timeout);

                /* Check for transfer completion. */
                METAL_I2C_REG_CHECK((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
                                     METAL_I2C_STATUS_TIP),
                                    timeout);

                /* Check for ACK from slave. */
                if ((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
                     METAL_I2C_STATUS_RXACK)) {
                    /* No ACK, return error */
                    METAL_I2C_LOG("I2C RX ACK failed.\r\n");
                    ret = -1;
                    break;
                }
            }
        }
        if (rxlen) {
            command = METAL_I2C_CMD_READ; /* Set command flags */
            /* Send address over I2C bus, current driver supports only 7bit
             * addressing */
            ret = driver_i2c0_write_addr(base, addr, METAL_I2C_READ);

            if (ret != 0) {
                /* Return error */
                METAL_I2C_LOG("I2C Read failed.\r\n");
                return ret;
            }
            for (i = 0; i < rxlen; i++) {
                /* check for last transfer */
                if (i == (rxlen - 1)) {
                    /* Set NACK to end read, generate STOP condition */
                    command |= (METAL_I2C_CMD_ACK | METAL_I2C_CMD_STOP);
                }
                /* Write command register */
                METAL_I2C_REGB(METAL_SIFIVE_I2C0_COMMAND) = command;
                /* Reset timeout */
                METAL_I2C_TIMEOUT_RESET(timeout);

                /* Wait for the read to complete */
                METAL_I2C_REG_CHECK((METAL_I2C_REGB(METAL_SIFIVE_I2C0_STATUS) &
                                     METAL_I2C_STATUS_TIP),
                                    timeout);
                /* Store the received byte */
                rxbuf[i] = METAL_I2C_REGB(METAL_SIFIVE_I2C0_TRANSMIT);
            }
        }

    return ret;
}
