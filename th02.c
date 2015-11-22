/* Copyright (c) 2015, Dimitar Dimitrov
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "th02.h"

enum th02_regid {
	TH02_STATUS = 0,
	TH02_DATA_H = 1,
	TH02_DATA_L = 2,
	TH02_CONFIG = 3,
	TH02_ID     = 17,
};

#define STATUS_NRDY_BIT	0

#define MAX_RETRIES	10
#define TH02_I2C_ADDR	0x40

static int th02_write_reg(int fd, enum th02_regid regn, uint8_t val)
{
	uint8_t buf[2] = { regn, val };
	struct i2c_msg msgs[1] = {
		{
			.addr = TH02_I2C_ADDR,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};
	struct i2c_rdwr_ioctl_data msgset = {
		.nmsgs = 1,
		.msgs = msgs,
	};
	int st;

	st = ioctl(fd, I2C_RDWR, &msgset);
	if (st != 1)
		perror("ioctl failed");

	return 0;
}
static int th02_read_reg(int fd, enum th02_regid regn, uint8_t *val)
{
	uint8_t regn_byte = regn;
	struct i2c_msg msgs[2] = {
		{
			.addr = TH02_I2C_ADDR,
			.flags = 0,
			.len = 1,
			.buf = &regn_byte,
		},
		{
			.addr = TH02_I2C_ADDR,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	struct i2c_rdwr_ioctl_data msgset = {
		.nmsgs = 2,
		.msgs = msgs,
	};
	int st;

	st = ioctl(fd, I2C_RDWR, &msgset);
	if (st != 2)
		perror("ioctl failed");

	return 0;
}

static int th02_run(int fd, bool temperature, bool heater, int *val)
{
	int st, retry;
	uint8_t bh, bl, status;

	/* set CONFIG */
	st = th02_write_reg(fd, TH02_CONFIG,
			(int)temperature << 4 | (int)heater << 1 | 1);
	if (st)
		return st;

	/* wait for /RDY */
	retry = 0;
	do {
		st = th02_read_reg(fd, TH02_STATUS, &status);
		if (st)
			return st;
		if (retry)
			usleep(20 * 1000);
		if (retry++ > MAX_RETRIES)
			return -ETIMEDOUT;
	} while (status & (1u << STATUS_NRDY_BIT));

	/* read 16bit raw data */
	st = th02_read_reg(fd, TH02_DATA_H, &bh);
	if (st)
		return st;
	st = th02_read_reg(fd, TH02_DATA_L, &bl);

	*val = (int)bh << 8 | bl;

	return st;
}

int th02_i2c_setup(const int busid)
{
	int fd;
	char filename[256];

	snprintf(filename, sizeof(filename) - 1, "/dev/i2c-%d", busid);
	fd = open(filename, O_RDWR);
	if (fd < 0) {
		perror("could not open i2c adapter");
		return -errno;
	}

	return fd;
}

void th02_i2c_close(int fd)
{
	close(fd);
}


int th02_measure(int fd, bool heater, int *rh, int *t_c)
{
	const double A0 = -4.7843;
	const double A1 = 0.4008;
	const double A2 = -0.00393;
	const double Q0 = 0.1973;
	const double Q1 = 0.00237;
	int st;
	double RH;

	if (0) {
		uint8_t b;
		st = th02_read_reg(fd, TH02_ID, &b);
		printf("id=%02x (st=%d)\n", b, st);
	}
	st = th02_run(fd, true, heater, t_c);
	if (st)
		return st;

	/* printf("raw t_c=%04x\n", *t_c); */
	*t_c >>= 2;
	*t_c = ((double)*t_c / 32.0) - 50.0;

	st = th02_run(fd, false, heater, rh);
	if (st)
		return st;

	/* printf("raw rh=%04x\n", *rh); */
	*rh >>= 4;
	RH = (*rh / 16) - 24;

	/* linearization */
	RH = RH - (RH * RH * A2 + RH * A1 + A0);
	/* temperature compensation */
	RH = RH + (*t_c - 30) * (RH * Q1 + Q0);

	*rh = round(RH);

	return 0;
}

