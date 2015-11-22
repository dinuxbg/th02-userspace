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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>

#include "th02.h"

int main(int argc, char *argv[])
{
	int st, rh, t_c, fd, i2c_bus_id;
	bool heater;

	/* TODO - set via getopt */
	(void)argc; (void)argv;
	heater = false;
	i2c_bus_id = 2;

	fd = th02_i2c_setup(i2c_bus_id);
	if (fd < 0) {
		perror("could not initialize TH02 I2C interface");
		exit(EXIT_FAILURE);
	}
	st = th02_measure(fd, heater, &rh, &t_c);
	if (st) {
		fprintf(stderr, "ERROR: could not perform a measurement!\n");
		exit(EXIT_FAILURE);
	}
	if (0) {
		printf("RH=%d%%, T=%d Celsius\n", rh, t_c);
	} else {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		printf("%lld, %d%%, %d\n", (long long)tv.tv_sec, rh, t_c);
	}
	th02_i2c_close(fd);

	return EXIT_SUCCESS;
}

