/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) Dr Silvio Cesare of InfoSect
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/fcntl.h>
#include <sys/time.h>
#include "flash.h"
#include "programmer.h"
#include "spi.h"
#include "busside_spi.h"

static unsigned char bs_sync_word[] = "\xfe\xca";
static uint32_t bs_sequence_number;
static char *dev;

struct busside_spispeeds {
        const char *name;
        const int speed;
	const int spispeed;
};

static const struct busside_spispeeds spispeeds[] = {
        {"30k",         0x0,	30000	},
        {"125k",        0x1, 	125000	},
        {"250k",        0x2,	250000	},
        {"1M",          0x3,	1000000	},
        {"2M",          0x4,	2000000	},
        {"2.6M",        0x5,	2600000	},
        {"4M",          0x6,	4000000	},
        {"8M",          0x7,	8000000	},
        {NULL,          0x0,	0	},
};

int spispeed = 0x7;

static int busside_get_spispeed();
static int bs_sync(int timeout);

static int busside_serialport_setup()
{
        /* 500000bps, 8 databits, no parity, 1 stopbit */
        sp_fd = sp_openserport(dev, 500000);
        if (sp_fd == SER_INV_FD)
                return 1;
        return 0;
}

static uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

static void
bs_next_sequence_number()
{
	int fd;

	bs_sequence_number++;
	fd = open(BS_SEQ_FILE, O_RDWR);
	write(fd, &bs_sequence_number, sizeof(bs_sequence_number));
	close(fd);
}

static unsigned int
crc_update(unsigned int crc, unsigned char data)
{
    unsigned char tbl_idx;

    tbl_idx = crc ^ (data >> (0 * 4));
    crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = crc_table[tbl_idx & 0x0f] ^ (crc >> 4);
    return crc;
}

static uint32_t
crc_mem(char *s, int n)
{
  unsigned int crc = ~0L;

  for (int i = 0; i < n; i++)
    crc = crc_update(crc, s[i]);
  crc = ~crc;
  return crc;
}

static int
busside_echo(int nretries, int timeout)
{
	struct bs_frame_s *request;
	struct bs_frame_s *reply;
	unsigned int i;

	request = (struct bs_frame_s *)malloc(BS_HEADER_SIZE);
	if (request == NULL) {
		return ERROR_OOM;
	}
	reply = (struct bs_frame_s *)malloc(BS_HEADER_SIZE);
	if (reply == NULL) {
		free(request);
		return ERROR_OOM;
	}
	request->bs_command = BUSSIDE_SPI_ECHO;
	request->bs_payload_length = 0;
	for (i = 0; i < nretries; i++) {
		unsigned int n;
		int rv;
		uint32_t checksum;

		request->bs_sequence_number = bs_sequence_number;
		bs_next_sequence_number();
		request->bs_checksum = 0;

		request->bs_checksum = crc_mem((char *)request, BS_HEADER_SIZE);
		serialport_write(bs_sync_word, 2);
		serialport_write((unsigned char *)request, BS_HEADER_SIZE);
		if (bs_sync(3) != 0)
			goto err;
		rv = serialport_read_nonblock((unsigned char *)reply, BS_HEADER_SIZE, timeout, &n);
		if (rv != 0 || n != BS_HEADER_SIZE)
			goto err;
		if (reply->bs_payload_length != 0)
			goto err;
		if (reply->bs_sequence_number != request->bs_sequence_number)
			goto err;
		checksum = reply->bs_checksum;
		reply->bs_checksum = 0;
		if (crc_mem((char *)reply, BS_HEADER_SIZE) != checksum)
			goto err;
		break;
err:
		printf("Retrying %d/%d\n", i, nretries);
		usleep(5000);
		sp_flush_incoming();
	}
	if (i == nretries) {
		free(request);
		free(reply);
		return -1;
	}
	free(request);
	free(reply);
	fflush(stdout);
	return 0;

}

int
busside_requestreply(int nretries, int timeout, unsigned int writecnt, unsigned int readcnt, const unsigned char *writearr, unsigned char *readarr)
{
	struct bs_frame_s *request;
	struct bs_frame_s *reply;
	unsigned int i;
	uint32_t *request_args;
	uint32_t request_size;

	request_size = BS_HEADER_SIZE + 7*sizeof(uint32_t) + writecnt;
	request = (struct bs_frame_s *)malloc(request_size);
	if (request == NULL) {
		return ERROR_OOM;
	}
	reply = (struct bs_frame_s *)malloc(BS_HEADER_SIZE + readcnt);
	if (reply == NULL) {
		free(request);
		return ERROR_OOM;
	}
	request->bs_command = BUSSIDE_SPI_COMMAND;
	request->bs_payload_length = request_size - BS_HEADER_SIZE;
	request_args = (uint32_t *)&request->bs_payload[0];
	request_args[0] = spispeeds[spispeed].spispeed;
	request_args[1] = 9;
	request_args[2] = 6;
	request_args[3] = 8;
	request_args[4] = 7;
	request_args[5] = writecnt;
	request_args[6] = readcnt;
	memcpy(&request_args[7], writearr, writecnt);
	for (i = 0; i < nretries; i++) {
		unsigned int n;
		int rv;
		uint32_t checksum;

		request->bs_sequence_number = bs_sequence_number;
		bs_next_sequence_number();
		request->bs_checksum = 0;

		request->bs_checksum = crc_mem((char *)request, request_size);
		serialport_write(bs_sync_word, 2);
		serialport_write((unsigned char *)request, request_size);
		if (bs_sync(3) != 0)
			goto err;
		rv = serialport_read_nonblock((unsigned char *)reply, BS_HEADER_SIZE, timeout, &n);
		if (rv != 0 || n != BS_HEADER_SIZE)
			goto err;
		if (reply->bs_payload_length != readcnt)
			goto err;
		rv = serialport_read_nonblock((unsigned char *)&reply->bs_payload[0], readcnt, timeout, &n);
		if (rv != 0 || n != readcnt)
			goto err;
		if (reply->bs_sequence_number != request->bs_sequence_number)
			goto err;
		checksum = reply->bs_checksum;
		reply->bs_checksum = 0;
		if (crc_mem((char *)reply, BS_HEADER_SIZE + reply->bs_payload_length) != checksum)
			goto err;
		break;
err:
		printf("Retrying %d/%d\n", i, nretries);
		usleep(5000);
		sp_flush_incoming();
	}
	if (i == nretries) {
		free(request);
		free(reply);
		return -1;
	}
	memcpy(readarr, reply->bs_payload, readcnt);
	free(request);
	free(reply);
	return 0;
}
static int busside_spi_send_command(struct flashctx *flash,
				    unsigned int writecnt, unsigned int readcnt,
				    const unsigned char *writearr,
				    unsigned char *readarr);

static const struct spi_master spi_master_busside = {
	.type		= SPI_CONTROLLER_BUSSIDE,
	.max_data_read	= 2048,
	.max_data_write	= 2048,
	.command	= busside_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= default_spi_read,
	.write_256	= default_spi_write_256,
	.write_aai	= default_spi_write_aai,
};

static int busside_spi_shutdown(void *data)
{
	serialport_shutdown(NULL);
	free(dev);
	return 0;
}

static int busside_get_spispeed()
{
	int speed = 0x7;
	char *tmp;
	int i;

        tmp = extract_programmer_param("spispeed");
        if (tmp) {
                for (i = 0; spispeeds[i].name; i++) {
                        if (!strncasecmp(spispeeds[i].name, tmp, strlen(spispeeds[i].name))) {
                                speed = spispeeds[i].speed;
                                break;
                        }
                }
                if (!spispeeds[i].name)
                        msg_perr("Invalid SPI speed, using default.\n");
        }
        free(tmp);
	return speed;
}

static int
bs_sync(int timeout)
{
	struct timeval tv_start, tv_current, tv_diff;

	gettimeofday(&tv_start, NULL);
	while (1) {
		unsigned char ch;
		unsigned int n;

		gettimeofday(&tv_current, NULL);
		timersub(&tv_current, &tv_start, &tv_diff);
		if (tv_diff.tv_sec >= timeout)
			return -1;
		if (serialport_read_nonblock(&ch, 1, 1000, &n) != 0)
			continue;
		if (n != 1)
			continue;
		if (ch == 0xfe) {
got1:
			if (serialport_read_nonblock(&ch, 1, 1000, &n) != 0)
				continue;
			if (n != 1)
				continue;
			if (ch == 0xca)
				return 0;
			else if (ch == 0xfe)
				goto got1;
		}
	}
}

static int
bs_sequence_number_init()
{
	int fd;
	ssize_t n;

	fd = open(BS_SEQ_FILE, O_RDWR | O_CREAT);
	if (fd == -1) {
		return -1;
	}
	n = read(fd, &bs_sequence_number, sizeof(bs_sequence_number));
	if (n != sizeof(bs_sequence_number)) {
		bs_sequence_number = 2;
		write(fd, &bs_sequence_number, sizeof(bs_sequence_number));
	}
	close(fd);
	return 0;
}

int busside_spi_init(void)
{
	int rv;

	if (bs_sequence_number_init() == -1) {
		return -1;
	}
	dev = extract_programmer_param("dev");
	if (dev && !strlen(dev)) {
		free(dev);
		dev = NULL;
	}
        if (!dev) {
                msg_perr("No serial device given. Use flashrom -p buspirate_spi:dev=/dev/ttyUSB0\n");
                return 1;
        }
	spispeed = busside_get_spispeed();
	rv = busside_serialport_setup();
	if (rv) {
		return rv;
        }
	if (busside_echo(10, 2000) < 0) {
		return -1;
	}
        if (register_shutdown(busside_spi_shutdown, NULL) != 0) {
                return 1;
        }
	register_spi_master(&spi_master_busside);
	return 0;
}

static int busside_spi_send_command(struct flashctx *flash,
				    unsigned int writecnt, unsigned int readcnt,
				    const unsigned char *writearr,
				    unsigned char *readarr)
{
	return busside_requestreply(10, 2000, writecnt, readcnt, writearr, readarr);
}
