#ifndef _busside_h
#define _busside_h

#include <sys/types.h>

struct bs_frame_s {
	uint32_t bs_command;
	uint32_t bs_payload_length;
	uint32_t bs_sequence_number;
	uint32_t bs_checksum;
	uint32_t bs_payload[0];
} __attribute__((packed));

#define bs_request_s bs_frame_s
#define bs_reply_s bs_frame_s

#define BS_SEQ_FILE		"/tmp/busside.seq"
#define BS_HEADER_SIZE		(4*4)
#define BUSSIDE_SPI_COMMAND	43
#define BUSSIDE_SPI_ECHO	0

int busside_requestreply(int nretries, int timeout, unsigned int writecnt, unsigned int readcnt, const unsigned char *writearr, unsigned char *readarr);


#endif
