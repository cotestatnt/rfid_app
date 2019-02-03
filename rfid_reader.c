#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <malloc.h>
#include "libusb.h"

#define AUTO_FORMAT 0
#define T5577_FORMAT 1
#define EM4305_FORMAT 2

#define VENDOR_ID 0x6688
#define PRODUCT_ID 0x6850

#define ENDPOINT_IN		0x85
#define ENDPOINT_OUT	0x03

#define MESSAGE_START_MARKER	0x01
#define MESSAGE_END_MARKER		0x04
#define MESSAGE_STRUCTURE_SIZE	5

/* Commands (from computer) */
#define CMD_BUZZER			0x03
#define CMD_EM4100ID_READ	0x10
#define CMD_T5557_BLOCK_WRITE	0x12
#define CMD_EM4305_CMD	0x13

/* Commands (to computer) */
#define CMD_EM4100ID_ANSWER	0x90
#define CMD_T5557_BLOCK_WRITE_ANSWER	0x92
#define CMD_EM4305_CMD_ANSWER	0x93

static int verbose = 0;
static int handle_events = 0;

static int timeout=1000; /* timeout in ms */
static uint8_t answer[48] = {0};


int prepare_message(uint8_t *out_buf, int endpoint, int command, uint8_t *pl_buf, int pl_buf_size) {

	int i,j;
	int x = 0;
	memset(out_buf, 0, 24);
	out_buf[0] = endpoint;
	out_buf[1] = MESSAGE_START_MARKER;
	out_buf[2] = 5 + pl_buf_size;	//size of message
	out_buf[3] = command;
	i = 4;
	if (pl_buf_size > 0) {
		memcpy(&out_buf[i], pl_buf, pl_buf_size);
		i+=pl_buf_size;
	}
	/* Calculate checksum byte */
	for (j=1 ; j<i ; j++) {
		x = x^out_buf[j];
	}
	out_buf[i] = x;
	out_buf[i+1] = MESSAGE_END_MARKER;


    for (i=0 ; i<24 ; i++) {
        if(i%16 == 0)
            if (verbose) fprintf(stdout,"\n");
        if (verbose) fprintf(stdout, "%02x ", out_buf[i]);
    }
    if (verbose) fprintf(stdout, "\n");

};

int send_message(struct libusb_device_handle * devh, uint8_t *message, uint8_t *answer) {
	int r,i;
	int bt = 0;
//usleep(20000);
	r = libusb_interrupt_transfer(devh, ENDPOINT_IN, answer, 48, &bt, timeout);
	r = libusb_interrupt_transfer(devh, ENDPOINT_OUT, message, 24, &bt, timeout);
//	usleep(50000);

    if (verbose) fprintf(stdout, "Answer:\n");
    for (i=0 ; i<48 ; i++) {
        if(i%16 == 0)
            if (verbose) fprintf(stdout,"\n");
        if (verbose) fprintf(stdout, "%02x ", answer[i]);
    }
    if (verbose) fprintf(stdout, "\n");

};

int handle_interrupt_answer(uint8_t *int_buf, int int_buf_size) {
	int i, x = 0;
	int msg_size = 0;
	uint8_t cmd = 0, checksum;

	if (int_buf_size == 48) {
		if (verbose) fprintf(stdout,"valid interrupt buffer size found (48)\n");

		/* parse buffer */
		if (int_buf[0] != 0x05)
			fprintf(stdout,"invalid endpoint value %02x!=5\n",int_buf[0]);

		if (int_buf[1] != MESSAGE_START_MARKER)
			fprintf(stdout,"invalid start marker %02x!=0x01\n",int_buf[1]);

		msg_size = int_buf[2];
		if (msg_size > (48-1))
			fprintf(stdout,"invalid msg size %d\n", msg_size);

		cmd = int_buf[3];

		/* the checksum on received data is not the same as outgoing data
		checksum = int_buf[msg_size-1];

		for (i=0 ; i<5 ; i++)
			x = x^int_buf[i];

		if (checksum != x)
			fprintf(stdout,"checksum missmatch %x!=%x\n",checksum, x);
		*/

		if (int_buf[msg_size] != MESSAGE_END_MARKER)
			fprintf(stdout,"invalid end marker !=0x04\n");

		switch (cmd) {
			case CMD_EM4100ID_ANSWER:
				if (msg_size != 0x06)
//				fprintf(stdout, "%02x%02x%02x%02x%02x\n",int_buf[5],int_buf[6],int_buf[7],int_buf[8],int_buf[9]);
				break;
			default:
				break;
		}
	}
	return 0;
};

void interrupt_cb(struct libusb_transfer *xfr)
{
	int i;
	uint8_t* l_answer;

    switch(xfr->status)
    {
        case LIBUSB_TRANSFER_COMPLETED:
			 handle_events-=1;
			l_answer = xfr->buffer;
			if (verbose) fprintf(stdout, "interrupt transfer actual_length: %d ", xfr->actual_length);
			if (xfr->actual_length == 48) {
				for (i=0 ; i<24 ; i++) {
					if(i%16 == 0)
						if (verbose) fprintf(stdout,"\n");
					if (verbose) fprintf(stdout, "%02x ", answer[i]);
				}
				if (verbose) fprintf(stdout, "\n");

//				fprintf(stdout, "%02x%02x%02x%02x%02x\n",answer[5],answer[6],answer[7],answer[8],answer[9]);
//			handle_interrupt_answer(xfr->buffer, xfr->actual_length);
			if (verbose) fprintf(stdout, "|%x| \n", (unsigned long)xfr->user_data);
			if (xfr->user_data)
				memcpy(xfr->user_data, l_answer, 48);	//only handle 48 byte answers
			} else {
			}
            break;
        case LIBUSB_TRANSFER_CANCELLED:
        case LIBUSB_TRANSFER_NO_DEVICE:
        case LIBUSB_TRANSFER_TIMED_OUT:
        case LIBUSB_TRANSFER_ERROR:
        case LIBUSB_TRANSFER_STALL:
        case LIBUSB_TRANSFER_OVERFLOW:
if (verbose) fprintf(stdout, "transfer error\n");
			handle_events = 0;
            break;
    }
};

int init_protocol(struct libusb_device_handle * devh) {

	// the protocol needs a previous sent interrupt in request
	// and it should not be handled
	// most likely to sync the device buffer somehow
	uint8_t* message;
	struct libusb_transfer *xfr_in;
	message = calloc(1, 48);
	xfr_in = libusb_alloc_transfer(0);
	libusb_fill_interrupt_transfer(xfr_in, devh, ENDPOINT_IN, message, 48,
		                      interrupt_cb, answer, 0);

	if(libusb_submit_transfer(xfr_in) < 0)
		libusb_free_transfer(xfr_in);
	else
		if (verbose) fprintf(stdout, "init succeeded\n");

	usleep(500 *1000);
}

int uninit_protocol(struct libusb_device_handle * devh) {

if (verbose) fprintf(stdout, "uninit_protocol\n");

	while(handle_events) {
		if(libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
		if (verbose) fprintf(stdout, "loop uninit\n");
	}


}

int send_message_async(struct libusb_device_handle * devh, uint8_t *message, uint8_t *answer) {
	int r,i;
	int bt = 0;
	struct libusb_transfer *xfr_out, *xfr_in;
	uint8_t* usb_msg_out = malloc(24);
	uint8_t* usb_msg_in = calloc(1, 48);

	xfr_out = libusb_alloc_transfer(0);
	xfr_in = libusb_alloc_transfer(0);

	memcpy(usb_msg_out, message, 24);
	libusb_fill_interrupt_transfer(xfr_out, devh, ENDPOINT_OUT, usb_msg_out, 24,
		                      interrupt_cb, NULL, timeout);

	libusb_fill_interrupt_transfer(xfr_in, devh, ENDPOINT_IN, usb_msg_in, 48,
		                      interrupt_cb, answer, 0);


	if(libusb_submit_transfer(xfr_out) < 0)
		libusb_free_transfer(xfr_out);
	handle_events = 2;
	usleep(50 * 1000);

	while(handle_events) {
		if(libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
		if (verbose) fprintf(stdout, "event %d handled\n", handle_events);
	}

	usleep(100 * 1000);


	handle_events = 1;
	if(libusb_submit_transfer(xfr_in) < 0)
		libusb_free_transfer(xfr_in);

	usleep(50 * 1000);

	return 0;
};


int send_read_em4100id(struct libusb_device_handle * devh) {
	uint8_t cmd[24] = {0};

	int cmd_answer_size = 0;
	int retry_cnt = 10;

	// flaky read, retry 10 times
	while ((cmd_answer_size < 5) && retry_cnt) {
		prepare_message(cmd, ENDPOINT_OUT, CMD_EM4100ID_READ, NULL, 0);
		send_message_async(devh, cmd, answer);
		handle_interrupt_answer(answer, 48);
		cmd_answer_size = answer[2] - MESSAGE_STRUCTURE_SIZE - 1;
		retry_cnt--;
	}
	if (cmd_answer_size < 5)
		fprintf(stdout, "NOTAG\n");
	else
		fprintf(stdout, "%02X%02X%02X%02X%02X\n",answer[5],answer[6],answer[7],answer[8],answer[9]);

	return 0;
};

int t55xx_reset(struct libusb_device_handle * devh) {
	uint8_t cmd[24] = {0};
	uint8_t answer[48] = {0};
	uint8_t reset[5] = {0x0, 0x0, 0x0, 0x0, 0x0};

	prepare_message(cmd, ENDPOINT_OUT, CMD_T5557_BLOCK_WRITE, reset, 5);
	send_message_async(devh, cmd, answer);

};


int em4100_column_parity(uint8_t* hex_buf, int shift) {
	int i, p=0;

	for (i=0 ; i<5 ; i++) {
		p += (hex_buf[i] >> shift+4) &1;
		p += (hex_buf[i] >> shift) &1;
	}
	return p&1;
}

int hex_to_em4100_layout(uint8_t* hex_buf, uint8_t* out_buf) {
	int p0,p1,p2,p3,p4,p5,p6,p7,p8,p9;
	int pc0,pc1,pc2,pc3;
	uint8_t ep[16] = {0,1,1,0, 1,0,0,1, 1,0,0,1, 0,1,1,0};

	p0 = ep[hex_buf[0]  >> 4];
	p1 = ep[hex_buf[0] & 0xf];
	p2 = ep[hex_buf[1]  >> 4];
	p3 = ep[hex_buf[1] & 0xf];
	p4 = ep[hex_buf[2]  >> 4];
	p5 = ep[hex_buf[2] & 0xf];
	p6 = ep[hex_buf[3]  >> 4];
	p7 = ep[hex_buf[3] & 0xf];
	p8 = ep[hex_buf[4]  >> 4];
	p9 = ep[hex_buf[4] & 0xf];

	pc0 = em4100_column_parity(hex_buf, 3);
	pc1 = em4100_column_parity(hex_buf, 2);
	pc2 = em4100_column_parity(hex_buf, 1);
	pc3 = em4100_column_parity(hex_buf, 0);

	out_buf[0] = 0xff;
	out_buf[1] = 0x80 | ((hex_buf[0]>>1)&0x78) | (p0<<2) | ((hex_buf[0]>>2)&0x03);
	out_buf[2] = (hex_buf[0]<<6) | (p1<<5) | ((hex_buf[1]>>3)&30) | p2;
	out_buf[3] = (hex_buf[1]<<4) | (p3<<3) | (hex_buf[2]>>5);
	out_buf[4] = ((hex_buf[2]<<3)&0x80) | (p4<<6) | ((hex_buf[2]<<2)&0x3c) | (p5<<1) | (hex_buf[3]>>7);
	out_buf[5] = ((hex_buf[3]<<1)&0xe0) | (p6<<4) | (hex_buf[3]&0xf);
	out_buf[6] = (p7<<7) | ((hex_buf[4]>>1)&0x78) | (p8<<2) | ((hex_buf[4]>>2)&0x03);
	out_buf[7] = (hex_buf[4]<<6) | (p9<<5) | (pc0<<4) | (pc1<<3) | (pc2<<2) | (pc3<<1);
};




int em4305_login(struct libusb_device_handle * devh) {
	uint8_t cmd[24] = {0};
	uint8_t answer[48] = {0};
	uint8_t login[7] = {0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

	prepare_message(cmd, ENDPOINT_OUT, CMD_EM4305_CMD, login, 7);
	send_message_async(devh, cmd, answer);
}


int send_buzzer(struct libusb_device_handle * devh, uint8_t duration) {
	uint8_t cmd[48] = {0};
	uint8_t answer[48] = {0};
	uint8_t buzz[1] = {9};
	buzz[1] = duration;
	prepare_message(cmd, ENDPOINT_OUT, CMD_BUZZER, &buzz[0], 1);
	send_message_async(devh, cmd, answer);
};

int hex_string_to_bytes(uint8_t *hex_string, uint8_t *byte_array) {
	uint8_t* c1;
	int i,idx=0;
	unsigned int bytes[2];
	for(i=0 ; i<10 ; i+=2) {
	    c1 = &hex_string[i];
	    sscanf(c1, "%02x", &bytes[0]);
	    byte_array[idx] = bytes[0];
	    idx++;
	}
}

int main(int argc,char** argv)
{
    int r = 1, i;
    struct libusb_device_handle *devh = NULL;

    int option = 0;
    int read_device = 0;
	int format = AUTO_FORMAT;
	int buzzer = 0;
    int semicolon=0, questionmark=0, split=0, enter=0;

    while ((option = getopt(argc, argv,"w:vrb:sqlef:")) != -1) {
        switch (option) {
            case 'v' : verbose = 1;
                break;
            case 'r' : read_device = 1;
                break;
            case 'b' : buzzer = optarg[0]+'0';
                break;
            case 's' : semicolon = 1;
                break;
            case 'q' : questionmark = 1;
                break;
            case 'l' : split = 1;
                break;
            case 'e' : enter = 1;
                break;
			case 'f' : format = atoi(optarg);
				break;
            default: ;/*print_usage()*/;
                 exit(EXIT_FAILURE);
        }
    }

    if (argc < 2) {
        //print_usage();
        goto exit;
    }

    if (verbose) fprintf(stdout, "Init usb\n");

    /* Init USB */
    r = libusb_init(NULL);
    if (r < 0) {
        fprintf(stderr, "Failed to initialise libusb\n");
        exit(1);
    }

    libusb_device **devs;
    ssize_t n = libusb_get_device_list(NULL, &devs);

    int num_dev = 0;
    for(int i=0; i<n; i++){
        struct libusb_device_descriptor desc;
        r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) {
            fprintf(stderr, "failed to get device descriptor");
        }
        if(desc.idVendor == 0x6688){
            num_dev++;
            libusb_open(devs[i], &devh);
        }
    }

    if (verbose) fprintf(stdout, "Found %d readers\n", num_dev);


    //devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);

    if (!devh) {
        if (verbose) fprintf(stdout, "USB device open failed\n");
        goto out;
    }
    if (verbose) fprintf(stdout, "Successfully found the RFID R/W device\n");

    r = libusb_detach_kernel_driver(devh, 0);
    if (r < 0 && r != LIBUSB_ERROR_NOT_FOUND && r != LIBUSB_ERROR_NOT_SUPPORTED) {
        fprintf(stderr, "libusb_detach_kernel_driver error %d\n", r);
        goto out;
    }


    r = libusb_claim_interface(devh, 0);
    if (r < 0) {
        fprintf(stderr, "libusb_claim_interface error %d\n", r);
        goto out;
    }

	init_protocol(devh);

    if (buzzer) {
		send_buzzer(devh, 9);
    }


    if (read_device) {
        send_read_em4100id(devh);
    }


if (verbose) fprintf(stdout, "uninit\n");

//	uninit_protocol(devh);
//	send_buzzer(devh);
    libusb_release_interface(devh, 0);
out:
    libusb_free_device_list(devs, 1);
    //	libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(NULL);
exit:
    return r >= 0 ? r : -r;
};
