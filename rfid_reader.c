#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <malloc.h>
#include "libusb.h"

#define AUTO_FORMAT     0
#define T5577_FORMAT    1
#define EM4305_FORMAT   2

#define VENDOR_ID       0x6688
#define PRODUCT_ID      0x6850

#define ENDPOINT_IN		0x85
#define ENDPOINT_OUT	0x03

#define MESSAGE_START_MARKER	0x01
#define MESSAGE_END_MARKER		0x04
#define MESSAGE_STRUCTURE_SIZE	5

/* Commands (from computer) */
#define CMD_BUZZER              0x03
#define CMD_EM4100ID_READ       0x10
#define CMD_EM4305_CMD          0x13

/* Commands (to computer) */
#define CMD_EM4100ID_ANSWER	    0x90
#define CMD_EM4305_CMD_ANSWER	0x93

static int verbose = 0;
static int handle_events = 0;

/* timeout in ms */
static int timeout=1000;        
static uint8_t answer[48] = {0};

void prepare_message(uint8_t *out_buf, int endpoint, int command, uint8_t *pl_buf, int pl_buf_size) {
    int i,j;
    int x = 0;
    memset(out_buf, 0, 24);
    out_buf[0] = endpoint;
    out_buf[1] = MESSAGE_START_MARKER;
    out_buf[2] = 5 + pl_buf_size;   //size of message
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

}

void send_message(struct libusb_device_handle * devh, uint8_t *message, uint8_t *answer) {
    int r,i;
    int bt = 0;

    r = libusb_interrupt_transfer(devh, ENDPOINT_IN, answer, 48, &bt, timeout);
    r = libusb_interrupt_transfer(devh, ENDPOINT_OUT, message, 24, &bt, timeout);

    if (verbose) fprintf(stdout, "Answer:\n");
    for (i=0 ; i<48 ; i++) {
        if(i%16 == 0)
            if (verbose) fprintf(stdout,"\n");
        if (verbose) fprintf(stdout, "%02x ", answer[i]);
    }
    if (verbose) fprintf(stdout, "\n");

}

void handle_interrupt_answer(uint8_t *int_buf, int int_buf_size) {
    int msg_size = 0;
    uint8_t cmd = 0;

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
                    if (verbose) fprintf(stdout, "%02x%02x%02x%02x%02x\n",int_buf[5],int_buf[6],int_buf[7],int_buf[8],int_buf[9]);
                break;
            default:
                break;
        }
    }    
}

void interrupt_cb(struct libusb_transfer *xfr){
    int i;
    uint8_t* l_answer;

    switch(xfr->status)    {
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
                handle_interrupt_answer(xfr->buffer, xfr->actual_length);
                if (verbose) fprintf(stdout, "|%x| \n", (unsigned long)xfr->user_data);
                if (xfr->user_data)
                    memcpy(xfr->user_data, l_answer, 48);   //only handle 48 byte answers
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
}

void init_protocol(struct libusb_device_handle * devh) {

    // the protocol needs a previous sent interrupt in request
    // and it should not be handled
    // most likely to sync the device buffer somehow
    uint8_t* message;
    struct libusb_transfer *xfr_in;
    message = calloc(1, 48);
    xfr_in = libusb_alloc_transfer(0);
    libusb_fill_interrupt_transfer(xfr_in, devh, ENDPOINT_IN, message, 48, interrupt_cb, answer, 0);

    if(libusb_submit_transfer(xfr_in) < 0)
        libusb_free_transfer(xfr_in);
    else
        if (verbose) fprintf(stdout, "init succeeded\n");

    //usleep(500 *1000);
    
}

void uninit_protocol(void) {
    if (verbose) fprintf(stdout, "uninit_protocol\n");
    while(handle_events) {
        if(libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
        if (verbose) fprintf(stdout, "loop uninit\n");
    }
}

void send_message_async(struct libusb_device_handle * devh, uint8_t *message, uint8_t *answer) {		
    struct libusb_transfer *xfr_out, *xfr_in;
    uint8_t* usb_msg_out = malloc(24);
    uint8_t* usb_msg_in = calloc(1, 48);

    xfr_out = libusb_alloc_transfer(0);
    xfr_in = libusb_alloc_transfer(0);

    memcpy(usb_msg_out, message, 24);
    libusb_fill_interrupt_transfer(xfr_out, devh, ENDPOINT_OUT, usb_msg_out, 24, interrupt_cb, NULL, timeout);
    libusb_fill_interrupt_transfer(xfr_in, devh, ENDPOINT_IN, usb_msg_in, 48, interrupt_cb, answer, 0);

    if(libusb_submit_transfer(xfr_out) < 0)
        libusb_free_transfer(xfr_out);
    handle_events = 2;
    //usleep(50 * 1000);

    while(handle_events) {
        if(libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
        if (verbose) fprintf(stdout, "event %d handled\n", handle_events);
    }

    //usleep(100 * 1000);

    handle_events = 1;
    if(libusb_submit_transfer(xfr_in) < 0)
        libusb_free_transfer(xfr_in);

    //usleep(50 * 1000);    
}


void send_read_em4100id(struct libusb_device_handle * devh) {
    uint8_t cmd[24] = {0};
    int cmd_answer_size = 0;
    int retry_cnt = 10;   // flaky read, retry 10 times
    
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
}


void send_buzzer(struct libusb_device_handle * devh) {
    uint8_t cmd[48] = {0};
    uint8_t answer[48] = {0};
    uint8_t duration = 9;
    prepare_message(cmd, ENDPOINT_OUT, CMD_BUZZER, &duration, 1);
    send_message_async(devh, cmd, answer);
}


int main(int argc, char** argv) {
    int r = 1;
    int option = 0;
    int read_device = 0;	
	int buzzer = 0;    

    while ((option = getopt(argc, argv,"vrb")) != -1) {
        switch (option) {
            case 'v' : 
                verbose = 1;
                break;
            case 'r' : 
                read_device = 1;
                break;
            case 'b' : 
                buzzer = 1;
                break;            
            default: read_device = 1;
                break;
        }
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
    
    struct libusb_device_handle *reader1 = NULL;
    struct libusb_device_handle *reader2 = NULL;

    int num_dev = 0;
    for(int i=0; i<n; i++){
        struct libusb_device_descriptor desc;
        r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) {
            fprintf(stderr, "failed to get device descriptor");
        }
        if(desc.idVendor == 0x6688){
            num_dev++;
            libusb_open(devs[i], &reader1);
        }
    }

    if (verbose) fprintf(stdout, "Found %d readers\n", num_dev);

    if (!reader1) {
        if (verbose) fprintf(stdout, "USB device open failed\n");
        goto out;
    }
    if (verbose) fprintf(stdout, "Successfully found the RFID R/W device\n");

    r = libusb_detach_kernel_driver(reader1, 0);
    if (r < 0 && r != LIBUSB_ERROR_NOT_FOUND && r != LIBUSB_ERROR_NOT_SUPPORTED) {
        fprintf(stderr, "libusb_detach_kernel_driver error %d\n", r);
        goto out;
    }


    r = libusb_claim_interface(reader1, 0);
    if (r < 0) {
        fprintf(stderr, "libusb_claim_interface error %d\n", r);
        goto out;
    }

    init_protocol(reader1);

    if (read_device) {
        send_read_em4100id(reader1);
    }
    
    if (buzzer) {
        send_buzzer(reader1);
    }

    if (verbose) fprintf(stdout, "uninit\n");

    libusb_release_interface(reader1, 0);
out:
    libusb_free_device_list(devs, 1);    
    libusb_close(reader1);
    libusb_exit(NULL);

}
