#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <algorithm> 
#include <thread>


#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

 
#ifdef MAX_CUR
#define CURRENT_LIMIT MAX_CUR
#endif

#ifndef MAX_CUR
#define CURRENT_LIMIT 1024
#endif

int s;
// int32_t cur_T = 0;
// bool output = true;
struct can_frame sendframe;
struct can_frame receivedframe;


typedef struct{
  int32_t enc; 
  int16_t rpm;
  int16_t cur;
  uint8_t tmp;
} DJI_Feedback;

typedef struct{
  uint16_t enc; // rotor encoder count
  int16_t rpm; // rotor rpm
  int16_t cur; // actual current output
  uint8_t tmp; // temperature
} DJI_Raw_Feedback;

DJI_Raw_Feedback dji_raw;
DJI_Feedback dji_fb;

int dji_init(){
    struct sockaddr_can addr;
	struct ifreq ifr;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 0;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;


	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

    return 1;
}

void dji_set_current(int32_t cur){
    // limit current
    cur = std::min(CURRENT_LIMIT, cur);
    cur = std::max(-CURRENT_LIMIT, cur);
    // cur = cur & 0xFFFF;
    // cur_T = cur;
    // assemble control message
    sendframe.can_id = 0x200;
    sendframe.can_dlc = 8;
    sendframe.data[2] = cur >> 8;
    sendframe.data[3] = cur & 0xff;
    // std::cout << sendframe.data[2] << std::endl;
    // std::cout << sendframe.data[3] << std::endl;
    if (write(s, &sendframe, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return;
    }
    return;
}

bool dji_get_feedback(){
    int nbytes;
    receivedframe.can_id = 0x202;
	receivedframe.can_dlc = 8;
    nbytes = read(s, &receivedframe, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("Read");
        std::cout << "false" << std::endl;
        return false;
    }
    else{
        static uint16_t prev_enc = 0;
        memcpy(&dji_raw, receivedframe.data, sizeof(dji_raw));
        ///////////////////////////////////////////////////////////////////////
        // handle encoder multiturn
        uint16_t raw_enc = (dji_raw.enc >> 8) | ((dji_raw.enc & 0xFF) << 8);
        int16_t dpos = (8192 + raw_enc - prev_enc) % 8192; // 8192+0-8191
        if (dpos > 4096)
            dpos = dpos - 8192;
        dji_fb.enc += dpos;
        prev_enc = raw_enc;
        ///////////////////////////////////////////////////////////////////////
            
        /////////////////////////////////////////////////////////////////////// 
        // handle rpm, current endian
        ///////////////////////////////////////////////////////////////////////
        dji_fb.rpm = (uint16_t)receivedframe.data[2] << 8 |receivedframe.data[3];
        dji_fb.cur = (uint16_t)receivedframe.data[4] << 8 |receivedframe.data[5];

    }
    return true;
}

void startThread(void (*get_command)(),void (*dji)()){
    std:: thread GetInput(get_command);
    std:: thread Dji(dji);
    GetInput.join();
    Dji.join();
}