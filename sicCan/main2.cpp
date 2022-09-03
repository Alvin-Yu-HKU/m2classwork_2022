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
#include <string>

#define CURRENT_LIMIT 1024

// control loop limit for safe
// DO NOT MODIFY UNTIL YOU ARE TOLD TO DO SO !
#define MAX_VEL 4000 // maximum velocity +- 2000 rpm
#define MAX_CUR 2048 // maximum current +-2.5 A
#define MAX_CUR_CHANGE 2048 // limit the change in current
#define MAX_POS 500000 // maximum position 100000 count
#define MIN_POS -500000 // maximum position 100000 count

// For testing max speed achieved by m3508 for pendulum
int max_speed = 0;
int32_t cur_enc;

// control loop parameters
#define P_KP 128
#define P_KD 1024
#define V_KP 256

int s;
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

enum Control_Mode
{
    MODE_CUR, // value = 0
    MODE_VEL, // value = 1
    MODE_POS  // value = 2
};

Control_Mode ctrl_mode;
int32_t ctrl_target;
int16_t iout;
bool enable_feedback = false;

//This part is for initializing socket port
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
        dji_fb.cur = (uint16_t)receivedframe.data[4] << 8 |receivedframe.data[5];;

        // if(output == true){
        //     std::cout << "Enc: " << dji_fb.enc << std::endl;
        //     std::cout << "Rpm: " << dji_fb.rpm << std::endl;
        //     std::cout << "Cur: " << dji_fb.cur << std::endl;
        // }

        // dji_set_current(cur_T);
    }
    return true;
}

//This part is for printing out feedback to users
void print_feedback()
{
    if(enable_feedback == true){
        std::cout<< "Mode: " << ctrl_mode << std::endl;
        std::cout<< "Val: " << ctrl_target << std::endl;
        std::cout<< "iout: " << iout << std::endl;
        std::cout<< "ENC: " << dji_fb.enc << std::endl;
    }
}

//This part is for limiting input val
int32_t constrain(int32_t val,int32_t min,int32_t max){
    if(val > max){
        return max;
    }
    else if(val < min){
        return min;
    }
    else{
        return val;
    }
}

// This function will get the command from the user and update corresbonding configurations
// General tasks of this function: 
void get_command()
{ 
    while(true){
        std::string Input; 
        std::string temp = "";
        int numV = 0;
        bool start = false;
        char cmd;
        int32_t val;

        std::getline(std::cin,Input);
        for(int i = 0; i < Input.length(); i++){
            if(Input[i] == ' '){
                numV++;
                start = true;
            }
            if(start == true){
                temp = temp + Input[i];
            }
        }
        std::cout << Input << std::endl;
        std::cout << numV << std::endl;
        if(numV >= 2 || (Input[0] != 'p' && Input[0] != 'i' && Input[0] != 'v' && Input[0] != 'f')){
            std::cout << "Not valid input" << std::endl;
        }
        else{
            cmd = Input[0];
            if(temp != ""){
                val = stoi(temp);
            }
            std::cout << "cmd: " << cmd << std::endl;
            std::cout << "val: " << val << std::endl;
            switch (cmd)
            {
            case 'i':
                ctrl_mode = MODE_CUR;
                ctrl_target = constrain(val, -MAX_CUR, MAX_CUR);
                break;
            case 'v':
                ctrl_mode = MODE_VEL;
                ctrl_target = constrain(val, -MAX_VEL, MAX_VEL);
                break;
            case 'p':
                ctrl_mode = MODE_POS;
                ctrl_target = constrain(val, MIN_POS, MAX_POS);
                break;
            case 'f':
                if(enable_feedback == true){
                    enable_feedback = false;
                }
                else{
                    enable_feedback = true;
                }
                std::cout << enable_feedback << std::endl;
            default:
                break;
            
            }
        }
    }
}

// This part is for control the motor(eg. position, speed, current)
int32_t control()
{
    // desired output, error of output, change in output error
    int32_t pdes, perr, dperr;
    // desired output, error of output
    int32_t vdes, verr;
    // desired current output
    int32_t ides; 
    // with 'static' keyword, these values will retent after this function returns
    static int32_t prev_perr; // error of position loop in the previous loop cycle
    static int32_t prev_iout; // current output the previous loop cycle

    // DO IT YOURSELF
    // use dji_fb.enc or dji_fb.rpm to calculate required current
    // such that the motor can move in the desired way given by ctrl_mode and ctrl_target
    
    if(ctrl_mode == MODE_CUR){
        // MODE_CUR
        ides = ctrl_target;
    }
    else{
        if(ctrl_mode == MODE_VEL){
            // MODE_VEL
            vdes = ctrl_target;
        }
        else{
            // MODE_POS
            pdes = ctrl_target;
            perr = pdes - dji_fb.enc;
            dperr = perr - prev_perr;
            prev_perr = perr;
            vdes = P_KP * perr + P_KD * dperr;
            vdes /= 128;
            vdes = constrain(vdes, -MAX_VEL, MAX_VEL);
        }

        verr = vdes - dji_fb.rpm;
        ides = V_KP * verr;
        ides /= 128;
        ides = constrain(ides, -MAX_CUR, MAX_CUR);
    }

    prev_iout += constrain(ides-prev_iout, -MAX_CUR_CHANGE, MAX_CUR_CHANGE);
    iout = prev_iout;

    // std::cout << "iout: " << iout << std::endl;
    return iout; // return the calculated current output
}

//This part is for geting a can signal feedback from m3508 and then send back a can signal to m3508 to control it
void dji(){
    while(true){
        if (dji_get_feedback())
        {        
            print_feedback();
            dji_set_current(control());
        }
    }
}

int main()
{
    if(dji_init() == 0){
        return 1;
    }
    //Start threading Thread1: get user input Thread2: send and receive can signal
    std:: thread GetInput(get_command);
    std:: thread Dji(dji);
    GetInput.join();
    Dji.join();
}
