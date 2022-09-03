#include "dji.h"
#include <string>
#include <iostream>
#include <thread>
#include <pthread.h>

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


// Pendulum's PID
// #define kP 4.5
// #define kD 8.5
// #define multiplier 4

// Create a encoder object
// It will be used to get the position of the rod when operating pendulum
// Encoder myEnc(3, 4);

enum Control_Mode
{
    MODE_CUR, // value = 0
    MODE_VEL, // value = 1
    MODE_POS  // value = 2
};

Control_Mode ctrl_mode;
int32_t ctrl_target;
int16_t iout;
bool enable_feedback = false;   // initially is 0 means that we do not accept feedback at the beginning
// bool enable_balance = 0;
// This function will print out the feedback on the serial monitor
void print_feedback()
{
    if(enable_feedback == true){
        std::cout<< "Mode: " << ctrl_mode << std::endl;
        std::cout<< "Val: " << ctrl_target << std::endl;
        std::cout<< "iout: " << iout << std::endl;
        std::cout<< "ENC: " << dji_fb.enc << std::endl;
    }
}

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
// 1. Read from user input
// 2. Update ctrl_mode, ctrl_target, enable_feedback
void get_command()
{ 
    while(true){
        std::string Input; 
        std::string temp = "";
        int numV = 0;
        bool start = false;
        char cmd;
        int32_t val;
        // void *ptr;

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
            // Task 5b.3 - position control loop (do it the last)
            // Steps to follow:
            // 1. Find perr, the error between desired position (pdes) and actual position (dji_fb.enc)
            // 2. Find dperr, the difference between the error calculated in current loop cycle (perr)
            //    and the previous loop cycle (prev_perr)
            // 3. Calculate vdes using this formula: (P_KP*perr + P_KD*dperr)/128
            // 4. Update prev_perr with perr
            // 5. Limit vdes with MAX_VEL using constrain()
            // TYPE YOUR CODE HERE:
            perr = pdes - dji_fb.enc;
            dperr = perr - prev_perr;
            prev_perr = perr;
            vdes = P_KP * perr + P_KD * dperr;
            vdes /= 128;
            vdes = constrain(vdes, -MAX_VEL, MAX_VEL);
        }

        // MODE_VEL and MODE_POS will go through this:
        // Task 5b.2 - velocity control loop
        // 1. find verr, the error between desired velocity (vdes) and actual velocity (dji_fb.rpm)
        // 2. calculate ides with V_KP * verr / 128
        // 3. limit ides with MAX_CUR using constrain()
        // TYPE YOUR CODE HERE:
        verr = vdes - dji_fb.rpm;
        ides = V_KP * verr;
        ides /= 128;
        ides = constrain(ides, -MAX_CUR, MAX_CUR);
    }

    // Task 5b.1 - limit the change in current (do it first)
    // 1. find the difference between the desired current output(ides) and previous current output (prev_iout)
    // 2. limit the difference with MAX_CUR_CHANGE using constrain()
    // 3. apply the limited change to the previous current output (prev_iout)
    // 4. prev_iout is now the calculated current, assign it to iout
    // TYPE YOUR CODE HERE:
    prev_iout += constrain(ides-prev_iout, -MAX_CUR_CHANGE, MAX_CUR_CHANGE);
    iout = prev_iout;

    // std::cout << "iout: " << iout << std::endl;
    return iout; // return the calculated current output
}


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
    // Do not change anything here
    startThread(&get_command,&dji);
}
