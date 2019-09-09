#include "sim.h"
#include <unistd.h>
#include <thread>
#include <random>
#include <iostream>
using namespace std;
#define Kp 20
#define Ki 0
#define Kd 0

float pitch_i = 0;
float roll_i = 0;
float pitch_error_before;
float roll_error_before;
void PID_controler_pitch(float input, float& output)
{
    float error = 0 - input;
    pitch_i += error * 0.001;
    if(pitch_i > 30)
        pitch_i = 30;
    if(pitch_i < -30)
        pitch_i = -30;
    output = Kp * error + Ki * pitch_i + Kd * (error - pitch_error_before) * 1000;
    pitch_error_before = error;
}
void PID_controler_roll(float input, float& output)
{
    float error = 0 - input;
    roll_i += error * 0.001;
    if(roll_i > 30)
        roll_i = 30;
    if(roll_i < -30)
        roll_i = -30;
    output = Kp * error + Ki * pitch_i + Kd * (error - roll_error_before) * 1000;
    roll_error_before = error;
}
void controler(float _pitch, float _roll, float _yaw, unsigned int& motor_1, unsigned int& motor_2, unsigned int& motor_3, unsigned int& motor_4)
{
    float pitch_output;
    PID_controler_pitch(_pitch, pitch_output);
    float roll_output;
    PID_controler_roll(_roll, roll_output);
    motor_3 += pitch_output;
    motor_4 += pitch_output;
    motor_1 -= pitch_output;
    motor_2 -= pitch_output;
    motor_1 += roll_output;
    motor_4 += roll_output;
    motor_2 -= roll_output;
    motor_3 -= roll_output;
    
}
int main()
{
    float pitch;
    float roll;
    float yaw;
    unsigned int motor_1;
    unsigned int motor_2;
    unsigned int motor_3;
    unsigned int motor_4;
    default_random_engine e; 
    sim s;
    thread t(&sim::_control, s);
    motor_1 = 1500;
    motor_2 = 1500;
    motor_3 = 1500;
    motor_4 = 1500;
    while(1)
    {
        motor_1 = 1500 + e() % 30;
        motor_2 = 1500 + e() % 30;
        motor_3 = 1500 + e() % 30;
        motor_4 = 1500 + e() % 30;
        //cout << "[motor_1]:\t" << motor_1 << "\t[motor_2]:\t" << motor_2 << "\t[motor_3]:\t" << motor_3 << "\t[motor_3]\t" << motor_4 << endl; 
        controler(pitch, roll, yaw, motor_1, motor_2, motor_3, motor_4);
        s.output_angle(pitch, roll, yaw);
        s.input_control_value(motor_1, motor_2, motor_3, motor_4);
        sleep(0.001);
    }
    t.join();
}