#include "sim.h"
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <thread>
#include <pthread.h>
using namespace std;
#define PI 3.1415926
unsigned int motor_1;
unsigned int motor_2;
unsigned int motor_3;
unsigned int motor_4;
float pitch_;
float roll_;
sim::sim()
{
    motor_1 = 0;
    motor_2 = 0;
    motor_3 = 0;
    motor_4 = 0;

    force_m1_x = 0;
    force_m1_y = 0;
    force_m1_z = 0;

    force_m2_x = 0;
    force_m2_y = 0;
    force_m2_z = 0;

    force_m3_x = 0;
    force_m3_y = 0;
    force_m3_z = 0;

    force_m4_x = 0;
    force_m4_y = 0;
    force_m4_z = 0;

    axles_distance = 450;      //轴距，单位厘米
    weight = 1000;            //重量，单位克

    force_x = 0;
    force_y = 0;
    force_z = 0;

    acc_x = 0;
    acc_y = 0;
    acc_z = 0;

    v_x = 0;
    v_y = 0;
    v_z = 0;

    pal_x = 0;
    pal_y = 0;
    pal_z = 0;

    pal_acc_x = 0;
    pal_acc_y = 0;

    pitch = 0;
    roll = 0;
    yaw = 0;
    
    control_rate = 1000.0;   //控制频率，单位Hz
    run = true;
    
    global_x = 0;
    global_y = 0;
    global_z = 0;

    channel_1 = 0;
    channel_2 = 0;
    channel_3 = 0;
    channel_4 = 0;
    //转动惯量
    M = (1.0 / 12.0) * (weight / 1000.0) * (axles_distance / 100.0 / 1.414) * (axles_distance / 100.0 / 1.414);
}

void sim::control()
{
    //t.detach();
}

void sim::_control()
{
    while(run)
    {
        gettimeofday(&time_begin, NULL);
        force_m1 = get_force(motor_1);
        force_m1_x = force_m1 * sin(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);
        force_m1_y = force_m1 * cos(pitch / 180.0 * PI) * sin(roll / 180.0 * PI);
        force_m1_z = force_m1 * cos(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);
        
        force_m2 = get_force(motor_2);
        force_m2_x = force_m2 * sin(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);
        force_m2_y = force_m2 * cos(pitch / 180.0 * PI) * sin(roll / 180.0 * PI);
        force_m2_z = force_m1 * cos(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);

        force_m3 = get_force(motor_3);
        force_m3_x = force_m3 * sin(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);
        force_m3_y = force_m3 * cos(pitch / 180.0 * PI) * sin(roll / 180.0 * PI);
        force_m3_z = force_m1 * cos(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);

        force_m4 = get_force(motor_4);
        force_m4_x = force_m4 * sin(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);
        force_m4_y = force_m4 * cos(pitch / 180.0 * PI) * sin(roll / 180.0 * PI);
        force_m4_z = force_m1 * cos(pitch / 180.0 * PI) * cos(roll / 180.0 * PI);

        force_x = force_m1_x + force_m2_x + force_m3_x + force_m4_x;
        force_y = force_m1_y + force_m2_y + force_m3_y + force_m4_y;
        force_z = force_m1_z + force_m2_z + force_m3_z + force_m4_z;

        force_turn_x = force_m3 + force_m4 - force_m1 - force_m2;
        force_turn_y = force_m1 + force_m4 - force_m3 - force_m2;

        pal_acc_x = force_turn_x * (axles_distance / 100.0 / 1.414) / M;
        pal_acc_y = force_turn_y * (axles_distance / 100.0 / 1.414) / M; 

        pitch += pal_x * (1.0 / control_rate);
        roll += pal_y * (1.0 / control_rate);

        pal_x += pal_acc_x * (1.0 / control_rate);
        pal_y += pal_acc_y * (1.0 / control_rate);

        pitch += 0.5 * pal_acc_x * (1.0 / control_rate) * (1.0 / control_rate);
        roll += 0.5 * pal_acc_y * (1.0 / control_rate) * (1.0 / control_rate);

        acc_x = force_x / weight;
        acc_y = force_y / weight;
        acc_z = (force_z - weight) / weight;

        global_x += v_x * (1.0 / control_rate);
        global_y += v_y * (1.0 / control_rate);
        global_z += v_z * (1.0 / control_rate);

        v_x += acc_x * (1.0 / control_rate);
        v_y += acc_y * (1.0 / control_rate);
        v_z += acc_z * (1.0 / control_rate);

        global_x += 0.5 * acc_x * (1.0 / control_rate) * (1.0 / control_rate);
        global_y += 0.5 * acc_y * (1.0 / control_rate) * (1.0 / control_rate);
        global_z += 0.5 * acc_z * (1.0 / control_rate) * (1.0 / control_rate); 
        //cout << "[global_x]:\t" << global_x << endl;
        //cout << "[global_y]:\t" << global_y << endl;
        //cout << "[global_z]:\t" << global_z << endl;
        //cout << "[pitch]:\t" << pitch << endl;
        //cout << "[roll]:\t\t" << roll << endl;
        cout << roll << endl;
        pitch_ = pitch;
        roll_ = roll;
        do
        {
            gettimeofday(&time_end, NULL);
        } while (((time_end.tv_sec-time_begin.tv_sec)*1000000+time_end.tv_usec-time_begin.tv_usec)/1000 < (1000/control_rate));
    }
}

void sim::output_angle(float& _pitch, float& _roll, float& _yaw)
{
    _pitch = pitch_;
    _roll = roll_;
    _yaw = yaw;
}

void sim::input_control_value(unsigned int m1, unsigned int m2, unsigned int m3, unsigned int m4)
{
    pthread_rwlock_rdlock(&rwlock);
    motor_1 = m1;
    motor_2 = m2;
    motor_3 = m3;
    motor_4 = m4;
    pthread_rwlock_unlock(&rwlock);
}

float sim::get_force(unsigned int motor)
{
    if(motor < 1000 || motor > 2000)
    {
        return 0;
    }
    else
    {
        motor -= 1000;
        float retvel = (float)motor;
        retvel /= 1000;
        retvel *= 900;
        return retvel;
    }
}