#define __SIM_H__
#ifdef __SIM_H__
#include <sys/time.h>
#include <pthread.h>
/**
 * @brief 针对四旋翼的模拟器
 * 
 */
class sim
{
    /*
    电机示意图
     ____         ____  
    |    | => M1 |    | => M2
    |____|       |____|
          \     /
     ____ /     \ ____
    |    | => M4 |    | => M3
    |____|       |____|
    */
    private:
    //电机

    float force_m1;
    float force_m1_x;
    float force_m1_y;
    float force_m1_z;

    float force_m2;
    float force_m2_x;
    float force_m2_y;
    float force_m2_z;

    float force_m3;
    float force_m3_x;
    float force_m3_y;
    float force_m3_z;

    float force_m4;
    float force_m4_x;
    float force_m4_y;
    float force_m4_z;
    //参数表
    float axles_distance;    //轴距，单位厘米
    float weight;            //重量，单位克
    //飞机
    float force_x;
    float force_y;
    float force_z;

    float force_turn_x;
    float force_turn_y;

    float acc_x;
    float acc_y;
    float acc_z;

    float v_x;
    float v_y;
    float v_z;

    float pal_x;
    float pal_y;
    float pal_z;

    float pal_acc_x;
    float pal_acc_y;
    float pal_acc_z;

    float M;

    float pitch;                    //俯仰角，单位角度
    float roll;                     //横滚角，单位角度
    float yaw;                      //偏航角，单位角度
    
    float control_rate;               //控制频率，单位Hz
    struct timeval time_begin;
    struct timeval time_end;
    bool run;
    float global_x;
    float global_y;
    float global_z;

    unsigned int channel_1;
    unsigned int channel_2;
    unsigned int channel_3;
    unsigned int channel_4;

    pthread_rwlock_t rwlock; 
    public:
    sim();
    void set_configuration();
    void control();
    void _control();
    void input_control_value(unsigned int m1, unsigned int m2, unsigned int m3, unsigned int m4);
    void output_angle(float& pitch, float& roll, float& yaw);

    private:
    int get_throttle(unsigned int channel_3);
    float get_force(unsigned int motor);
};

#endif