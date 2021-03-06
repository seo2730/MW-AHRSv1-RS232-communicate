#include "ros/ros.h"
#include "Serial.h"
#include <iostream>
#include <sensor_msgs/Imu.h>
//#include <../include/mw_ahrsv1/imu.h>


typedef struct
{
    // Euler
    float roll;
    float pitch;
    float yaw;

    float a_x;
    float a_y;
    float a_z;

    float w_x;
    float w_y;
    float w_z;

}Euler;

class MW_AHRS
{

public:
    MW_AHRS()
    {
        for(int i=0; i<100; i++)
            buffer[i] = 0;

        dev = open_serial((char*)"/dev/ttyUSB1", 115200, 0, 0);

        Tx[0] = A;     // a
        Tx[1] = N;     // n
        Tx[2] = G;     // g
        Tx[3] = CR;     // CR
        Tx[4] = LF;     // LF

        Tx[5] = A;     // a
        Tx[6] = C;     // n
        Tx[7] = C;     // g
        Tx[8] = CR;     // CR
        Tx[9] = LF;     // LF

        Tx[10] = G;     // a
        Tx[11] = Y;     // n
        Tx[12] = R;     // g
        Tx[13] = CR;     // CR
        Tx[14] = LF;     // LF

    }
    ~MW_AHRS()
    {
        close_serial(dev);
    }

    Euler get_data(void)
    {
        write(dev,Tx,5);
        read(dev, &buffer, 100);

        if(buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g')
        {
            char *ptr = strtok(buffer, " ");

            ang_count=0;

            while(ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if(ang_count == 1)
                {
                    euler.roll = atof(ptr);
                }
                else if(ang_count == 2)
                {
                    euler.pitch = atof(ptr);
                }
                else if(ang_count == 3)
                {
                    euler.yaw = atof(ptr);
                }

            }
        }

        else if(buffer[0] == 'a' && buffer[1] == 'c' && buffer[2] == 'c')
        {
            char *ptr = strtok(buffer, " ");

            ang_count=0;

            while(ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if(ang_count == 1)
                {
                    euler.a_x = atof(ptr);
                }
                else if(ang_count == 2)
                {
                    euler.a_y = atof(ptr);
                }
                else if(ang_count == 3)
                {
                    euler.a_z = atof(ptr);
                }

            }
        }

        else if(buffer[0] == 'g' && buffer[1] == 'y' && buffer[2] == 'r')
        {
            char *ptr = strtok(buffer, " ");

            ang_count=0;

            while(ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if(ang_count == 1)
                {
                    euler.w_x = atof(ptr);
                }
                else if(ang_count == 2)
                {
                    euler.w_y = atof(ptr);
                }
                else if(ang_count == 3)
                {
                    euler.w_z = atof(ptr);
                }

            }
        }


        std::cout << "roll = " << euler.roll << std::endl;
        std::cout << "pitch = " << euler.pitch << std::endl;
        std::cout << "yaw = " << euler.yaw << std::endl;

        std::cout << "a_x = " << euler.a_x << std::endl;
        std::cout << "a_y = " << euler.a_y << std::endl;
        std::cout << "a_z = " << euler.a_z << std::endl;

        std::cout << "w_x = " << euler.w_x << std::endl;
        std::cout << "w_y = " << euler.w_y << std::endl;
        std::cout << "w_z = " << euler.w_z << std::endl;
        std::cout << std::endl;

        return euler;

    }


private:
    // Device Name
    int dev = 0;

    Euler euler;

    // Data buffer
    char buffer[100];
    unsigned char Tx[15];

    // Serperate Euler Angle Variable
    int ang_count = 0;

    // ASCII CODE
    const unsigned char A = 0x61;
    const unsigned char N = 0x6E;
    const unsigned char G = 0x67;

    const unsigned char C = 0x63;

    const unsigned char Y = 0x79;
    const unsigned char R = 0x72;

    const unsigned char M = 0x67;

    const unsigned char CR = 0x0D;
    const unsigned char LF = 0x0A;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mw_ahrsv1");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

//    ros::Publisher imu_pub_ = nh.advertise<mw_ahrsv1::imu>("imu", 100);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("mw_ahrsv1/imu", 1);
//    mw_ahrsv1::imu msg_;

    sensor_msgs::Imu msg;

    MW_AHRS ahrs_obj;
    Euler euler;

    while (ros::ok())
    {
        euler = ahrs_obj.get_data();


//        msg_.roll = euler.roll;
//        msg_.pitch = euler.pitch;
//        msg_.yaw = euler.yaw;

//        imu_pub_.publish(msg_);

        msg.orientation.x = euler.roll;
        msg.orientation.y = euler.pitch;
        msg.orientation.z = euler.yaw;

        msg.linear_acceleration.x = euler.a_x;
        msg.linear_acceleration.y = euler.a_y;
        msg.linear_acceleration.z = euler.a_z;

        msg.angular_velocity.x = euler.w_x;
        msg.angular_velocity.y = euler.w_y;
        msg.angular_velocity.z = euler.w_z;

        imu_pub.publish(msg);

        loop_rate.sleep();

    }


    return 0;
}



