#include "ros/ros.h"
#include "Serial.h"
#include <iostream>
#include <sensor_msgs/Imu.h>
//#include <../include/mw_ahrsv1/imu.h>


typedef struct
{
    // Euler
    // float roll;
    // float pitch;
    // float yaw;

    float ax;
    float ay;
    float az;

    float wx;
    float wy;
    float wz;

}Euler;

class MW_AHRS
{

public:
    MW_AHRS()
    {
        for(int i=0; i<100; i++)
            buffer[i] = 0;

        dev = open_serial((char*)"/dev/ttyUSB0", 115200, 0, 0);

        Tx[0] = S;    
        Tx[1] = S;    
        Tx[2] = equal;    
	Tx[3] = 0x33;    
        Tx[4] = CR;     // CR
        Tx[5] = LF;     // LF
    }
    ~MW_AHRS()
    {
        close_serial(dev);
    }

    Euler get_data(void)
    {
        write(dev,Tx,5);
        read(dev, &buffer, 100);

        /*if(buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g')
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
        }*/

	if(buffer[0] == 's' && buffer[1] == 's' && buffer[2] == '=' && buffer[3] == '3')
        {
            char *ptr = strtok(buffer, " ");

            ang_count=0;

            while(ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if(ang_count == 1)
                {
                    euler.ax = atof(ptr);
                }
                else if(ang_count == 2)
                {
                    euler.ay = atof(ptr);
                }
                else if(ang_count == 3)
                {
                    euler.az = atof(ptr);
                }
		else if(ang_count == 4)
                {
                    euler.wx = atof(ptr);
                }
                else if(ang_count == 5)
                {
                    euler.wy = atof(ptr);
                }
		else if(ang_count == 6)
                {
                    euler.wz = atof(ptr);
                }

            }
        }

        //std::cout << "roll = " << euler.roll << std::endl;
        //std::cout << "pitch = " << euler.pitch << std::endl;
        //std::cout << "yaw = " << euler.yaw << std::endl;

	std::cout << "ax = " << euler.ax << std::endl;
        std::cout << "ay = " << euler.ay << std::endl;
        std::cout << "az = " << euler.az << std::endl;

        std::cout << "wx = " << euler.wx << std::endl;
        std::cout << "wy = " << euler.wy << std::endl;
        std::cout << "wz = " << euler.wz << std::endl;

	//std::cout << "buffer = " << buffer << std::endl;
        std::cout << std::endl;

        return euler;

    }


private:
    // Device Name
    int dev = 0;

    Euler euler;

    // Data buffer
    char buffer[100];
    unsigned char Tx[5];

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

    const unsigned char S = 0x73;
    const unsigned char equal = 0x3D;

    const unsigned char CR = 0x0D;
    const unsigned char LF = 0x0A;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mw_ahrsv1");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    //ros::Publisher imu_pub = nh.advertise<mw_ahrsv1::imu>("imu", 100);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu0",1000);
//    mw_ahrsv1::imu msg_;

    sensor_msgs::Imu msg;

    MW_AHRS ahrs_obj;
    Euler euler;

    while (ros::ok())
    {
        euler = ahrs_obj.get_data();

        //msg.orientation.x = euler.roll;
        //msg.orientation.y = euler.pitch;
        //msg.orientation.z = euler.yaw;

	msg.header.stamp = ros::Time::now();
	msg.linear_acceleration.x = euler.ax;
	msg.linear_acceleration.y = euler.ay;
	msg.linear_acceleration.z = euler.az;

	msg.angular_velocity.x = euler.wx;
	msg.angular_velocity.y = euler.wy;
	msg.angular_velocity.z = euler.wz;	
	
        imu_pub.publish(msg);

        loop_rate.sleep();

    }


    return 0;
}
