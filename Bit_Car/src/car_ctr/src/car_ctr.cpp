#include <common/public.h>
#include <common/myudp.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <car_ctr/car_state_msgs.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// #include <autoware_msgs/VehicleStatus.h>
// #include <autoware_msgs/ControlCommandStamped.h>

using namespace std;


const float RPM2SPD = 3.1416 * 0.32 / 60;
const float SPD2RPM = 1 / RPM2SPD;
const float RAD2DEG = 57.29578;

class TCarCtr : public Thread
{
private:
    TUDP *udp;
    ros::NodeHandle *node_local;
	TTimer acctimer;
    ros::Subscriber Car_sub, vel_sub, gps_speed_sub, mpc_carcmd_sub;
    ros::Publisher carstate_pub, cartest_pub, vehicle_state_pub, wheel_spd_pub;
	ros::Publisher carmsg_data_pub;

    void CarCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        giv_spd=msg->linear.x;
        giv_angle = msg->angular.z * RAD2DEG;
        giv_angle = - std::min(std::max(giv_angle, -20.0f), 20.0f);

        
        // printf("giv_spd = %.2f giv_angle = %.2f\n",giv_spd, giv_angle);
    }

	// void mpcCtrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr &msg)
    // {
    //     giv_spd=msg->cmd.linear_velocity;
    //     giv_angle = -msg->cmd.steering_angle*180.0/3.1415926;
    // }
 
/*
    void TargetVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        giv_spd = msg->linear.x;
    }
*/
    void GpsSpeedCallback(const std_msgs::Float32::ConstPtr &msg) 
    {
        // autoware_msgs::VehicleStatus state_msg;
        // state_msg.speed =  msg->data;
        // cout << "gps_speed = " << gps_speed << endl;
        // state_msg.speed = gps_speed;
        // state_msg.angle = turn_angle;
        // vehicle_state_pub.publish(state_msg);
        
		
		float accelerate = 0;
		static float pre_speed = gps_speed;

		if(acctimer.GetValue() > 0.5)
		{
            accelerate = (gps_speed - pre_speed)/acctimer.GetValue();
			acctimer.Clear();
			pre_speed = gps_speed;
		}

		std_msgs::Float32MultiArray tempmsgs;
		tempmsgs.data.push_back(giv_spd);
		

		tempmsgs.data.push_back(giv_angle);
		tempmsgs.data.push_back(gps_speed);  //实际速度
		tempmsgs.data.push_back(turn_angle);  //实际角度
		tempmsgs.data.push_back(accelerate);  //实际角度

		carmsg_data_pub.publish(tempmsgs);
    }


public:
    float giv_spd = 0, giv_angle = 0;
    car_ctr::car_state_msgs car_state_msg;
    float act_runspd; //  小车实际速度
    double gps_speed = 0;
    double turn_angle = 0;
    

    TCarCtr()
    {
        node_local=new ros::NodeHandle("~");
        
        string pub_topic, sub_topic, car_ip;
        node_local->getParam("input_topic", sub_topic);
        node_local->getParam("output_topic", pub_topic);   
        node_local->getParam("car_ip", car_ip);  

        udp = new TUDP(8090);
        udp->AddRemoteIP(car_ip, 8080);
        udp->Send("OK");
        usleep(10000);

        carstate_pub = node_local->advertise<car_ctr::car_state_msgs>(pub_topic, 10);
        wheel_spd_pub = node_local->advertise<geometry_msgs::Twist>("/wheel_spd",10);
        // vehicle_state_pub = node.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);
        Car_sub = node_local->subscribe<geometry_msgs::Twist>(sub_topic, 10, &TCarCtr::CarCmdCallback, this);
        // gps_speed_sub = node.subscribe<std_msgs::Float32>("/gps_pro/act_speed", 10, &TCarCtr::GpsSpeedCallback, this);
        // mpc_carcmd_sub = node.subscribe<autoware_msgs::ControlCommandStamped>("ctrl_raw", 10, &TCarCtr::mpcCtrlCmdCallback, this);
        // cartest_pub = node.advertise<std_msgs::Float32MultiArray>("/car_cmd",10);
        // pos_sub = node.subscribe<geometry_msgs::Point>("/target_pos", 10, &TCarCtr::TargetPosCallback,this);
        // vel_sub = node.subscribe<geometry_msgs::Twist>("vel_cmd", 10, &TCarCtr::TargetVelCallback, this);
        // carmsg_data_pub = node.advertise<std_msgs::Float32MultiArray>("carmsg_data", 1);
        
        // Enable(true);
        giv_angle = 0;
        giv_spd = 0;
        start();
    }
    ~TCarCtr()
    {
        delete udp;
    }

    void CarSpdAngleCtr()
    {
        // if(!car_state_msg.Car_Remote_Enable) return;

        char str[500];
        // cout << giv_spd << "  " << giv_angle << endl;
        sprintf(str, "Car Run %.2f %.2f", giv_spd, giv_angle);
        //sprintf(str, "$Motor B %d %d %d", 1, giv_spd, giv_angle);
        udp->Send(str);
        usleep(5000);
        // printf("giv=%.0f %.0f\n",giv_spd, giv_angle);
    }

    void Stop(int flag)
    {
        char str[500];
        sprintf(str, "Car Stop %d", flag);
        udp->Send(str);
    }

    void Enable(bool e)
    {
        char str[500];
        if (e)
            sprintf(str, "Car Enable");
        else
            sprintf(str, "Car Disable");
        udp->Send(str);
        usleep(20000);
    }

    void UDP_Proc() //  用于处理小车上传的状态信息
    {
        if (udp->rec_count == 0)
            return;

        udp->rec_count = 0;

        char recbuf[1000];
        strcpy(recbuf, udp->rec_buf);
        // printf("%s\n",recbuf);

        vector<string> strs = split(recbuf, ";");
        for (int i = 0; i < strs.size(); i++)
        {
            vector<string> substrs = split(strs.at(i), " ");
            if (substrs[0] == "Car")
            {
                car_state_msg.Car_Enable = atoi(substrs.at(1).c_str());
                car_state_msg.Car_Remote_Enable = atoi(substrs.at(2).c_str());
            }
            if (substrs[0] == "BMS")
            {
                car_state_msg.BMS_SOC = atof(substrs.at(1).c_str());
                car_state_msg.BMS_Voltage = atof(substrs.at(2).c_str());
                car_state_msg.BMS_Current = atof(substrs.at(3).c_str());
            }
            if (substrs[0].find("Wheel") != -1) // || substrs[0].substr(0,4)=="Turn" )
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());
                // printf("id=%d %s\n",id, substrs[1].c_str());
                car_state_msg.WheelMotor_Enable[id] = atoi(substrs.at(1).c_str());
                car_state_msg.WheelMotor_Spd[id] = atof(substrs.at(2).c_str());
                gps_speed = car_state_msg.WheelMotor_Spd[0];
            }
            if (substrs[0].find("Turn") != -1) // || substrs[0].substr(0,4)=="Turn" )
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());

                car_state_msg.TurnMotor_Enable[id] = atoi(substrs.at(1).c_str());
                car_state_msg.TurnMotor_Angle[id] = atof(substrs.at(2).c_str());
                turn_angle = car_state_msg.TurnMotor_Angle[0];
            }
        }
        carstate_pub.publish(car_state_msg);

        act_runspd = 0;
        for (int i = 0; i < 4; i++)
            act_runspd += car_state_msg.WheelMotor_Spd[i];
        act_runspd *= RPM2SPD;
        gps_speed =  act_runspd*0.25;


        geometry_msgs::Twist temp_twist;
        temp_twist.linear.x = gps_speed;
        temp_twist.angular.z = turn_angle;
        wheel_spd_pub.publish(temp_twist);
        


        // printf("%.2f\n",act_runspd);
    }

    void run()
    {
        TTimer tmr;
        while (node_local->ok())
        {
            CarSpdAngleCtr();
            UDP_Proc();
            usleep(20000);
            ros::spinOnce();

            // std_msgs::Float32MultiArray msg;
            // msg.data.push_back(100*sin(6.28*0.05*tmr.GetValue()));
            // msg.data.push_back(40*sin(6.28*0.1*tmr.GetValue()));
            // cartest_pub.publish(msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Car_Ctr");
    TCarCtr myCar;

    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
