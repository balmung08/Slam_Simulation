#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QDebug>
#include <std_msgs/String.h>
#include <common/public.h>

#include "carctr_panel.h"


using namespace std;

namespace rviz_gui
{

// 构造函数，初始化变量
Panel_CarCtr::Panel_CarCtr(QWidget *parent)
    : rviz::Panel(parent), ui(new Ui::Panel_CarCtr)
{
    ui->setupUi(this);

    nh = new ros::NodeHandle();
    nh_local = new ros::NodeHandle("~");

    sub = nh->subscribe("/car_ctr/car_state", 10, &Panel_CarCtr::CarMsgCallback, this);
    // pub = nh->advertise<std_msgs::String>("/turntable_ctr", 10);

    // qtmr.start(20);
    //connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));
}

void Panel_CarCtr::qtmrfunc()
{
    //if(turntable_cmd=="")  return;
    //ROS_INFO("%s",turntable_cmd.c_str());

}

void Panel_CarCtr::CarMsgCallback(const car_ctr::car_state_msgs::ConstPtr &msg)
{
    // ROS_INFO("%d", msg->BMS_SOC);

    char buf[500];
    sprintf(buf,"BATTERY: SOC %d    Voltage %d",  msg->BMS_SOC, msg->BMS_Voltage);
    ui->Label_Battery->setText(QString::fromUtf8(buf));


    sprintf(buf,"WHEEL_1: Enable %d    Speed %.2f", msg->WheelMotor_Enable[0], msg->WheelMotor_Spd[0]);
    ui->Label_Wheel1->setText(QString::fromUtf8(buf));

    sprintf(buf,"WHEEL_2: Enable %d    Speed %.2f", msg->WheelMotor_Enable[1], msg->WheelMotor_Spd[1]);

    ui->Label_Wheel2->setText(QString::fromUtf8(buf));
    sprintf(buf,"WHEEL_3: Enable %d    Speed %.2f",  msg->WheelMotor_Enable[2], msg->WheelMotor_Spd[2]);
    ui->Label_Wheel3->setText(QString::fromUtf8(buf));
    sprintf(buf,"WHEEL_4: Enable %d    Speed %.2f",  msg->WheelMotor_Enable[3], msg->WheelMotor_Spd[3]);
    ui->Label_Wheel4->setText(QString::fromUtf8(buf));


    sprintf(buf,"TURN_F : Enable %d    Angle %.2f",  msg->TurnMotor_Enable[0], msg->TurnMotor_Angle[0]);
    ui->Label_Turn1->setText(QString::fromUtf8(buf));
    sprintf(buf,"TURN_R : Enable %d    Angle %.2f",  msg->TurnMotor_Enable[1], msg->TurnMotor_Angle[1]);
    ui->Label_Turn2->setText(QString::fromUtf8(buf));
}

// void Panel_CarCtr::TTCallback(const std_msgs::StringConstPtr &msg)
// {
//     vector<string> ss=split(msg->data,";");
//     char buf[500];
    
//     for(int i=0;i<ss.size();i++)
//     {
//         // printf("%s\n",ss[i].c_str());
//         vector<string> subs=split(ss[i]," ");
//         float angle=atof(subs[2].c_str());
//         float err=atof(subs[3].c_str());
//         int e=atoi(subs[1].c_str());
//         if(subs[0]=="azimuth") 
//         {
//             sprintf(buf,"Azimuth: %5.1f  Err: %3.1f  Enabled: %d ",angle, err, e);
//             ui->Label_Azimuth_Angle->setText(QString::fromUtf8(buf));
//         }
//         else if(subs[0]=="pitch") 
//         {
//             sprintf(buf,"Pitch:     %5.1f  Err: %3.1f  Enabled: %d ",angle, err, e);
//             ui->Label_Pitch_Angle->setText(QString::fromUtf8(buf));
//         }
        
//     }
// }



} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gui::Panel_CarCtr, rviz::Panel)
// END_TUTORIAL
