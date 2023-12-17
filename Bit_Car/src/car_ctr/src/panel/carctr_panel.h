#ifndef ABC_H
#define ABC_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <std_msgs/String.h>
#include <QTimer>
#endif

#include "ui_carctr.h"
#include "car_ctr/car_state_msgs.h"

using namespace std;


namespace rviz_gui
{
// 所有的plugin都必须是rviz::Panel的子类
class Panel_CarCtr: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    Panel_CarCtr(QWidget *parent = 0);

// 公共槽.
public Q_SLOTS:


// 内部槽.
protected Q_SLOTS:
    void qtmrfunc();

    // 内部变量.
protected:
    ros::NodeHandle *nh, *nh_local;
    ros::Subscriber sub;
    ros::Publisher pub;

    void CarMsgCallback(const car_ctr::car_state_msgs::ConstPtr &msg);

private:
    Ui::Panel_CarCtr *ui;
    QTimer qtmr;
};

} 

#endif 
