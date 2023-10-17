#include "teleop_pad.h"

namespace rviz_teleop_commander
{
// 构造函数，初始化变量
TeleopPanel::TeleopPanel(QWidget *parent) : rviz::Panel(parent), linear_velocity_(0), angular_velocity_(0)
{
    /**
     * @brief RVIZ panel 布局
     *
     */

    // 初始化 QVBoxLayout
    QVBoxLayout *topic_layout = new QVBoxLayout;

    // 创建一个输入topic命名的窗口
    topic_layout->addWidget(new QLabel("Teleop Topic:"));
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget(output_topic_editor_);

    // 创建一个输入线速度的窗口
    topic_layout->addWidget(new QLabel("Linear Velocity:"));
    output_topic_editor_1 = new QLineEdit;
    topic_layout->addWidget(output_topic_editor_1);

    // 创建一个输入角速度的窗口
    topic_layout->addWidget(new QLabel("Angular Velocity:"));
    output_topic_editor_2 = new QLineEdit;
    topic_layout->addWidget(output_topic_editor_2);

    // 创建按钮
    QHBoxLayout *button_layout = new QHBoxLayout;

    button_startPub_ = new QPushButton;
    button_startPub_->setText("start pub cmd_vel");
    button_stopPub_ = new QPushButton;
    button_stopPub_->setText("stop pub cmd_vel");
    button_layout->addWidget(button_startPub_);
    button_layout->addWidget(button_stopPub_);

    // 初始化 QHBoxLayout
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(topic_layout);
    layout->addLayout(button_layout);
    setLayout(layout);

    /**
     * @brief QT UI element SLOT
     *
     */

    // 设置信号与槽的连接
    connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));  // 输入topic命名，回车后，调用updateTopic()
    connect(
        output_topic_editor_1,
        SIGNAL(editingFinished()),
        this,
        SLOT(update_Linear_Velocity()));  // 输入线速度值，回车后，调用update_Linear_Velocity()
    connect(
        output_topic_editor_2,
        SIGNAL(editingFinished()),
        this,
        SLOT(update_Angular_Velocity()));  // 输入角速度值，回车后，调用update_Angular_Velocity()

    connect(button_startPub_, SIGNAL(pressed()), this, SLOT(startPub()));  // 开始发布按钮
    connect(button_stopPub_, SIGNAL(pressed()), this, SLOT(stopPub()));    // 停止发布按钮

    // 设置定时器的回调函数，按周期调用sendVel()
    // 创建一个定时器，用来定时发布消息
    QTimer *output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(sendVel()));
    // 设置定时器的周期，100ms
    output_timer->start(100);

    // init pub flag
    this->pub_flag_ = false;
}

// 更新线速度值
void TeleopPanel::update_Linear_Velocity()
{
    // 获取输入框内的数据
    QString temp_string = output_topic_editor_1->text();

    // 将字符串转换成浮点数
    float lin = temp_string.toFloat();

    // 保存当前的输入值
    linear_velocity_ = lin;
}

// 更新角速度值
void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();
    float ang = temp_string.toFloat();
    angular_velocity_ = ang;
}

// 更新topic命名
void TeleopPanel::updateTopic()
{
    setTopic(output_topic_editor_->text());
}

// 开始发布消息
void TeleopPanel::startPub()
{
    ROS_WARN("start pub cmd_vel!");
    this->pub_flag_ = true;
}

// 停止发布消息
void TeleopPanel::stopPub()
{
    ROS_WARN("stop pub cmd_vel!");
    this->pub_flag_ = false;
}

// 设置topic命名
void TeleopPanel::setTopic(const QString &new_topic)
{
    // 检查topic是否发生改变.
    if (new_topic != output_topic_)
    {
        output_topic_ = new_topic;

        // 如果命名为空，不发布任何信息
        if (output_topic_ == "")
        {
            velocity_publisher_.shutdown();
        }
        // 否则，初始化publisher
        else
        {
            velocity_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_.toStdString(), 1);
        }

        Q_EMIT configChanged();
    }
}

// 发布消息
void TeleopPanel::sendVel()
{
    if (ros::ok() && velocity_publisher_ && this->pub_flag_)
    {
        geometry_msgs::TwistStamped msg;
        msg.header.frame_id = "cmd_vel";
        msg.header.stamp = ros::Time::now();
        msg.twist.linear.x = linear_velocity_;
        msg.twist.linear.y = 0;
        msg.twist.linear.z = 0;
        msg.twist.angular.x = 0;
        msg.twist.angular.y = 0;
        msg.twist.angular.z = angular_velocity_;
        velocity_publisher_.publish(msg);
    }
}

/**
 * @brief 重载rviz::Panel
 *
 */

// 重载父类的功能, 保存当前的topic名称
void TeleopPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    // 设置topic名称到"Topic"
    // config.mapSetValue(key, value);
    config.mapSetValue("Topic", output_topic_);
}

// 重载父类的功能，加载配置数据
void TeleopPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    QString topic;
    if (config.mapGetString("Topic", &topic))
    {
        output_topic_editor_->setText(topic);
        updateTopic();
    }
}

}  // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel, rviz::Panel)
// END_TUTORIAL
