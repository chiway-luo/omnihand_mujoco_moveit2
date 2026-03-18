#include "rclcpp/rclcpp.hpp"
#include "omnihand_node_msgs/msg/motor_angle.hpp" //关节电机角度控制
#include "omnihand_node_msgs/msg/control_mode.hpp" //电机控制模式
#include "sensor_msgs/msg/joint_state.hpp" //关节状态消息
/*
    需求:接收robot_state_publisher_gui发布的消息,并将其转换为电机控制命令发布出去,实现对机械手的控制
    流程:
        1.包含头文件
        2.初始化ros2客户端
        3.自定义节点类
            3-1
            3-2
            3-3
        4.调用spin函数,并传入节点对象指针
        5.释放资源
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class HandTest :public rclcpp::Node{
public:
    HandTest(std::string str1):Node(str1),last_time_(this->now()){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());
        //创建电机控制方式发布方对象(仅发布一次)
        // | `/agihand/omnihand/left/control_mode_cmd`  | 关节电机控制模式 |    订阅    | [omnihand_node_msgs.msg.ControlMode](#omnihand_node_msgs::msg::ControlMode) |
        //创建发布方对象 发布电机控制模式
        pub_control_mode_ = this->create_publisher<omnihand_node_msgs::msg::ControlMode>("/agihand/omnihand/left/control_mode_cmd",10);
        //创建消息对象
        msg_control_mode_ = std::make_shared<omnihand_node_msgs::msg::ControlMode>();
        msg_control_mode_->modes = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 10个电机全部设为位置控制模式 //设置为关节角度控制模式
        //创建订阅方对象 订阅robot_state_publisher_gui发布的消息
        sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&HandTest::sub_joint_cb,this,_1)
        );
        //创建发布方对象 发布电机控制消息
        //  `/agihand/omnihand/left/motor_angle_cmd`  | 关节电机角度 |    订阅    | [omnihand_node_msgs.msg.MotorAngle](#omnihand_node_msgs::msg::MotorAngle) | 
        pub_ = this->create_publisher<omnihand_node_msgs::msg::MotorAngle>(
            "/agihand/omnihand/left/motor_angle_cmd",10
        );
        //创建定时器,周期1s 防止底层串口通信过快导致丢包
        timer_ = this->create_wall_timer(1s,std::bind(&HandTest::control_cb,this));
        //初始化消息
        msg_motor_ = std::make_shared<omnihand_node_msgs::msg::MotorAngle>();
        angles_ = std::vector<double>(10, 0.0);
        last_angle_ = std::vector<double>(10, 0.0);

        //创建关节名称顺序关系
        // MoveIt 关节名 → MotorAngle.angles[] 索引（左手10个主动关节）
        joint_index_map_ = {
            {"L_thumb_roll_joint",  0},
            {"L_thumb_abad_joint",  1},
            {"L_thumb_mcp_joint",   2},
            {"L_index_abad_joint",  3},
            {"L_index_pip_joint",   4},
            {"L_middle_pip_joint",  5},
            {"L_ring_abad_joint",   6},
            {"L_ring_pip_joint",    7},
            {"L_pinky_abad_joint",  8},
            {"L_pinky_pip_joint",   9},
        };
    }

private:
    rclcpp::Publisher<omnihand_node_msgs::msg::ControlMode>::SharedPtr pub_control_mode_;//发布方对象(电机控制模式)
    rclcpp::Publisher<omnihand_node_msgs::msg::MotorAngle>::SharedPtr pub_;//发布方对象(电机控制)
    rclcpp::TimerBase::SharedPtr timer_;//定时器对象
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;//订阅方对象(关节状态)
    //创建消息对象
    std::shared_ptr<omnihand_node_msgs::msg::ControlMode> msg_control_mode_;//电机控制模式消息对象
    std::vector<double> angles_;//接收到的角度数据
    std::vector<double> last_angle_;//上一次接收到的角度数据
    bool is_init_ = false;//是否初始化角度

    std::shared_ptr<omnihand_node_msgs::msg::MotorAngle> msg_motor_;//电机角度消息对象
    rclcpp::Time last_time_; //上次发布消息的时间

    std::unordered_map<std::string, int> joint_index_map_;//关节名称与电机角度索引的映射关系

    //回调函数
    void control_cb(){
        if (!is_init_) {
            return; //第一次接收到消息时,不发布控制命令,只初始化角度数据,防止机械手突然动作
        }
        //判断角度是否变化超过阈值,如果没有超过阈值则不发布消息,防止机械手抖动
        if(is_angle_changed(angles_,last_angle_,0.01)){ //阈值设置为0.01弧度,根据实际情况调整
            last_angle_ = angles_; //更新上一次的角度数据
        }else{
            last_angle_ = angles_; //更新上一次的角度数据,防止微小变化造成误差积累,但不发布消息
            return; //角度没有变化超过阈值,直接返回,不发布消息
        }
        msg_motor_->angles = angles_; //设置消息中的角度数据
        msg_motor_->header.stamp = this->get_clock()->now(); //设置消息头的时间戳
        msg_motor_->header.frame_id = "angle_frame";
        pub_->publish(*msg_motor_); //发布消息
    }

    void sub_joint_cb(const sensor_msgs::msg::JointState& msg){
        /* 
            std_msgs/Header header
                    builtin_interfaces/Time stamp
                            int32 sec
                            uint32 nanosec
                    string frame_id

            string[] name
            float64[] position 弧度
            float64[] velocity
            float64[] effort
        */
        //根据关节名称找到对应的索引,将接收到的角度数据转换为电机控制命令
        for (size_t i = 0; i < msg.name.size(); ++i) {
            auto it = joint_index_map_.find(msg.name[i]);
            if (it != joint_index_map_.end()) {
                // msg_motor_->angles[it->second] = msg.position[i];
                angles_[it->second] = msg.position[i]; //直接使用弧度值,底层驱动会进行转换
            }
        }
        if (!is_init_) {
            pub_control_mode_->publish(*msg_control_mode_); //第一次接收到消息时,发布电机控制模式,确保底层驱动知道我们要使用位置控制模式
            is_init_ = true; //第一次接收到消息后,设置初始化标志为true
        }
    }

    //创建角度转换弧度工具函数
    double degree_to_radian(double degree){
        return degree * M_PI / 180.0;
    }
    // //弧度转换角度
    // double radian_to_degree(double radian){
    //     return radian * 180 / M_PI;
    // }

    //判断角度是否变化的函数 当前角度 上一次角度 变化阈值
    bool is_angle_changed(const std::vector<double>& current_angles, const std::vector<double>& last_angles, double threshold) {
        for (size_t i = 0; i < current_angles.size(); ++i) {
            if (std::abs(current_angles[i] - last_angles[i]) > threshold) {
                return true; //如果有任意一个角度变化超过阈值,则认为角度发生了变化
            }
        }
        return false; //所有角度变化都没有超过阈值,则认为角度没有发生变化
    }
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<HandTest>("HandTest_node_cpp"));//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}