#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include "omnihand_node_msgs/srv/set_hand_shape.hpp"

/*
    需求:通过接收客户端的请求来动态加载param参数文件,实现向moveit控制节点发送机械手的目标姿态,
        并通过moveit规划出机械手的运动轨迹,
        最终通过底层驱动节点控制机械手执行规划好的轨迹
    流程:
        1. 启动时从 YAML 参数文件加载预设手型 (presets.xxx) 和当前手型 (current_shape)
        2. 创建服务端对象,等待客户端调用服务来切换手型,服务回调函数中设置一个标志位,通知主线程执行规划
        3. 当 current_shape 被修改时, 查找对应预设角度, 通过 MoveGroupInterface 规划并执行
    使用:
        ros2 service call /hand_shape/set_shape omnihand_node_msgs/srv/SetHandShape "{shape_name: catch}"

    增加: 电机堵转检测,如果堵转,则停止发送指令,
        存储上n帧数据,回溯目标位置,发送回退指令,让机械手回退到未堵转位置,发送error日志,等待下一次请求
*/

using namespace std::chrono_literals;
using namespace std::placeholders;

// 10个主动关节名（顺序与 YAML 角度数组对应）
static const std::vector<std::string> ACTIVE_JOINT_NAMES = {
    "L_thumb_roll_joint", "L_thumb_abad_joint", "L_thumb_mcp_joint",
    "L_index_abad_joint", "L_index_pip_joint",
    "L_middle_pip_joint",
    "L_ring_abad_joint",  "L_ring_pip_joint",
    "L_pinky_abad_joint", "L_pinky_pip_joint",
};

class HandShape : public rclcpp::Node {
public:
    // 手动声明参数, 使 ros2 param list / describe 可查看
    HandShape(): 
    Node("hand_shape",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
        RCLCPP_INFO(this->get_logger(), "hand_shape 节点已创建");
        // --- 显式声明参数 (带描述) ---
        // current_shape
        // 注意：automatically_declare_parameters_from_overrides(true) 已从 YAML 自动声明了该参数，
        //       此处仅在未声明时补充声明（如直接 ros2 run 而未传 YAML 时的兜底默认值）
        if (!this->has_parameter("current_shape")) {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.description = "当前激活的手型名称, 修改后自动规划执行. "
                               "可选值: default, catch, full, pinch, point, joke 等 (见 hand_shape.yaml)";
            desc.read_only = false; // 允许运行时修改
            this->declare_parameter("current_shape", "default", desc);
        }

        // 预设手型参数 —— 遍历已加载的 presets.* 参数并补充描述
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.description = "预设手型角度数组 (单位:度), 10个值依次为: "
                               "thumb_roll, thumb_abad, thumb_mcp, "
                               "index_abad, index_pip, middle_pip, "
                               "ring_abad, ring_pip, pinky_abad, pinky_pip";
            // 获取所有已通过 YAML 自动加载的 presets.* 参数名
            // 2 表示只列出一级子参数 (即 presets.xxx, 不包括更深层次的参数)
            auto param_names = this->list_parameters({"presets"}, 2).names;
            for (const auto & name : param_names) {
                RCLCPP_INFO(this->get_logger(), "已加载预设参数: %s", name.c_str());
            }
        }

        // 创建 service，接收手型切换请求
        // 用法: ros2 service call /hand_shape/set_shape omnihand_node_msgs/srv/SetHandShape "{shape_name: catch}"
        service_ = this->create_service<omnihand_node_msgs::srv::SetHandShape>(
            "/hand_shape/set_shape",
            //服务端回调函数,接收客户端请求,设置标志位通知主线程执行规划,并等待执行结果返回给客户端
            [this](const omnihand_node_msgs::srv::SetHandShape::Request::SharedPtr request,
                   omnihand_node_msgs::srv::SetHandShape::Response::SharedPtr response) {
                // service 回调在 spin 线程中，不能直接调 MoveIt（会死锁）
                // 设置标志，由主线程执行，等待执行结果返回给客户端
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_shape_ = request->shape_name;
                    shape_changed_ = true;//通知主线程有新请求
                    result_ready_ = false;//结果未准备好, 等主线程执行完成后再设置为 true
                }
                cv_.notify_one();//通知主线程有新请求 转到run_moveit_loop函数的主循环等待处理
                RCLCPP_INFO(this->get_logger(), "收到手型请求: %s", request->shape_name.c_str());

                // 等待主线程执行完成（最长等待 30 秒）
                std::unique_lock<std::mutex> lock(mutex_);
                if (cv_result_.wait_for(lock, std::chrono::seconds(30),
                        [this]{ return result_ready_; })) {//如果在30秒内,result_ready_被设置为true, 则说明执行完成, 可以返回结果
                    response->success = result_success_;
                    response->message = result_message_;
                } else {
                    response->success = false;
                    response->message = "执行超时";
                }
            }
        );

        // sub_motor_states_ = this->create_subscription<


    }

    // 在 spin 线程启动后调用, 初始化 MoveGroupInterface 并进入主循环
    void run_moveit_loop()
    {
        // MoveGroupInterface 构造时需要节点正在 spin, 所以放在这里而不是构造函数
        RCLCPP_INFO(this->get_logger(), "正在连接 MoveGroupInterface (规划组: hand) ...");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "hand");//连接 MoveGroupInterface, 参数为当前节点和规划组名称(需与 moveit 配置一致)
        move_group_->setMaxVelocityScalingFactor(1.0);// 限制速度, 避免机械手动作过快
        move_group_->setMaxAccelerationScalingFactor(1.0);// 限制加速度, 避免机械手动作过快
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface 连接成功");

        // 主循环：等待 service 请求，执行 MoveIt 规划
        while (rclcpp::ok()) {
            std::string shape_name;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]{ return shape_changed_ || !rclcpp::ok(); });//等待有新请求或者节点关闭
                if (!rclcpp::ok()) break;
                shape_name = pending_shape_;
                shape_changed_ = false;//重置标志位,没有手型需要请求
            }
            auto [success, message] = execute_shape(shape_name);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                result_success_ = success;
                result_message_ = message;
                result_ready_ = true;//执行完成
            }
            cv_result_.notify_one();//通知服务回调执行结果已经准备好，可以返回给客户端(不保证moveit执行成功)
        }
    }

private:
    rclcpp::Service<omnihand_node_msgs::srv::SetHandShape>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;// MoveIt 规划接口
    std::mutex mutex_;//保护共享数据访问
    std::condition_variable cv_;// 通知主线程有新请求
    std::condition_variable cv_result_;// 通知 service 回调执行完成
    //保护的变量
    std::string pending_shape_;//待执行的手型名称
    bool shape_changed_ = false;//是否有新的手型需要执行
    
    bool result_ready_ = false;//是否执行完成, service 回调可以返回结果
    bool result_success_ = false;//执行结果: 是否成功
    std::string result_message_;//执行结果: 额外信息,如错误原因等

    

    // ---- 执行手型切换，返回 {成功?, 信息} ----
    std::pair<bool, std::string> execute_shape(const std::string & shape_name)
    {
        // 1. 从参数服务器读取预设角度 (presets.<shape_name>)
        std::string param_name = "presets." + shape_name;
        if (!this->has_parameter(param_name)) {
            RCLCPP_ERROR(this->get_logger(),"未找到预设手型参数: %s  (可用预设请查看 YAML 文件)", param_name.c_str());
            return {false, "未找到预设: " + shape_name};
        }

        std::vector<double> angles_deg;//预设角度(单位:度)
        try {
            angles_deg = this->get_parameter(param_name).as_double_array();
        } catch (const rclcpp::ParameterTypeException & e) {
            RCLCPP_ERROR(this->get_logger(), "参数 %s 类型错误: %s", param_name.c_str(), e.what());
            return {false, "参数类型错误"};
        }

        if (angles_deg.size() != ACTIVE_JOINT_NAMES.size()) {//如果预设角度数量不匹配, 则报错并返回
            RCLCPP_ERROR(this->get_logger(),
                "预设 \"%s\" 角度数量 = %zu, 应为 %zu",
                shape_name.c_str(), angles_deg.size(), ACTIVE_JOINT_NAMES.size());
            return {false, "角度数量不匹配"};
        }

        // 2. 构造关节目标 (度 → 弧度)
        std::map<std::string, double> joint_target;
        RCLCPP_INFO(this->get_logger(), "=== 手型: %s ===", shape_name.c_str());
        for (size_t i = 0; i < ACTIVE_JOINT_NAMES.size(); ++i) {
            double rad = angles_deg[i] * M_PI / 180.0;//将角度转换为弧度
            joint_target[ACTIVE_JOINT_NAMES[i]] = rad;
        }

        // 3. 设置 MoveIt 关节目标
        if (!move_group_->setJointValueTarget(joint_target)) {
            RCLCPP_ERROR(this->get_logger(), "setJointValueTarget 失败, 目标可能超出关节限制");
            return {false, "目标超出关节限制"};
        }

        // 4. 规划
        RCLCPP_INFO(this->get_logger(), "正在规划运动轨迹...");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_ok = (move_group_->plan(plan) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (!plan_ok) {
            RCLCPP_ERROR(this->get_logger(), "手型 \"%s\" 轨迹规划失败!", shape_name.c_str());
            return {false, "轨迹规划失败"};
        }

        // 5. 执行
        auto exec_result = move_group_->execute(plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            // execute 返回成功后，等待2秒让机械手完成动作（可根据实际情况调整）
            std::this_thread::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(this->get_logger(), "手型 \"%s\" 执行完成 ✓", shape_name.c_str());
            return {true, "执行完成"};
        } else {
            RCLCPP_ERROR(this->get_logger(), "手型 \"%s\" 执行失败 (错误码: %d)",
                shape_name.c_str(), exec_result.val);
            return {false, "执行失败，错误码: " + std::to_string(exec_result.val)};
        }
    }
};

int main(int argc, char * argv[])
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);

    // 2. 创建节点
    auto node = std::make_shared<HandShape>();

    // 3. 在后台线程中 spin (MoveGroupInterface 需要节点持续 spin)
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    // 4. 主线程运行 MoveIt 循环 (阻塞, 直到 Ctrl+C)
    node->run_moveit_loop();

    // 5. 退出清理
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}