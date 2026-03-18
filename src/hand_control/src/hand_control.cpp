#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
/*
    工作流程:
    1. 订阅灵巧手驱动反馈 /agihand/omnihand/left/motor_angle (100Hz, 真实关节角度)
    2. 计算 mimic 关节角度，发布完整 /joint_states 给 MoveIt + robot_state_publisher
       → MoveIt 据此做碰撞检测，robot_state_publisher 据此发布 TF
    3. 订阅 /controller_joint_states (MoveIt fake controller 轨迹插值输出，
       已从 /joint_states 重映射，避免和真实状态冲突)
    4. 将轨迹指令转发到 /agihand/omnihand/left/motor_angle_cmd 控制真实灵巧手

    该节点为单线程,对于共享变量不需要使用锁,但需要注意回调函数之间的执行顺序和数据一致性。
*/

using namespace std::placeholders;

class HandControl : public rclcpp::Node {
public:
    HandControl() : Node("hand_control_node") {
        RCLCPP_INFO(this->get_logger(), "HandControl 节点已启动");

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

        // 所有16个关节名（10主动 + 6 mimic）
        all_joint_names_ = {
            "L_thumb_roll_joint", "L_thumb_abad_joint", "L_thumb_mcp_joint",
            "L_thumb_pip_joint", "L_thumb_dip_joint",
            "L_index_abad_joint", "L_index_pip_joint", "L_index_dip_joint",
            "L_middle_pip_joint", "L_middle_dip_joint",
            "L_ring_abad_joint", "L_ring_pip_joint", "L_ring_dip_joint",
            "L_pinky_abad_joint", "L_pinky_pip_joint", "L_pinky_dip_joint",
        };


        // 发布 mujoco 角度 指令（转发 MoveIt 轨迹到mujoco）10关节
        pub_motor_angle_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/agihand/omnihand/left/motor_angle_cmd", 10);

        // // 发布真实关节状态给 MoveIt + robot_state_publisher 16关节 由mujoco反馈回调发布
        // pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10);

        // 订阅mujoco 反馈（真实关节角度 100Hz） 10关节
        sub_motor_angle_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/agihand/omnihand/left/motor_angle", 10,
            std::bind(&HandControl::motor_angle_callback, this, _1));

        // 订阅 MoveIt fake controller 轨迹插值输出（重映射到 /controller_joint_states）
        sub_controller_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/controller_joint_states", 10,
            std::bind(&HandControl::controller_states_callback, this, _1));

        // 定时器：50Hz 周期性重发 /joint_states，防止驱动断更导致 MoveIt 超时
        cached_positions_.resize(all_joint_names_.size(), 0.0);
        timer_republish_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HandControl::republish_joint_states, this));
        timer_republish_->cancel(); // 初始时不启动，等收到过关节数据后再启动

        //初始化关节消息数据
        js_msg_.name = all_joint_names_;
        js_msg_.position.resize(all_joint_names_.size(), 0.0);

    }

private:
    std::unordered_map<std::string, int> joint_index_map_;// MoveIt 关节名 → MotorAngle.angles[] 索引
    std::vector<std::string> all_joint_names_;// 全部16个关节名（10主动 + 6 mimic）
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_control_mode_;// 发布控制模式
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_motor_angle_;//发布电机角度指令
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;//发布关节状态
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_motor_angle_;//订阅驱动反馈的关节角度
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_controller_states_;//订阅 MoveIt fake controller 输出的轨迹指令
    rclcpp::TimerBase::SharedPtr timer_republish_;// 定时器：周期性重发 /joint_states，防止驱动断更导致 MoveIt 超时
    rclcpp::TimerBase::SharedPtr timer_control_mode_;// 定时器：周期性发送控制模式，确保驱动收到
    int control_mode_send_count_ = 0;// 已发送控制模式的次数
    std::vector<double> cached_positions_;//最近一次收到的关节状态
    bool has_joint_data_ = false;//是否已收到过关节状态数据
    rclcpp::Time last_motor_time_;  // 最近一次收到驱动反馈的时间
    std::vector<double> last_cmd_angles_ = std::vector<double>(10, 0.0);  // 上次下发的指令角度
    bool gate_open_ = false;      // 门控：fake controller 与真实手对齐后才允许转发
    bool driver_connected_ = false;  // 驱动是否在线（收到过反馈且未超时）

    sensor_msgs::msg::JointState js_msg_; // 关节状态消息对象，避免每次回调都重新创建


    // 驱动反馈回调：真实关节 → /joint_states（MoveIt 碰撞检测 + TF 可视化）消息,不发布指令
    void motor_angle_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 10) return;

        double thumb_roll = msg->position[0];
        double thumb_abad = msg->position[1];
        double thumb_mcp  = msg->position[2];
        double index_abad = msg->position[3];
        double index_pip  = msg->position[4];
        double middle_pip = msg->position[5];
        double ring_abad  = msg->position[6];
        double ring_pip   = msg->position[7];
        double pinky_abad = msg->position[8];
        double pinky_pip  = msg->position[9];

        // SDK 多项式计算被动关节（kinematics_solver.cc: GetAllJointPos / CalculatePower）
        // 线性近似（已弃用，仅作参考）：
        // double thumb_pip = 1.33 * thumb_mcp;
        // double thumb_dip = 1.42 * thumb_mcp;
        // double index_dip  = 1.29 * index_pip;
        // double middle_dip = 1.29 * middle_pip;
        // double ring_dip   = 1.29 * ring_pip;
        // double pinky_dip  = 1.29 * pinky_pip;

        // SDK 多项式计算被动关节（更精确，已启用）待测试
        // thumb_mcp2pip_poly_ = {0.0, 1.33}
        double thumb_pip = 1.33 * thumb_mcp;
        // thumb_mcp2dip_poly_(左手) = {0.0, 1.846, +0.853, 0.280}
        // 左手构造函数中对 [2] 取反：-(-0.853) = +0.853
        double thumb_dip = 1.846 * thumb_mcp
                         + 0.853 * thumb_mcp * thumb_mcp
                         + 0.280 * thumb_mcp * thumb_mcp * thumb_mcp;
        // finger_pip2dip_poly_ = {0.0, 2.192, -1.425, 0.747, -0.167}
        double index_dip  = 2.192 * index_pip  - 1.425 * index_pip  * index_pip  + 0.747 * index_pip  * index_pip  * index_pip  - 0.167 * index_pip  * index_pip  * index_pip  * index_pip;
        double middle_dip = 2.192 * middle_pip - 1.425 * middle_pip * middle_pip + 0.747 * middle_pip * middle_pip * middle_pip - 0.167 * middle_pip * middle_pip * middle_pip * middle_pip;
        double ring_dip   = 2.192 * ring_pip   - 1.425 * ring_pip   * ring_pip   + 0.747 * ring_pip   * ring_pip   * ring_pip   - 0.167 * ring_pip   * ring_pip   * ring_pip   * ring_pip;
        double pinky_dip  = 2.192 * pinky_pip  - 1.425 * pinky_pip  * pinky_pip  + 0.747 * pinky_pip  * pinky_pip  * pinky_pip  - 0.167 * pinky_pip  * pinky_pip  * pinky_pip  * pinky_pip;

        
        //填充关节状态数据，顺序必须与 all_joint_names_ 定义的顺序一致
        // js_msg_.position = {
        //     thumb_roll, thumb_abad, thumb_mcp, thumb_pip, thumb_dip,
        //     index_abad, index_pip, index_dip,
        //     middle_pip, middle_dip,
        //     ring_abad, ring_pip, ring_dip,
        //     pinky_abad, pinky_pip, pinky_dip,
        // };

        // 缓存最新关节状态，供定时器重发,并且在门控机制中判断 fake controller 是否已对齐
        cached_positions_ = {
            thumb_roll, thumb_abad, thumb_mcp, thumb_pip, thumb_dip,
            index_abad, index_pip, index_dip,
            middle_pip, middle_dip,
            ring_abad, ring_pip, ring_dip,
            pinky_abad, pinky_pip, pinky_dip,
        };

        // js.position[0] = msg->angles[0];
        // js.position[1] = msg->angles[1];
        // js.position[2] = msg->angles[2];
        // js.position[3] = msg->angles[2] * 1.33; // thumb_pip
        // js.position[4] = msg->angles[2] * 1.42; // thumb_dip
        // js.position[5] = msg->angles[3];
        // js.position[6] = msg->angles[4];
        // js.position[7] = msg->angles[4] * 1.29; // index_dip
        // js.position[8] = msg->angles[5];
        // js.position[9] = msg->angles[5] * 1.29; // middle_dip
        // js.position[10] = msg->angles[6];
        // js.position[11] = msg->angles[7];
        // js.position[12] = msg->angles[7] * 1.29; // ring_dip
        // js.position[13] = msg->angles[8];
        // js.position[14] = msg->angles[9];
        // js.position[15] = msg->angles[9] * 1.29; // pinky_dip

        has_joint_data_ = true;//已经收到关节数据
        last_motor_time_ = this->get_clock()->now();//更新最近一次收到驱动反馈的时间
        if (!driver_connected_) {
            driver_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "灵巧手驱动已连接，开始接收反馈");
        }

        // 定时器：50Hz 周期性重发 /joint_states，防止驱动断更导致 MoveIt 超时
        // 只有在收到过关节数据后才启动定时器，避免初始阶段发送全零状态
        if (has_joint_data_ && timer_republish_->is_canceled()) {
            timer_republish_->reset();
            RCLCPP_INFO(this->get_logger(), "已收到关节数据，启动定时重发 /joint_states");
        }

    }

    // 定时重发 /joint_states，确保 MoveIt 始终看到带新时间戳的关节状态
    // MoveIt 在执行轨迹前会验证关节状态时间戳，必须持续发布
    void republish_joint_states() {
        if (!has_joint_data_) return;

        // 检测驱动是否掉线（1秒无反馈）
        if (driver_connected_) {
            auto now = this->get_clock()->now();
            if ((now - last_motor_time_).seconds() > 1.0) {
                driver_connected_ = false;
                RCLCPP_ERROR(this->get_logger(),
                    "灵巧手驱动掉线！已超过1秒无反馈。请检查USB连接并重启驱动。");
                rclcpp::shutdown(); // 直接关闭节点，避免 MoveIt 显示"成功"但真实手没动
            }
        }

        js_msg_.header.stamp = this->get_clock()->now();// 更新时间戳
        js_msg_.position = cached_positions_;// 使用缓存的最新关节状态
        pub_joint_states_->publish(js_msg_);
    }

    // MoveIt 轨迹回调：将规划指令转发给真实驱动
    // 只有当指令位置相对上次有变化时才转发，避免 idle 状态下
    // joint_state_broadcaster 的 100Hz 静态重复消息冲击真实驱动
    void controller_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 驱动掉线时不转发指令，防止MoveIt显示"成功"但真实手没动
        if (!driver_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "驱动已掉线，忽略轨迹指令。请重启灵巧手驱动。");
            return;
        }
        sensor_msgs::msg::JointState motor_msg;
        motor_msg.header.stamp = this->get_clock()->now();
        motor_msg.header.frame_id = "angle_frame";
        motor_msg.position.resize(10, 0.0);

        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = joint_index_map_.find(msg->name[i]);
            if (it != joint_index_map_.end()) {
                motor_msg.position[it->second] = msg->position[i];
            }
        }

        //判断是否已经收到过关节的状态数据,如果没有收到过,说明还没有建立起与驱动的通信,此时不转发指令,避免发送无效指令导致驱动异常
        if (!has_joint_data_) {
                last_cmd_angles_ = motor_msg.position;
                return;  // 驱动未连接，等待
        }
        // 门控机制：等待 fake controller 内部状态与真实手位置对齐后再开放转发
        // 只在开头被触发
        //
        // 背景：mock_components/GenericSystem 初始状态为 0（initial_positions.yaml）。
        // 当第一条 set_shape 指令下发时，joint_trajectory_controller 会在轨迹执行前
        // 将 fake controller 从 0 插值过渡到轨迹起点（真实手位置 P）。
        // 若将这段 0→P 的插值帧直接转发给驱动，真实手会先反向运动到 0 附近，
        // 再回到 P、最终到目标 T，即大 角度抖动→执行命令 现象。
        //
        // 解决方案：检测 fake controller 状态是否已接近真实手位置，
        // 只有对齐后（gate_open_ = true）才允许转发，从而跳过 0→P 插值阶段。
        if (!gate_open_) {
            
            // 检查 fake controller 各关节是否已接近真实手位置（容差 0.1 rad ≈ 5.7°）
            bool all_close = true;
            for (const auto& [name, motor_idx] : joint_index_map_) {
                for (size_t j = 0; j < all_joint_names_.size(); ++j) {
                    if (all_joint_names_[j] == name) {
                        if (std::abs(motor_msg.position[motor_idx] - cached_positions_[j]) > 0.1) {
                            all_close = false;
                        }
                        break;
                    }
                }
                if (!all_close) break;
            }
            last_cmd_angles_ = motor_msg.position;  // 始终跟踪，避免对齐后第一帧 diff 过大
            if (all_close) {
                gate_open_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[Gate] fake controller 已与真实手对齐，开始转发轨迹指令");
            }
            return;  // 对齐前本帧始终不转发
        }

        // 检测指令是否有变化（阈值 0.001 rad ≈ 0.06°）
        bool changed = false;
        double diff;
        for (int i = 0; i < 10; ++i) {
            diff = std::abs(motor_msg.position[i] - last_cmd_angles_[i]);
            if (diff > 0.001) {
                changed = true;
                break;
            }
        }
        if (!changed){
            last_cmd_angles_ = motor_msg.position; // 更新基准，防止后续微小变化被忽略
            return;
        }   // 静态重复消息，跳过

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        //     "转发轨迹指令 → 驱动 (最大变化: 关节[%d] %.4f rad)", max_diff_idx, max_diff);
        last_cmd_angles_ = motor_msg.position;
        pub_motor_angle_->publish(motor_msg);
    }

    // void relink_sdk() {
    //     // 该函数用于在检测到驱动掉线后尝试重新建立连接
    //     // 目前实现为重启节点，实际应用中可改为更细粒度的重连逻辑
    //     rclcpp::Rate rate(0.5); // 重连频率 0.5Hz
    //     while (rclcpp::ok())
    //     {
    //         RCLCPP_INFO(this->get_logger(), "尝试重新连接驱动...");
    //         rate.sleep();
    //     }
    // }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandControl>());
    rclcpp::shutdown();
    return 0;
}