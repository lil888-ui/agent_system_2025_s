#include <cnoid/SimpleController>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <mutex>
#include <vector> // std::vector を使用するために追加
#include <iostream> // デバッグ用にインクルード

using namespace cnoid;

class RttTankController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Twist latest_command_velocity;
    std::mutex command_velocity_mutex;

    // 無限軌道から車輪に変更
    std::vector<Link*> wheelsL; // 左側の車輪
    std::vector<Link*> wheelsR; // 右側の車輪
    
    Link* turretJoint[2];
    double q_ref[2], q_prev[2];
    double dt;

    // 車輪の半径とトレッド幅を定義
    static constexpr double WHEEL_RADIUS = 0.05; // bodyファイルから取得した車輪の半径
    static constexpr double TREAD_WIDTH = 0.35;  // WHEEL_L_1/R_1 のY座標の差から計算 (0.175 * 2)

public:
    // initialize() の先頭で ROS 初期化と Subscriber 登録を行う
    virtual bool initialize(SimpleControllerIO* io) override
    {
        // ———— ROS ノード初期化 & NodeHandle 作成 ————
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "rtt_tank_controller", // ノード名を変更
                      ros::init_options::AnonymousName);
        }
        node.reset(new ros::NodeHandle());

        // ———— /cmd_vel をグローバルに subscribe ————
        subscriber = node->subscribe(
            "/cmd_vel", 1,
            &RttTankController::command_velocity_callback, this);

        // ———— Choreonoid 側のセットアップ ————
        Body* body = io->body();
        dt = io->timeStep();

        if (!body) {
            std::cerr << "Error: Choreonoid Body not found!" << std::endl;
            return false; // Bodyがない場合は初期化失敗
        }
        std::cout << "Choreonoid Body found: " << body->name() << std::endl;



        // 車輪リンクの取得と出力有効化
        // 左側の車輪
        wheelsL.push_back(body->link("WHEEL_L_1"));
        wheelsL.push_back(body->link("WHEEL_L_2"));
        wheelsL.push_back(body->link("WHEEL_L_3"));
        // 右側の車輪
        wheelsR.push_back(body->link("WHEEL_R_1"));
        wheelsR.push_back(body->link("WHEEL_R_2"));
        wheelsR.push_back(body->link("WHEEL_R_3"));
        // 各車輪リンクが正しく取得できたか確認
        for(size_t i = 0; i < wheelsL.size(); ++i) {
            if (!wheelsL[i]) {
                std::cerr << "Error: Left wheel link WHEEL_L_" << (i+1) << " not found!" << std::endl;
                return false; // リンクが見つからない場合は初期化失敗
            }
            std::cout << "Found link: " << wheelsL[i]->name() << std::endl;
            io->enableOutput(wheelsL[i], JointVelocity);
        }
        for(size_t i = 0; i < wheelsR.size(); ++i) {
            if (!wheelsR[i]) {
                std::cerr << "Error: Right wheel link WHEEL_R_" << (i+1) << " not found!" << std::endl;
                return false; // リンクが見つからない場合は初期化失敗
            }
            std::cout << "Found link: " << wheelsR[i]->name() << std::endl;
            io->enableOutput(wheelsR[i], JointVelocity);
        }

        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");

        // タレットジョイントの確認
        if (!turretJoint[0]) {
            std::cerr << "Error: Turret Yaw link TURRET_Y not found!" << std::endl;
            return false;
        }
        std::cout << "Found link: " << turretJoint[0]->name() << std::endl;

        if (!turretJoint[1]) {
            std::cerr << "Error: Turret Pitch link TURRET_P not found!" << std::endl;
            return false;
        }
        std::cout << "Found link: " << turretJoint[1]->name() << std::endl;

        for (int i = 0; i < 2; ++i) {
            Link* joint = turretJoint[i];
            // joint が nullptr でないことを確認してからアクセス
            if (joint) {
                q_ref[i] = q_prev[i] = joint->q();
                joint->setActuationMode(JointTorque);
                io->enableIO(joint);
            } else {
                std::cerr << "Error: Turret joint " << i << " is nullptr!" << std::endl;
                return false;
            }
        }

        return true;
    }

    void command_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(command_velocity_mutex);
        latest_command_velocity = *msg;
    }

    virtual bool control() override
    {
        // ———— 制御ループ内で ROS コールバックを回す ————
        ros::spinOnce();

        geometry_msgs::Twist cmd;
        {
            std::lock_guard<std::mutex> lock(command_velocity_mutex);
            cmd = latest_command_velocity;
        }

        // 車輪の角速度指令
        double linear_vel = cmd.linear.x;
        double angular_vel_z = cmd.angular.z;

        // 左右の車輪の線形速度を計算
        // 左車輪の線形速度 = 前進速度 - (旋回速度 * 半トレッド幅)
        double left_wheel_linear_vel = linear_vel - (angular_vel_z * TREAD_WIDTH / 2.0);
        // 右車輪の線形速度 = 前進速度 + (旋回速度 * 半トレッド幅)
        double right_wheel_linear_vel = linear_vel + (angular_vel_z * TREAD_WIDTH / 2.0);
        
        // 線形速度を角速度に変換して各車輪に設定
        double left_wheel_angular_vel = left_wheel_linear_vel / WHEEL_RADIUS;
        double right_wheel_angular_vel = right_wheel_linear_vel / WHEEL_RADIUS;

        for(Link* wheel : wheelsL) {
            if (wheel) wheel->dq_target() = left_wheel_angular_vel;
        }
        for(Link* wheel : wheelsR) {
            if (wheel) wheel->dq_target() = right_wheel_angular_vel;
        }

        // タレット PID 制御（例）
        static constexpr double P = 200.0;
        static constexpr double D =  50.0;
        for (int i = 0; i < 2; ++i) {
            Link* joint = turretJoint[i];
            double q  = joint->q();
            double dq = (q - q_prev[i]) / dt;
            joint->u() = P * (q_ref[i] - q) + D * (0.0 - dq);
            q_prev[i] = q;
        }

        return true;
    }

    virtual void stop() override
    {
        subscriber.shutdown();
    }
};

// コントローラーの名前も分かりやすいように変更
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttTankController)
