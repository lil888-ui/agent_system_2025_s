#include <cnoid/SimpleController>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <iostream>
#include <Eigen/Geometry>
#include <fstream>    // 追加
#include <geometry_msgs/PoseStamped.h>     // ← 追加
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // ← ここを追加
#include <tf2/LinearMath/Quaternion.h>//追加

using namespace cnoid;

class RttTankController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    ros::Publisher pose_pub;//追加
    geometry_msgs::Twist latest_command_velocity;
    std::mutex command_velocity_mutex;

    Link* root_link;  // ルートリンク（ワールド座標取得用）
    std::vector<Link*> wheelsL;  // 左車輪
    std::vector<Link*> wheelsR;  // 右車輪
    Link* turretJoint[2];       // タレットジョイント（Yaw, Pitch）
    double q_ref[2], q_prev[2];
    double dt;

    //7.6
    SimpleControllerIO* io_;
    long long loop_count{0};


    // 車輪パラメータ
    static constexpr double WHEEL_RADIUS = 0.05; // メートル
    static constexpr double TREAD_WIDTH  = 0.35; // メートル

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        //7.6
        io_ = io;


        // ROSノード初期化
        if(!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "rtt_tank_controller", ros::init_options::AnonymousName);
        }
        node.reset(new ros::NodeHandle());
        subscriber = node->subscribe(
            "/cmd_vel", 1,
            &RttTankController::command_velocity_callback, this);

        Body* body = io->body();
        dt = io->timeStep();
        if(!body) {
            std::cerr << "Error: Choreonoid Body not found!" << std::endl;
            return false;
        }
        std::cout << "Choreonoid Body found: " << body->name() << std::endl;

        // ルートリンクの絶対位置・姿勢入力を有効化
        io->enableInput(body->rootLink(), LinkPosition);
        root_link = body->rootLink();

        // 車輪リンクを取得し、JointVelocity出力を有効化
        // 左側車輪
        wheelsL.push_back(body->link("WHEEL_L_1"));
        wheelsL.push_back(body->link("WHEEL_L_2"));
        wheelsL.push_back(body->link("WHEEL_L_3"));
        // 右側車輪
        wheelsR.push_back(body->link("WHEEL_R_1"));
        wheelsR.push_back(body->link("WHEEL_R_2"));
        wheelsR.push_back(body->link("WHEEL_R_3"));
        for(size_t i = 0; i < wheelsL.size(); ++i) {
            if(!wheelsL[i]) {
                std::cerr << "Error: Left wheel link WHEEL_L_" << (i+1) << " not found!" << std::endl;
                return false;
            }
            std::cout << "Found link: " << wheelsL[i]->name() << std::endl;
            io->enableOutput(wheelsL[i], JointVelocity);
        }
        for(size_t i = 0; i < wheelsR.size(); ++i) {
            if(!wheelsR[i]) {
                std::cerr << "Error: Right wheel link WHEEL_R_" << (i+1) << " not found!" << std::endl;
                return false;
            }
            std::cout << "Found link: " << wheelsR[i]->name() << std::endl;
            io->enableOutput(wheelsR[i], JointVelocity);
        }

        // タレットジョイント設定
        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i = 0; i < 2; ++i) {
            if(!turretJoint[i]) {
                std::cerr << "Error: Turret joint " << i << " not found!" << std::endl;
                return false;
            }
            q_ref[i] = q_prev[i] = turretJoint[i]->q();
            turretJoint[i]->setActuationMode(JointTorque);
            io->enableIO(turretJoint[i]);
        }
        //追加
        pose_pub = node->advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
        return true;
    }

    void command_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(command_velocity_mutex);
        latest_command_velocity = *msg;
    }

    virtual bool control() override
    {
        // ROSコールバック処理
        ros::spinOnce();

        //7.6
        double sim_t = io_->currentTime();
        ++loop_count;

        // ルートリンクの絶対位置・姿勢を取得
        Isometry3 T = root_link->position();
        Vector3 pos = T.translation();
        ROS_INFO("Position: [%.3f, %.3f, %.3f]", pos.x(), pos.y(), pos.z());
        Matrix3 rot = T.rotation();
        Vector3 rpy = rot.eulerAngles(2, 1, 0); // yaw(Z), pitch(Y), roll(X)
        ROS_DEBUG("Orientation (RPY): [%.3f, %.3f, %.3f] rad",
                  rpy.z(), rpy.y(), rpy.x());

        // ========= ここからログ出力を追加 =========
        if(loop_count%5==0){
            std::ofstream ofs("/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log_temp.csv", std::ios::app);
            if(ofs) {
                ofs << sim_t << ","
                    << pos.x() << "," 
                    << pos.y() << "," 
                    << pos.z() << "\n";
            }
        }
    // ========================================


        // 最新のcmd_vel取得
        geometry_msgs::Twist cmd;
        {
            std::lock_guard<std::mutex> lock(command_velocity_mutex);
            cmd = latest_command_velocity;
        }

        // 車輪速度制御
        double linear_vel    = cmd.linear.x;
        double angular_vel_z = cmd.angular.z;
        double left_lin  = linear_vel - (angular_vel_z * TREAD_WIDTH / 2.0);
        double right_lin = linear_vel + (angular_vel_z * TREAD_WIDTH / 2.0);
        double left_ang  = left_lin  / WHEEL_RADIUS;
        double right_ang = right_lin / WHEEL_RADIUS;
        for(Link* wheel : wheelsL) {
            if(wheel) wheel->dq_target() = left_ang;
        }
        for(Link* wheel : wheelsR) {
            if(wheel) wheel->dq_target() = right_ang;
        }

        // タレットPID制御（例）
        static constexpr double P = 200.0;
        static constexpr double D =  50.0;
        for(int i = 0; i < 2; ++i) {
            Link* joint = turretJoint[i];
            double q  = joint->q();
            double dq = (q - q_prev[i]) / dt;
            joint->u() = P * (q_ref[i] - q) + D * (0.0 - dq);
            q_prev[i] = q;
        }


        // ─── ここから rostopic 公開 ───
        geometry_msgs::PoseStamped msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = "world";  // お好みで

        // 位置をセット
        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();

        // 姿勢をクォータニオンでセット
        // roll = rpy.z(), pitch = rpy.y(), yaw = rpy.x()
        tf2::Quaternion quat;
        quat.setRPY(rpy[2], rpy[1], rpy[0]);  // roll, pitch, yaw
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
        msg.pose.orientation.w = quat.w();

        pose_pub.publish(msg);
        // ───────────────────────────────



        return true;
    }

    virtual void stop() override
    {
        subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttTankController)
