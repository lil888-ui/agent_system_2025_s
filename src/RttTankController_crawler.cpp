#include <cnoid/SimpleController>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <mutex>

using namespace cnoid;

class RttTankController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Twist latest_command_velocity;
    std::mutex command_velocity_mutex;

    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double q_ref[2], q_prev[2];
    double dt;

public:
    // initialize() の先頭で ROS 初期化と Subscriber 登録を行う
    virtual bool initialize(SimpleControllerIO* io) override
    {
        // ———— ROS ノード初期化 & NodeHandle 作成 ————
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "rtt_tank_controller",
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

        trackL = body->link("TRACK_L");
        trackR = body->link("TRACK_R");
        io->enableOutput(trackL, JointVelocity);
        io->enableOutput(trackR, JointVelocity);

        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for (int i = 0; i < 2; ++i) {
            Link* joint = turretJoint[i];
            q_ref[i] = q_prev[i] = joint->q();
            joint->setActuationMode(JointTorque);
            io->enableIO(joint);
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

        // トラックの速度指令
        trackL->dq_target() = 0.5 * cmd.linear.x - 0.3 * cmd.angular.z;
        trackR->dq_target() = 0.5 * cmd.linear.x + 0.3 * cmd.angular.z;

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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttTankController)
