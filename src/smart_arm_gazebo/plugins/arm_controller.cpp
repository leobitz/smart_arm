#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "std_msgs/Float32.h"
#include <smart_arm_lib/ArmPose.h>
#include "std_msgs/String.h"
#include <thread>
#include <cstdlib>
#include <math.h>

namespace gazebo
{
    class ArmController : public ModelPlugin
    {
        private: physics::ModelPtr model;
        private: common::PID pid;
        private: physics::JointControllerPtr jointController;
        private: event::ConnectionPtr updateConnection;
        private: std::unique_ptr<ros::NodeHandle> rosNode;
                
        private: ros::Subscriber    rosSub;
        private: ros::CallbackQueue rosQueue;
        private: std::thread        rosQueueThread;
        
        private:std::thread rosDataPublishThread;
        private:ros::Publisher data_pub;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }
            else
            {
                ROS_INFO("Starting +++ArmController+++");
            }

            this->pid = common::PID(0.1, 0, 0);
            
            std::string move_fingers   = "/" + this->model->GetName() + "/smart_arm/move_arm";

            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            this->jointController = this->model->GetJointController();
            this->jointController->Reset();
            this->jointController->AddJoint(model->GetJoint("robot_carrier_to_arm_base_joint"));
            this->jointController->AddJoint(model->GetJoint("arm_base_to_arm_joint"));
            this->jointController->AddJoint(model->GetJoint("arm_to_arm_rotetor_joint"));
            this->jointController->AddJoint(model->GetJoint("arm_rotetor_to_wrist_rx_joint"));
            this->jointController->AddJoint(model->GetJoint("wrist_rx_to_wrist_ry_joint"));
            this->jointController->AddJoint(model->GetJoint("wrist_ry_to_wrist_rz_joint"));


            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<smart_arm_lib::ArmPose>
                (
                    move_fingers,
                    1,
                    boost::bind( &ArmController::set_angle, this, _1), 
                    ros::VoidPtr(), 
                    &this->rosQueue
                );

            this->rosSub = this->rosNode->subscribe(so);
            this->rosQueueThread = std::thread(std::bind(&ArmController::QueueThread, this));
            
            ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
        }

        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        public: void set_angle(const smart_arm_lib::ArmPose::ConstPtr &_msg)
        {

            this->SetTrans("robot_carrier_to_arm_base_joint", _msg->trans_x);            
            this->SetTrans("arm_base_to_arm_joint",           _msg->trans_z);
            this->SetAngle("arm_to_arm_rotetor_joint",        _msg->rot_angle);
            this->SetAngle("arm_rotetor_to_wrist_rx_joint",   _msg->rx_angle);
            this->SetAngle("wrist_rx_to_wrist_ry_joint",      _msg->ry_angle);
            this->SetAngle("wrist_ry_to_wrist_rz_joint",      _msg->rz_angle);

        }

        private: void SetAngle(std::string joint_name, float degree)
        {
            float rad = M_PI * degree / 180;
            std::string name = this->model->GetJoint(joint_name)->GetScopedName();
            this->jointController->SetPositionPID(name, pid);
            this->jointController->SetPositionTarget(name, rad);
            this->jointController->Update();
        }

        private: void SetTrans(std::string joint_name, float loc)
        {
            std::string name = this->model->GetJoint(joint_name)->GetScopedName();
            this->jointController->SetPositionPID(name, pid);
            this->jointController->SetPositionTarget(name, loc);
            this->jointController->Update();
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(ArmController);
}