#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// MADE by TheConstruct , contact duckfrots@theconstructsim.com for any doubts or questions
// Or leave a request in the public git that contains this code.

namespace gazebo
{
  class PlannarMover : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PlannarMover::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();
      
      // Create a topic name
      std::string plannar_pos_topicName = "/cmd_vel";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "plannar_rosnode",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("plannar_rosnode"));
      
      // Plannar Pose
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            plannar_pos_topicName,
            1,
            boost::bind(&PlannarMover::OnRosMsg_Pos, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&PlannarMover::QueueThread, this));

      ROS_WARN("Loaded PlannarMover Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", this->model->GetName().c_str());
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

    }


    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel)
    {

        std::string model_name = this->model->GetName();

        ROS_DEBUG("Moving model=%s",model_name.c_str());

        this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
        this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));

        ROS_DEBUG("Moving model=%s....END",model_name.c_str());

    }
    
    
    public: void OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &_msg)
    {
        this->MoveModelsPlane(_msg->linear.x, _msg->linear.y,_msg->linear.z, _msg->angular.x, _msg->angular.y, _msg->angular.z);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Direction Value
    int direction = 1;
    // Frequency of earthquake
    double x_axis_pos = 1.0;
    // Magnitude of the Oscilations
    double y_axis_pos = 1.0;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PlannarMover)
}