#include <geometry_msgs/PoseStamped.h>

#ifndef OBJECT
#define OBJECT

namespace stage_ros
{
    class Object
    {
      public:
        Object(std::string id, std::string type) :
          id_(id),
          type_(type)
        {
        }

        virtual ~Object() {}

        void setTrajectory(std::vector<geometry_msgs::PoseStamped> trajectory)
        {
          trajectory_ = trajectory;
        }

        geometry_msgs::PoseStamped getPose(ros::Time now)
        {
          int size = trajectory_.size();
          geometry_msgs::PoseStamped current_pose;
          for(size_t i = 0; i < size ; ++i)
          {
            ros::Duration diff = trajectory_.at(i).header.stamp - now;
            if (diff < ros::Duration(0.0))
            {
              current_pose = trajectory_.at(i);
            }
          }

          return current_pose;
        }

      private:
        std::string id_;
        std::string type_;
        std::vector<geometry_msgs::PoseStamped> trajectory_;

    };
}
#endif
