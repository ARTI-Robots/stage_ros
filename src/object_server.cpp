#include <stage_ros/object_server.h>

namespace stage_ros
{

ObjectServer::ObjectServer(ros::NodeHandle nh, StageNode* stage) :
        node_(nh),
        stage_(stage),
        create_action_server_(node_, "/stage/create", boost::bind(&ObjectServer::executeCreate, this, _1), false)
{
  create_action_server_.start();
}

ObjectServer::~ObjectServer()
{
}

void ObjectServer::executeCreate(const stage_ros::createGoalConstPtr&)
{
  stage_ros::createResult res;

  Stg::Model *model = new Stg::ModelPosition(stage_->world, NULL, "asdfjhasf");
  model->SetColor(Stg::Color::blue);
  Stg::Geom geom;
  geom.size.x = 1.0;
  geom.size.y = 1.0;
  geom.size.z = 1.0;
  model->SetGeom(geom);
  stage_->addModel(model);

  create_action_server_.setSucceeded(res);

//    tedusar_exploration_msgs::MoveToPathResult res;
//
//    res.move_to_path.header.frame_id = "map";
//
//    tf::Stamped<tf::Pose> robot_pose_tf;
//    costmap_2d_ros_move_to_->getRobotPose(robot_pose_tf);
//
//    geometry_msgs::PoseStamped start_pose;
//    tf::poseStampedTFToMsg(robot_pose_tf, start_pose);
//
//    if(!hector_move_to_planner_->makePlan(start_pose, goal->goto_point, res.move_to_path.poses))
//    {
//        move_to_server_.setAborted(res, "exporation of hector failed");
//        return;
//    }
//
//    res.move_to_path.header.stamp = ros::Time::now();
//
//    if(move_to_server_.isPreemptRequested())
//    {
//        move_to_server_.setPreempted();
//        return;
//    }
//
//    plan_pub_.publish(res.move_to_path);
//    move_to_server_.setSucceeded(res);
}

}  // namespace stage_ros
