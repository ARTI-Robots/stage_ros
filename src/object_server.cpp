#include <stage_ros/object_server.h>
#include <stage_ros/stageros.h>

namespace stage_ros
{

ObjectServer::ObjectServer(ros::NodeHandle nh, StageNode* stage) :
        node_(nh),
        stage_(stage),
        create_action_server_(node_, "/stage/create", boost::bind(&ObjectServer::executeCreate, this, _1), false),
        move_action_server_(node_, "/stage/move", boost::bind(&ObjectServer::executeMove, this, _1), false),
        remove_action_server_(node_, "/stage/remove", boost::bind(&ObjectServer::executeRemove, this, _1), false)
{
  create_action_server_.start();
  move_action_server_.start();
  remove_action_server_.start();
}

ObjectServer::~ObjectServer()
{
}

void ObjectServer::executeCreate(const stage_ros::createGoalConstPtr& goal)
{
  stage_ros::createResult res;

  Stg::Model *model = new Stg::ModelPosition(stage_->world, NULL, "tmp");
  model->SetColor(Stg::Color::blue);
  Stg::Geom geom;
  geom.size.x = 1.0;
  geom.size.y = 1.0;
  geom.size.z = 1.0;
  model->SetGeom(geom);
  std::string id = stage_->addModel(model);
  res.id = id;

  stage_ros::Object* object = new stage_ros::Object(id, goal->type);
  objects_[id] = object;
  create_action_server_.setSucceeded(res);

//    if(!hector_move_to_planner_->makePlan(start_pose, goal->goto_point, res.move_to_path.poses))
//    {
//        move_to_server_.setAborted(res, "exporation of hector failed");
//        return;
//    }
//
//    if(move_to_server_.isPreemptRequested())
//    {
//        move_to_server_.setPreempted();
//        return;
//    }
}

void ObjectServer::executeMove(const stage_ros::moveGoalConstPtr& goal)
{
  stage_ros::moveResult res;

  std::string id = goal->id;

  Stg::Size size;
  size.x = 2.0;
  size.y = 2.0;
  size.z = 2.0;

  objects_[id]->setTrajectory(goal->trajectory);

  geometry_msgs::PoseStamped spose = objects_[id]->getPose(ros::Time::now());

  Stg::Pose pose;
  pose.x = spose.pose.position.x;
  pose.y = spose.pose.position.y;

  stage_->setModelPose(id, pose);
//  stage_->setModelSize(id, size);
  res.response = 0;

  move_action_server_.setSucceeded(res);
}

void ObjectServer::executeRemove(const stage_ros::removeGoalConstPtr& goal)
{
  stage_ros::removeResult res;
  std::string name = goal->id;

  stage_->removeModel(name);
  remove_action_server_.setSucceeded(res);
}

}  // namespace stage_ros
