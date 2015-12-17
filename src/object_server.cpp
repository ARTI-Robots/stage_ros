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

  double control_frequence = 30.0;
  trajectory_timer_ = node_.createTimer(ros::Duration(1.0/control_frequence), &ObjectServer::timerTrajectories, this, false );
}

ObjectServer::~ObjectServer()
{
  std::map<std::string,stage_ros::Object*>::iterator it;
  for (it = objects_.begin(); it != objects_.end(); ++it)
    delete it->second;

  objects_.clear();
}

void ObjectServer::timerTrajectories(const ros::TimerEvent& e)
{
  std::map<std::string,stage_ros::Object*>::iterator it;
  for (it = objects_.begin(); it != objects_.end(); ++it)
  {
    geometry_msgs::PoseStamped spose = it->second->getPose(ros::Time::now());

    Stg::Pose pose;
    pose.x = spose.pose.position.x;
    pose.y = spose.pose.position.y;
    stage_->setModelPose(it->first, pose);
  }
}

void ObjectServer::executeCreate(const stage_ros::createGoalConstPtr& goal)
{
  stage_ros::createResult res;

  // TODO create object with respect to yaml configuration, otherwise error!

  Stg::Model *model = new Stg::ModelPosition(stage_->world, NULL, goal->type);
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
}

void ObjectServer::executeMove(const stage_ros::moveGoalConstPtr& goal)
{
  stage_ros::moveResult res;

  std::string id = goal->id;

  if ( objects_.find(id) == objects_.end() )
  {
    //not found
    res.response = -2;
    move_action_server_.setAborted(res);
  }

  objects_[id]->setTrajectory(goal->trajectory);
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
