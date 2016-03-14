#include <stage_ros/object_server.h>
#include <stage_ros/stageros.h>
#include <tf/transform_datatypes.h>

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

    //Set Coordinates
    Stg::Pose pose;
    pose.x = spose.pose.position.x;
    pose.y = spose.pose.position.y;

    //Calculate rotation
    geometry_msgs::Quaternion q = spose.pose.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    pose.a = yaw;

    //Send the values to stage
    stage_->setModelPose(it->first, pose);
  }
}

void ObjectServer::executeCreate(const stage_ros::createGoalConstPtr& goal)
{
  stage_ros::createResult res;

  // TODO create object with respect to yaml configuration, otherwise error!

  Stg::Model *model = new Stg::Model(stage_->world, NULL, goal->type);
  model->SetColor(Stg::Color::blue);
  Stg::Geom geom;
  geom.size.x = 0.8;
  geom.size.y = 0.1;
  geom.size.z = 0.3;
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
    ROS_ERROR_STREAM("ObjectServer::executeMove: Could not find id: " << id);
    res.response = -2;
    move_action_server_.setAborted(res);
    return;
  }

  if (goal->trajectory.size() == 0)
  {
    ROS_ERROR_STREAM("ObjectServer::executeMove: The trajectory size is zero");
    res.response = -3;
    move_action_server_.setAborted(res);
    return;
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
