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
  objects_[id]->setTrajectory(goal->trajectory);

  //Get latest pose value (by time)
  geometry_msgs::PoseStamped spose = objects_[id]->getPose(ros::Time::now());

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
  stage_->setModelPose(id, pose);
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
