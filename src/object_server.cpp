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

void ObjectServer::executeMove(const stage_ros::moveGoalConstPtr&)
{

}

void ObjectServer::executeRemove(const stage_ros::removeGoalConstPtr&)
{

}

}  // namespace stage_ros
