/*
 * object_server.h
 *
 *  Created on: Dez 15, 2015
 *      Author: Konstantin Lassnig
 */

#ifndef OBJECT_SERVER
#define OBJECT_SERVER

#include <ros/node_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <stage_ros/createAction.h>

//#include <stage_ros/stageros.h>

namespace stage_ros
{

typedef actionlib::SimpleActionServer<stage_ros::createAction> CreateActionServer;

class ObjectServer
{
  public:
    ObjectServer(ros::NodeHandle nh, StageNode* stage);
    virtual ~ObjectServer();

    void executeCreate(const stage_ros::createGoalConstPtr&);

  private:
    ros::NodeHandle node_;
    StageNode* stage_;
    CreateActionServer create_action_server_;
};

}
#endif  /* OBJECT_SERVER */
