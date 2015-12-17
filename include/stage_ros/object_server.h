/*
 * object_server.h
 *
 *  Created on: Dez 15, 2015
 *      Author: Konstantin Lassnig
 */

#ifndef OBJECT_SERVER
#define OBJECT_SERVER

#include <map>

#include <ros/node_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <stage_ros/createAction.h>
#include <stage_ros/moveAction.h>
#include <stage_ros/removeAction.h>
#include <stage_ros/object.h>

class StageNode;

namespace stage_ros
{

typedef actionlib::SimpleActionServer<stage_ros::createAction> CreateActionServer;
typedef actionlib::SimpleActionServer<stage_ros::moveAction> MoveActionServer;
typedef actionlib::SimpleActionServer<stage_ros::removeAction> RemoveActionServer;

class ObjectServer
{
  public:
    ObjectServer(ros::NodeHandle nh, StageNode* stage);
    virtual ~ObjectServer();

    void executeCreate(const stage_ros::createGoalConstPtr& goal);
    void executeMove(const stage_ros::moveGoalConstPtr& goal);
    void executeRemove(const stage_ros::removeGoalConstPtr& goal);

  private:
    ros::NodeHandle node_;
    StageNode* stage_;
    CreateActionServer create_action_server_;
    MoveActionServer move_action_server_;
    RemoveActionServer remove_action_server_;

    std::map<std::string,stage_ros::Object*> objects_;
};

}

#endif  /* OBJECT_SERVER */
