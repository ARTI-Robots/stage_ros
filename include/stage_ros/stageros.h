#ifndef STAGE
#define STAGE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>

#include <mrpt_msgs/ObservationRangeBearing.h>
#include <stage_ros/object_server.h>
#include <stage_ros/SetRobotPose.h>
#include <stage_ros/WheelCmdVel.h>

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
#define LANDMARK_WITHOUT_ID "landmarks_without_id"
#define LANDMARK_WITH_ID "landmarks_with_id"
#define WHEEL_CMD_VEL "wheel_cmd_vel"
#define SET_POSE "set_robot_pose"

// Our node
class StageNode {
private:

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<Stg::ModelFiducial *> fiducialmodels;
    std::vector<Stg::Model *> othermodels;

    //a structure representing a robot inthe simulator
    struct StageRobot
    {
        double wheele_radius;
        double wheele_base;
        //stage related models
        Stg::ModelPosition *positionmodel; //one position
        std::vector<Stg::ModelCamera *> cameramodels; //multiple cameras per position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position
        std::vector<Stg::ModelFiducial *> fiducialmodels; //multiple rangers per position

        //ros publishers
        ros::Publisher odom_pub; //one odom
        ros::Publisher ground_truth_pub; //one ground truth

        std::vector<ros::Publisher> image_pubs; //multiple images
        std::vector<ros::Publisher> depth_pubs; //multiple depths
        std::vector<ros::Publisher> camera_pubs; //multiple cameras
        std::vector<ros::Publisher> laser_pubs; //multiple lasers

        ros::Subscriber cmdvel_sub; //one cmd_vel subscriber

        std::vector<ros::Publisher> landmarks_with_id_pubs;
        std::vector<ros::Publisher> landmarks_without_id_pubs;

        ros::Subscriber wheelcmdvel_subs_;

        ros::ServiceServer set_robot_pose_srvs_;
    };

    std::vector<StageRobot const *> robotmodels_;

    // Used to remember initial poses for soft reset
    std::vector<Stg::Pose> initial_poses;
    ros::ServiceServer reset_srv_;

    ros::Publisher clock_pub_;

    bool isDepthCanonical;
    bool use_model_names;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model *mod, StageNode *node);

    static bool s_update(Stg::World *world, StageNode *node) {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model *mod) const;

    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model *mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;

    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

    stage_ros::ObjectServer* object_server_;

public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char **argv, bool gui, const char *fname, bool use_model_names);

    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();

    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const> &msg);

    // Service callback for soft reset
    bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // Message callback for a Wheel Vel message, which set velocities of the left and right wheel.
    void wheelcmdvelCB(int idx, const boost::shared_ptr<stage_ros::WheelCmdVel const>& msg);

    // Service callback for SetRobotPose, which set the robot to a pose.
    bool setRobotPoseCB(int idx, stage_ros::SetRobotPose::Request &request, stage_ros::SetRobotPose::Response &response);

    //Add a model to the world
    std::string addModel(Stg::Model *);

    void removeModel(const std::string& name);

    Stg::Model* getModel(const std::string& name);
    Stg::Pose getModelPose(const std::string& name);
    Stg::Pose setModelPose(const std::string& name, Stg::Pose pose);
    Stg::Size getModelSize(const std::string& name);
    Stg::Size setModelSize(const std::string& name, Stg::Size size);

    // The main simulator object
    Stg::World *world;
};

#endif  /* STAGE */
