#ifndef TGF_LOCAL_PLANNER_H_
#define TGF_LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265

using namespace std;

namespace tgf_local_planner{

    struct position{
        double x, y, az;
    };
    
    struct edges{
    	int down;
    	double degree, depth;
    };
    
    struct regions{
    	double start, finish;
    	double startDepth, finishDepth, length, midAngle;
    	bool checked;
    };

class TGFPlannerROS : public nav_core::BaseLocalPlanner{
public:

    TGFPlannerROS();
    TGFPlannerROS(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~TGFPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    bool initialized_;

    position now;
    position next;
    position error;
    position midError;
    position midGoal;
    double distance_;
    double midDistance_;
    int plan_count;
    int length;
    bool first_time;
    bool finish;

    edges edge[540]; //maximum number of gaps in 720 samples
    regions region[270]; // maximum of regions is 540/2
    edges closestObstacle;
    
    //MAY NOT BE USEFUL
    double FTGYaw;
    double conto;

    bool obstaclesNear;
    double sgAngle;
    double vgAngle;
    int numberRegions;
    int numberEdges;
    bool thereIsRegion;
    double arc;
    double upValue;
    double downValue;
    bool highSafety;
    bool freePath;
    int step = 0;

    //laser has 720 samples
    const int laserMax = 720;
    double R;
    
    ros::Publisher vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber amcl_sub;
    geometry_msgs::Twist cmd;
    sensor_msgs::LaserScan laserData;
    std::vector<geometry_msgs::PoseStamped> plan;

    //laserCallback: function called whenever a laser msg is published

    void laserCallback(const sensor_msgs::LaserScan::Ptr& msg);

    //amclCallback: function called whenever an amcl message is published
    
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

    //setNowError: calculates the current distance to the goal and the angle error to the goal
    
    void setNowError();

    //getYaw: calculates yaw angle with the message the amcl gave
    
    double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

    //setGoal: sets the next subgoal
    
    void setGoal();

    //setVel: sets the velocity parameters of the robot
    
    void setVel();

    //setRot: sets the rotation of the robot if the angle to the next subgoal is too wide
    
    void setRot();

    //setRegions: calculates the regions the laser can provide with edges
    
    void setRegions();

    //checkRegions: verifies that the regions calculated are safe to drive in and chooses the max region
    
    void checkRegions();

    //checkSituation: selects between the 4 different situations in Tangencial Gap Flow and returns it in an int value

    int checkSituation();

    //setVGAngle: computes the angle of ner obstacle avoidance related to the safety frontier

    void setVGAngle();
    
    //setGoal: sets the next mid goal
    
    void setMidGoal();

    //setMidError: calculates the current distance to the mid goal and the angle error to the mid goal
    
    void setMidError();
};
};

#endif
