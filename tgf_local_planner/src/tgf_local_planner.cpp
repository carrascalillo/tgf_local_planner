#include "tgf_local_planner/tgf_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tgf_local_planner::TGFPlannerROS, nav_core::BaseLocalPlanner)

namespace tgf_local_planner{

TGFPlannerROS::TGFPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

TGFPlannerROS::TGFPlannerROS(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

TGFPlannerROS::~TGFPlannerROS() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void TGFPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }

        //initializing nodes
    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    laser_sub = n.subscribe("/front/scan", 10, &TGFPlannerROS::laserCallback, this);
    amcl_sub = n.subscribe("amcl_pose", 10, &TGFPlannerROS::amclCallback, this);

        //in order to get the global time just one time
    first_time = true;

        //set velocity to 0
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    
    numberRegions =0;
    numberEdges = 0;
    thereIsRegion = false;
    R = 0.25;
    conto = 0;

    obstaclesNear = false;
}

bool TGFPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    if(first_time){
        plan_count = 0;
        plan = orig_global_plan;
        length = plan.size();
        setGoal();
        //ROS_INFO("Longitud del plan: %d", length);
        //ROS_INFO("%f", plan[3].pose.position.x);
        next.x = plan[plan_count].pose.position.x;
        next.y = plan[plan_count].pose.position.y;
        first_time = false;
    }
    return true;
}

bool TGFPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    setNowError();
    setRegions();
    checkRegions();
    checkSituation();
    setVGAngle();
    setMidGoal();
    setMidError();

    //funcionamiento FTG
    //setFTGYaw();
    //region[0].start = edge[0].degree;
    
    if(distance_ < 0.3){
        if((length - 1 - plan_count) <= 1){
            finish = true;
        } else {
            if((length - 10 - plan_count) < 0){
                step = length - plan_count - 1;
                plan_count += step;
            } else {
                plan_count += 10;
                setGoal();
            }
        }
    } else {
        if(fabs(midError.az) > (25*PI/180)){
            setRot();
        } else {
            setVel();
        }
    }
    cmd_vel = cmd;
    vel_pub.publish(cmd_vel);
    
    /*if(conto > 6){
        ROS_INFO("GOAL: %f, %f", next.x, next.y);
        ROS_INFO("MIDGOAL: %f, %f", midGoal.x, midGoal.y);
        if(checkSituation() == 2){ ROS_INFO("vgAngle: %f", vgAngle*180/PI);}
        if(checkSituation() == 3){ ROS_INFO("sgAngle: %f", vgAngle*180/PI);}
        if(checkSituation() == 4){ ROS_INFO("vsgAngle: %f", (vgAngle+sgAngle)*180/PI);}
        conto = 0;
    }
    conto++;*/

    return true;
}

bool TGFPlannerROS::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}

void TGFPlannerROS::laserCallback(const sensor_msgs::LaserScan::Ptr& msg){
    laserData = *msg;

    //reset before rescanning
    obstaclesNear = false;
    closestObstacle.depth = laserData.ranges[120];
    closestObstacle.degree = (120-360)*0.006554075051099062;

    //ROS_INFO("longitud: %ld", laserData.ranges.size());
    for(int i = 0; i < laserMax; i++){
        if(laserData.ranges[i] > 4){
            laserData.ranges[i] = 4;
            
            //security condition
            /*if(laserData.ranges[i] < 0.5){
                securityFlag = true;
            }*/
        }

        if(laserData.ranges[i] <= 0.5){
            obstaclesNear = true;
        }

        //closest obstacle to robot
        if(laserData.ranges[i] < closestObstacle.depth){
            closestObstacle.depth = laserData.ranges[i];
            closestObstacle.degree = (i-360)*0.006554075051099062;
            //ROS_INFO("dmin: %f", closestObstacle.depth);
        }
    }
    
    /*if(securityFlag && (length - 2 - plan_count) < 1 && cooldown == 0){
        plan_count++;
        cooldown = 29;
    }
    
    if(securityFlag){
        cooldown--;
    }
    
    securityFlag = false;*/
    
    /*if(conto < 1){
        for(int i = 0; i<720; i++){
            ROS_INFO("%f", laserData.ranges[i]);
        }
    }
    conto++;*/
}

void TGFPlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg){
    position before;
    //if(hasStarted){
    //  before.x = now.x;
    //  before.y = now.y;
    //}
    before.x = now.x;
    before.y = now.y;
    now.x = msg->pose.pose.position.x;
    now.y = msg->pose.pose.position.y;
    now.az = getYaw(*msg);
    setNowError();

}

double TGFPlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg){

    double quaternion[4];
    quaternion[0] = msg.pose.pose.orientation.x;
    quaternion[1] = msg.pose.pose.orientation.y;
    quaternion[2] = msg.pose.pose.orientation.z;
    quaternion[3] = msg.pose.pose.orientation.w;
    
    double t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
    double t4 = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    
    return std::atan2(t3, t4);
}

void TGFPlannerROS::setNowError(){

    double ang;

    error.x = (next.x - now.x);
    error.y = (next.y - now.y);

    if (error.y == 0 & error.x == 0){  
        ang = now.az;
    }else{  
    ang = std::atan2(error.y, error.x);
    }

    distance_ = std::sqrt(error.x*error.x + error.y*error.y);
    error.az = ang - now.az;

    if ( error.az > PI ){
    error.az -= 2*PI;
    }
    if ( error.az < -PI ){
    error.az += 2*PI;
    }
    //ROS_INFO("Distancia: %f", distance_);
    //ROS_INFO("error.x: %f, error.y: %f, error.az: %f", error.x, error.y, error.az);
    //ROS_INFO("now.x: %f, now.y: %f, next.x: %f, next.y: %f", now.x, now.y, next.x, next.y);

}

void TGFPlannerROS::setGoal(){

    next.x = plan[plan_count].pose.position.x;
    next.y = plan[plan_count].pose.position.y;
    
    /*if(conto > 10){
    
        ROS_INFO("NEXT.X: %f, NEXT.Y: %f", plan[plan_count].pose.position.x, plan[plan_count].pose.position.y);
        conto = 0;
    }
    
    conto++;*/
}

void TGFPlannerROS::setVel(){
    cmd.linear.x = 1.5 * midDistance_;
    cmd.angular.z = 0.75 * midError.az;
    //cmd.linear.x = 1.25 * distance_;
    //cmd.angular.z = 0.75 * FTGYaw;
}

void TGFPlannerROS::setRot(){
    cmd.linear.x = 0.1;
    cmd.angular.z = midError.az;
    //cmd.linear.x = 0.1;
    //cmd.angular.z = FTGYaw;
}

void TGFPlannerROS::setRegions(){

    int j = 1;
    int k = 0;
    
    /*for(int i = 0; i < 720; i++){
    
        if(fabs(laserData.ranges[i] - laserData.ranges[i+1]) > 2*R){
        
        //conto = fabs(laserData.ranges[i] - laserData.ranges[i+1]);
        //ROS_INFO("diferencia: %f", conto);
        
            if(laserData.ranges[i] < laserData.ranges[i+1]){
            
                edge[j].down = -1;
                edge[j].degree = (i-360)*0.375;
                edge[j].depth = laserData.ranges[i];
                
            } else {
            
                edge[j].down = 1;
                edge[j].degree = (i+1-360)*0.375;
                edge[j].depth = laserData.ranges[i];
            }
            j++;
        }
    }*/
    
    for(int i = 120; i < 600; i++){
    
        if(fabs(laserData.ranges[i] - laserData.ranges[i+1]) > 2*R){
        
        //conto = fabs(laserData.ranges[i] - laserData.ranges[i+1]);
        //ROS_INFO("diferencia: %f", conto);
        
            if(laserData.ranges[i] < laserData.ranges[i+1]){
            
                edge[j].down = -1;
                edge[j].degree = (i-360)*0.375;
                edge[j].depth = laserData.ranges[i];
                
            } else {
            
                edge[j].down = 1;
                edge[j].degree = (i+1-360)*0.375;
                edge[j].depth = laserData.ranges[i];
            }
            j++;
        }
    }
    
    edge[0].down = -edge[1].down;
    edge[0].degree = -90;
    edge[0].depth = laserData.ranges[120];
    
    edge[j].down = -edge[j-1].down;
    edge[j].degree = 90;
    edge[j].depth = laserData.ranges[599];
    
    /*edge[0].down = -edge[1].down;
    edge[0].degree = -135;
    edge[0].depth = laserData.ranges[0];
    
    edge[j].down = -edge[j-1].down;
    edge[j].degree = 135;
    edge[j].depth = laserData.ranges[719];*/
    
    j++;
    
    /*for(int i = 0; i < j; i++){
        ROS_INFO("edge[%d].degree: %f", i, edge[i].degree);
    }*/
    
    //ROS_INFO("j: %d", j);
    

    for(int i = 0; i < (j - 1); i++){
    
        if(edge[i].down == -1){
        
            region[k].start = edge[i].degree;
            region[k].startDepth = edge[i].depth;
            region[k].finish = edge[i+1].degree;
            region[k].finishDepth = edge[i+1].depth;
            region[k].checked = 0;
            k++;
            
        } else if (edge[i].down == 1){
        
            if(edge[i+1].down == 1){
            
                region[k].start = edge[i].degree;
                region[k].startDepth = edge[i].depth;
                region[k].finish = edge[i+1].degree;
                region[k].finishDepth = edge[i+1].depth;
                region[k].checked = 0;
                k++;
            }
        }
        
    }
    
    numberEdges = j;
    numberRegions = k;
    
    //ROS_INFO("NUMBER REGIONS: %d", numberRegions);
    /*conto++;
    if(conto > 5){
        for(int i = 0; i < numberEdges; i++){
            ROS_INFO("DEGREE: %f, DOWN: %d", edge[i].degree, edge[i].down);
        }
        conto = 0;
        ROS_INFO("cmd.linear.x: %f, cmd.angular.z; %f", cmd.linear.x, cmd.angular.z);
        ROS_INFO("------------------------------------");
    }*/
}

void TGFPlannerROS::checkRegions(){

    int j;
    double b = 0;
    double c = 0;
    double angle = 0;
    double maxLength = 0;
    double closestAngle = 360;
    
    thereIsRegion = false;
    
    for(int i = 0; i < numberRegions; i++){
    
        if(!region[i].checked){
        
            region[i].checked = true;
            //if((region[i].start > -90 && region[i].start < 90) && (region[i].finish > -90 && region[i].finish < 90)){
            
                angle = (region[i].finish - region[i].start)*PI/180;
                b = region[i].startDepth;
                c = region[i].finishDepth;
                region[i].length = std::sqrt(b*b + c*c - 2 * b * c * std::cos(angle));
                //ROS_INFO("b: %f,  c: %f, LENGTH: %f, i: %d", b, c, region[i].length, i);
                arc = ((region[i].finish + region[i+1].start)*PI/360 - error.az);
            //}
            
            if((region[i].length > 2*R) && (arc < closestAngle)){
                j = i;
                thereIsRegion = true;
                region[j].midAngle = (region[i].finish + region[i].start)/2;
            }
        }
    }
    
    if(thereIsRegion){
        sgAngle = region[j].midAngle - error.az;
    }
    
}

int TGFPlannerROS::checkSituation(){

    //three posible situations, free-path and high-safety, dangerous-path and high safety and low-safety situation.
    //SITUATION 1: FREE PATH AND HIGH SAFETY
    //SITUATION 2: FREE PATH AND LOW SAFETY
    //SITUATION 3: DANGEROUS PATH AND HIGH SAFETY
    //SITUATION 4: DANGEROUS PATH AND LOW SAFETY
    //firstly we check for the path situation

    if(obstaclesNear){
        highSafety = false;
    } else {
        highSafety = true;
    }

    //checks if distance to goal is bigger than distance to obstacles in that direction

    upValue = laserData.ranges[ceil(error.az/0.375)+360];
    downValue = laserData.ranges[floor(error.az/0.375)+360];

    if((distance_ < upValue) && (distance_ < downValue)){
        freePath = true;
    } else {
        freePath = false;
    }

    if (freePath && highSafety) {                       //does not need any action
        return 1;
    } else if (freePath && !highSafety) {               //needs to rotate with vgAngle
        return 2;
    } else if (!freePath && highSafety) {               //needs to rotate with sgAngle
        return 3;
    } else {                                            //needs to rotate with both sgAngle and vgAngle
        return 4;
    }
}

void TGFPlannerROS::setVGAngle(){

    if((fabs(error.az - closestObstacle.degree) < PI) && (std::signbit(error.az) != std::signbit(closestObstacle.degree))){

        if(std::signbit(closestObstacle.degree)){
            vgAngle = PI/2 - (error.az - closestObstacle.degree);
        } else {
            vgAngle = -PI/2 - (error.az - closestObstacle.degree);
        }

    } else if((fabs(error.az - closestObstacle.degree) >= PI) && (std::signbit(error.az) != std::signbit(closestObstacle.degree))){

        if(std::signbit(closestObstacle.degree)){
            vgAngle = 3*PI/2 - (error.az - closestObstacle.degree);
        } else {
            vgAngle = -3*PI/2 - (error.az - closestObstacle.degree);
        }

    } else if((fabs(closestObstacle.degree) >= fabs(error.az)) && (std::signbit(error.az) == std::signbit(closestObstacle.degree))){

        if(std::signbit(closestObstacle.degree)){
            vgAngle = PI/2 - (error.az - closestObstacle.degree);
        } else {
            vgAngle = -PI/2 - (error.az - closestObstacle.degree);
        }

    } else if((fabs(closestObstacle.degree) < fabs(error.az)) && (std::signbit(error.az) == std::signbit(closestObstacle.degree))){

        if(std::signbit(closestObstacle.degree)){
            vgAngle = -PI/2 - (error.az - closestObstacle.degree);
        } else {
            vgAngle = PI/2 - (error.az - closestObstacle.degree);
        }

    } else {
        vgAngle = 0;
    }
    
}

void TGFPlannerROS::setMidGoal(){

    //pos midGoal;
    position relative;
    double distance;
    
    relative.x = next.x - now.x;
    relative.y = next.y - now.y;

    if(checkSituation() == 1){

        midGoal.x = next.x;
        midGoal.y = next.y;

    } else if(checkSituation() == 2){

        midGoal.x = std::cos(vgAngle)*relative.x - std::sin(vgAngle)*relative.y + now.x;
        midGoal.y = std::sin(vgAngle)*relative.x + std::cos(vgAngle)*relative.y + now.y;

    } else if(checkSituation() == 3){

        midGoal.x = std::cos(sgAngle)*relative.x - std::sin(sgAngle)*relative.y + now.x;
        midGoal.y = std::sin(sgAngle)*relative.x + std::cos(sgAngle)*relative.y + now.y;

    } else if(checkSituation() == 4){

        midGoal.x = std::cos(sgAngle + vgAngle)*relative.x - std::sin(sgAngle + vgAngle)*relative.y + now.x;
        midGoal.y = std::sin(sgAngle + vgAngle)*relative.x + std::cos(sgAngle + vgAngle)*relative.y + now.y;

    }
}

void TGFPlannerROS::setMidError(){

    double ang;

    midError.x = (midGoal.x - now.x);
    midError.y = (midGoal.y - now.y);

    if (midError.y == 0 & midError.x == 0){  
        ang = now.az;
    }else{  
    ang = std::atan2(midError.y, midError.x);
    }

    midDistance_ = std::sqrt(midError.x*midError.x + midError.y*midError.y);
    midError.az = ang - now.az;

    if ( midError.az > PI ){
    midError.az -= 2*PI;
    }
    if ( midError.az < -PI ){
    midError.az += 2*PI;
    }
    //ROS_INFO("Distancia: %f", distance_);
    //ROS_INFO("error.x: %f, error.y: %f, error.az: %f", error.x, error.y, error.az);
    //ROS_INFO("now.x: %f, now.y: %f, next.x: %f, next.y: %f", now.x, now.y, next.x, next.y);

}

}
