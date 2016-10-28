#include <ros/ros.h> 
#include <geometry_msgs/Point.h> 
#include <davinci_kinematics/davinci_kinematics.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <davinci_traj_streamer/trajAction.h>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>

using namespace std;

bool g_got_new_pose = false;
int g_count = 0;
int iter = 0;
double pos_in_cam[3][10];

Eigen::Vector3d g_des_point;
Eigen::Vector3d g_point;

Davinci_fwd_solver g_davinci_fwd_solver; //instantiate a forward-kinematics solver    
Davinci_IK_solver g_ik_solver;
Eigen::Affine3d g_des_gripper_affine1, g_des_gripper_affine2;
Eigen::Affine3d g_affine_lcamera_to_psm_one, g_affine_camera_to_one_tool_tip;
Eigen::Affine3d g_des_gripper1_wrt_base, g_real_gripper1_wrt_base; 
Vectorq7x1 g_q_vec1_start, g_q_vec1_goal, g_q_vec2_start, g_q_vec2_goal, g_q_real;
Matrix base, camera;

trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory 
trajectory_msgs::JointTrajectoryPoint g_trajectory_point1, g_trajectory_point2;
davinci_traj_streamer::trajGoal g_goal;

// here is a "goal" object compatible with the server, as defined in example_action_server/action
davinci_traj_streamer::trajGoal goal;

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}


void move_tip(){
        //g_got_new_pose = true;
        g_trajectory_point1 = g_trajectory_point2; //now 40
        g_trajectory_point2.positions[0] += 0.025;
        g_trajectory_point2.positions[1] += 0.0175;
        g_trajectory_point2.positions[2] += 0.005;
        g_trajectory_point2.positions[3] += 0.05;
        g_trajectory_point2.positions[4] += 0.0375;
        g_trajectory_point2.positions[5] += 0.0325;
        g_trajectory_point2.positions[6] += 0;
        //should fix up jaw-opening values...do this later
        g_des_trajectory.header.stamp = ros::Time::now();

        g_des_trajectory.points[0] = g_des_trajectory.points[1]; //former goal is new start
        g_des_trajectory.points[1] = g_trajectory_point2;
        // copy traj to goal:
        g_goal.trajectory = g_des_trajectory;
        g_got_new_pose = true;
        //iter+=1;
        ros::Duration(0.5).sleep();
}

void inJointCallback(const sensor_msgs::JointState& jt_msg){
    g_q_real(2) = jt_msg.position[0]; 
    g_q_real(1) = jt_msg.position[1];
    g_q_real(3) = jt_msg.position[7];
    g_q_real(6) = jt_msg.position[8];
    g_q_real(4) = jt_msg.position[10];
    g_q_real(5) = jt_msg.position[11];
    g_q_real(0) = jt_msg.position[12];
} 

void inPointCallback(const geometry_msgs::Point& pt_msg) {
    Eigen::Affine3d des_gripper1_wrt_base;

    g_des_point(0) = pt_msg.x;
    g_des_point(1) = pt_msg.y;
    g_des_point(2) = pt_msg.z;
    cout << "received des point = " << g_des_point.transpose() << endl;
    g_des_gripper_affine1.translation() = g_des_point;
    //convert this to base coords:
    g_des_gripper1_wrt_base = g_affine_lcamera_to_psm_one.inverse() * g_des_gripper_affine1;
    //try computing IK:
    if (g_ik_solver.ik_solve(g_des_gripper1_wrt_base)) {
        ROS_INFO("found IK soln");
        g_got_new_pose = true;
        g_q_vec1_start = g_q_vec1_goal;
        g_q_vec1_goal = g_ik_solver.get_soln();
        cout << g_q_vec1_goal.transpose() << endl;
        g_trajectory_point1 = g_trajectory_point2; //former goal is new start

        for (int i = 0; i < 6; i++) {
            g_trajectory_point2.positions[i] = g_q_vec1_goal[i]; //leave psm2 angles alone; just update psm1 goal angles
        }
        //gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
        //    des_trajectory.joint_names.push_back("right_s0"); //yada-yada should fill in names
        g_des_trajectory.header.stamp = ros::Time::now();

        g_des_trajectory.points[0] = g_des_trajectory.points[1]; //former goal is new start
        g_des_trajectory.points[1] = g_trajectory_point2;



        // copy traj to goal:
        g_goal.trajectory = g_des_trajectory;
    } else {
        ROS_WARN("NO IK SOLN FOUND");
    }

}

void doneCb(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

int do_inits() {
    Eigen::Matrix3d R;
    Eigen::Vector3d nvec, tvec, bvec, tip_pos;
    bvec << 0, 0, 1; //default for testing: gripper points "down"
    nvec << 1, 0, 0;
    tvec = bvec.cross(nvec);
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    g_des_gripper_affine1.linear() = R;
    tip_pos << -0.15, -0.03, 0.07;
    g_des_gripper_affine1.translation() = tip_pos; //will change this, but start w/ something legal

    //hard-coded camera-to-base transform, useful for simple testing/debugging
    g_affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
    nvec << -1, 0, 0;
    tvec << 0, 1, 0;
    bvec << 0, 0, -1;
    //Eigen::Matrix3d R;
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    g_affine_lcamera_to_psm_one.linear() = R;

    g_q_vec1_start.resize(7);
    g_q_vec1_goal.resize(7);
    g_q_vec2_start.resize(7);
    g_q_vec2_goal.resize(7);

    g_des_gripper1_wrt_base = g_affine_lcamera_to_psm_one.inverse() * g_des_gripper_affine1;
    g_ik_solver.ik_solve(g_des_gripper1_wrt_base); // compute IK:
    g_q_vec1_goal = g_ik_solver.get_soln();
    g_q_vec1_start = g_q_vec1_goal;
    g_q_vec2_start << 0, 0, 0, 0, 0, 0, 0; //just put gripper 2 at home position
    g_q_vec2_goal << 0, 0, 0, 0, 0, 0, 0;
    //repackage q's into a trajectory;
    //populate a goal message for action server; 
    // initialize with 2 poses that are identical
    g_trajectory_point1.positions.clear();
    g_trajectory_point2.positions.clear();
    //resize these:
    for (int i=0;i<14;i++)  {
        g_trajectory_point1.positions.push_back(0.0); 
        g_trajectory_point2.positions.push_back(0.0); 
    }
        
    for (int i = 0; i < 7; i++) { 
        g_trajectory_point1.positions[i] = g_q_vec1_start[i];
        g_trajectory_point1.positions[i + 7] = g_q_vec2_start[i];
        //should fix up jaw-opening values...do this later
    }

    g_trajectory_point1.time_from_start = ros::Duration(0.0);
    g_trajectory_point2.time_from_start = ros::Duration(2.0);


    g_des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    g_des_trajectory.joint_names.clear();
    //    des_trajectory.joint_names.push_back("right_s0"); //yada-yada should fill in names
    g_des_trajectory.header.stamp = ros::Time::now();

    g_des_trajectory.points.push_back(g_trajectory_point1); // first point of the trajectory 
    g_des_trajectory.points.push_back(g_trajectory_point2);

    // copy traj to goal:
    g_goal.trajectory = g_des_trajectory;
    g_got_new_pose = true; //send robot to start pose
    
     ROS_INFO("getting transforms from camera to PSMs");
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one, tfResult_two;
    bool tferr = true;
    int ntries = 0;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        if (ntries > 5) break; //give up and accept default after this many tries
          tferr= false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("left_camera_optical_frame", "one_psm_base_link", ros::Time(0), tfResult_one);
            //tfListener.lookupTransform("left_camera_optical_frame", "two_psm_base_link", ros::Time(0), tfResult_two);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
        }
    }
    //default transform: need to match this up to camera calibration!
    if (tferr) {
        g_affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
        Eigen::Vector3d nvec, tvec, bvec;
        nvec << -1, 0, 0;
        tvec << 0, 1, 0;
        bvec << 0, 0, -1;
        Eigen::Matrix3d R;
        R.col(0) = nvec;
        R.col(1) = tvec;
        R.col(2) = bvec;
        g_affine_lcamera_to_psm_one.linear() = R;
        g_affine_lcamera_to_psm_one.linear() = R;
        g_affine_lcamera_to_psm_one.translation() << 0.145, -0.03265, 0.0;
        ROS_WARN("using default transform");
    } else {

        ROS_INFO("tf is good");

        //affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
        // need to extend this to camera optical frame
        g_affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
        //affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two);
    }
    ROS_INFO("transform from left camera to psm one:");
    cout << g_affine_lcamera_to_psm_one.linear() << endl;
    cout << g_affine_lcamera_to_psm_one.translation().transpose() << endl;
    //ROS_INFO("transform from left camera to psm two:");
    //cout << affine_lcamera_to_psm_two.linear() << endl;
    //cout << affine_lcamera_to_psm_two.translation().transpose() << endl;    
    
    
    ROS_INFO("done w/ inits");

    return 0;
    
}

double e(double x, double y, double z, double r, double p, double w){
    double error=0;
    for (int i = 0; i<40; i++){
        error += pow(abs((base(i,0)*cos(w)*cos(p)+base(i,1)*(cos(w)*sin(p)*sin(r)-sin(w)*cos(r))+ base(i, 2)*(cos(w)*sin(p)*cos(r)+sin(r)*sin(w))+x-camera(i, 0))), 2)+ pow(abs((base(i, 0)*sin(w)*cos(p)+base(i, 1)*(sin(w)*sin(p)*sin(r)+cos(w)*cos(r))+ base(i, 2)*(sin(w)*sin(p)*cos(r)-cos(w)*sin(r))+y-camera(i, 1))), 2)+ pow(abs((-base(i, 0)*sin(p)+base(i, 1)*cos(p)*sin(r)+ base(i, 2)*cos(p)*cos(r)+z-camera(i, 2))), 2);
    }
    return error;
}

double x_partial(double x, double y, double z, double r, double p, double w) {

    double x_prime = (e((x + 0.000001),y,z,r,p,w) - e((x - 0.000001), y,z,r,p,w)) / 0.000002; 
    return x_prime;
}

double y_partial(double x, double y, double z, double r, double p, double w) {

    double y_prime = (e(x,y + 0.000001,z,r,p,w) - e(x, y - 0.000001,z,r,p,w)) / 0.000002; 
    return y_prime;
}

double z_partial(double x, double y, double z, double r, double p, double w) {

    double z_prime = (e(x,y,z + 0.000001,r,p,w) - e(x, y,z - 0.000001,r,p,w)) / 0.000002;
    return z_prime;
}

double r_partial(double x, double y, double z, double r, double p, double w) {

    double r_prime = (e(x,y,z,r + 0.000001,p,w) - e(x, y,z,r - 0.000001,p,w)) / 0.000002; 
    return r_prime;
}

double p_partial(double x, double y, double z, double r, double p, double w) {

    double p_prime = (e(x,y,z,r,p + 0.000001,w) - e(x, y,z,r,p - 0.000001,w)) / 0.000002; 
    return p_prime;
}

double w_partial(double x, double y, double z, double r, double p, double w) {

    double w_prime = (e(x,y,z,r,p,w + 0.000001) - e(x, y,z,r,p,w - 0.000001)) / 0.000002; 
    return w_prime;
}


void isd(){
    double tol = 0.000001; // tolerance for convergence
    int max_iter = 1000000; // maximum number of iterations
    double dmin = 0.000001;
    double alpha = 0.01;
    double gnorm = 100000;
    double a, b, c;
    double g = 0; 
    //double xopt, yopt, zopt, ropt, popt, wopt;
    double gradx, grady, gradz, gradr, gradp, gradw, x=0, y=0, z=0, r=0, p=0, w=0;
    //double x0 = 0, y0 =  0, z0 = 0, r0 = 0, p0 = 0, w0 = 0;
    int niter =0; 
    double last_fit;
    double dnorm=100000;
    double d = 0;
    //last_fit = e(x0,y0,z0,r0,p0,w0);
    while (niter<=max_iter && gnorm>=tol && dnorm>=dmin){
        //a = e(0.000001,0,0,0,0,0) ;
        //b = e((-0.000001), 0,0,0,0,0) ;                // why????? b = 2a ??? 
        //c = e(0,0,0,0,0,0);
        gradx = x_partial(x, y, z, r, p, w);
        grady = y_partial(x, y, z, r, p, w);
        gradz = z_partial(x, y, z, r, p, w);
        gradr = r_partial(x, y, z, r, p, w);
        gradp = p_partial(x, y, z, r, p, w);
        gradw = w_partial(x, y, z, r, p, w);
        x = x - alpha*gradx;
        y = y - alpha*grady;
        z = z - alpha*gradz;
        r = r - alpha*gradr;
        p = p - alpha*gradp;
        w = w - alpha*gradw;
        g = pow(abs(gradx), 2) + pow(abs(grady), 2) + pow(abs(gradz), 2) + pow(abs(gradr), 2) + pow(abs(gradp), 2) + pow(abs(gradw), 2);
        gnorm = sqrt(g);
        d = pow(abs(alpha*gradx), 2) + pow(abs(alpha*grady), 2) + pow(abs(alpha*gradz), 2) + pow(abs(alpha*gradr), 2) + pow(abs(alpha*gradp), 2) + pow(abs(alpha*gradw), 2);
        dnorm = sqrt(d);
        niter++;
    }

    
    cout<<"The transform between base & camera is: " << endl << endl;
    cout<<"x: "<< x << endl;
    cout<<"y: "<< y << endl;
    cout<<"z: "<< z << endl;
    cout<< "roll: "<< r << endl;
    cout<< "pitch: "<< p << endl;
    cout<< "yaw: " << w << endl;    

    cout<< "number of iteration: " << niter << endl;
    //last_fit = e(x0,y0,z0,r0,p0,w0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pointer");
    ros::NodeHandle n;
    //ros::Subscriber theJoints = n.subscribe("/davinci/joint_states", 1, inJointCallback);

    
    if (do_inits()!=0) return 1; //poor-man's constructor       
    
     actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> g_action_client("trajActionServer", true);
   // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    int max_tries = 0;
    while (!server_exists) {
        server_exists = g_action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("retrying...");
        max_tries++;
        if (max_tries > 100)
            break;
    }

    if (!server_exists) {
        ROS_WARN("could not connect to server; quitting");
        return 1; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); // if here, then we connected to the server;    
    
    for (int i=0;i<40;i++){
        move_tip();
        //ros::Subscriber theJoints = n.subscribe("/davinci/joint_states", 1, inJointCallback);
        if (g_got_new_pose) {
            ROS_INFO("sending new goal");
            g_got_new_pose = false;
               // stuff a goal message:
                g_count++;
            g_goal.traj_id = g_count; // this merely sequentially numbers the goals sent
            ROS_INFO("sending traj_id %d", g_count);
            //g_action_client.sendGoal(g_goal, &doneCb);
            g_action_client.sendGoal(g_goal);
            bool finished_before_timeout = g_action_client.waitForResult(ros::Duration(5.0));
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result for goal number %d", g_count);
                return 0;
            } else {
                ROS_INFO("finished before timeout");
            }
            ros::Duration(3).sleep();

            ros::Subscriber theJoints = n.subscribe("/davinci/joint_states", 1, inJointCallback);
            //Eigen::Affine3d Davinci_fwd_solver::fwd_kin_solve(const Vectorq7x1& q_vec)
            g_real_gripper1_wrt_base = g_davinci_fwd_solver.fwd_kin_solve(g_q_real); 
            cout<< "the "<< i+1 <<"th point's coord wrt base:"<< g_real_gripper1_wrt_base.translation().transpose() <<endl;
            for (int j=0;j<3;j++){
                base(i,j)=g_real_gripper1_wrt_base.translation().transpose()(j);
            }
            //base(i)<< g_real_gripper1_wrt_base.translation().transpose();

            //get the transform to know coordinates in camear frame    
            ROS_INFO("getting transforms from camera to tip");
            tf::TransformListener tfListener;
            tf::StampedTransform tfResult;
            bool tferr = true;
            int ntries = 0;
            ROS_INFO("waiting for tf between base and right_hand...");
            while (tferr) {
                if (ntries > 5) break; //give up and accept default after this many tries
                tferr= false;
                try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                    tfListener.lookupTransform("camera", "one_tool_tip_link", ros::Time(0), tfResult);
                } catch (tf::TransformException &exception) {
                    ROS_WARN("%s", exception.what());
                    tferr = true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                    ros::spinOnce();
                    ntries++;
                }
            }
            if(!tferr){
                ROS_INFO("tf is good");
            }
            g_affine_camera_to_one_tool_tip = transformTFToEigen(tfResult);
            cout<< "the "<< i+1 <<"th point's coord wrt camera:" << g_affine_camera_to_one_tool_tip.translation().transpose() <<endl; 
            for (int j=0;j<3;j++){
                camera(i,j)= g_affine_camera_to_one_tool_tip.translation().transpose()(j);
            }
            //camera(i)<< g_affine_camera_to_one_tool_tip.translation().transpose();
            //for(int j=0; j<2; j++){
            //    cout << pos_in_cam[j][i] << endl;
            //}
            //cout<< "the"<< i+1 <<"th point's coord wrt camera:" << pos_in_cam[][i] << endl;        


            //while (ros::ok()) {
            //  ros::spinOnce();
            //if (g_got_new_pose) {
            //  ROS_INFO("sending new goal");
            //g_got_new_pose = false;
               // stuff a goal message:
              //  g_count++;
            //g_goal.traj_id = g_count; // this merely sequentially numbers the goals sent
            //ROS_INFO("sending traj_id %d", g_count);
            //g_action_client.sendGoal(g_goal, &doneCb);
            //g_action_client.sendGoal(g_goal);
            //bool finished_before_timeout = g_action_client.waitForResult(ros::Duration(5.0));
            //            if (!finished_before_timeout) {
            //              ROS_WARN("giving up waiting on result for goal number %d", g_count);
            //               return 0;
            //           } else {
            //               ROS_INFO("finished before timeout");
             //          }
            //          ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
        //ros::spinOnce();
    }
    cout<< endl;
    cout<< "base dataset:"<< endl;
    cout<< base << endl << endl;
    cout<< "camera dataset:" << endl; 
    cout<< camera << endl << endl;
    /*
    base(0,0) = -0.0096675;
    base(1,0) = -0.012446;
    base(2,0) = -0.02596;
    base(3,0) = -0.0364659;
    base(4,0) = -0.037012;
    base(5,0) = -0.0251765;
    base(6,0) = -0.014556;
    base(7,0) = 0.020327;
    base(8,0) = 0.027307;
    base(9,0) = 0.03942;

    base(0,1) = -0.01171;
    base(1,1) = -0.010012;
    base(2,1) = 0.002715;
    base(3,1) = 0.0092682;
    base(4,1) = 0.0072007;
    base(5,1) = -0.0046494;
    base(6,1) = -0.02061;
    base(7,1) = -0.05972;
    base(8,1) = -0.056387;
    base(9,1) = -0.071921;

    base(0,2) = -0.002297;
    base(1,2) = -0.0009375;
    base(2,2) = -0.01262;
    base(3,2) = -0.03381;
    base(4,2) = -0.0624234;
    base(5,2) = -0.09182;
    base(6,2) = -0.11049;
    base(7,2) = -0.19364;
    base(8,2) = -0.11681;
    base(9,2) = -0.12028;

    camera(0,0) = -0.138;
    camera(1,0) = -0.135;
    camera(2,0) =-0.121;
    camera(3,0) =-0.111;
    camera(4,0) =-0.111;
    camera(5,0) =-0.124;
    camera(6,0) =-0.135;
    camera(7,0) =-0.171;
    camera(8,0) =-0.179;
    camera(9,0) =-0.191;

    camera(0,1) =0.06;
    camera(1,1) =0.058;
    camera(2,1) =0.045;
    camera(3,1) =0.039;
    camera(4,1) =0.042;
    camera(5,1) =0.055;
    camera(6,1) =0.072;
    camera(7,1) =0.11;
    camera(8,1) =0.108;
    camera(9,1) =0.124;

    camera(0,2) =0.496;
    camera(1,2) =0.497;
    camera(2,2) =0.485;
    camera(3,2) =0.461;
    camera(4,2) =0.431;
    camera(5,2) =0.401;
    camera(6,2) =0.383;
    camera(7,2) =0.3;
    camera(8,2) =0.377;
    camera(9,2) =0.374;
*/
    isd(); 
    ros::spinOnce();
    return 0; // should never get here, unless roscore dies 
}
