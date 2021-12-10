#pragma once
#include <Eigen/Core>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mutex>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <quadrotor_msgs/Trajectory.h>

class Quintic{
public:
    Eigen::Matrix<float,6,1> coeffs_;
    Eigen::Matrix<float,5,1> coeffs_d_;
    Eigen::Matrix<float,4,1> coeffs_dd_;
    Eigen::Matrix<float,3,1> coeffs_ddd_;
    float t_=0;
    Quintic(){

    }
    Quintic(Eigen::Matrix<float,6,1> coeffs , float t){

        t_ = t;
        coeffs(2) *=0.5f;

        coeffs_ = coeffs;
        if(fabs(coeffs(3)-coeffs(0))<0.0001){
            t_ = 0;
            return;
        }
        Eigen::Matrix<float,3,3> A;

        float t_sq = t*t;
        float t_th = t_sq*t;
        float t_fo = t_th*t;
        float t_fi = t_fo*t;
        A << t_th,t_fo,t_fi,
                3*t_sq,4*t_th,5*t_fo,
                6*t,12*t_sq,20*t_th;

        Eigen::Matrix<float,3,1> b;
        b<< coeffs(3)-coeffs(0)-coeffs(1)*t-coeffs(2)*t_sq,
                coeffs(4)-coeffs(1)-2*coeffs_(2)*t,
                coeffs(5) - 2*coeffs_(2);
        coeffs_.tail(3) = A.inverse() * b;

        coeffs_d_(2) = coeffs_(3) *3;
        coeffs_d_(3) = coeffs_(4) * 4;
        coeffs_d_(4) = coeffs_(5) * 5;

        coeffs_dd_(1) =coeffs_(3) * 6;
        coeffs_dd_(2) =coeffs_(4) * 12;
        coeffs_dd_(3) =coeffs_(5) * 20;

        coeffs_ddd_(0) = coeffs_dd_(1);
        coeffs_ddd_(1) =coeffs_(4)* 24;
        coeffs_ddd_(2) =coeffs_(5)* 60;
    }

    float ft(float t){

        return coeffs_(0)+coeffs_(1)*t+coeffs_(2)*t*t+coeffs_(3)*t*t*t + coeffs_(4)*t*t*t*t+coeffs_(5)*t*t*t*t*t;
    }

    float ft_d(float t){
        return coeffs_(1) + 2 * coeffs_(2) * t +
                3 * coeffs_(3) * t *t + 4 * coeffs_(4) * t *t*t + 5 * coeffs_(5) * t *t*t*t;
    }

    float ft_dd(float t){
        //        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return  2 * coeffs_(2) +
                6 * coeffs_(3) * t  + 12 * coeffs_(4) * t *t + 20 * coeffs_(5) * t *t*t;

    }

    double evaluate(double t, int der){
        if(t_ < 0.0001){
            return coeffs_(der);
        }
        switch(der){
        case 0:
            return ft(t);
        case 1:
            return ft_d(t);
        case 2:
            return ft_dd(t);
        }

        return 0;
    }

    float ft_ddd(float t){
        Eigen::Matrix<float,3,1> t_v; t_v << 1,t;
        return coeffs_ddd_.dot(t_v);
    }

    void applyConstraint(double constraint_v,double constraint_a){
        if(t_ == 0){
            return;
        }
        double delta_t = 0.02;
        constexpr double kTolerance = 1e-3;
        constexpr size_t kMaxCounter = 20;

        bool within_range = false;
        for(size_t i = 0; i < kMaxCounter; ++i){
            double maxv=0,maxa=0;
            for(double t = 0; t <= t_; t+=delta_t){
                double v = ft_d(t);
                double a = ft_dd(t);
                v*=v;
                a*=a;
                if(v > maxv){
                    maxv = v;
                }

                if(a > maxa){
                    maxa = a;
                }
            }
            maxv = sqrt(maxv);
            maxa = sqrt(maxa);
            double velocity_violation = maxv / constraint_v;
            double acceleration_violation = maxa / constraint_a;

            within_range = velocity_violation <= 1.0 + kTolerance &&
                    acceleration_violation <= 1.0 + kTolerance;
            if (within_range) {
                break;
            }
            double violation_scaling = std::max(
                        1.0, std::max(velocity_violation, sqrt(acceleration_violation)));
            if(violation_scaling>1.8)
                violation_scaling*=0.6;
            double violation_scaling_inverse = 1.0 / violation_scaling;
            scaleQuintic(violation_scaling_inverse);

            t_*=violation_scaling;
        }

    }

    void scaleQuintic(double scale_factor){
        double scale =1.0;
        for(int i = 0; i<coeffs_.size();++i){
            coeffs_[i] *= scale;
            scale *=scale_factor;
        }
    }

};

class SegmentManager4D{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SegmentManager4D(){

    }
    SegmentManager4D(const Eigen::Vector3d &start_position, mav_trajectory_generation::Segment::Vector xyz_segment, std::vector<Quintic> yaw_segments){
        segments_xyz_=xyz_segment;
        segments_yaw_=yaw_segments;
        size_t size = std::max(xyz_segment.size(),yaw_segments.size());
        start_position_ = start_position;
        //set max time, set maxtime segment
        segments_max_time_.resize(size);
        segments_xyz_time_.resize(size);
        segments_yaw_time_.resize(size);
        for(int i = 0;i < size;++i){
            segments_max_time_[i] = std::fmax(xyz_segment[i].getTime(),segments_yaw_[i].t_);
            segments_xyz_time_[i] = xyz_segment[i].getTime();
            segments_yaw_time_[i] = segments_yaw_[i].t_;
            max_time_ += segments_max_time_[i];
        }

    }

    bool sampleTrajectoryAtTime(double t, mav_msgs::EigenTrajectoryPoint *p){
        if(t > max_time_)
            return false;
        Eigen::Vector4d pos = evaluate(t,0);
        Eigen::Vector4d vel = evaluate(t,1);
        Eigen::Vector4d acc = evaluate(t,2);
        p->setFromYaw(pos.w());
        p->setFromYawRate(vel.w());
        p->setFromYawAcc(acc.w());


        p->position_W = pos.head(3);
        p->velocity_W = vel.head(3);
        p->acceleration_W = acc.head(3);

        p->degrees_of_freedom = mav_msgs::MavActuation::DOF4;
        return true;
    }

    Eigen::Vector4d evaluate(double t,int der){
        Eigen::Vector4d res;
        double accumulated_time = 0;
        size_t i;
        for(i = 0; i < segments_max_time_.size();++i){
            accumulated_time += segments_max_time_[i];
            if (accumulated_time > t) {
                break;
            }

        }

        if (i >= segments_max_time_.size()) {
            i = segments_max_time_.size() - 1;
        }
        float relative_t = accumulated_time;
        relative_t -= segments_max_time_[i];
        relative_t = t - relative_t;
        if(segments_xyz_.empty()){
            res.head(3) = start_position_;
        }else{
            if(relative_t > segments_xyz_time_[i]){
                res.head(3) = segments_xyz_[i].evaluate(segments_xyz_[i].getTime()-0.0001,der).head(3);
            }else{
                res.head(3) = segments_xyz_[i].evaluate(relative_t,der).head(3);
            }
        }

        if(relative_t > segments_yaw_time_[i]){
            res(3) = segments_yaw_[i].evaluate(segments_yaw_time_[i],der);
        }else{
            res(3) = segments_yaw_[i].evaluate(relative_t,der);
        }
        return res;
    }

    void sample3D(double start,double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points){
        points.reserve((int) max_time_/dt+1);
        double t = start;
        while(t < max_time_){

            points.push_back(evaluate(t,0).head(3));
            t+=dt;
        }

    }

    double getMaxTime() const{
        return max_time_;
    }
    double max_time_=0;

    mav_trajectory_generation::Segment::Vector segments_xyz_;
    std::vector<Quintic> segments_yaw_;

    std::vector<double> segments_max_time_;
    std::vector<double> segments_xyz_time_;
    std::vector<double> segments_yaw_time_;
    Eigen::Vector3d start_position_;
};

class PolyTrajInterface
{
public:
    typedef std::shared_ptr<PolyTrajInterface> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolyTrajInterface(){

    }

    PolyTrajInterface(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private){
        nh_private_ = (nh_private);
        nh_ = nh;
        nh_private_.param("dt", dt_, dt_);


        nh_.param("max_vel", max_vel_, max_vel_);
        nh_.param("max_acc", max_acc_, max_acc_);


        nh_.param("max_yaw_rate", max_yaw_rate_, max_yaw_rate_);
        nh_.param("max_yaw_acc", max_yaw_acc_, max_yaw_acc_);

        trajectory_pub_ = nh_.advertise<quadrotor_msgs::Trajectory>(
                    "autopilot/trajectory", 10);

        command_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>(
                    mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
        publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                         &PolyTrajInterface::commandTimerCallback,
                                         this, false, false);


    }

    ~PolyTrajInterface(){

    }
    bool computeTrajectoryAndYaw(const Eigen::Vector4d &start_pos_i, const Eigen::Vector4d &start_vel,
                                 const Eigen::Vector4d &goal_pos_i, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &way_pts_i,
                                 mav_trajectory_generation::Trajectory *trajectory, bool nearestYaw = true){

        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > wps = way_pts_i;

        Eigen::Vector4d start_pos = start_pos_i;
        Eigen::Vector4d goal_pos ;
        if(nearestYaw){
            if(wps.empty()){
                goal_pos = goal_pos_i;
                while (start_pos(3) < 0) { start_pos(3) = start_pos(3)+ 2 * M_PI; }
                while (goal_pos(3) < 0) { goal_pos(3) = goal_pos(3)+ 2 * M_PI; }
                while(std::fabs(start_pos(3)-goal_pos(3)) > M_PI){
                    if(start_pos(3) < goal_pos(3)){
                        goal_pos(3) -= 2 * M_PI;
                    }else{
                        goal_pos(3) += 2 * M_PI;
                    }

                }

            }else{
                while (start_pos(3) < 0) { start_pos(3) = start_pos(3)+ 2 * M_PI; }

                Eigen::Vector4d tmp_start = start_pos;

                for(size_t i = 0; i < wps.size();++i){
                    while (wps[i](3) < 0) { wps[i](3) = wps[i](3)+ 2 * M_PI; }

                    while(std::fabs(tmp_start(3)-wps[i](3)) > M_PI){
                        if(tmp_start(3) < wps[i](3)){
                            wps[i](3) -= 2 * M_PI;
                        }else{
                            wps[i](3) += 2 * M_PI;
                        }

                    }
                    tmp_start = wps[i];
                }
                goal_pos = goal_pos_i;

                while (goal_pos(3) < 0) { goal_pos(3) = goal_pos(3)+ 2 * M_PI; }
                while(std::fabs(tmp_start(3)-goal_pos(3)) > M_PI){
                    if(tmp_start(3) < goal_pos(3)){
                        goal_pos(3) -= 2 * M_PI;
                    }else{
                        goal_pos(3) += 2 * M_PI;
                    }

                }
            }
        }else{
            goal_pos = goal_pos_i;
        }
        std::cout << "New Trajectory 3D with waypoints" << start_pos_i.transpose() << " "<< start_vel.transpose() << " " << goal_pos_i .transpose()<< " " << goal_vel.transpose() <<std::endl;
        std::cout << "Dist=: " << (start_pos_i.head(3)-goal_pos_i.head(3)).norm() << std::endl;
        mav_trajectory_generation::Segment::Vector segment_xyz;

        //    if((start_pos.head(3)-goal_pos.head(3)).squaredNorm()> 0.0001)
        //    {
        const int dimension = 3;
        mav_trajectory_generation::Vertex::Vector vertices;

        const int derivative_to_optimize =
                mav_trajectory_generation::derivative_order::ACCELERATION;


        //    optimalTraj.computeTrajPoints(0.01,pts);
        mav_trajectory_generation::Vertex start(dimension), end(dimension);
        start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(start_pos.head<3>()));
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            start_vel.head(3));
        vertices.push_back(start);

        mav_trajectory_generation::Vertex p(dimension);
        for(size_t i = 0; i < wps.size(); ++i){
            const Eigen::Vector4d &next_pts = wps[i];
            p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(next_pts.head<3>()));
            p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            Eigen::Vector3d::Zero());
            p.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                            Eigen::Vector3d(0,0,0));
            vertices.push_back(p);
        }


        end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos.head<3>());
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                          goal_vel.head(3));
        end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                          Eigen::Vector3d(0,0,0));

        vertices.push_back(end);
        // setimate initial segment times
        std::vector<double> segment_times;
        segment_times = estimateSegmentTimes(vertices, max_vel_, max_acc_);

        for(size_t i =0; i < segment_times.size(); ++i){
            if(segment_times[i] < 0.001)
                segment_times[i] = 0.1;
        }

        // Set up polynomial solver with default params
        mav_trajectory_generation::NonlinearOptimizationParameters parameters;
        parameters.max_iterations = 1000;
        parameters.f_rel = 0.05;
        parameters.x_rel = 0.1;
        parameters.time_penalty = 2000.0;
        parameters.initial_stepsize_rel = 0.1;
        parameters.inequality_constraint_tolerance = 0.1;

        // set up optimization problem
        const int N = 6;
        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


        // constrain velocity and acceleration
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_vel_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_acc_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, 5.);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, 5.);

        // solve trajectory
        opt.optimize();

        // get trajectory as polynomial parameters
        opt.getTrajectory(&(*trajectory));
        trajectory->scaleSegmentTimesToMeetConstraints(max_vel_, max_acc_);



        trajectory->getSegments(&segment_xyz);
        //    }

        std::vector<Quintic> segments_yaw;
        float prev_yaw = start_pos.w();
        float prev_vel_yaw = start_vel.w();
        double prev_time = 0;
        if(wps.size() > 0){
            for(size_t i = 0; i < wps.size(); ++i){
                const float next_yaw = wps[i].w();
                Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,next_yaw,0,0;
                double t = fabs(prev_yaw-next_yaw)/max_yaw_rate_;
                Quintic q(state,t);
                //applyconstraint
                q.applyConstraint(max_yaw_rate_,max_yaw_rate_);
                segments_yaw.push_back(q);

                prev_yaw = next_yaw;

                prev_vel_yaw = 0;

            }
        }

        Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,goal_pos.w(),0,0;
        Quintic q(state,fabs(prev_yaw-goal_pos.w())/max_yaw_rate_);
        //apply constraint
        q.applyConstraint(max_yaw_rate_,max_yaw_acc_);
        segments_yaw.push_back(q);
        //    std::cout << goal_pos.w() <<std::endl;

        //    std::cout << q.t_ <<std::endl;
        trajectory_4D_ = SegmentManager4D(start_pos.head(3),segment_xyz,segments_yaw);


        return true;

    }



    void sendTrajectory(){
        mav_msgs::EigenTrajectoryPoint pt;
        float dt = 0.02;
        quadrotor_msgs::Trajectory quad_traj;
        quad_traj.points.reserve(trajectory_4D_.max_time_/dt+1);
        for(float t = 0; t <= trajectory_4D_.max_time_;t+=dt){
            trajectory_4D_.sampleTrajectoryAtTime(t,&pt);
            quadrotor_msgs::TrajectoryPoint quad_pt;
            quad_pt .time_from_start = ros::Duration(t);

            quad_pt.pose.position.x = pt.position_W.x();
            quad_pt.pose.position.y = pt.position_W.y();
            quad_pt.pose.position.z = pt.position_W.z();


            quad_pt.velocity.linear.x = pt.velocity_W.x();
            quad_pt.velocity.linear.y = pt.velocity_W.y();
            quad_pt.velocity.linear.z = pt.velocity_W.z();


            quad_pt.acceleration.linear.x = pt.acceleration_W.x();
            quad_pt.acceleration.linear.y = pt.acceleration_W.y();
            quad_pt.acceleration.linear.z = pt.acceleration_W.z();

            quad_pt.jerk.linear.x = pt.jerk_W.x();
            quad_pt.jerk.linear.y = pt.jerk_W.y();
            quad_pt.jerk.linear.z = pt.jerk_W.z();


            quad_pt.snap.linear.x = pt.snap_W.x();
            quad_pt.snap.linear.y = pt.snap_W.y();
            quad_pt.snap.linear.z = pt.snap_W.z();

            quad_pt.heading = pt.getYaw();
            quad_pt.heading_rate = pt.getYawRate();
            quad_pt.heading_acceleration = pt.getYawAcc();


            quad_traj.points.push_back(quad_pt);
        }

        {
            quadrotor_msgs::TrajectoryPoint quad_pt = quad_traj.points.back();

            quad_pt .time_from_start = ros::Duration(trajectory_4D_.max_time_);

            quad_pt.velocity.linear.x = 0;
            quad_pt.velocity.linear.y = 0;
            quad_pt.velocity.linear.z = 0;


            quad_pt.acceleration.linear.x = 0;
            quad_pt.acceleration.linear.y = 0;
            quad_pt.acceleration.linear.z = 0;

            quad_pt.jerk.linear.x = 0;
            quad_pt.jerk.linear.y = 0;
            quad_pt.jerk.linear.z = 0;


            quad_pt.snap.linear.x = 0;
            quad_pt.snap.linear.y = 0;
            quad_pt.snap.linear.z = 0;

            quad_pt.heading_rate = 0;
            quad_pt.heading_acceleration = 0;


            quad_traj.points.push_back(quad_pt);
        }

        quad_traj.header.frame_id = "";
        quad_traj.header.stamp = ros::Time::now();
        quad_traj.type = quadrotor_msgs::Trajectory::JERK;
        trajectory_pub_.publish(quad_traj);

    }
    void commandTimerCallback(const ros::TimerEvent&){
        current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
        if (current_sample_time_ <= trajectory_4D_.getMaxTime()) {
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            bool success = trajectory_4D_.sampleTrajectoryAtTime(
                        current_sample_time_, &trajectory_point);

            if (!success) {
                publish_timer_.stop();
            }
            mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
            quadrotor_msgs::TrajectoryPoint t;

            t.pose.position.x = trajectory_point.position_W.x();
            t.pose.position.y = trajectory_point.position_W.y();
            t.pose.position.z = trajectory_point.position_W.z();

            t.velocity.linear.x = trajectory_point.velocity_W.x();
            t.velocity.linear.y = trajectory_point.velocity_W.y();
            t.velocity.linear.z = trajectory_point.velocity_W.z();

            t.acceleration.linear.x = trajectory_point.acceleration_W.x();
            t.acceleration.linear.y = trajectory_point.acceleration_W.y();
            t.acceleration.linear.z = trajectory_point.acceleration_W.z();


            t.jerk.linear.x = trajectory_point.jerk_W.x();
            t.jerk.linear.y = trajectory_point.jerk_W.y();
            t.jerk.linear.z = trajectory_point.jerk_W.z();

            t.snap.linear.x = trajectory_point.snap_W.x();
            t.snap.linear.y = trajectory_point.snap_W.y();
            t.snap.linear.z = trajectory_point.snap_W.z();


            t.heading = trajectory_point.getYaw();
            t.heading_rate = trajectory_point.getYawRate();
            t.heading_acceleration = trajectory_point.getYawAcc();

            command_pub_.publish(t);
        } else if(current_sample_time_ <= trajectory_4D_.getMaxTime() ){//time shift to go at goal
            current_sample_time_ = trajectory_4D_.getMaxTime();
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            bool success = trajectory_4D_.sampleTrajectoryAtTime(
                        current_sample_time_, &trajectory_point);

            if (!success) {
                publish_timer_.stop();
            }
            mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);

            quadrotor_msgs::TrajectoryPoint t;

            t.pose.position.x = trajectory_point.position_W.x();
            t.pose.position.y = trajectory_point.position_W.y();
            t.pose.position.z = trajectory_point.position_W.z();

            t.velocity.linear.x = trajectory_point.velocity_W.x();
            t.velocity.linear.y = trajectory_point.velocity_W.y();
            t.velocity.linear.z = trajectory_point.velocity_W.z();

            t.acceleration.linear.x = trajectory_point.acceleration_W.x();
            t.acceleration.linear.y = trajectory_point.acceleration_W.y();
            t.acceleration.linear.z = trajectory_point.acceleration_W.z();


            t.jerk.linear.x = trajectory_point.jerk_W.x();
            t.jerk.linear.y = trajectory_point.jerk_W.y();
            t.jerk.linear.z = trajectory_point.jerk_W.z();

            t.snap.linear.x = trajectory_point.snap_W.x();
            t.snap.linear.y = trajectory_point.snap_W.y();
            t.snap.linear.z = trajectory_point.snap_W.z();


            t.heading = trajectory_point.getYaw();
            t.heading_rate = trajectory_point.getYawRate();
            t.heading_acceleration = trajectory_point.getYawAcc();

            command_pub_.publish(t);



        }else {
            publish_timer_.stop();
        }
    }
    void yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point);


    void publishVizualization(const mav_trajectory_generation::Trajectory &trajectory);

    void startTrajectory(){
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        publish_timer_.start();
    }
    void stopTrajectory(){
        publish_timer_.stop();
    }

    float remainingTime(){

        current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
        float t = current_sample_time_ - trajectory_.getMaxTime();
        return t;

    }

    bool isRunning(){
        bool tmp = publish_timer_.hasStarted();
        return tmp;

    }

    bool inputFeasability(const mav_trajectory_generation::Trajectory *trajectory)
    {
        for(const auto &segment: trajectory->segments()){
            if (feasibility_check_.checkInputFeasibility(segment) !=
                    mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                return false;
            }
        }
        return true;
    }



    //    bool getPredictedPose(Eigen::Vector4d & pos_o, Eigen::Vector4d & vel_o);


    void setTrajectory(const mav_trajectory_generation::Trajectory &trajectory){
        publish_timer_.stop();
        trajectory_.clear();
        trajectory_ = trajectory;
    }


    void get3DTraj(double start, double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points){
        trajectory_4D_.sample3D(start,dt,points);
    }
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher command_pub_;
    ros::Publisher trajectory_pub_;

    ros::Timer publish_timer_;
    ros::Time start_time_;


    double dt_=0.01;
    std::atomic<double> current_sample_time_;


    std::vector<Quintic> yaw_traj_;
    mav_trajectory_generation::Trajectory trajectory_;

    mav_trajectory_generation::Trajectory current_trajectory_;
    mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;

    bool useYaw = true;


    std::atomic_bool setup_start;

    Eigen::Vector3d  predicted_velocity_;
    Eigen::Vector3d predicted_acc_;
    Eigen::Vector3d predicted_pos_;

    SegmentManager4D trajectory_4D_;




    //constraint

    double max_yaw_rate_ = M_PI/2.;
    double max_yaw_acc_ = M_PI/2.;

    double max_vel_ = 1.5;
    double max_acc_ = 2.5;
};
