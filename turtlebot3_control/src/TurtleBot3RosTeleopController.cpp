/*
 * TurtleBot3RosTeleopController.cpp
 *
 *  Created on: 2020/03/25
 *      Author: Tsuyoshi Anazawa
 *
 *  Outline: The keyboard teleoperation controller for operating TurtleBot3 model.
 */
// For choreonoid related.
#include <cnoid/SimpleController>
#include <cnoid/RangeSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <fmt/format.h>

// For ros related.
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

using namespace std;
//using namespace cnoid;
using fmt::format;

class TurtleBot3RosTeleopController : public cnoid::SimpleController
{
	ros::NodeHandle nh_;
	// Odometry publisher.
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster tfBroadcaster_;
	// 0: odom, 1: imu, 2: cmd_vel
	double currentTime[3], lastTime[3], update_period[3];
	double update_rate[3] = { 3.0, 20.0, 10.0 };

	// Imu publisher.
	ros::Publisher imu_pub_;
	// For accelerator and gyro sensor.
	cnoid::AccelerationSensorPtr accel;
	cnoid::RateGyroSensorPtr gyro;
	std::string deviceName[2] = { "AccelSensor", "GyroSensor" };
	cnoid::ScopedConnection accel_conn;
	cnoid::ScopedConnection gyro_conn;

	// cmd_vel subscriber.
	ros::Subscriber cmdvel_sub_;
	// Constant to store the number of wheels.
	static const int WHEEL_NUM = 2;
	// The array containing the wheel names defined in the body file.
	const std::string wheelNames[WHEEL_NUM] = { "Left_wheel", "Right_wheel" };
	// The variable that stores the actuation mode of the link.
	int actuationMode;
	// The array to store wheels.
	cnoid::Link* wheels[2];
	double qprev[2];
	double dt;
	double qref[2];
	cnoid::SimpleControllerIO* io;
	// Current coordinate position.
	double x, y, th;
	double omega;
	// The array used in accelerometers and angular velocity sensors.
	// Current velocity[m/s] and angular velocity[rad/s].
	cnoid::Vector3 v;
	// Last velocity[m/s].
	cnoid::Vector3 prevVth;
	double imuTh;
	cnoid::Vector3 cmd_vel, cmd_vel_th;
	const std::string ODOM_FRAME_ID = "odom";
	const std::string CHILD_FRAME_ID = "base_footprint";
	const std::string IMU_FRAME_ID = "base_footprint";
	// Limit velocities and accelerations.
	const double MAX_VEL = 0.26;
	const double MIN_VEL = -0.26;
	const double MAX_ACCEL = 1.82;
	const double MIN_ACCEL = -1.82;
	// half of the tread.
	double d = 0.1435;
	// Wheel radius.
	double r = 0.033;
	cnoid::Vector2 prevWheelTh;
	cnoid::Vector3 dv_sum[2], w_sum[2];
	int sensingSteps[2];

public:
	double clamp(const double x, const double min, const double max)
	{
	  return std::min(std::max(min, x), max);
	}

	virtual bool configure(cnoid::SimpleControllerConfig* config) override
	{
		return true;
	}

	virtual bool initialize(cnoid::SimpleControllerIO* io) override
	{
		this->io = io;
		ostream& os = io->os();
		// Body variable.
		cnoid::Body* body = io->body();

		// Set joint's actuation mode.
		actuationMode = cnoid::Link::JointTorque;
		// Get controller option.
		std::string option = io->optionString();

		if(!option.empty()){
			// When controller option is empty.
			if(option == "velocity" || option == "position"){
				// When controller option is velocity or position.
				actuationMode = cnoid::Link::JointVelocity;
			} else if(option == "torque"){
				// When controller option is torque.
				actuationMode = cnoid::Link::JointTorque;
			} else {
				// In other cases.
				os << format("Warning: Unknown option {}.", option) << endl;
			}
		}

		for(int i = 0; i < WHEEL_NUM; ++i){
			// Store wheel links in array.
			wheels[i] = body->link(wheelNames[i]);
			if(!wheels[i]){
				// When there is no wheel links.
				os << format("{0} of {1} is not found.", wheelNames[i], body->name()) << endl;
				return false;
			}

			// Set the wheel array's actuation mode.
			wheels[i]->setActuationMode(actuationMode);
			// Enable output to wheel links.
			io->enableIO(wheels[i]);
			// Store current joint angles in array.
			qprev[i] = wheels[i]->q();
		}

		accel = body->findDevice<cnoid::AccelerationSensor>(deviceName[0]);
		if(!accel){
			os << format("{0} of {1} is not found.", deviceName[0], body->name()) << endl;
			//return false;
		} else {
			io->enableInput(accel);
			accel_conn.disconnect();
			accel_conn = accel->sigStateChanged().connect(
					[&](){ updateImu(); });
		}
		
		gyro = body->findDevice<cnoid::RateGyroSensor>(deviceName[1]);
		if(!gyro){
			os << format("{0} of {1} is not found.", deviceName[1], body->name()) << endl;
			//return false;
		} else {
			io->enableInput(gyro);
			gyro_conn.disconnect();
			gyro_conn = gyro->sigStateChanged().connect(
					[&](){ updateImu(); });
		}

		// cmd_vel subscriber.
		cmdvel_sub_ = nh_.subscribe("cmd_vel", 1, &TurtleBot3RosTeleopController::cmdvelCallback, this);
		// Create the publisher of odometry(topic: odom, buffer_size: 10).
		odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
		// Create the publisher of imu(topic: imu, buffer_size: 10).
		imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);

		// Suppose the robot starts from the origin of the odom coordinate system.
		nh_.param("initial_x", x, 0.0);
		nh_.param("initial_y", y, 0.0);
		nh_.param("initial_th", th, 0.0);

		imuTh = th;

		prevVth[2] = 0.0;

		for(int i = 0; i < sizeof(wheels) / sizeof(wheels[0]); ++i){
			prevWheelTh[i] = wheels[i]->q();
		}

		currentTime[3] = lastTime[3] = io->currentTime();
		for(int i = 0; i < sizeof(currentTime) / sizeof(currentTime[0]); ++i){
			update_period[i] = 1.0 / update_rate[i];
		}

		dt = io->timeStep();

		return true;
	}

	void cmdvelCallback(const geometry_msgs::Twist& msg)
	{
		cmd_vel[0] = msg.linear.x;
		cmd_vel[1] = msg.linear.y;
		cmd_vel[2] = msg.linear.z;
		cmd_vel_th[0] = msg.angular.x;
		cmd_vel_th[1] = msg.angular.y;
		cmd_vel_th[2] = msg.angular.z;
	}

	void updateLocalization()
	{
		cnoid::Vector2 wheel_th, wheel_omg, wheel_v;

		for(int i = 0; i < sizeof(wheels) / sizeof(wheels[0]); ++i){
			double latest_wheel_angle = wheels[i]->q();
			wheel_th[i] = (latest_wheel_angle - prevWheelTh[i]);
			wheel_omg[i] = wheel_th[i] / update_period[0];
			wheel_v[i] = r * wheel_omg[i];
			prevWheelTh[i] = latest_wheel_angle;
		}

		v[0] = TurtleBot3RosTeleopController::clamp((wheel_v[0] + wheel_v[1]) / 2.0, MIN_VEL, MAX_VEL);
		v[1] = 0.0;

		double deltaX = (v[0] * cos(th) - v[1] * sin(th)) * update_period[0];
		double deltaY = (v[0] * sin(th) + v[1] * cos(th)) * update_period[0];
		double deltaTh = ((omega + prevVth[2]) / 2.0) * update_period[0];

		x += deltaX;
		y += deltaY;
		th += deltaTh;

		prevVth[2] = omega;
	}

	void updateOdom()
	{
		currentTime[0] = io->currentTime();
		double updateSince = currentTime[0] - lastTime[0];
		w_sum[0] += gyro->w();
		sensingSteps[0]++;

		if(updateSince > update_period[0]){
			cnoid::Vector3 w = w_sum[0] / sensingSteps[0];
			// Convert yaw values to quaternions.
			geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);
			// Create a TransformedStamped message to send to tf.
			geometry_msgs::TransformStamped odomTrans;
			odomTrans.header.stamp.fromSec(currentTime[0]);
			odomTrans.header.frame_id = ODOM_FRAME_ID;
			odomTrans.child_frame_id = CHILD_FRAME_ID;

			// Substitute coordinate transformation data from odometry data and send using Transform Broadcaster.
			odomTrans.transform.translation.x = x;
			odomTrans.transform.translation.y = y;
			odomTrans.transform.translation.z = 0.0;
			odomTrans.transform.rotation = odomQuat;
			tfBroadcaster_.sendTransform(odomTrans);

			// The navigation stack publishes a nav_msgs/Odometry message for velocity information.
			nav_msgs::Odometry odom;
			odom.header.stamp.fromSec(currentTime[0]);
			odom.header.frame_id = ODOM_FRAME_ID;

			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odomQuat;

			odom.child_frame_id = CHILD_FRAME_ID;
			odom.twist.twist.linear.x = v[0];
			odom.twist.twist.linear.y = v[1];
			odom.twist.twist.angular.z = omega = w.z();

			w_sum[0].setZero();
			sensingSteps[0] = 0;

			updateLocalization();

			ros::spinOnce();
			odom_pub_.publish(odom);

			lastTime[0] += update_period[0];
		}
	}

	void updateImu()
	{
		currentTime[1] = io->currentTime();
		double updateSince = currentTime[1] - lastTime[1];

		dv_sum[1] += accel->dv();
		w_sum[1] += gyro->w();
		sensingSteps[1]++;

		if(updateSince > update_period[1]){
			auto dv = dv_sum[1] / sensingSteps[1];
			auto w = w_sum[1] / sensingSteps[1];
			sensor_msgs::Imu imu;

			imuTh += w.z() * update_period[1];

			geometry_msgs::Quaternion imuQuat = tf::createQuaternionMsgFromYaw(imuTh);
			imu.header.stamp.fromSec(currentTime[1]);
			imu.header.frame_id = IMU_FRAME_ID;
			imu.linear_acceleration.x = dv.x();
			imu.linear_acceleration.y = dv.y();
			imu.linear_acceleration.z = dv.z();
			imu.angular_velocity.x = w.x();
			imu.angular_velocity.y = w.y();
			imu.angular_velocity.z = w.z();
			imu.orientation = imuQuat;

			dv_sum[1].setZero();
			w_sum[1].setZero();
			sensingSteps[1] = 0;

			ros::spinOnce();
			imu_pub_.publish(imu);

			lastTime[1] += update_period[1];
		}
	}

	virtual bool control() override
	{
		double dq_target[2];		// Target angular velocity.
		dq_target[0] = clamp(cmd_vel[0] - clamp(cmd_vel_th[2], MIN_ACCEL, MAX_ACCEL) * d, MIN_VEL, MAX_VEL) / r;
		dq_target[1] = clamp(cmd_vel[0] + clamp(cmd_vel_th[2], MIN_ACCEL, MAX_ACCEL) * d, MIN_VEL, MAX_VEL) / r;

		if(actuationMode == cnoid::Link::JointVelocity){
			// When actuation mode is velocity.
			static const double K = 1.0;
			wheels[0]->dq_target() = K * dq_target[0];
			wheels[1]->dq_target() = K * dq_target[1];

		} else if(actuationMode == cnoid::Link::JointTorque){
			// When actuation mode is torque.
			static const double K = 35.0;

			for(int i = 0; i < 2; ++i){
				double dq_current = (wheels[i]->q() - qprev[i]) / dt;	// Current angular velocity.
				wheels[i]->u() = K * (dq_target[i] - dq_current);	// Torque value given to tire.
				qprev[i] = wheels[i]->q();	// Update previous joint angular.
			}
		}

		updateImu();
		updateOdom();

		return true;
	}

	virtual void stop() override
	{
		x = y = th = 0.0;
		for(int i = 0; i < sizeof(v)/sizeof(v[0]); ++i){
			v[i] = 0.0;
		}

		cmdvel_sub_.shutdown();
		odom_pub_.shutdown();
		imu_pub_.shutdown();
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurtleBot3RosTeleopController)
