#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class GenerateCSV{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_wheel_odom;
		ros::Subscriber sub_slam_odom;
		/*objects*/
		double ini_r;
		double ini_l;
		double ini_t;
		nav_msgs::Odometry slam_odom;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		/*flags*/
		bool first_callback_odom = true;
		/*file*/
		FILE *fp;
	public:
		GenerateCSV();
		void CallbackWheelOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackSLAMOdom(const nav_msgs::OdometryConstPtr& msg);
		void Close(void);
};

GenerateCSV::GenerateCSV()
	: nhPrivate("~")
{
	sub_wheel_odom = nh.subscribe("/odom", 1, &GenerateCSV::CallbackWheelOdom, this);
	sub_slam_odom = nh.subscribe("/loamvelodyne_odometry", 1, &GenerateCSV::CallbackSLAMOdom, this);
	if((fp = fopen("/home/amsl/Desktop/data.csv", "w")) == NULL){
		printf("file open error!!\n");
		exit(EXIT_FAILURE);
	}

	nhPrivate.param("ini_r", ini_r, 0.125);
	nhPrivate.param("ini_l", ini_l, 0.125);
	nhPrivate.param("ini_t", ini_t, 0.45);
}

void GenerateCSV::CallbackWheelOdom(const nav_msgs::OdometryConstPtr& msg)
{
	time_odom_now = ros::Time::now();
	double dt;
	try{
		dt = (time_odom_now - time_odom_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_odom_last = time_odom_now;

	if(first_callback_odom){
		dt = 0.0;
		first_callback_odom = false;
	}

	double v = msg->twist.twist.linear.x;
	double w = msg->twist.twist.angular.z;

	double wr = (2*v - w*ini_t)/(2*ini_r);
	double wl = (2*v + w*ini_t)/(2*ini_l);

	fprintf(fp, "%f,%f\n", wr*dt, wl*dt);
}

void GenerateCSV::CallbackSLAMOdom(const nav_msgs::OdometryConstPtr& msg)
{
	slam_odom = *msg;
}

void GenerateCSV::Close(void)
{
	tf::Quaternion q_orientation;
	quaternionMsgToTF(slam_odom.pose.pose.orientation, q_orientation);
	double r, p, y;
	tf::Matrix3x3(q_orientation).getRPY(r, p, y);

	fprintf(fp, "NAN, NAN\n");
	fprintf(fp, "%f,%f,%f\n", slam_odom.pose.pose.position.x, slam_odom.pose.pose.position.y, y);
	fclose(fp);
	std::cout << "Closed" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generate_csv");

	GenerateCSV generate_csv;

	ros::spin();

	generate_csv.Close();
}
