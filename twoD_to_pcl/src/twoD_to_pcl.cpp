#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/extract_indices.h>
#include <lidar2gps_msgs/lidar2gps.h>

using namespace std;
typedef pcl::PointXYZI pointType;

class twoD_to_pcl{

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_2d_,sub_imu_;
	ros::Publisher pub_points_, pub_center_, pub_waypoint_, pub_angle_, pub_speed_, pub_control_;

	std_msgs::Float64 speed_ ;
	std_msgs::Float64 angle_ ;

	lidar2gps_msgs::lidar2gps control_;
	
	//local variable
	pcl::PointCloud<pointType>::Ptr msg;
	vector<pointType> center_;
	//laser_geometry::LaserProjection projector_;
	//tf::TransformListener listener_;

	//parmeter
	float min_x_, max_x_,min_y_, max_y_, cluster_tolerance_,
	finish_point1_, finish_point3_, sep_dynamic_, static_start_dist_;
	int cluster_minsize_, cluster_maxsize_, static_param_, dynamic_param_;
	
	bool heading_record_ = true, finish_static_=false, is_dynamic_ = false;
	double start_heading_ = 0, cur_heading_ = 0, flag2_offset_;

	float yyy_=1000;	
	int mission_state_ = 6, flag_= 0, dynamic_count_ = 0, static_count_ = 0;
	
	clock_t start, finish;

public:
	//creator
	twoD_to_pcl() {
		// show log in terminal
		cout << "twoD_to_pcl Initialized" << endl;
		
		//pointcloud vector reset
		msg.reset(new pcl::PointCloud<pointType>());
		//center_.reset(new vector<pointType>());
		
		// initialize Settings
		initSetup();	
	}
	
	// destructor
	~twoD_to_pcl(){
		// show log in terminal
		cout << "twoD_to_pcl Terminated" << endl;
	}
	
	void initSetup();	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& data);
	bool clustering();
	void visualize_center(vector<pointType> car);
	void visualize_waypoint(geometry_msgs::Point wayPoint);
	float cal_steer(float x, float y);
	void small_static();
	void print();
	void dynamin_static();
};

	void twoD_to_pcl::initSetup() {
		control_.is_dynamic = false;
		control_.is_static = false;
		// initialize Flag
		ros::param::get("~min_x", min_x_);
		ros::param::get("~max_x", max_x_);
		ros::param::get("~min_y", min_y_);
		ros::param::get("~max_y", max_y_);

		ros::param::get("~cluster_tolerance", cluster_tolerance_);
		ros::param::get("~cluster_minsize", cluster_minsize_);
		ros::param::get("~cluster_maxsize", cluster_maxsize_);

		ros::param::get("~flag1_finish_dist", finish_point1_);
		ros::param::get("~flag3_finish_dist", finish_point3_);
		ros::param::get("~flag2_time_offset", flag2_offset_);
		ros::param::get("~static_start_dist", static_start_dist_);

		ros::param::get("~sep_dynamic", sep_dynamic_);
		ros::param::get("~dynamic_param", dynamic_param_);
		ros::param::get("~static_param", static_param_);

		// set subscriber
		sub_2d_ = nh_.subscribe("/lidar2D", 10, &twoD_to_pcl::scanCallback, this);
		//sub_imu_ = nh_.subscribe("/imu", 10, &twoD_to_pcl::imuCallback, this);


		// set publisher
		pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/passed_points", 100);
		pub_center_ = nh_.advertise<visualization_msgs::Marker>("/obstacle_center", 10);
		pub_waypoint_ = nh_.advertise<visualization_msgs::Marker>("/waypoint", 10);
		pub_speed_ = nh_.advertise<std_msgs::Float64>("/commands/motor/speed", 10);
		pub_angle_ = nh_.advertise<std_msgs::Float64>("/commands/servo/position", 10);
        pub_control_ = nh_.advertise<lidar2gps_msgs::lidar2gps>("/lidar_control", 10);
	}

	void twoD_to_pcl::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
			double angle = 0;
			pcl::PointCloud<pointType> tmp_cloud;	
			pointType tmp_point;
			for (int i=0;i<(*scan_in).ranges.size();i++){
				//cout<< (*scan_in).ranges[i] <<endl;
				//wayPoint.x = center_[0].x * cos(h) + (center_[0].y+offset_) * sin(h) ;
				//wayPoint.y = center_[0].x * sin(h) *(-1) + (center_[0].y+offset_) * cos(h) ;
				tmp_point.x = (*scan_in).ranges[i] * cos(angle) ;
				tmp_point.y = (*scan_in).ranges[i] * sin(angle) ;
				tmp_point.z = 0.0;
				tmp_point.intensity = float((*scan_in).intensities[i]);
				
				double h = abs(cur_heading_ - start_heading_) ;
				//tmp_point.x = x * cos(h) + y * sin(h);
				//tmp_point.y = x * sin(h) * (-1) + y * cos(h); 
				tmp_cloud.push_back(tmp_point);
				angle = angle + (1.0/180*M_PI);
				//cout<<tmp_point.x << " || "<< tmp_point.y<<" || "<< tmp_point.intensity <<endl;
			}
			
			*msg = tmp_cloud;
	}

/*
	void twoD_to_pcl::imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
			tf::Quaternion q(data->orientation.x, data->orientation.y, data->orientation.z, data->orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			cur_heading_ = yaw;
			//ROS_INFO("Imu YAW: [%f]", yaw);
			//ROS_INFO("Imu Seq: [%d]", data->header.seq);
			//ROS_INFO("EULER Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
			//ROS_INFO("QUATERNION Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", data->orientation.x,data->orientation.y,data->orientation.z,data->orientation.w);
			if (heading_record_) {
					start_heading_ = cur_heading_;
					heading_record_ = false;
					ROS_INFO("START_HEADING: %lf",start_heading_);
			}
	}
*/

	bool cmp(pointType a, pointType b) { return a.x < b.x; }

	bool twoD_to_pcl::clustering(){
		vector<pointType> v;
		center_.clear();
		center_.swap(v);

		pcl::PassThrough<pointType> pass;

		pass.setInputCloud(msg);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits(min_x_, max_x_);
		pass.filter(*msg);

		pass.setInputCloud(msg);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(min_y_, max_y_);
		pass.filter(*msg);


		sensor_msgs::PointCloud2 filteredOutput;
		pcl::toROSMsg(*msg, filteredOutput);
		filteredOutput.header.frame_id = "velodyne";
		pub_points_.publish(filteredOutput);

		
		pcl::search::KdTree<pointType>::Ptr kdtree(new pcl::search::KdTree<pointType>);
		kdtree -> setInputCloud(msg);

		std::vector<pcl::PointIndices> clusterIndices;
		pcl::EuclideanClusterExtraction<pointType> ec;

		ec.setClusterTolerance(cluster_tolerance_);
		ec.setMinClusterSize(cluster_minsize_);
		ec.setMaxClusterSize(cluster_maxsize_);
		ec.setSearchMethod(kdtree);
		ec.setInputCloud(msg);
		ec.extract(clusterIndices);

		//ROS_INFO("cluster number %lu", clusterIndices.size());

		for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
		    pointType Pt_center;
		    int count = 0;         
		    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			    Pt_center.x += msg->points[*pit].x;
			    Pt_center.y += msg->points[*pit].y;
			    count ++;
		    }            
		    Pt_center.x = Pt_center.x/count;
		    Pt_center.y = Pt_center.y/count;
			Pt_center.z = 0;
			Pt_center.intensity = 0;
		    center_.push_back(Pt_center);
		}

		if (clusterIndices.size()>0){
			sort(center_.begin(),center_.end(), cmp);
			if (fabs(yyy_-center_[0].y) > sep_dynamic_ && fabs(yyy_-center_[0].y) < 50) dynamic_count_ +=1;
			else static_count_ +=1;
			cout <<"yyyy____: " << fabs(yyy_ - center_[0].y) << endl;
			//cout <<"is dynamic: " << is_dynamic_ << endl;
			yyy_ = center_[0].y;
			twoD_to_pcl::visualize_center(center_);
			return true;
		}
		
		else return false;  
	}

	void twoD_to_pcl::visualize_center(vector<pointType> car) {
		visualization_msgs::Marker car_pt;
		geometry_msgs::Point point;

		car_pt.header.frame_id = "velodyne";
		car_pt.header.stamp = ros::Time::now();
		car_pt.ns = "parked car";
		car_pt.action = visualization_msgs::Marker::ADD;
		car_pt.pose.orientation.w = 1.0;
		car_pt.id = 1;
		car_pt.type = visualization_msgs::Marker::POINTS;
		car_pt.scale.x = 0.3;
		car_pt.scale.y = 0.3;
		car_pt.color.a = 1.0;
		car_pt.color.g = 1.0f;

		for (int i = 0 ; i<car.size();i++){
		    point.x = car[i].x;
		    point.y = car[i].y;
		    point.z = 0;
		    car_pt.points.push_back(point);
		    pub_center_.publish(car_pt);
		}
	}
	
	void twoD_to_pcl::visualize_waypoint(geometry_msgs::Point wayPoint) {
		visualization_msgs::Marker way_pt;

		way_pt.header.frame_id = "velodyne";
		way_pt.header.stamp = ros::Time::now();
		way_pt.ns = "way";
		way_pt.pose.orientation.w = 1.0;
		way_pt.id = 1;
		way_pt.type = visualization_msgs::Marker::POINTS;
		way_pt.scale.x = 0.3;
		way_pt.scale.y = 0.3;
		way_pt.color.a = 1.0;
		way_pt.color.r = 1.0f;

		way_pt.points.push_back(wayPoint);
		pub_waypoint_.publish(way_pt);

	}

	float twoD_to_pcl::cal_steer(float x,float y){
		//float degree = atan(2L*sin(alpha)/ld)*180/M_PI;
		float degree = atan(y/(x-0.145))*180/M_PI;

		if (degree > 19.5) degree = 19.5;
		else if (degree < -19.5) degree = -19.5;

		return (degree)*(-1) / 39 + 0.5;
	}

	void twoD_to_pcl::print(){
		cout << "=======================================================================" << endl;
		cout << "FLAG: " << flag_ << endl;
		cout << "OBS_X: " << center_[0].x << endl;
		//cout << "START_HEADING: " << start_heading_ << endl;
		//cout << "CUR_HEADING: " << cur_heading_ << endl;
		//cout << "ANGLE: " << angle_.data << endl;
		//cout << "SPEED: " << speed_.data << endl;
	}

	void twoD_to_pcl::small_static(){
		geometry_msgs::Point wayPoint;
		twoD_to_pcl::print();
		switch (flag_){

		case 0:
			if(center_[0].x < static_start_dist_){
				control_.is_static = true;
				flag_=1;
			}
			break;

		case 1:
			if (center_[0].x <= finish_point1_) {
				start = clock();
				flag_ = 2;		
			}
			break;

		case 2:{			
			min_y_ = 0.01;
			finish = clock();

			double duration = (double)(finish - start) / CLOCKS_PER_SEC;
			cout << "duration:  " << duration << endl;
			if (duration > flag2_offset_) flag_ = 3;
		
			break;
		}

		case 3:
			min_y_ = -1;
			if (center_[0].x <= finish_point3_) {
				cout << "====================DONE====================" << endl;
				control_.is_static = false;
				flag_=4;
			}

			break;

		case 4:
			min_y_ = -1;
			flag_ = 0;
			static_count_ = 0;
			break;
		
		default:
			break;

		}
	}

	void twoD_to_pcl::dynamin_static(){

		if (twoD_to_pcl::clustering()){
			cout  << "dynamic_count_: " << dynamic_count_ << endl;
			cout  << "static_count_: " << static_count_ << endl;

			if (static_count_ >= static_param_){
				cout << "STATIC_ON" << endl;
				control_.is_dynamic = false;
				twoD_to_pcl::small_static();
			}

		
			else if(dynamic_count_ >= dynamic_param_){
				cout << "DYNAMIC_ON" << endl;
				min_y_ = -0.3;
				control_.is_dynamic = true;
				control_.is_static = false;
				static_count_ = 0;
			}
		}

		else {
			is_dynamic_ = false;
			dynamic_count_ = 0;	
		}

		pub_control_.publish(control_);
		
	}



// ############################# Main Code #############################

int main(int argc, char **argv) {
	// set Node
	ros::init(argc, argv, "twoD_to_pcl_node");

	// 2d data to pcl data
	twoD_to_pcl p;

	//ros roop 
	ros::Rate loop_rate(30);
	while(ros::ok()){
		ros::spinOnce();
		p.dynamin_static();
		loop_rate.sleep();
	}
	
	return 0;
}