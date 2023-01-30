#include "twoD_to_pcl/twoD_to_pcl.h"
#include "twoD_to_pcl/delaunay_struct.h"

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

		//labacon_param
		ros::param::get("~labacone_speed", speed_.data);
		ros::param::get("~labacone_offset", labacone_offset_);
		ros::param::get("~labacone_gradient", labacone_gradient_);


		//static param
		ros::param::get("~static_start_dist", static_start_dist_); //mission start dist
		ros::param::get("~flag1_finish_dist", finish_point1_); //first obs
		ros::param::get("~flag3_finish_dist", finish_point3_); //second obs
		ros::param::get("~flag2_time_offset", flag2_offset_); //after first obs

		//decision param
		ros::param::get("~sep_dynamic", sep_dynamic_);
		ros::param::get("~dynamic_param", dynamic_param_);
		ros::param::get("~static_param", static_param_);

		// set subscriber
		sub_2d_ = nh_.subscribe("/lidar2D", 10, &twoD_to_pcl::scanCallback, this);
		//sub_imu_ = nh_.subscribe("/imu", 10, &twoD_to_pcl::imuCallback, this);


		// set publisher
		pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/passed_points", 100);
		pub_center_ = nh_.advertise<visualization_msgs::Marker>("/obstacle_center", 10);
		pub_gps_ = nh_.advertise<lidar2gps_msgs::lidar2gps>("/lidar_control", 10);
		
		pub_waypoint_ = nh_.advertise<visualization_msgs::Marker>("/waypoint", 10);
		pub_speed_ = nh_.advertise<std_msgs::Float64>("/commands/motor/speed", 10);
		pub_angle_ = nh_.advertise<std_msgs::Float64>("/commands/servo/position", 10);
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
				
				//double h = abs(cur_heading_ - start_heading_) ;
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
			visualize_center(center_);
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
	

// ############################# Labacone Code ############################


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

	geometry_msgs::Point twoD_to_pcl::get_waypoint(){
		//pcl::PointCloud<pointType> waypoint_list;
		//pointType w;
		geometry_msgs::Point waypoint;
		float dist;
		if (center_.size()>1){
			for (int i = 1 ; i < center_.size(); i++){
				dist = sqrt(pow(center_[0].x - center_[i].x, 2) + pow(center_[0].y - center_[i].y, 2));
				if (dist < 0.5 && (center_[0].x - center_[i].x)/(center_[0].y - center_[i].y) < labacone_gradient_){
					waypoint.x = (center_[0].x + center_[i].x)/2;
					waypoint.y = (center_[0].y + center_[i].y)/2;
					waypoint.z = 0;
					break;
				}

				else {
					waypoint.x = center_[0].x ;
					waypoint.y = center_[0].y + labacone_offset_*(center_[0].y/fabs(center_[0].y));
					waypoint.z = 0;
				}
			}
		}

		else{ //size = 1
			waypoint.x = center_[0].x ;
			waypoint.y = center_[0].y + labacone_offset_*(center_[0].y/fabs(center_[0].y));
			waypoint.z = 0;
		}

		return waypoint;
	}

	
	void twoD_to_pcl::laba_control(){
		geometry_msgs::Point laba;
		laba = get_waypoint();
		angle_.data = cal_steer(laba.x,laba.y);  
		pub_speed_.publish(speed_);
		pub_angle_.publish(angle_);
		visualize_waypoint(laba);		
	}
	


// ############################# Static Code #############################

	void twoD_to_pcl::print_static(){
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
		print_static();
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


// ############################# Decision Code #############################

	void twoD_to_pcl::dynamin_static(){
		if (clustering()){
			cout  << "dynamic_count_: " << dynamic_count_ << endl;
			cout  << "static_count_: " << static_count_ << endl;

			if (labacone_sign_){
				dynamic_count_ = 0;
				static_count_ = 0;
				laba_control();
			}

			else if (static_count_ >= static_param_){
				cout << "STATIC_ON" << endl;
				control_.is_dynamic = false;
				small_static();
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
			control_.is_dynamic = false;
			dynamic_count_ = 0;	
		}

		pub_gps_.publish(control_);
		
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
