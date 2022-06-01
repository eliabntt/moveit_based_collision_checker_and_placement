#include <memory>

#include "collision_check/collision_check.h"
#include <filesystem>

void CollisionChecker::randomize_position(double& x, double& y, double& z, double& yaw) {
	x = dis_x(gen);
	y = dis_y(gen);
	z = dis_z(gen);
	yaw = dis_yaw(gen);
}

void CollisionChecker::move_collision_object() {
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(
		0).position.x = planning_scene_->getWorldNonConst()->getObject(
		check_object_ids_.at(0))->global_shape_poses_.at(0).translation().x();
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(
		0).position.y = planning_scene_->getWorldNonConst()->getObject(
		check_object_ids_.at(0))->global_shape_poses_.at(0).translation().y();
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(
		0).position.z = planning_scene_->getWorldNonConst()->getObject(check_object_ids_.at(0))->global_shape_poses_.at(
		0).translation().z();
	Eigen::Quaterniond q(
		planning_scene_->getWorldNonConst()->getObject(check_object_ids_.at(0))->global_shape_poses_.at(0).rotation());
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(0).orientation.x = q.x();
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(0).orientation.y = q.y();
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(0).orientation.z = q.z();
	collision_objects.at(collision_objects.size() - 1).mesh_poses.at(0).orientation.w = q.w();
}

bool CollisionChecker::onSegment(PointSimple p, PointSimple q, PointSimple r) {
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return true;
	return false;
}

int CollisionChecker::orientation(PointSimple p, PointSimple q, PointSimple r) {
	double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) return 0;
	return (val > 0) ? 1 : 2;
}

bool CollisionChecker::doIntersect(PointSimple p1, PointSimple q1, PointSimple p2, PointSimple q2) {
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	if (o1 != o2 && o3 != o4)
		return true;

	if (o1 == 0 && onSegment(p1, p2, q1)) return true;
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false;
}

bool CollisionChecker::isInside(PointSimple p){
	int n = polygon.size();
	if (n < 3) {
		ROS_ERROR_STREAM("Polygon has less than 3 vertices");
		return true;
	}
	// Create a point for line segment from p to infinite
	PointSimple extreme = {INF, p.y};

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		int next = (i+1)%n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect({polygon[i].x, polygon[i].y}, {polygon[next].x, polygon[next].y}, p, extreme))
		{
			// If the point 'p' is collinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation({polygon[i].x, polygon[i].y}, p, {polygon[next].x, polygon[next].y}) == 0)
				return onSegment({polygon[i].x, polygon[i].y}, p, {polygon[next].x, polygon[next].y});

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count&1; // Same as (count%2 == 1)
}

bool
CollisionChecker::get_position(double &x, double &y, double &z, double &yaw, const int &limit_collision, const double &forced_z) {
	std::vector<moveit_msgs::CollisionObject> tmp;
	tmp.push_back(collision_objects.at(collision_objects.size() - 1));
	ps.world.collision_objects = tmp;
	planning_scene_->usePlanningSceneMsg(ps);

	Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
	for (auto i : min_limits){
		ROS_INFO_STREAM("min_limits: " << i);
	}
	for (auto i : max_limits){
		ROS_INFO_STREAM("max_limits: " << i);
	}

	dis_x = std::uniform_real_distribution<double>(min_limits.at(0),  max_limits.at(0));
	dis_y = std::uniform_real_distribution<double>(min_limits.at(1),  max_limits.at(1));
	dis_z = std::uniform_real_distribution<double>(min_limits.at(2), max_limits.at(2));
	dis_yaw = std::uniform_real_distribution<double>(-M_PI, M_PI);

	randomize_position(x,y,z, yaw);
	while (not (isInside({x, y}))) {
//		ROS_INFO_STREAM("x: " << x << " y: " << y << " z: " << z);
//		ROS_INFO_STREAM("isInside: " << isInside({x, y}));
		randomize_position(x,y,z, yaw);
	}

	pose = Eigen::Isometry3d::Identity();
	pose.translation() = Eigen::Vector3d(x, y, z);
	pose = Eigen::Translation3d(pose.translation()) * (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

	planning_scene_->getWorldNonConst()->setObjectPose(check_object_ids_.at(0), pose);

	for (int i = 0; i < 99; i++) {
		ROS_INFO_STREAM("Checking collision for the " << i << " time");
		try {
			auto res = planning_scene_->getCollisionEnv()->checkCollisionBetweenObjectGroups(check_world_id_,
			                                                                                 check_object_ids_);
			if (!res.collision or res.contact_count < limit_collision) {
				pose.translation() = Eigen::Vector3d(x, y, forced_z != -1 ? forced_z : z);
				planning_scene_->getWorldNonConst()->setObjectPose(check_object_ids_.at(0), pose);
				move_collision_object();
				return true;
			} else if (i < 99) {
				randomize_position(x,y,z, yaw);

				while (not (isInside({x, y}))) {
//					ROS_INFO_STREAM("isInside: " << isInside({x, y}));
//					ROS_INFO_STREAM("x: " << x << " y: " << y << " z: " << z);
					randomize_position(x,y,z, yaw);
				}

				pose = Eigen::Isometry3d::Identity();
				pose.translation() = Eigen::Vector3d(x, y, z);
				pose = Eigen::Translation3d(pose.translation()) * (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

				planning_scene_->getWorldNonConst()->setObjectPose(check_object_ids_.at(0), pose);
			}
		}
		catch (...) {

		}
	}
	x = max_limits.at(0) + 10;
	y = max_limits.at(1) + 10;
	z = -10;
	yaw = 0;

	pose.translation() = Eigen::Vector3d(x, y, z);
	pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	planning_scene_->getWorldNonConst()->setObjectPose(check_object_ids_.at(0), pose);

	move_collision_object();

	return false;
}

CollisionChecker::CollisionChecker(ros::NodeHandle &nh) {
	nh_ = nh;
	gen.seed(rd_());
	std::string robot_mesh_path;
	nh.getParam("robot_mesh_path",robot_mesh_path);
	if (robot_mesh_path.empty()) {
		ROS_ERROR("robot_mesh_path is empty");
		return;
	}
	m_cam = shapes::createMeshFromResource("file://"+robot_mesh_path);
	shapes::constructMsgFromShape(m_cam, mesh_msg);
	mesh_cam = boost::get<shape_msgs::Mesh>(mesh_msg);

	ros::ServiceClient get_ps_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	get_ps_client_.waitForExistence();
	moveit_msgs::GetPlanningScene get_ps_srv_msg;
	static const std::string PLANNING_GROUP = "my_fake_group";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	planning_scene_ = std::make_unique<planning_scene::PlanningScene>(move_group.getRobotModel());
}

bool CollisionChecker::check_collision(collision_check::collision_check_srv::Request &req,
                                       collision_check::collision_check_srv::Response &res) {
	int index = 0;
	polygon = req.env_polygon;

	moveit_msgs::CollisionObject bin;
	bin.header.frame_id = "world";
	bin.operation = bin.ADD; //add object to collitions
	bin.meshes.resize(1); //scale
	bin.mesh_poses.resize(1); //vector resize
	bin.mesh_poses[0].position.x = 0; //pose
	bin.mesh_poses[0].position.y = 0;
	bin.mesh_poses[0].position.z = 0;
	bin.mesh_poses[0].orientation.w = 1;
	bin.mesh_poses[0].orientation.x = 0;
	bin.mesh_poses[0].orientation.y = 0;
	bin.mesh_poses[0].orientation.z = 0;


	ps.robot_state.is_diff = true;
	ps.is_diff = true;

	if (req.env_stl_path == old_env_stl_path and not req.reset) {
		ROS_INFO("Environment name is same as old");
		collision_objects.clear();
	} else {
		ps.is_diff = false;
		collision_objects.clear();

		ROS_INFO("Environment name is different from old / resetting");
		old_env_stl_path = req.env_stl_path;
		m = shapes::createMeshFromResource("file://"+old_env_stl_path);

		oc.id = "world";
		oc.color.g = 0.0;
		oc.color.r = 255.0;
		oc.color.b = 0.0;
		oc.color.a = 255.0;
		object_colors.push_back(oc);

		shapes::constructMsgFromShape(m, mesh_msg);
		mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

		bin.meshes[0] = mesh; //mesh
		bin.id = "world"; //rename object
		collision_objects.emplace_back(bin);
		ps.world.collision_objects = collision_objects;
		planning_scene_->usePlanningSceneMsg(ps);
		planning_scene_interface.applyCollisionObjects(collision_objects, object_colors);

		object_colors.clear();
		collision_objects.clear();
		ps.is_diff = true;

		check_world_id_.clear();
		check_world_id_.emplace_back("world");
		index++;
	}

	if (req.ob_names.empty() && req.is_cam == false) {
		ROS_ERROR("Robot name is empty and is not camera");
		return false;
	}
	check_object_ids_.clear();
	if (req.is_cam == false) {
		int cnt = 0;
		for (auto ob: req.ob_names) {
			oc.id = ob;
			oc.color.g = 255.0;
			oc.color.r = 0.0;
			oc.color.a = 255.0;
			object_colors.push_back(oc);

			m = shapes::createMeshFromResource(
				"file://" + req.ob_stl_paths[cnt]);
			cnt += 1;

			shapes::constructMsgFromShape(m, mesh_msg);
			mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

			bin.meshes[0] = mesh; //mesh
			bin.id = ob; //rename object
			collision_objects.emplace_back(bin);

			ps.world.collision_objects = collision_objects;
			index++;

			check_object_ids_.clear();
			check_object_ids_.emplace_back(ob);

			double x, y, yaw, z;
			min_limits = req.min_limits;
			max_limits = req.max_limits;
			bool success = get_position(x, y, z, yaw, req.limit_collision, req.forced_z);
			res.x.emplace_back(x);
			res.y.emplace_back(y);
			res.z.emplace_back(z);
			res.yaw.emplace_back(yaw);

			check_world_id_.emplace_back(ob);
		}
	} else {
		// placing camera
		oc.id = "camera";
		oc.color.g = 255.0;
		oc.color.r = 0.0;
		oc.color.a = 255.0;
		object_colors.push_back(oc);


		bin.meshes[0] = mesh_cam; //mesh
		bin.id = "camera"; //rename object
		collision_objects.push_back(bin);

		ps.world.collision_objects = collision_objects;
		index++;

		check_object_ids_.emplace_back("camera");
		double x, y, yaw, z;
		min_limits = req.min_limits;
		max_limits = req.max_limits;
		polygon = req.env_polygon;
		bool success = get_position(x, y, z, yaw, req.limit_collision, req.forced_z);
		res.x.emplace_back(x);
		res.y.emplace_back(y);
		res.z.emplace_back(z);
		res.yaw.emplace_back(yaw);
	}

	planning_scene_interface.applyCollisionObjects(collision_objects, object_colors);
	return true;
}

CollisionChecker::~CollisionChecker() {

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "collision_check");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(8);
	spinner.start();
	CollisionChecker c(node_handle);
	ros::ServiceServer service = node_handle.advertiseService("collision_checker/check",
	                                                          &CollisionChecker::check_collision,
	                                                          (CollisionChecker *) &c);

	while (ros::ok()) {
	}
}