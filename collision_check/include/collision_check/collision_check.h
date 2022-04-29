//
// Created by ebonetto on 02.03.22.
//

#ifndef TRY_COLLISION_COLLISION_CHECK_H
#define TRY_COLLISION_COLLISION_CHECK_H

#include <ros/ros.h>

#include "moveit_msgs/GetPlanningScene.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/ObjectColor.h"
#include "moveit/collision_detection_fcl/collision_common.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/mesh_operations.h"
#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
#include "chrono"
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "geometry_msgs/Point.h"

#include "collision_check/collision_check_srv.h"

#include <octomap/OcTree.h>
#include <random>

#define INF 10000

class CollisionChecker {
public:
		struct PointSimple {
				double x;
				double y;
		};

		CollisionChecker(ros::NodeHandle &nh);

		~CollisionChecker();
		bool check_collision(collision_check::collision_check_srv::Request &req,
		                     collision_check::collision_check_srv::Response &res);
private:
		std::string old_env_stl_path;

		std::vector<moveit_msgs::CollisionObject> collision_objects{}; //vector of objects
		std::vector<moveit_msgs::ObjectColor> object_colors{}; //vector of colors

		std::vector<std::string> check_world_id_;
		std::vector<std::string> check_object_ids_;

		moveit_msgs::ObjectColor oc{};
		shape_msgs::Mesh mesh, mesh_cam;
		shapes::ShapeMsg mesh_msg;
		shapes::Mesh *m{}, *m_cam;

		std::random_device rd_;
		std::mt19937 gen;
		std::uniform_real_distribution<double> dis_x;
		std::uniform_real_distribution<double> dis_y;
		std::uniform_real_distribution<double> dis_z;
		std::uniform_real_distribution<double> dis_yaw;

		moveit_msgs::PlanningScene ps{};

		void move_collision_object();

		bool get_position(double &x, double &y, double &z, double &yaw, const int &limit_collision, const double &forced_z);

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
		ros::NodeHandle nh_;

		bool onSegment(PointSimple p, PointSimple q, PointSimple r);
		int orientation(PointSimple p, PointSimple q, PointSimple r);
		bool doIntersect(PointSimple p1, PointSimple q1, PointSimple p2, PointSimple q2);
		bool isInside(PointSimple p);
		std::vector<geometry_msgs::Point> polygon;
		std::vector<double> min_limits, max_limits;
		void randomize_position(double& x, double& y, double& z, double& yaw);
};

#endif //TRY_COLLISION_COLLISION_CHECK_H
