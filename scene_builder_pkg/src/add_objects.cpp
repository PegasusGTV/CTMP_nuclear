// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Loading Meshes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Geometry>

void addMeshToCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                               const std::string& mesh_resource, 
                               const std::string& object_id, 
                               const std::string& frame_id, 
                               const geometry_msgs::Pose& pose, 
                               double scale_factor = 1.0)
{
    // Create a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = object_id;
    collision_object.header.frame_id = frame_id;

    // Load the mesh from the resource
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_resource);
    if (!mesh)
    {
        ROS_ERROR("Failed to load mesh from resource: %s", mesh_resource.c_str());
        return;
    }

    // Scale the mesh
    for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
        mesh->vertices[3 * i + 0] *= scale_factor;
        mesh->vertices[3 * i + 1] *= scale_factor;
        mesh->vertices[3 * i + 2] *= scale_factor;
    }

    // Convert the mesh to a ROS message
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);
    shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

    // Add the mesh to the collision object
    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO("Mesh %s added to the planning scene", object_id.c_str());
}

void addBoxToCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                               const std::string& object_id, 
                               const std::string& frame_id, 
                               const geometry_msgs::Pose& pose, 
                               const std::vector<double>& dimensions)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = dimensions;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO("Box %s added to the planning scene", object_id.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_scene");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::string base_frame = "world";

    // Load the objects parameters from the parameter server (already loaded through the launch file)
    XmlRpc::XmlRpcValue objects;
    if (!nh.getParam("objects", objects)) {
        ROS_ERROR("Failed to get 'objects' from the parameter server.");
        return 1;
    }

    for (int i = 0; i < objects.size(); ++i)
    {
        // Get the object details
        std::string object_id = static_cast<std::string>(objects[i]["id"]);
        std::string type = static_cast<std::string>(objects[i]["type"]);
        std::string frame_id = static_cast<std::string>(objects[i]["frame_id"]);
        
        // Get the pose (position and orientation)
        geometry_msgs::Pose pose;
        pose.position.x = static_cast<double>(objects[i]["pose"]["position"][0]);
        pose.position.y = static_cast<double>(objects[i]["pose"]["position"][1]);
        pose.position.z = static_cast<double>(objects[i]["pose"]["position"][2]);
        pose.orientation.x = static_cast<double>(objects[i]["pose"]["orientation"][0]);
        pose.orientation.y = static_cast<double>(objects[i]["pose"]["orientation"][1]);
        pose.orientation.z = static_cast<double>(objects[i]["pose"]["orientation"][2]);
        pose.orientation.w = static_cast<double>(objects[i]["pose"]["orientation"][3]);

        // Add the object to the planning scene based on its type
        if (type == "box")
        {
            // Get the dimensions for the box
            std::vector<double> dimensions;
            for (int j = 0; j < 3; ++j)
                dimensions.push_back(static_cast<double>(objects[i]["dimensions"][j]));

            addBoxToCollisionObjects(planning_scene_interface, object_id, frame_id, pose, dimensions);
        }
        else if (type == "mesh")
        {
            // Get the mesh resource and scale factor
            std::string mesh_resource = static_cast<std::string>(objects[i]["mesh_resource"]);
            double scale_factor = 1.0;  // Default scale factor
            if (objects[i].hasMember("scale_factor"))
                scale_factor = static_cast<double>(objects[i]["scale_factor"]);

            addMeshToCollisionObjects(planning_scene_interface, mesh_resource, object_id, frame_id, pose, scale_factor);
        }
        else
        {
            ROS_WARN("Unsupported object type: %s", type.c_str());
        }
    }

    ros::shutdown();
    return 0;
}
