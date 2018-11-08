#include "ros/ros.h"
#include <ros/package.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace octomap;

Eigen::Matrix4d getMatrixFromYamlFile(string file)
{
    YAML::Node config = YAML::LoadFile(file);
    std::vector<double> matrix;

    matrix = config["T"].as<std::vector<double> >();
    Eigen::Matrix4d T;

    for (int i=0; i<16; i++) {
        T << matrix[i];
    }
    T << matrix[0], matrix[1], matrix[2], matrix[3], 
         matrix[4], matrix[5], matrix[6], matrix[7],
         matrix[8], matrix[9], matrix[10], matrix[11],
         matrix[12], matrix[13], matrix[14], matrix[15];
    
    return T;
}

point3d transformPoint(point3d current_point, Eigen::Matrix4d T)
{
    Eigen::Vector4d X;
    X << current_point.x(), current_point.y(), current_point.z(), 1;
    X = T*X;
    point3d return_point(X(0), X(1), X(2));
    return return_point;
}

int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "rotate_octomap");
    ros::NodeHandle nh_private("~");

    // Collect parameters
    cout << "Loading map and config." << endl;
    string input_file_path, config_file_path, output_map_path;
    nh_private.param("input_map", input_file_path, 
        ros::package::getPath("larics_gazebo_worlds") + 
        string("/models/city/city.binvox.bt"));
    nh_private.param("output_map", output_map_path, 
        ros::package::getPath("larics_gazebo_worlds") + 
        string("/models/city/city_rotated.binvox.bt"));
    nh_private.param("config_file", config_file_path, 
        string("/home/antun/catkin_ws/src/larics_gazebo_worlds/config/transform.yaml"));
    Eigen::Matrix4d T = getMatrixFromYamlFile(config_file_path);

    // Create octomap
    OcTree map(input_file_path);

    // Create empty map filled with rotated points and same resolution
    // as input map
    OcTree rotated_map(map.getResolution());
    // Get minimum and maximum in metric units
    double xm, ym, zm;
    map.getMetricMax(xm, ym, zm);
    int x_max = xm/map.getResolution();
    int y_max = ym/map.getResolution();
    int z_max = zm/map.getResolution();
    map.getMetricMin(xm, ym, zm);
    int x_min = xm/map.getResolution();
    int y_min = ym/map.getResolution();
    int z_min = zm/map.getResolution();
    // Auxiliary variable for resolution
    double resolution = map.getResolution();

    // Go through all points on map and check for occupancy
    double trigger_output = 20.0;
    double progress = 0.0;
    double full = (x_max-x_min)*(y_max-y_min)*(z_max-z_min);
    cout << "Rotating octomap." << endl;
    cout << "Progress: " << progress << "%." << endl;
    for (int x = x_min; x < x_max; x++) {
        for (int y = y_min; y < y_max; y++) {
            for (int z = z_min; z < z_max; z++) {
                for (int w = -1; w <= 1; w++) {
                    // A little perturbation for 1 voxel in order to get more
                    // precise occupancy map
                    point3d current_point(
                        float(x)*resolution + float(w)*resolution/2.0, 
                        float(y)*resolution + float(w)*resolution/2.0, 
                        float(z)*resolution + float(w)*resolution/2.0);
                    // Search for result node defined by current point in 
                    // highest depth
                    OcTreeNode* result = map.search(current_point, 0);
                    // Check occupancy
                    bool occupied = false;
                    if (result != NULL) {
                        if (map.isNodeOccupied(result) == true) occupied = true;
                    }
                    // Rotate point and add it to new map
                    point3d rotated_point = transformPoint(current_point, T);
                    rotated_map.updateNode(rotated_point, occupied);

                    // Write progress
                    progress = (x-x_min)*(y-y_min)*(z-z_min)/full*100.0;
                    if (progress >= trigger_output) {
                        cout << "Progress: " << progress << "%" << endl;
                        trigger_output += 20.0;
                    }
                }
            }
        }
    }
    cout << "Progress: 100%" << endl;

    // Write rotated map
    rotated_map.prune();
    rotated_map.writeBinary(output_map_path);

    cout << "Octomap rotated and saved to: " << output_map_path << endl;
    return 0;
}