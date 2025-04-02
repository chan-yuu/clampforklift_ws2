#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <chrono>

ros::Publisher pub;
std::chrono::time_point<std::chrono::system_clock> last_log_time;

void logWithColor(const std::string& color_code, const std::string& message) {
    ROS_INFO_STREAM("\033[" << color_code << "m" << message << "\033[0m");
}

bool shouldLog() {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time).count();
    if (elapsed_time >= 1) { // Log every 1 second
        last_log_time = current_time;
        return true;
    }
    return false;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& data) {
    static auto start_time = ros::Time::now();  // Record start time

    // Get parameters from the parameter server for the rectangular area
    double x_min = ros::param::param<double>("~x_min", -1);
    double x_max = ros::param::param<double>("~x_max", 1);
    double y_min = ros::param::param<double>("~y_min", -1);
    double y_max = ros::param::param<double>("~y_max", 1);
    double z_min = ros::param::param<double>("~z_min", -2.0);
    double z_max = ros::param::param<double>("~z_max", 3);
    double max_x = ros::param::param<double>("~max_x", 50);
    double max_y = ros::param::param<double>("~max_y", 50);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*data, *cloud);

    std::vector<std::pair<double, double>> filtered_points;
    double min_z = std::numeric_limits<double>::max();

    for (const auto& point : cloud->points) {
        if (z_min <= point.z && point.z <= z_max &&
            ((point.x >= x_max || point.x <= x_min) ||
             (point.y >= y_max || point.y <= y_min)) &&
            (point.x <= max_x && point.x >= -max_x) && (point.y <= max_y && point.y >= -max_y))
        {
            filtered_points.emplace_back(point.x, point.y);
            if (point.z < min_z) {
                min_z = point.z;
            }
        }
    }

    auto end_time = ros::Time::now();  // Record end time
    double process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds

    // Only log if enough time has passed
    if (shouldLog()) {
        logWithColor("32", "Filtering time: " + std::to_string(process_time) + " seconds");
    }

    if (!filtered_points.empty()) {
        const double resolution = 0.05;
        const int width = 800;
        const int height = 800;
        const double min_x = -20.0;
        const double min_y = -20.0;

        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        try {
            // Transform points from lidar frame to base_footprint frame
            geometry_msgs::TransformStamped transform_to_base;
            transform_to_base = tf_buffer.lookupTransform("base_footprint", data->header.frame_id, ros::Time(0), ros::Duration(1.0));

            std::vector<std::pair<double, double>> transformed_points;
            for (const auto& point : filtered_points) {
                geometry_msgs::PointStamped point_stamped;
                point_stamped.point.x = point.first;
                point_stamped.point.y = point.second;
                point_stamped.point.z = 0.0;
                point_stamped.header.frame_id = data->header.frame_id;
                point_stamped.header.stamp = ros::Time(0);

                geometry_msgs::PointStamped transformed_point_stamped;
                tf2::doTransform(point_stamped, transformed_point_stamped, transform_to_base);
                transformed_points.emplace_back(transformed_point_stamped.point.x, transformed_point_stamped.point.y);
            }

            // Create grid_map in base_footprint frame
            nav_msgs::OccupancyGrid grid_map;
            grid_map.header.frame_id = "base_footprint";  // Set to base_footprint coordinate frame
            grid_map.header.stamp = ros::Time::now();
            grid_map.info.resolution = resolution;
            grid_map.info.width = width;
            grid_map.info.height = height;
            grid_map.info.origin.position.x = min_x;
            grid_map.info.origin.position.y = min_y;
            grid_map.info.origin.orientation.w = 1.0;

            std::vector<int8_t> grid_data(width * height, 0);

            for (const auto& point : transformed_points) {
                int grid_x = static_cast<int>((point.first - min_x) / resolution);
                int grid_y = static_cast<int>((point.second - min_y) / resolution);
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    grid_data[grid_y * width + grid_x] = 100;
                }
            }

            grid_map.data = grid_data;

            // Lookup transform from base_footprint to map
            geometry_msgs::TransformStamped transform_to_map;
            transform_to_map = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));

            // Transform the origin of grid_map to map frame
            geometry_msgs::PoseStamped pose_in_base, pose_in_map;
            pose_in_base.pose = grid_map.info.origin;
            pose_in_base.header.frame_id = "base_footprint";
            pose_in_base.header.stamp = ros::Time(0); // Use latest available transform

            // Perform the transformation
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(pose_in_base, transformed_pose, transform_to_map);

            // Update grid_map with new origin in map frame
            grid_map.header.frame_id = "map";
            grid_map.info.origin = transformed_pose.pose;

            pub.publish(grid_map);

            end_time = ros::Time::now();  // Record end time
            process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds

            // Only log if enough time has passed
            if (shouldLog()) {
                logWithColor("32", "Processing and publishing occupancy grid took: " + std::to_string(process_time) + " seconds");
            }
        } catch (tf2::LookupException& ex) {
            logWithColor("31", "TF lookup failed: " + std::string(ex.what()));
            end_time = ros::Time::now();  // Record end time on exception
            process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds
            logWithColor("31", "Total processing time (including exception): " + std::to_string(process_time) + " seconds");
        } catch (tf2::ConnectivityException& ex) {
            logWithColor("31", "TF connectivity failed: " + std::string(ex.what()));
            end_time = ros::Time::now();  // Record end time on exception
            process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds
            logWithColor("31", "Total processing time (including exception): " + std::to_string(process_time) + " seconds");
        } catch (tf2::ExtrapolationException& ex) {
            logWithColor("31", "TF extrapolation failed: " + std::string(ex.what()));
            end_time = ros::Time::now();  // Record end time on exception
            process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds
            logWithColor("31", "Total processing time (including exception): " + std::to_string(process_time) + " seconds");
        }
    } else {
        // Only log if enough time has passed
        if (shouldLog()) {
            logWithColor("31", "Failed to find any points!");
            end_time = ros::Time::now();  // Record end time on failure
            process_time = (end_time - start_time).toSec();  // Calculate processing time in seconds
            logWithColor("31", "Failed processing, took: " + std::to_string(process_time) + " seconds");
        }
    }

    // Reset start time for next iteration
    start_time = ros::Time::now();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rslidar_points_processor");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    ros::Subscriber sub = nh.subscribe("rslidar_points", 1, pointCloudCallback);
    ros::spin();
    return 0;
}