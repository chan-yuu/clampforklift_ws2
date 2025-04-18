#include "roi_clip.h"


RoiClip::RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{  
    private_node_handle.param("roi_x_min", roi_x_min_, 0.0);
    private_node_handle.param("roi_x_max", roi_x_max_, 100.0);
    private_node_handle.param("roi_y_min", roi_y_min_, -10.0);
    private_node_handle.param("roi_y_max", roi_y_max_, 10.0);
    private_node_handle.param("roi_z_min", roi_z_min_, -1.75);
    private_node_handle.param("roi_z_max", roi_z_max_, 2.0);
    // 转换到车辆坐标系下后，将车身点云切除，车辆坐标系中心为后轴中心０点
    // 坐标系参数不确定，暂时不转换
    private_node_handle.param("vehicle_x_min", vehicle_x_min_, -1.2);
    private_node_handle.param("vehicle_x_max", vehicle_x_max_, 3.0);
    private_node_handle.param("vehicle_y_min", vehicle_y_min_, -1.0);
    private_node_handle.param("vehicle_y_max", vehicle_y_max_, 1.0);
    private_node_handle.param("vehicle_z_min", vehicle_z_min_, -1.7);
    private_node_handle.param("vehicle_z_max", vehicle_z_max_, 0.2);
}



 bool RoiClip::IsIn(const float x, const float x_min, const float x_max)
 {
      return (x < x_max) && (x > x_min);
 }

pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,pcl::PointCloud<pcl::PointXYZI>::Ptr &out,Eigen::Matrix4f base_to_sensor_matrix)
{
    float  roi_z_min = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipCloud = ClipVehicle(in,base_to_sensor_matrix);
    
    if (clipCloud->points.size() > 0)
    {
        for (auto &p : clipCloud->points)
        {
            if (IsIn(p.x, roi_x_min_, roi_x_max_) && IsIn(p.y, roi_y_min_, roi_y_max_) && IsIn(p.z, roi_z_min_, roi_z_max_))
            out->push_back(p);
        }
       
        return out;

    }

   
    return nullptr;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,Eigen::Matrix4f base_to_sensor_matrix){
   
    if(in->points.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

        for (auto &p : in->points)
        {
            
            Eigen::Vector4f point(p.x,p.y,p.z,1);
           Eigen::Vector4f transPoint  = base_to_sensor_matrix*point;
        
            if (IsIn(transPoint(0), vehicle_x_min_, vehicle_x_max_) && IsIn(transPoint(1), vehicle_y_min_, vehicle_y_max_) && IsIn(transPoint(2), vehicle_z_min_, vehicle_z_max_)){
                
            }
            else out->push_back(p);
        }
        return out;
    }
    return nullptr; 
}

