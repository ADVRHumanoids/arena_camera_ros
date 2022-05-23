#include <ros/ros.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr cloud (new PointCloud);
ros::Publisher pub_cloud;

#define f_x 523.136597
#define f_y 523.136597

#define c_x 325.644318
#define c_y 238.680389
#define depth_scaling 1
#define data_scaling 0.29944

void acquire_image(const sensor_msgs::ImagePtr& msg)
{
    
    ROS_INFO("Acquire Image");
    
    int w = msg->width;
    int h = msg->height;
    
    
    std::vector<uint8_t> data = msg->data;
           
    int size  = data.size();
        
    uint32_t i;
    
    cloud->points.clear();
    
    ROS_INFO("Build Cloud");
                
    for (int r = 0; r< h; r++){
        for (int c = 0; c< w; c++){
            i = r * w + c;
                        
            if (data[i] < 254){// and (C*0.25f) > 280.0f/*mm*/){
                                
                cloud->points.push_back(pcl::PointXYZ(
                                    (((c - c_x)*(255-data[i]*data_scaling)/f_x)),
                                    (((r - c_y)*(255-data[i]*data_scaling)/f_y)),
                                    ((255-data[i])*data_scaling*depth_scaling)
                                    ));
                
            }            
        }
    }
    
    ROS_INFO("Publish");
    
    cloud->header.frame_id = "arena_camera";
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub_cloud.publish(cloud);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "denoisecloud");
    ros::NodeHandle nh;
    
    ros::Subscriber subscriber = nh.subscribe("/arena_camera_node/image_raw", 1, acquire_image);
        
    pub_cloud = nh.advertise<PointCloud>("arena_camera/depth/points", 1);

    
    ros::Rate loop_rate(100);
    //Publish the original Point Cloud
    while(ros::ok()){


        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}


