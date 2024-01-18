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

#define TLX 225//160
#define TLY 90//120
#define W   210//320
#define H   185//240

void acquire_image(const sensor_msgs::ImagePtr& msg)
{
    int w = msg->width;
    int h = msg->height;
    
    
    std::vector<uint8_t> data = msg->data;
           
    int size  = data.size();

    std::vector<float> buf;
    buf.resize(size);
        
    uint32_t i, o;
    uint16_t A, B, C;
    
    cloud->points.clear();

                
    for (int r = TLY; r< (TLY+H); r++){
        for (int c = TLX; c< (TLX+W); c++){
            i = r * 640 + c;
            o = (r - TLY)*W + (c-TLX);

            A = uint16_t(data[o*3 + 0]);
            B = uint16_t(data[o*3 + 1]);
            C = uint16_t(data[o*3 + 2]);

            if (A!=0xFFFF and B!=0xFFFF and C!=0xFFFF && (C*0.25f/*+offZ*/)>280.0f/*mm*/){// and (C*0.25f) > 280.0f/*mm*/){
                buf[i*3+0] = (A*0.25f);
                buf[i*3+1] = (B*0.25f);
                buf[i*3+2] = (C*0.25f);
            }
            else{
                buf[i*3+0] = buf[i*3+1] = buf[i*3+2] = 0;
            }          
        }
    }
        
    cloud->header.frame_id = "arena_camera";
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub_cloud.publish(cloud);
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "image_to_cloud");
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


