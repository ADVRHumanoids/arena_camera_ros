#include <ros/ros.h>
#include <iostream>
#include "ArenaApi.h"
#include <vector>
#include <thread>
#include <iomanip> // For setw, setfill

#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

double f_x, f_y, c_x, c_y;


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_cloud");
    ros::NodeHandle n;
    
    bool exceptionThrown = false;
    
    
    PointCloud::Ptr cloud (new PointCloud);
    ros::Publisher pub_cloud = n.advertise<PointCloud>("arena_camera/depth/points", 1);
    
    Arena::ISystem* pSystem;
//     ImageCallback* pCallbackHandler;
    Arena::IDevice* pDevice;
    
    double xyz_scale_mm, x_offset_mm, y_offset_mm, z_offset_mm;
    
    double k1, k2, k3, p1, p2;
    
    try
    {
        // prepare example
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(100);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        if (deviceInfos.size() == 0)
        {
                std::cout << "\nNo camera connected\nPress enter to complete\n";
                std::getchar();
                return 0;
        }
        //Get Serial Number
        pDevice = pSystem->CreateDevice(deviceInfos[0]);
        
        
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
                
        Arena::SetNodeValue<bool>( pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true); // so no points are marked as invalid
        Arena::SetNodeValue<double>( pDevice->GetNodeMap(), "Scan3dAmplitudeGain", 5.0); // should make the intensity image look brighter
        
//         Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
        
        xyz_scale_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale");
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateA");
        x_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateB");
        y_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");
        z_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");
        
//         GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode");
//         Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance3000mmSingleFreq"); //Find the right value
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance5000mmMultiFreq"); 
        
//         Arena::SetNodeValue<bool>( pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
//         Arena::SetNodeValue<bool>( pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
//         Arena::SetNodeValue<int64_t>( pDevice->GetTLStreamNodeMap(), "StreamMaxNumResendRequestsPerImage", 500);
        
        ROS_WARN("PARAMS: %f, %f, %f, %f", xyz_scale_mm, x_offset_mm, y_offset_mm, z_offset_mm);

        Arena::SetNodeValue<int64_t>( pDevice->GetNodeMap(), "GevSCPD", 8000); 
        Arena::SetNodeValue<int64_t>( pDevice->GetNodeMap(), "DeviceLinkThroughputReserve", 30); 
        

        //In a similar way you can also read out parameters:
        f_x = Arena::GetNodeValue<double>( pDevice->GetNodeMap(), "CalibFocalLengthX");
        f_y = Arena::GetNodeValue<double>( pDevice->GetNodeMap(), "CalibFocalLengthY");
        c_x = Arena::GetNodeValue<double>( pDevice->GetNodeMap(), "CalibOpticalCenterX");
        c_y = Arena::GetNodeValue<double>( pDevice->GetNodeMap(), "CalibOpticalCenterY");
                
        ROS_WARN("Calib param: %f, %f, %f, %f", f_x, f_y, c_x, c_y);
        
        
//         Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value0");
//         double k1 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value0");
        k1 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value1");
        k2 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value2");
        p1 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value3");
        p2 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", "Value4");
        k3 = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue");
        
        ROS_WARN("DISTORTION: %f, %f, %f, %f, %f", k1, k2, p1, p2, k3);
        
        //You can read out the calibration parameters:
        //CalibFocalLengthX, CalibFocalLengthY, CalibOpticalCenterX, CalibOpticalCenterY
        //and
        //CalibLensDistortionValue for CalibLensDistortionValueSelector = Value0 (k1), Value1 (k2), Value2 (p1), Value3 (p2), Value4 (k3)
        

        std::cout << "Using camera with serial number: " << deviceInfos[0].SerialNumber() << std::endl;

        // Allocate the image callback handler object
//         pCallbackHandler = new ImageCallback(std::string(deviceInfos[0].SerialNumber()));
        
//         pCallbackHandler->SetPublisher(&n, "arena_camera/depth/points");

        // Register the callback handler to the device
//         pDevice->RegisterImageCallback(pCallbackHandler);

        pDevice->StartStream();
    }
    catch (...)
    {
        std::cout << "\nUnexpected exception thrown\n";
        exceptionThrown = true;
    }
    
    ros::Rate loop_rate(20);
    //Publish the original Point Cloud
    while(ros::ok()){
            
        Arena::IImage* pImage;
        ROS_INFO("Ask for image");
        try{
            pImage = pDevice->GetImage(2000);
        }
        catch(...){
            ROS_INFO("EXCEPTION!!");
            continue;
        }
        
        if (pImage->IsIncomplete()){
                //std::cerr << "lucidlabs_helios2: ERROR - incomplete frame" << std::endl;
            ROS_INFO("Image not found !!");
            pDevice->RequeueBuffer(pImage);
            continue;
        }
            // std::cout << "lucidlabs_helios2: GOOD frame" << std::endl;

        ROS_INFO("Image found !");
        
        const uint16_t *data = (uint16_t*)pImage->GetData();
        uint16_t A,B,C;
        uint32_t i, o;
    
        double H = pImage->GetHeight();
        double W = pImage->GetWidth();
        
        cloud->points.clear();
        
        for (int r = 0; r < H; r++){
            for (int c = 0; c < W; c++){
                
                pcl::PointXYZI point;
                /*
                if(data[0] > 65530 || data[1] > 65530 || data[2] > 65530 || data[3] > 65530)
                    continue ;*/
                
                point.x = (float)(double(data[0]) * xyz_scale_mm + x_offset_mm)/1000;
                point.y = (float)(double(data[1]) * xyz_scale_mm + y_offset_mm)/1000;
                point.z = (float)(double(data[2]) * xyz_scale_mm + z_offset_mm)/1000;
                point.intensity = (float)(data[3]);
               
                /*
                double c_x = 0, c_y = 0;
                double r = sqrt(pow(point.x - c_x, 2) + pow(point.y - c_y, 2));
                
                double dist_x = point.x, dist_y = point.y;
                
                // Distortion correction
                point.x = dist_x + (dist_x - c_x)*(k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6)) + (p1*(r*r + 2*pow(dist_x - c_x,2)) + 2*p2*(dist_x - c_x)*(dist_y - c_y));
                point.y = dist_y + (dist_y - c_y)*(k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6)) + (2*p1*(dist_x - c_x)*(dist_y - c_y) + p2*(r*r + 2*pow(dist_y - c_y,2)));
                
                // Only radial
                point.x = c_x + (dist_x - c_x)/(1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6));
                point.y = c_y + (dist_y - c_y)/(1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6));
                */
                
                cloud->points.push_back(point);

                data += 4;
            }
        }
        
        ROS_INFO("Publish !");
        cloud->header.frame_id = "arena_camera";
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub_cloud.publish(cloud);
                
        ros::spinOnce();
        loop_rate.sleep();
        
        pDevice->RequeueBuffer(pImage);
        
//         delete data;
//         delete pImage;
    }
    
    ROS_WARN("STOPPED");
    
    
    pDevice->StopStream();
    // Deregister the callback handler
//     pDevice->DeregisterImageCallback(pCallbackHandler);

    // Free the callback handler object
//     delete pCallbackHandler;
    // clean up example
    pSystem->DestroyDevice(pDevice);
    Arena::CloseSystem(pSystem);
    
    return 0;
}
