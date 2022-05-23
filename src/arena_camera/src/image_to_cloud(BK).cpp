#include <ros/ros.h>
/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>*/

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <cassert>
#include <chrono>
#include "ArenaApi.h"

namespace py = pybind11;

// system timeout
#define SYSTEM_TIMEOUT 2000

// image timeout
#define IMAGE_TIMEOUT 2000

#define TLX 225//160
#define TLY 90//120
#define W   210//320
#define H   185//240


// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

class PyHALLucidLabsHelios2 {
	Arena::ISystem* pSystem = nullptr;
	std::vector<Arena::DeviceInfo> deviceInfos;
	Arena::IDevice* pDevice = nullptr;
    GenApi::INodeMap* pNodeMap = nullptr;
    GenApi::INodeMap* pStreamNodeMap = nullptr;
    Arena::IImage* pImage = nullptr;
    float offX, offY, offZ;

public:
	uint64_t _s0, _e0, _t0;
	py::array_t<int, py::array::f_style> frame_size = py::array_t<int, py::array::f_style>(2);
	PyHALLucidLabsHelios2(){}
	~PyHALLucidLabsHelios2(){
		if (pDevice){
		    pDevice->StopStream();
			pSystem->DestroyDevice(pDevice);
		}
		if (pSystem){
			Arena::CloseSystem(pSystem);
		}
	}

	bool init(const int threshold = 0, const bool spatial_filter = true, const bool flying_filter = true, const int accum = 1) try {
		((int*)(frame_size.request().ptr))[0] = W*H;
		((int*)(frame_size.request().ptr))[1] = 1;
		pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(SYSTEM_TIMEOUT);
		deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0){
			std::cerr << "\nNo camera connected" << std::endl;
			return false;
		}

		unsigned int id;
		unsigned int found_id;
		bool found = false;
		for (id = 0; id<deviceInfos.size(); id++){
			std::cout << "Model name: " << deviceInfos[id].ModelName() << std::endl;
			if (deviceInfos[id].ModelName() == "HLT003S-001"){
				found = true;
				found_id = id;
			}
		}

		if (not found){
			std::cerr << "No Helios 2 Camera found." << std::endl;
			return false;
		}
		try{
			pDevice = pSystem->CreateDevice(deviceInfos[found_id]);
		}
		catch (GenICam::GenericException& ge)
		{
		        std::cout << "\nGenICam CreateDevice - exception thrown: " << ge.what() << "\n";
			return false;
		}
		pNodeMap = pDevice->GetNodeMap();
		pStreamNodeMap = pDevice->GetTLStreamNodeMap();

		GenApi::CEnumerationPtr pModeSel = pNodeMap->GetNode("Scan3dModeSelector");
		GenApi::CEnumEntryPtr pModeProcessed = pModeSel->GetEntryByName("Processed");
		pModeSel->SetIntValue(pModeProcessed->GetValue());

		Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "ChunkModeActive", false);

		GenApi::CEnumerationPtr pStreamBufferHandlingMode = pStreamNodeMap->GetNode("StreamBufferHandlingMode");
		GenApi::CEnumEntryPtr pNewestOnly = pStreamBufferHandlingMode->GetEntryByName("NewestOnly");
		pStreamBufferHandlingMode->SetIntValue(pNewestOnly->GetValue());

		GenApi::CEnumerationPtr pPixelMode = pNodeMap->GetNode("PixelFormat");
		GenApi::CEnumEntryPtr pCoord3D_ABC16s = pPixelMode->GetEntryByName("Coord3D_ABC16");
		pPixelMode->SetIntValue(pCoord3D_ABC16s->GetValue());

		/*
		GenApi::CIntegerPtr pDeviceStreamChannelPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
		pDeviceStreamChannelPacketSize->SetValue(pDeviceStreamChannelPacketSize->GetMax());
		std::cout << "MTU: " << pDeviceStreamChannelPacketSize->GetMax() << std::endl;
		*/
	
		Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
		Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

		GenApi::CEnumerationPtr pExposureTime = pNodeMap->GetNode("ExposureTimeSelector");
		GenApi::CEnumEntryPtr pExp250Us = pExposureTime->GetEntryByName("Exp62_5Us");
		pExposureTime->SetIntValue(pExp250Us->GetValue());

		GenApi::CEnumerationPtr pOperatingMode = pNodeMap->GetNode("Scan3dOperatingMode");
		GenApi::CEnumEntryPtr pDistance1250mmSingleFreqs = pOperatingMode->GetEntryByName("Distance1250mmSingleFreq");
		pOperatingMode->SetIntValue(pDistance1250mmSingleFreqs->GetValue());

		GenApi::CEnumerationPtr pScan3DCoordinateSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
		GenApi::CEnumEntryPtr pCoordinateA = pScan3DCoordinateSelector->GetEntryByName("CoordinateA");
		GenApi::CEnumEntryPtr pCoordinateB = pScan3DCoordinateSelector->GetEntryByName("CoordinateB");
		GenApi::CEnumEntryPtr pCoordinateC = pScan3DCoordinateSelector->GetEntryByName("CoordinateC");
		GenApi::CFloatPtr pScan3DCoordinateOffset = pNodeMap->GetNode("Scan3dCoordinateOffset");

	    Arena::SetNodeValue<bool>(pNodeMap, "Scan3dConfidenceThresholdEnable", true);

		GenApi::CIntegerPtr pThresh = pNodeMap->GetNode("Scan3dConfidenceThresholdMin");
		pThresh->SetValue(threshold);

		GenApi::CBooleanPtr pSpatialFilter = pNodeMap->GetNode("Scan3dSpatialFilterEnable");
		pSpatialFilter->SetValue(spatial_filter);

		GenApi::CBooleanPtr pFlyingFilter = pNodeMap->GetNode("Scan3dFlyingPixelsRemovalEnable");
		pFlyingFilter->SetValue(flying_filter);

		Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", accum);


		pScan3DCoordinateSelector->SetIntValue(pCoordinateA->GetValue());
		offX = pScan3DCoordinateOffset->GetValue();
		pScan3DCoordinateSelector->SetIntValue(pCoordinateB->GetValue());
		offY = pScan3DCoordinateOffset->GetValue();
		pScan3DCoordinateSelector->SetIntValue(pCoordinateC->GetValue());
		offZ = pScan3DCoordinateOffset->GetValue();

		pDevice->StartStream();
		return true;
	} catch (GenICam::GenericException& ge) {
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		return false;
	} catch (std::exception& ex) {
		std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
		return false;
	} catch (...){
		std::cout << "\nUnexpected exception thrown\n";
		return false;
	}

	bool get_point_cloud(float* buf){
		pImage = pDevice->GetImage(IMAGE_TIMEOUT);
		assert(pImage != nullptr);

		if (pImage->IsIncomplete()){
            		//std::cerr << "lucidlabs_helios2: ERROR - incomplete frame" << std::endl;
			pDevice->RequeueBuffer(pImage);
            		return true;
       		}
            	// std::cout << "lucidlabs_helios2: GOOD frame" << std::endl;

		_s0 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		const uint16_t *data = (uint16_t*)pImage->GetData();
		uint16_t A,B,C;
		uint32_t i, o;

		for (int r = TLY; r<(TLY+H); r++){
			for (int c = TLX; c<(TLX+W); c++){
				o = r * 640 + c;
				i = (r-TLY) * W + (c-TLX);
				A = data[o*3+0];
				B = data[o*3+1];
				C = data[o*3+2];
				if (A!=0xFFFF and B!=0xFFFF and C!=0xFFFF and (C*0.25f+offZ)>280.0f/*mm*/){
					buf[i*3+0] = (A*0.25f+offX);
					buf[i*3+1] = (B*0.25f+offY);
					buf[i*3+2] = (C*0.25f+offZ);
				} else {
					buf[i*3+0] = buf[i*3+1] = buf[i*3+2] = 0;
				}
			}
		}
		_e0 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		_t0 = _e0 - _s0;
		pDevice->RequeueBuffer(pImage);
		return true;
	}

	bool py_get_point_cloud(py::array_t<float, py::array::c_style | py::array::forcecast> arr,
			    				py::array_t<unsigned int, py::array::c_style | py::array::forcecast> arr_size){
		assert(arr_size.request().ptr != nullptr);
		((unsigned int*)arr_size.request().ptr)[0] = (unsigned int) W*H;
		((unsigned int*)arr_size.request().ptr)[1] = (unsigned int) 1;
        auto data_ptr = (float*)arr.request().ptr;
        auto gil_release = py::gil_scoped_release();
		
		return get_point_cloud(data_ptr);
	}
};

/*
PYBIND11_MODULE(lucidlabs_helios2, m) {
    py::class_<PyHALLucidLabsHelios2>(m, "HAL")
		.def(py::init<>())
		.def("init", &PyHALLucidLabsHelios2::init,  py::arg("threshold") = 0,
													py::arg("spatial_filter") = true,
													py::arg("flying_filter") = true,
													py::arg("accum")=1)
		.def_readonly("frame_size", &PyHALLucidLabsHelios2::frame_size)
		.def("get_data", &PyHALLucidLabsHelios2::py_get_point_cloud)
		.def_readonly("_s0", &PyHALLucidLabsHelios2::_s0)
		.def_readonly("_e0", &PyHALLucidLabsHelios2::_e0)
		.def_readonly("_t0", &PyHALLucidLabsHelios2::_t0);
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_cloud");
    ros::NodeHandle n;
    
    ROS_INFO("Start !");
    
    
    PyHALLucidLabsHelios2 p;
    
    float * buf = new float[W*H];
    
    p.init();
    p.get_point_cloud(buf);
    
    
    ROS_INFO("Running ...");
    ros::spin();
    ROS_WARN("STOPPED");
    
//     pDevice->StopStream();
//     // Deregister the callback handler
//     pDevice->DeregisterImageCallback(pCallbackHandler);

//     // Free the callback handler object
//     delete pCallbackHandler;
//     // clean up example
//     pSystem->DestroyDevice(pDevice);
//     Arena::CloseSystem(pSystem);
    
    return 0;
}
