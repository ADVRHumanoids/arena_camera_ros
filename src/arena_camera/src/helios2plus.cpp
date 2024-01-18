#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <chrono>
#include "ArenaApi.h"

// system timeout
#define SYSTEM_TIMEOUT 2000

// image timeout
#define IMAGE_TIMEOUT 2000

#define TLX 225//160
#define TLY 90//120
#define W   210//320
#define H   185//240

Arena::ISystem* pSystem = nullptr;
std::vector<Arena::DeviceInfo> deviceInfos;
Arena::IDevice* pDevice = nullptr;
GenApi::INodeMap* pNodeMap = nullptr;
GenApi::INodeMap* pStreamNodeMap = nullptr;
Arena::IImage* pImage = nullptr;
float offX, offY, offZ;

uint64_t _s0, _e0, _t0;
std::array<int, 2> frame_size;

bool init(const int threshold = 0, const bool spatial_filter = true, const bool flying_filter = true, const int accum = 1){
	frame_size[0] = W*H;
	frame_size[1] = 1;
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
	for (id = 0; id < deviceInfos.size(); id++){
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
}

bool get_point_cloud(std::vector<float> buf){
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

	buf.resize(W*H);

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

	for (int r = 0; r < H; r++){
		for (int c = 0; c < W; c++){
			ROS_WARN("%f ", buf[r*W+c]);
		}
	}

	_e0 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	_t0 = _e0 - _s0;
	pDevice->RequeueBuffer(pImage);
	return true;
}

 int main(int argc, char** argv){

    ros::init(argc, argv, "image_to_cloud");
    ros::NodeHandle nh;

	ros::Rate r(20);

	std::vector<float> vect;
	ROS_WARN("STARTED");

	init();

	while (ros::ok())
	{
		get_point_cloud(vect);
		r.sleep();
	}

    return 0;
}