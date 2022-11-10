
//
#include "stdafx.h"
#include <time.h>
#include <cmath>
#include "mfc_lvx_sdk.h"
#include "mfc_lvx_file.h"
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_utils.hpp"
#include "io.h"
#include "direct.h"

#define WRITE_BUFFER_LEN 1024 * 1024
#define MAGIC_CODE       (0xac0ea767)
#define RAW_POINT_NUM     100
#define SINGLE_POINT_NUM  96
#define DUAL_POINT_NUM    48
#define IMU_POINT_NUM     1
#define M_PI             3.14159265358979323846
LvxFileHandle::LvxFileHandle() : cur_frame_index_(0), cur_offset_(0), frame_duration_(kDefaultFrameDurationTime) {}
CString g_strSavePath;
bool LvxFileHandle::InitLvxFile() {
	time_t curtime = time(nullptr);
	char strFileNameTemp[80] = { 0 };
	char* strFullFileName = new char[80];
	tm* local_time = localtime(&curtime);
	char strLvxFolderName[] = "lvxFiles";
	
	if (g_strSavePath == "")
	{
		// 文件夹不存在则创建文件夹
		if (_access(strLvxFolderName, 0) == -1)
		{
			_mkdir(strLvxFolderName);
		}
		strftime(strFileNameTemp, sizeof(strFileNameTemp), "lvxFiles\\%Y-%m-%d_%H-%M-%S.lvx", local_time);
		strcpy(strFullFileName, strFileNameTemp);
	}
	else
	{
		strftime(strFileNameTemp, sizeof(strFileNameTemp), "lvxFiles\\%Y-%m-%d_%H-%M-%S.lvx", local_time);
		USES_CONVERSION;
		char *strFullFileNameTemp = new char[strlen(g_strSavePath) + strlen(strFileNameTemp) + 3];
		strcpy(strFullFileNameTemp, g_strSavePath);
		if(strlen(g_strSavePath) > 3)
			strcat(strFullFileNameTemp, "\\");
		strcat(strFullFileNameTemp, strFileNameTemp);
		strcpy(strFullFileName, strFullFileNameTemp);
		//create \\lvx folder
		char* FullFilePath = new char[strlen(strLvxFolderName) + strlen(g_strSavePath) + 3];
		strcpy(FullFilePath, g_strSavePath);
		if(strlen(g_strSavePath) > 3)
			strcat(FullFilePath, "\\");
		strcat(FullFilePath, strLvxFolderName);
		// 文件夹不存在则创建文件夹
		if (_access(FullFilePath, 0) == -1)
		{
			_mkdir(FullFilePath);
		}
		delete [] FullFilePath;
		FullFilePath = NULL;
		delete [] strFullFileNameTemp;
		strFullFileNameTemp = NULL;
	}
	lvx_file_.open(strFullFileName, std::ios::out | std::ios::binary);
	if (!lvx_file_.is_open()) {
		return false;
	}
	delete [] strFullFileName;
	strFullFileName = NULL;
	return true;
}

void LvxFileHandle::InitLvxFileHeader() {
	LvxFilePublicHeader lvx_file_public_header = { 0 };
	std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
	std::string signature = "livox_tech";
	memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());

	lvx_file_public_header.version[0] = 1;
	lvx_file_public_header.version[1] = 1;
	lvx_file_public_header.version[2] = 0;
	lvx_file_public_header.version[3] = 0;

	lvx_file_public_header.magic_code = MAGIC_CODE;
	cur_offset_ = 0;//重设为0
	memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
	cur_offset_ += sizeof(LvxFilePublicHeader);

	uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
	LvxFilePrivateHeader lvx_file_private_header = { 0 };
	lvx_file_private_header.frame_duration = frame_duration_;
	lvx_file_private_header.device_count = device_count;

	memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
	cur_offset_ += sizeof(LvxFilePrivateHeader);

	for (int i = 0; i < device_count; i++) {
		memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
		cur_offset_ += sizeof(LvxDeviceInfo);
	}

	lvx_file_.write((char *)write_buffer.get(), cur_offset_);
}

void LvxFileHandle::SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {//数据写入本地
	uint64_t cur_pos = 0;
	FrameHeader frame_header = { 0 };
	std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

	frame_header.current_offset = cur_offset_;
	frame_header.next_offset = cur_offset_ + sizeof(FrameHeader);
	auto iterator = point_packet_list_temp.begin();
	for (iterator = point_packet_list_temp.begin(); iterator != point_packet_list_temp.end(); iterator++) {
		frame_header.next_offset += iterator->pack_size;
	}

	frame_header.frame_index = cur_frame_index_;

	memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(FrameHeader));
	cur_pos += sizeof(FrameHeader);

	auto iter = point_packet_list_temp.begin();//写入点云
	for (; iter != point_packet_list_temp.end(); iter++) {
		if (cur_pos + iter->pack_size >= WRITE_BUFFER_LEN) {
			lvx_file_.write((char*)write_buffer.get(), cur_pos);
			cur_pos = 0;
			memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
			cur_pos += iter->pack_size;
		}
		else {
			memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), iter->pack_size);
			cur_pos += iter->pack_size;
		}
	}
	lvx_file_.write((char*)write_buffer.get(), cur_pos);

	cur_offset_ = frame_header.next_offset;
	cur_frame_index_++;
}

void LvxFileHandle::CloseLvxFile() {
	if (lvx_file_.is_open())
		lvx_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet) {//设备获取点数据->packet
	packet.version = data->version;
	packet.port_id = data->slot;
	packet.lidar_index = data->id;
	packet.rsvd = data->rsvd;
	packet.error_code = data->err_code;
	packet.timestamp_type = data->timestamp_type;
	packet.data_type = data->data_type;
	memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
	//mid40 笛卡尔坐标系
	packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + RAW_POINT_NUM * sizeof(LivoxRawPoint);// = 1523-1500-4+100*13=
	memcpy(packet.raw_point, (void *)data->data, RAW_POINT_NUM * sizeof(LivoxRawPoint));
}

void ParseExtrinsicXml(DeviceItem &item, LvxDeviceInfo &info) {
	rapidxml::file<> extrinsic_param("extrinsic.xml");
	rapidxml::xml_document<> doc;
	doc.parse<0>(extrinsic_param.data());
	rapidxml::xml_node<>* root = doc.first_node();
	if ("Livox" == (std::string)root->name()) {
		for (rapidxml::xml_node<>* device = root->first_node(); device; device = device->next_sibling()) {
			if ("Device" == (std::string)device->name() && (strncmp(item.info.broadcast_code, device->value(), kBroadcastCodeSize) == 0)) {
				memcpy(info.lidar_broadcast_code, device->value(), kBroadcastCodeSize);
				memset(info.hub_broadcast_code, 0, kBroadcastCodeSize);
				info.device_type = item.info.type;
				info.device_index = item.handle;
				for (rapidxml::xml_attribute<>* param = device->first_attribute(); param; param = param->next_attribute()) {
					if ("roll" == (std::string)param->name()) info.roll = static_cast<float>(atof(param->value()));
					if ("pitch" == (std::string)param->name()) info.pitch = static_cast<float>(atof(param->value()));
					if ("yaw" == (std::string)param->name()) info.yaw = static_cast<float>(atof(param->value()));
					if ("x" == (std::string)param->name()) info.x = static_cast<float>(atof(param->value()));
					if ("y" == (std::string)param->name()) info.y = static_cast<float>(atof(param->value()));
					if ("z" == (std::string)param->name()) info.z = static_cast<float>(atof(param->value()));
				}
			}
		}
	}
}
