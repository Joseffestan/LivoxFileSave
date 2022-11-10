
// LivoxFileSaveDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "LivoxFileSave.h"
#include "LivoxFileSaveDlg.h"
#include "afxdialogex.h"
#include "apr/apr_general.h"
#include "apr/apr_getopt.h"
#include <algorithm>
#include "mfc_lvx_file.h"
#include <string.h>
#include <chrono>
#include <thread>
#include <time.h>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers> 
#include <osgViewer/CompositeViewer>
#include <osgUtil/optimizer>
#include <osg/LineWidth>
#include <osg/Point>
#include <osgDB/WriteFile>
#include <windows.h>
#include "iostream"
#include "io.h"
#include "direct.h"

using namespace std;
using namespace osg;
using namespace std::chrono;
bool g_bLastFrameDataShowing = FALSE;
DeviceItem devices[1];
LvxFileHandle lvx_file_handler;
list<LvxBasePackDetail> point_packet_list;
vector<std::string> broadcast_code_rev;
condition_variable lidar_arrive_condition;
condition_variable extrinsic_condition;
condition_variable point_pack_condition;
mutex mtx;//ȫ�ֻ�����
mutex mtxRealTimeFile;
mutex mtxLastFrameFile;
bool g_bFinishExtrinsicParameter = false;
bool g_bReadExtrinsicFromXML = false;//extrinsic xml
uint8_t  g_nConnectedLidarCount = 0;
extern CString g_strSavePath;
ref_ptr<Group> rootRealTime = new Group();
ref_ptr<Group> rootLastFrame = new Group();
#define FRAME_RATE 20 

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CLivoxFileSaveDlg* gLivoxFileSaveDlg = nullptr;
std::vector<std::string> broadcast_code_list = {/*"000000000000001","000000000000002","000000000000003","000000000000004"*/ };

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
	static uint32_t error_message_count = 0;
	if (message != NULL) {
		++error_message_count;
		if (0 == (error_message_count % 100)) {
			/*printf("handle: %u\n", handle);
			printf("temp_status : %u\n", message->lidar_error_code.temp_status);
			printf("volt_status : %u\n", message->lidar_error_code.volt_status);
			printf("motor_status : %u\n", message->lidar_error_code.motor_status);
			printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
			printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
			printf("pps_status : %u\n", message->lidar_error_code.device_status);
			printf("fan_status : %u\n", message->lidar_error_code.fan_status);
			printf("self_heating : %u\n", message->lidar_error_code.self_heating);
			printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
			printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
			printf("system_status : %u\n", message->lidar_error_code.system_status);*/
		}
	}
}
// Receiving point cloud data from Livox LiDAR. 
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {//��ȡ�������ݽ���packet list
	if (data) {
		if (handle < g_nConnectedLidarCount && g_bFinishExtrinsicParameter) {
			unique_lock<std::mutex> lock(mtx);
			LvxBasePackDetail packet;
			packet.device_index = handle;
			lvx_file_handler.BasePointsHandle(data, packet);//������packet
			point_packet_list.push_back(packet);//packet����list
		}
	}
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
	//printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
	if (status == kStatusSuccess) {
		if (response != 0) {
			devices[handle].device_state = kDeviceStateConnect;
		}
	}
	else if (status == kStatusTimeout) {
		devices[handle].device_state = kDeviceStateConnect;
	}
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Callback function of get LiDARs' extrinsic parameter. */
void OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data) {
	if (status == kStatusSuccess) {
		if (response != 0) {
			//printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
			std::unique_lock<std::mutex> lock(mtx);
			LvxDeviceInfo lidar_info;
			strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
			memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
			lidar_info.device_index = handle;
			lidar_info.device_type = devices[handle].info.type;
			lidar_info.extrinsic_enable = true;
			lidar_info.pitch = response->pitch;
			lidar_info.roll = response->roll;
			lidar_info.yaw = response->yaw;
			lidar_info.x = static_cast<float>(response->x / 1000.0);
			lidar_info.y = static_cast<float>(response->y / 1000.0);
			lidar_info.z = static_cast<float>(response->z / 1000.0);
			lvx_file_handler.AddDeviceInfo(lidar_info);
			if (lvx_file_handler.GetDeviceInfoListSize() == g_nConnectedLidarCount) {
				g_bFinishExtrinsicParameter = true;
				extrinsic_condition.notify_one();
			}
		}
	}
	else if (status == kStatusTimeout) {
		//printf("GetLidarExtrinsicParameter timeout! \n");
	}
}

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
void LidarGetExtrinsicFromXml(uint8_t handle) {
	LvxDeviceInfo lidar_info;
	ParseExtrinsicXml(devices[handle], lidar_info);
	lvx_file_handler.AddDeviceInfo(lidar_info);
	if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
		g_bFinishExtrinsicParameter = true;
		extrinsic_condition.notify_one();
	}
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
	if (status != kStatusSuccess) {
		//printf("Device Query Informations Failed %d\n", status);
	}
	if (ack) {
		/*printf("firm ver: %d.%d.%d.%d\n",
		ack->firmware_version[0],
		ack->firmware_version[1],
		ack->firmware_version[2],
		ack->firmware_version[3]);*/
	}
}

void LidarConnect(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	QueryDeviceInformation(handle, OnDeviceInformation, NULL);
	if (devices[handle].device_state == kDeviceStateDisconnect) {
		devices[handle].device_state = kDeviceStateConnect;
		devices[handle].info = *info;
	}
}

void LidarDisConnect(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	devices[handle].info = *info;
}
/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
	if (info == nullptr) {
		return;
	}
	//printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
	uint8_t handle = info->handle;
	if (handle >= kMaxLidarCount) {
		return;
	}

	if (type == kEventConnect) {
		LidarConnect(info);
		//printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
	}
	else if (type == kEventDisconnect) {
		LidarDisConnect(info);
		//printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
	}
	else if (type == kEventStateChange) {
		LidarStateChange(info);
		//printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
	}

	if (devices[handle].device_state == kDeviceStateConnect) {
		//printf("Device Working State %d\n", devices[handle].info.state);
		if (devices[handle].info.state == kLidarStateInit) {
			//printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
		}
		else {
			//printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
		}
		//printf("Device feature %d\n", devices[handle].info.feature);
		SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
		if (devices[handle].info.state == kLidarStateNormal) {
			if (!g_bReadExtrinsicFromXML) {
				LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
			}
			else {
				LidarGetExtrinsicFromXml(handle);
			}
			LidarStartSampling(handle, OnSampleCallback, nullptr);
			devices[handle].device_state = kDeviceStateSampling;
		}
	}
}

/** Callback function when broadcast message received.
* You need to add listening device broadcast code and set the point cloud data callback in this function.
*/
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
	if (info == nullptr || info->dev_type == kDeviceTypeHub) {
		return;
	}

	//printf("Receive Broadcast Code %s\n", info->broadcast_code);
	if ((broadcast_code_rev.size() == 0) ||
		(std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info->broadcast_code) == broadcast_code_rev.end())) {
		broadcast_code_rev.push_back(info->broadcast_code);
		lidar_arrive_condition.notify_one();
	}
}

/** Wait until no new device arriving in 2 second. */
void WaitForDevicesReady() {
	bool device_ready = false;
	seconds wait_time = seconds(2);
	steady_clock::time_point last_time = steady_clock::now();
	while (!device_ready) {
		std::unique_lock<std::mutex> lock(mtx);
		lidar_arrive_condition.wait_for(lock, wait_time);
		if ((steady_clock::now() - last_time + milliseconds(50)) >= wait_time) {
			device_ready = true;
		}
		else {
			last_time = steady_clock::now();
		}
	}
}
//extrinsic parameter wait
void WaitForExtrinsicParameter() {
	std::unique_lock<std::mutex> lock(mtx);
	extrinsic_condition.wait(lock);
}

void AddDevicesToConnect() {
	if (broadcast_code_rev.size() == 0)
		return;

	for (int i = 0; i < broadcast_code_rev.size(); ++i) {
		if ((broadcast_code_list.size() != 0) &&
			(std::find(broadcast_code_list.begin(), broadcast_code_list.end(), broadcast_code_rev[i]) == broadcast_code_list.end())) {
			continue;
		}
		uint8_t handle = 0;
		bool result = AddLidarToConnect(broadcast_code_rev[i].c_str(), &handle);
		if (result == kStatusSuccess) {
			/** Set the point cloud data for a specific Livox LiDAR. */
			SetDataCallback(handle, GetLidarData, nullptr);//���ݻ�ȡ�ص�����
			devices[handle].handle = handle;
			devices[handle].device_state = kDeviceStateDisconnect;
			g_nConnectedLidarCount++;
		}
	}
}

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
int SetProgramOption(int argc, const char *argv[]) {
	apr_status_t rv;
	apr_pool_t *mp = nullptr;
	static const apr_getopt_option_t opt_option[] = {
		/** Long-option, short-option, has-arg flag, description */
		{ "code", 'c', 1, "Register device broadcast code" },
		{ "log", 'l', 0, "Save the log file" },
		{ "time", 't', 1, "Time to save point cloud to the lvx file" },
		{ "param", 'p', 0, "Get the extrinsic parameter from extrinsic.xml file" },
		{ "help", 'h', 0, "Show help" },
		{ nullptr, 0, 0, nullptr },
	};
	apr_getopt_t *opt = nullptr;
	int optch = 0;
	const char *optarg = nullptr;

	if (apr_initialize() != APR_SUCCESS) {
		return -1;
	}

	if (apr_pool_create(&mp, NULL) != APR_SUCCESS) {
		return -1;
	}

	rv = apr_getopt_init(&opt, mp, argc, argv);
	if (rv != APR_SUCCESS) {
		//printf("Program options initialization failed.\n");
		return -1;
	}

	/** Parse the all options based on opt_option[] */
	bool is_help = false;
	while ((rv = apr_getopt_long(opt, opt_option, &optch, &optarg)) == APR_SUCCESS) {
		switch (optch) {
		case 'c': {
			//printf("Register broadcast code: %s\n", optarg);
			char *sn_list = (char *)malloc(sizeof(char)*(strlen(optarg) + 1));
			strncpy(sn_list, optarg, sizeof(char)*(strlen(optarg) + 1));
			char *sn_list_head = sn_list;
			sn_list = strtok(sn_list, "&");
			int i = 0;
			broadcast_code_list.clear();//��չ㲥���
			while (sn_list) {
				broadcast_code_list.push_back(sn_list);
				sn_list = strtok(nullptr, "&");
				i++;
			}
			free(sn_list_head);
			sn_list_head = nullptr;
			break;
		}
		case 'l': {
			//printf("Save the log file.\n");
			SaveLoggerFile();
			break;
		}
		case 't': {
			//printf("Time to save point cloud to the lvx file:%s.\n", optarg);
			//g_nLvxFileSaveTime = atoi(optarg);
			break;
		}
		case 'p': {
			//printf("Get the extrinsic parameter from extrinsic.xml file.\n");
			g_bReadExtrinsicFromXML = true;
			break;
		}
		case 'h': {
			/*printf(
			" [-c] Register device broadcast code\n"
			" [-l] Save the log file\n"
			" [-t] Time to save point cloud to the lvx file\n"
			" [-p] Get the extrinsic parameter from extrinsic.xml file\n"
			" [-h] Show help\n"
			);*/
			is_help = true;
			break;
		}
		}
	}
	if (rv != APR_EOF) {
		//printf("Invalid options.\n");
	}

	apr_pool_destroy(mp);
	mp = nullptr;
	if (is_help)
		return 1;
	return 0;
}

//ʮ����תΪ������ decimal to binary
int DecToBin(uint8_t nDec)
{
	int nBin = 0, nDecTemp = nDec, j = 1;
	while (nDecTemp) {
		nBin = nBin + j * (nDecTemp % 2);
		nDecTemp = nDecTemp / 2;
		j = j * 10;
	}
	return nBin;
}
//4��uint8_t��ʽ����ת��Ϊint
int FourDecsToBin(uint8_t nDec1, uint8_t nDec2, uint8_t nDec3, uint8_t nDec4)
{
	int nBin1 = DecToBin(nDec1);
	int nBin2 = DecToBin(nDec2);
	int nBin3 = DecToBin(nDec3);
	int nBin4 = DecToBin(nDec4);
	char* strResultBin = new char[33];
	sprintf(strResultBin, "%.8d%.8d%.8d%.8d", nBin4, nBin3, nBin2, nBin1);
	int i = 0, n = 0, m = 1, nResultDec = 0;
	while (strResultBin[i++])
		n++; //ͳ��ʵ���ж���λ
	for (i = 0; i < n; i++)
	{
		nResultDec = nResultDec + (strResultBin[n - i - 1] - '0')*m;
		//�����ַ���'0'�õ���Ӧ��ֵ����λ�ȼ�
		m *= 2;  //��λȨ�ǵ�λ��2��������һλȨ��2
	}
	delete[] strResultBin;
	strResultBin = NULL;
	return  nResultDec;
}
//8��uint8_tת��Ϊint
uint64_t EightDecsToBin(uint8_t nDecs[8])
{
	int nBins[8];
	for (int i = 0; i < 8; i++)
		nBins[i] = DecToBin(nDecs[i]);
	char* strResultBin = new char[65];
	sprintf(strResultBin, "%.8d%.8d%.8d%.8d%.8d%.8d%.8d%.8d", nBins[7], nBins[6], nBins[5], nBins[4], nBins[3], nBins[2], nBins[1], nBins[0]);
	int i = 0, n = 0;
	long long m = 1;
	uint64_t nResultDec = 0;
	while (strResultBin[i++])
		n++; //ͳ��ʵ���ж���λ
	for (i = 0; i < n; i++)
	{
		nResultDec = nResultDec + (strResultBin[n - i - 1] - '0')*m;
		m *= 2;
	}
	delete[] strResultBin;
	strResultBin = NULL;
	return  nResultDec;
}
//���ݵ�ǰϵͳʱ���ȡ�ļ���
void GetFileName(char* strFullFileName)
{
	time_t curtime = time(nullptr);
	char strFileNameTemp[80] = { 0 };
	tm* local_time = localtime(&curtime);
	char *strTxtFolderName = new char[10];
	strcpy(strTxtFolderName, "txtFiles");
	if (g_strSavePath == "")
	{
		// �ļ��в������򴴽��ļ���
		if (_access(strTxtFolderName, 0) == -1)
			_mkdir(strTxtFolderName);
		strftime(strFileNameTemp, sizeof(strFileNameTemp), "txtFiles\\%Y-%m-%d_%H-%M-%S.txt", local_time);
		strcpy(strFullFileName, strFileNameTemp);
	}
	else
	{
		strftime(strFileNameTemp, sizeof(strFileNameTemp), "txtFiles\\%Y-%m-%d_%H-%M-%S.txt", local_time);
		USES_CONVERSION;
		char *strFullFileNameTemp = new char[strlen(g_strSavePath) + strlen(strFileNameTemp) + 3];
		strcpy(strFullFileNameTemp, g_strSavePath);
		if (strlen(g_strSavePath) > 3)
			strcat(strFullFileNameTemp, "\\");
		strcat(strFullFileNameTemp, strFileNameTemp);
		strcpy(strFullFileName, strFullFileNameTemp);
		//create "\\txt" folder
		char* strFullFilePath = new char[strlen(strTxtFolderName) + strlen(g_strSavePath) + 3];
		strcpy(strFullFilePath, g_strSavePath);
		if (strlen(g_strSavePath) > 3)
			strcat(strFullFilePath, "\\");
		strcat(strFullFilePath, strTxtFolderName);
		// �ļ��в������򴴽��ļ���
		if (_access(strFullFilePath, 0) == -1)
			_mkdir(strFullFilePath);
		delete[] strFullFilePath;
		strFullFilePath = NULL;
		delete[] strFullFileNameTemp;
		strFullFileNameTemp = NULL;
		delete[] strTxtFolderName;
		strTxtFolderName = NULL;
	}
}
//���ݵ�ǰϵͳʱ���ȡ��ʱ�ļ���
void GetRealTimeFileName(char* strFileTempName)
{
	time_t curtime = time(nullptr);
	char strFileNameTemp[80] = { 0 };
	tm* local_time = localtime(&curtime);
	char *strTxtFolderName = new char[8];
	strcpy(strTxtFolderName, "temp");
	// �ļ��в������򴴽��ļ���
	if (_access(strTxtFolderName, 0) == -1)
		_mkdir(strTxtFolderName);
	strftime(strFileNameTemp, sizeof(strFileNameTemp), "temp\\%Y-%m-%d_%H-%M-%S.txt", local_time);
	string strFilenmTemp = strFileNameTemp;
	string strShortName = strFilenmTemp.substr(23, 5);
	char* strShortPathName = new char[15];
	strcpy(strShortPathName, "temp\\");
	strcat(strShortPathName, (char*)strShortName.c_str());
	strcpy(strFileTempName, strShortPathName);
	delete[] strShortPathName;
	strShortPathName = NULL;
	delete[] strTxtFolderName;
	strTxtFolderName = NULL;
}
//�ж�����ֵ�Ƿ�����
bool IsDataCorrect(int x, int y, int z, int ref)
{
	if (abs(x) < 100000 && abs(x) > 800 && abs(y) < 100000 && z > -20000 && z < 60000 && ref >= 5 && ref < 256)
		return true;
	else
		return false;
}
//���ݸ����ļ��� �����ݰ�����ά�����������д��txt�ļ�
void WriteFile(int nPointData[100][4], uint64_t nTimeStamp, char *strFilenm)
{
	char* strFileNameTemp = new char[80];
	strcpy(strFileNameTemp, strFilenm);
	FILE* fOutput = fopen(strFileNameTemp, "a");//�ļ�ָ�룬add��ʽ
	if (fOutput == NULL)
		return;
	for (int i = 0; i < 100; i++)
	{
		if (IsDataCorrect(nPointData[i][0], nPointData[i][1], nPointData[i][2], nPointData[i][3]) == 1)//�����ڹ涨��Χ��
		{
			std::unique_lock<std::mutex> lock(mtxLastFrameFile);
			for (int j = 0; j < 3; j++)
				fprintf(fOutput, "%.3f,", nPointData[i][j] / 1000.0);
			fprintf(fOutput, "%d,", nPointData[i][3]);
			fprintf(fOutput, "%lld,\n", nTimeStamp);
		}
	}
	fclose(fOutput);
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}

void WriteFileArray(int nPointData[100][4], uint64_t nTimeStamp, char *strFilenm)
{
	char* strWriteFileNameTemp = new char[80];
	strcpy(strWriteFileNameTemp, strFilenm);
	char* strFileNameArray[10] = { "0.txt","1.txt","2.txt","3.txt","4.txt","5.txt","6.txt","7.txt","8.txt","9.txt" };
	string strFilenmTemp = strFilenm;
	string strShortName = strFilenmTemp.substr(23, 5);
	char* strShortPathName = new char[15];
	strcpy(strShortPathName, "temp\\");
	strcat(strShortPathName, (char*)strShortName.c_str());
	int i = 0;
	for (int it = 0; it < 10; it++)
	{
		if (strcmp(strShortPathName, strFileNameArray[it]) == 0)
			i = it;
	}
	FILE* fOutput = fopen(strFileNameArray[i], "a");//�ļ�ָ�룬addд��
	if (fOutput == NULL)
		return;
	for (int i = 0; i < 100; i++)
	{
		if (IsDataCorrect(nPointData[i][0], nPointData[i][1], nPointData[i][2], nPointData[i][3]) == 1)
		{
			std::unique_lock<std::mutex> lock(mtxLastFrameFile);
			for (int j = 0; j < 4; j++)
			{
				fprintf(fOutput, "%d", nPointData[i][j]);
				fputc(',', fOutput);
			}
			fprintf(fOutput, "%lld", nTimeStamp);
			fputc(',', fOutput);
			fprintf(fOutput, "\n");
		}
	}
	fclose(fOutput);
	delete[] strWriteFileNameTemp;
	strWriteFileNameTemp = NULL;
}
//���ݸ����ļ��� �����ݰ�����ά�����������д��txt�ļ�
void WriteFileRealTime(int nPointData[100][4], char *strFilenm)
{
	FILE* fOutput = fopen(strFilenm, "a");//�ļ�ָ�룬add��ʽ
	if (fOutput == NULL)
		return;
	for (int i = 0; i < 100; i++)
	{
		if (IsDataCorrect(nPointData[i][0], nPointData[i][1], nPointData[i][2], nPointData[i][3]) == 1)//�����ڹ涨��Χ��
		{
			std::unique_lock<std::mutex> lock(mtxRealTimeFile);
			for (int j = 0; j < 3; j++)
				fprintf(fOutput, "%.3f,", nPointData[i][j] / 1000.0);
			fprintf(fOutput, "%d,\n", nPointData[i][3]);
		}
	}
	fclose(fOutput);
}
//ÿ֡��������д��txt�ļ�
void WriteTXTFile(std::list<LvxBasePackDetail> point_packet_list_to_save, char* strFileName)
{
	char* strFileNameTemp = new char[80];
	strcpy(strFileNameTemp, strFileName);
	list<LvxBasePackDetail>::iterator itrPointPack;
	itrPointPack = point_packet_list_to_save.begin();//��point packet list temp���������point packet
	for (itrPointPack = point_packet_list_to_save.begin(); itrPointPack != point_packet_list_to_save.end(); itrPointPack++)
	{
		int nRawPoint[100][13] = { 0 };//�����λ����
		uint8_t nRawTimeStamp[8] = { 0 };//����ʱ���
		for (int nFlag = 0; nFlag < 1300; nFlag++)//100����*13byte�ռ�
			nRawPoint[nFlag / 13][nFlag % 13] = itrPointPack->raw_point[nFlag];//100�����raw_point����д��RawPoint��ά����
		int nCookedPoint[100][4] = { 0 };
		for (int i = 0; i < 100; i++)//uint32_t���͵�rawpointתint����
		{
			nCookedPoint[i][0] = FourDecsToBin(nRawPoint[i][0], nRawPoint[i][1], nRawPoint[i][2], nRawPoint[i][3]);
			nCookedPoint[i][1] = FourDecsToBin(nRawPoint[i][4], nRawPoint[i][5], nRawPoint[i][6], nRawPoint[i][7]);
			nCookedPoint[i][2] = FourDecsToBin(nRawPoint[i][8], nRawPoint[i][9], nRawPoint[i][10], nRawPoint[i][11]);
			nCookedPoint[i][3] = nRawPoint[i][12];
		}
		for (int i = 0; i < 8; i++)
			nRawTimeStamp[i] = itrPointPack->timestamp[i];
		uint64_t nCookedTimeStamp = 0;
		nCookedTimeStamp = EightDecsToBin(nRawTimeStamp);
		WriteFile(nCookedPoint, nCookedTimeStamp, strFileNameTemp);
	}
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}
//ÿ֡��������д��txt�ļ�,��ʱ���
void WriteTXTFileRealTime(std::list<LvxBasePackDetail> point_packet_list_to_save, char* strFileName)
{
	char* strFileNameTemp = new char[80];
	strcpy(strFileNameTemp, strFileName);
	list<LvxBasePackDetail>::iterator itrPointPack;
	itrPointPack = point_packet_list_to_save.begin();//��point packet list temp���������point packet
	for (itrPointPack = point_packet_list_to_save.begin(); itrPointPack != point_packet_list_to_save.end(); itrPointPack++)
	{
		int nRawPoint[100][13] = { 0 };//�����λ����
		for (int nFlag = 0; nFlag < 1300; nFlag++)//100����*13byte�ռ�
			nRawPoint[nFlag / 13][nFlag % 13] = itrPointPack->raw_point[nFlag];//100�����raw_point����д��RawPoint��ά����
		int nCookedPoint[100][4] = { 0 };
		for (int i = 0; i < 100; i++)//uint32_t���͵�rawpointתint����
		{
			nCookedPoint[i][0] = FourDecsToBin(nRawPoint[i][0], nRawPoint[i][1], nRawPoint[i][2], nRawPoint[i][3]);
			nCookedPoint[i][1] = FourDecsToBin(nRawPoint[i][4], nRawPoint[i][5], nRawPoint[i][6], nRawPoint[i][7]);
			nCookedPoint[i][2] = FourDecsToBin(nRawPoint[i][8], nRawPoint[i][9], nRawPoint[i][10], nRawPoint[i][11]);
			nCookedPoint[i][3] = nRawPoint[i][12];
		}
		WriteFileRealTime(nCookedPoint, strFileNameTemp);
	}
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}
void DeleteTempFileSoon(char* strFileToDelete)
{
	char* strFileToDeleteTemp = new char[80];
	strcpy(strFileToDeleteTemp, strFileToDelete);
	Sleep(1000);
	remove(strFileToDeleteTemp);
	cout << "remove:" << strFileToDeleteTemp << endl;
	delete[] strFileToDeleteTemp;
	strFileToDeleteTemp = NULL;
}
//osg��ʾʵʱ�����ļ�
void OSGShowPointFileRealTime(char* strFileName, HWND hWnd)
{
	char* strFileNameTemp = new char[80];
	strcpy(strFileNameTemp, strFileName);
	Sleep(1000);
	{
		//��������
		ref_ptr<osg::Vec3Array> coords = new Vec3Array();
		ref_ptr<osg::Vec4Array> color = new Vec4Array();
		//��ȡ�ļ�

		FILE* pfData;
		fopen_s(&pfData, strFileNameTemp, "r");
		int num = 0;//���ݵ�����
		if (pfData == NULL)
		{
			cout << "File does not exist." << endl;
			return;
		}
		else
		{
			cout << "show:" << strFileNameTemp << endl;
			char strLine[50];
			int fx = 1000, fy = 0, fz = 0, fref = 0;
			//uint64_t timestp = 0;
			char *ptr = NULL;
			char *p = NULL;
			char strs[] = " ";
			int result_r(0), result_g(0), result_b(0);
			while (!feof(pfData) && num < 300000)
			{

				fgets(strLine, 50, pfData);
				int DataIntegrity = 0;//������������

				if (strcmp(strLine, strs) != 0)
				{
					ptr = strtok_s(strLine, ",", &p);
					if (ptr != NULL)
					{
						fx = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fy = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fz = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fref = atoi(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					ptr = strtok_s(NULL, "\n", &p);
					if (ptr != NULL)
						DataIntegrity++;

				}
				if (DataIntegrity == 4 && IsDataCorrect(fx, fy, fz, fref) == 1)
				{
					coords->push_back(osg::Vec3(fx, fy, fz));
					if (fref < 30)
						color->push_back(osg::Vec4(0, fref / 30.0, 255, 1));
					else if (fref < 90)
						color->push_back(osg::Vec4(0, 255, (90.0 - fref) / 60.0, 1));
					else if (fref < 150)
						color->push_back(osg::Vec4((fref - 90) / 60.0, 255, 0, 1));
					else
						color->push_back(osg::Vec4(255, (255 - fref) / 100.0, 0, 1));
					num++;
				}
				DataIntegrity = 0;
			}
			delete ptr;
			fclose(pfData);
		}
		thread tDeleteFile(DeleteTempFileSoon, strFileNameTemp);
		tDeleteFile.detach();
		//����������
		ref_ptr<Geometry> geometry = new Geometry();
		//���ö�������
		geometry->setVertexArray(coords.get());
		geometry->setColorArray(color.get());
		geometry->setColorBinding(Geometry::BIND_PER_VERTEX);
		Vec3Array *normals = new Vec3Array;
		normals->push_back(Vec3(0.0f, 1.0f, 0.0f));
		geometry->setNormalArray(normals);
		geometry->setNormalBinding(Geometry::BIND_OVERALL);
		geometry->addPrimitiveSet(new DrawArrays(PrimitiveSet::POINTS, 0, num, 0)); //���ù�����ʽ
		osg::StateSet* stateSet = rootRealTime->getOrCreateStateSet();
		osg::Point* pointSize = new osg::Point;
		pointSize->setSize(2.0);
		stateSet->setAttribute(pointSize);
		ref_ptr<Geode> geode = new Geode();
		geode->addDrawable(geometry.get());//����ͼҶ�ڵ�
		CLivoxFileSaveDlg* pWndDlg = gLivoxFileSaveDlg;
		if (pWndDlg != nullptr)
		{
			if (rootRealTime->getNumParents() == 0U)
			{
				rootRealTime->addChild(geode.get());
				pWndDlg->m_spViewer3DRealTime->AddNode(rootRealTime.get());
			}
			else
			{
				if (rootRealTime->getNumChildren() > (pWndDlg->m_nSamplingDuration - 1U > 12U ? 12U : pWndDlg->m_nSamplingDuration - 1U))
					pWndDlg->m_spViewer3DRealTime->RemoveChild(rootRealTime.get(), 0U, 1U);
				while (rootRealTime->getNumChildren() > (pWndDlg->m_nSamplingDuration - 1U > 12U ? 12U : pWndDlg->m_nSamplingDuration - 1U))
					pWndDlg->m_spViewer3DRealTime->RemoveChild(rootRealTime.get(), 0U, 1U);
				pWndDlg->m_spViewer3DRealTime->AddNode(geode.get(), rootRealTime.get());
				cout << rootRealTime->getNumChildren() << " children." << endl;
			}
		}
		delete[] strFileNameTemp;
		strFileNameTemp = NULL;
	}
	return;
}
//osg��ʾ��һ֡�����ļ�
void OSGShowPointFileLastFrame(char* strFileName, HWND hWnd)
{
	//��ȡ�ļ�
	char* strFileNameTemp = new char[80];
	strcpy_s(strFileNameTemp, 80, strFileName);
	FILE* pfData;
	fopen_s(&pfData, strFileNameTemp, "r");
	int nPointNum = 0;//���ݵ�����
	//����������	
	ref_ptr<Geode> geode = new Geode();
	if (pfData == NULL)
	{
		cout << "File does not exist." << endl;
		return;
	}
	else
	{
		cout << "show:" << strFileNameTemp << endl;
		char strLine[50];
		int fx = 1000, fy = 0, fz = 0, fref = 0;
		char *ptr = NULL;
		char *p = NULL;
		char strs[] = " ";
		int result_r(0), result_g(0), result_b(0);
		for (int i = 0; i < 4; i++)
		{
			nPointNum = 0;
			//��������
			ref_ptr<osg::Vec3Array> coords = new Vec3Array();
			ref_ptr<osg::Vec4Array> color = new Vec4Array();
			while (!feof(pfData) && nPointNum < 300000)
			{
				fgets(strLine, 50, pfData);
				int DataIntegrity = 0;//������������
				if (strcmp(strLine, strs) != 0)
				{
					ptr = strtok_s(strLine, ",", &p);
					if (ptr != NULL)
					{
						fx = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fy = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fz = 1000 * atof(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					if (ptr != NULL)
					{
						fref = atoi(ptr);
						ptr = strtok_s(NULL, ",", &p);
						DataIntegrity++;
					}
					ptr = strtok_s(NULL, "\n", &p);
					if (ptr != NULL)
						DataIntegrity++;
				}
				if (DataIntegrity == 4 && IsDataCorrect(fx, fy, fz, fref) == 1)
				{
					coords->push_back(osg::Vec3(fx, fy, fz));
					if (fref < 30)
						color->push_back(osg::Vec4(0, fref / 30.0, 255, 1));
					else if (fref < 90)
						color->push_back(osg::Vec4(0, 255, (90.0 - fref) / 60.0, 1));
					else if (fref < 150)
						color->push_back(osg::Vec4((fref - 90) / 60.0, 255, 0, 1));
					else
						color->push_back(osg::Vec4(255, (255 - fref) / 100.0, 0, 1));
					nPointNum++;
				}
				DataIntegrity = 0;
			}
			ref_ptr<Geometry> geometry = new Geometry();
			geometry->setVertexArray(coords.get());
			geometry->setColorArray(color.get());
			geometry->setColorBinding(Geometry::BIND_PER_VERTEX);
			Vec3Array *normals = new Vec3Array;//����
			normals->push_back(Vec3(0.0f, 1.0f, 0.0f));
			geometry->setNormalArray(normals);
			geometry->setNormalBinding(Geometry::BIND_OVERALL);
			geometry->addPrimitiveSet(new DrawArrays(PrimitiveSet::POINTS, 0, nPointNum)); //���ù�����ʽ
			geode->addDrawable(geometry.get());//drawableҶ�ڵ�
		}
		delete ptr;
		fclose(pfData);
	}
	osg::StateSet* stateSet = rootLastFrame->getOrCreateStateSet();
	osg::Point* pointSize = new osg::Point;
	pointSize->setSize(1.8);
	stateSet->setAttribute(pointSize);
	CLivoxFileSaveDlg* pWndDlg = gLivoxFileSaveDlg;
	if (pWndDlg != nullptr)
	{
		if (rootLastFrame->getNumParents() == 0U)
		{
			rootLastFrame->addChild(geode.get());
			pWndDlg->m_spViewer3DLastFrame->AddNode(rootLastFrame.get());
		}
		else
		{
			pWndDlg->m_spViewer3DLastFrame->RemoveChild(rootLastFrame.get(), 0U, 1U);
			pWndDlg->m_spViewer3DLastFrame->AddNode(geode.get(), rootLastFrame.get());
			cout << rootLastFrame->getNumChildren() << endl;
		}
	}
	pWndDlg->GetDlgItem(IDC_BTN_START_SAMPLING)->EnableWindow(TRUE);
	pWndDlg->OnBnClickedBtnRealTimeData();//���������ʼ��ʾԤ��ͼ
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
	return;
}
//��ʾʵʱ����
void ShowRealTimePoint()
{
	HWND hWndDlg = ::FindWindow(NULL, _T("LivoxFileSave"));//��ȡ���ھ��
	CLivoxFileSaveDlg* pWndDlg = (CLivoxFileSaveDlg*)CLivoxFileSaveDlg::FromHandle(hWndDlg);
	CEdit* editRealTime = (CEdit*)pWndDlg->GetDlgItem(IDC_PCT_REAL_TIME_DATA);
	HWND hWndEditRealTime = editRealTime->m_hWnd;//editRealTime��nullptr

	char* strFileNameTemp = new char[80];
	GetRealTimeFileName(strFileNameTemp);
	thread temp(OSGShowPointFileRealTime, strFileNameTemp, hWndEditRealTime);
	temp.detach();
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}
//��ʾ��һ֡����
void ShowLastFramePoint(char* strLastFrameFileName)
{
	char* strFileNameTemp = new char[80];
	strcpy(strFileNameTemp, strLastFrameFileName);
	HWND hWndDlg = ::FindWindow(NULL, _T("LivoxFileSave"));//��ȡ���ھ��
	CLivoxFileSaveDlg* pWndDlg = (CLivoxFileSaveDlg*)CLivoxFileSaveDlg::FromHandle(hWndDlg);
	CEdit* editLastFrame = (CEdit*)pWndDlg->GetDlgItem(IDC_PCT_LAST_FRAME_DATA);
	HWND hWndEditLastFrame;
	hWndEditLastFrame = editLastFrame->m_hWnd;
	thread temp(OSGShowPointFileLastFrame, strFileNameTemp, hWndEditLastFrame);
	temp.detach();
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}
//�����ʱ�ļ���
void ClearTempFolder()
{
	char *chDoc = ".";
	char *strFileNameTemp = new char[80];
	WIN32_FIND_DATA FileData;//WIN32_FIND_DATA���ݽṹ����
	HANDLE hSearch;
	BOOL bFinished = FALSE;
	string strFullFolder = string("temp") + string("\\*.*");
	hSearch = FindFirstFile(strFullFolder.c_str(), &FileData);
	if (hSearch == INVALID_HANDLE_VALUE)
	{
		cout << "No files found!" << endl;
		return;
	}
	while (!bFinished)
	{
		if (FileData.cFileName[0] != chDoc[0])
		{
			strcpy(strFileNameTemp, FileData.cFileName);
			string addr = string("temp") + string("\\") + string(strFileNameTemp);
			DeleteFile(addr.c_str());
		}
		if (!FindNextFile(hSearch, &FileData))
		{
			if (GetLastError() == ERROR_NO_MORE_FILES)
				bFinished = TRUE;
			else
				return;
		}
	}
	FindClose(hSearch);
	delete[] strFileNameTemp;
	strFileNameTemp = NULL;
}

// CLivoxFileSaveDlg �Ի���
CLivoxFileSaveDlg::CLivoxFileSaveDlg(CWnd* pParent /*=NULL*/) : CDialogEx(IDD_LIVOXFILESAVE_DIALOG, pParent)
, m_strSavePath(_T(""))
, m_nSamplingDuration(4)

{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_spViewer3DRealTime = nullptr;
	m_spViewer3DLastFrame = nullptr;
	gLivoxFileSaveDlg = this;
}

void CLivoxFileSaveDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDT_SAVE_PATH, m_strSavePath);
	DDX_Text(pDX, IDC_EDT_SAMPLING_DURATION, m_nSamplingDuration);
}

BEGIN_MESSAGE_MAP(CLivoxFileSaveDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//	ON_BN_CLICKED(IDC_BTN_START_SAMPLING, &CLivoxFileSaveDlg::OnBnClickedBtnStartSampling)
	ON_BN_CLICKED(IDC_BTN_CHOOSE_SAVE_PATH, &CLivoxFileSaveDlg::OnBnClickedBtnChooseSavePath)
	ON_BN_CLICKED(IDC_BTN_SHOW_TEST_DATA, &CLivoxFileSaveDlg::OnBnClickedBtnShowTestData)
	ON_WM_SIZE()
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDCANCEL, &CLivoxFileSaveDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BTN_REAL_TIME_DATA, &CLivoxFileSaveDlg::OnBnClickedBtnRealTimeData)
	ON_BN_CLICKED(IDC_BTN_START_SAMPLING, &CLivoxFileSaveDlg::OnBnClickedBtnStartSampling)
	ON_STN_CLICKED(IDC_PCT_LAST_FRAME_DATA, &CLivoxFileSaveDlg::OnStnClickedPctLastFrameData)
	ON_STN_CLICKED(IDC_PCT_REAL_TIME_DATA, &CLivoxFileSaveDlg::OnStnClickedPctRealTimeData)
	ON_EN_CHANGE(IDC_EDT_SAMPLING_DURATION, &CLivoxFileSaveDlg::OnEnChangeEdtSamplingDuration)
END_MESSAGE_MAP()


// CLivoxFileSaveDlg ��Ϣ�������

BOOL CLivoxFileSaveDlg::OnInitDialog()
{
	// TODO: �ڴ���Ӷ���ĳ�ʼ������
	CDialogEx::OnInitDialog();
	//��ʼ�������Ӵ��������
	{
		CEdit* editRealTime = (CEdit*)GetDlgItem(IDC_PCT_REAL_TIME_DATA);
		m_spViewer3DRealTime = new CViewer3D();
		if (!m_spViewer3DRealTime->Create((size_t)editRealTime->GetSafeHwnd()))
		{
			m_spViewer3DRealTime = nullptr;
			return FALSE;
		}
		//�����������
		osg::Vec3d eye, center, up;
		eye = osg::Vec3d(-2000, 0, 0);
		center = osg::Vec3d(0, 0, 0);
		up = osg::Vec3d(0, 0, 5);
		//���������ø����
		m_spViewer3DRealTime->SetCamera(eye, center, up);
	}
	{
		CEdit* editLastFrame = (CEdit*)GetDlgItem(IDC_PCT_LAST_FRAME_DATA);
		m_spViewer3DLastFrame = new CViewer3D();
		if (!m_spViewer3DLastFrame->Create((size_t)editLastFrame->GetSafeHwnd()))
		{
			m_spViewer3DLastFrame = nullptr;
			return FALSE;
		}
		//�����������
		osg::Vec3d eye, center, up;
		eye = osg::Vec3d(-2000, 0, 0);
		center = osg::Vec3d(0, 0, 0);
		up = osg::Vec3d(0, 0, 5);
		//���������ø����
		m_spViewer3DLastFrame->SetCamera(eye, center, up);
	}
	// ���ô˶Ի����ͼ�ꡣ  ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��
	//��������̨
	AllocConsole();
	freopen("CONOUT$", "w+t", stdout);
	freopen("CONIN$", "r+t", stdin);
	CRect rcOldRect;
	GetClientRect(&rcOldRect); //ȡ�ͻ�����С   
	pOldDlgSize.x = rcOldRect.right - rcOldRect.left;
	pOldDlgSize.y = rcOldRect.bottom - rcOldRect.top;
	CRect rectPictureCtlLastFrame;
	GetDlgItem(IDC_PCT_LAST_FRAME_DATA)->GetWindowRect(&rectPictureCtlLastFrame);
	ScreenToClient(&rectPictureCtlLastFrame);
	GetDlgItem(IDC_PCT_LAST_FRAME_DATA)->MoveWindow(rectPictureCtlLastFrame.left, rectPictureCtlLastFrame.top, 400, 400, true);//�趨Picture Control�ؼ��ĳ�ʼ�ߴ� 
	CRect rectPictureCtlRealTime;
	GetDlgItem(IDC_PCT_REAL_TIME_DATA)->GetWindowRect(&rectPictureCtlRealTime);
	ScreenToClient(&rectPictureCtlRealTime);
	GetDlgItem(IDC_PCT_REAL_TIME_DATA)->MoveWindow(rectPictureCtlRealTime.left, rectPictureCtlRealTime.top, 400, 400, true);
	/** Initialize Livox-SDK. */
	if (!Init()) {
		m_strSavePath = "Lidar initialize failed.";
		UpdateData(FALSE);
		return 0;
	}
	printf("Livox SDK has been initialized.\n");

	LivoxSdkVersion _sdkversion;
	GetLivoxSdkVersion(&_sdkversion);
	memset(devices, 0, sizeof(devices));
	/** Set the callback function receiving broadcast message from Livox LiDAR. */
	SetBroadcastCallback(OnDeviceBroadcast);
	/** Set the callback function called when device state change,
	* which means connection/disconnection and changing of LiDAR state.
	*/
	SetDeviceStateUpdateCallback(OnDeviceInfoChange);
	/** Start the device discovering routine. */
	if (!Start()) {
		Uninit();
		return 0;
	}
	WaitForDevicesReady();
	AddDevicesToConnect();
	if (g_nConnectedLidarCount == 0) {
		printf("No device will be connected.\n");
		Uninit();
		m_strSavePath = "Lidar not found.";
		UpdateData(FALSE);
		return FALSE;
	}
	WaitForExtrinsicParameter();//��ȡ���
	m_strSavePath = "Lidar connected.";
	UpdateData(FALSE);
	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ  ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�
void CLivoxFileSaveDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������
		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
		CDialogEx::OnPaint();
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù����ʾ��
HCURSOR CLivoxFileSaveDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}
//֡�ɼ���ť��Ӧ�̻߳ص�����
static UINT StartSampling(LPVOID pParam)
{
	printf("Start initialize lvx file.\n");
	if (!lvx_file_handler.InitLvxFile()) {
		Uninit();
		return 0;
	}
	CLivoxFileSaveDlg* pWndDlg = gLivoxFileSaveDlg;
	int nLvxFileSaveTime;//�źŻ�ȡ����ʱ��(��)
	if (pWndDlg->m_nSamplingDuration <= 0)
		nLvxFileSaveTime = 4;
	else if (pWndDlg->m_nSamplingDuration > 100)
		nLvxFileSaveTime = 100;
	else
		nLvxFileSaveTime = pWndDlg->m_nSamplingDuration;
	point_packet_list.clear();//��յ�ǰ�������ݻ���
	lvx_file_handler.InitLvxFileHeader();
	char* strSaveFileName = new char[80];
	GetFileName(strSaveFileName);
	int i = 0;
	steady_clock::time_point last_time = steady_clock::now();
	for (i = 0; i < nLvxFileSaveTime*FRAME_RATE; ++i) {//20*����
		std::list<LvxBasePackDetail> point_packet_list_temp;//ÿһ֡�½�һ��point packet��list temp
		{
			std::unique_lock<std::mutex> lock(mtx);//��Χ������������
			point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
			last_time = steady_clock::now();
			point_packet_list_temp.swap(point_packet_list);
			thread tWriteFile(WriteTXTFile, point_packet_list_temp, strSaveFileName);//д��txt�ļ�
			tWriteFile.join();
		}
		if (point_packet_list_temp.empty()) {
			printf("Point cloud packet is empty.\n");
			pWndDlg->GetDlgItem(IDC_BTN_START_SAMPLING)->EnableWindow(TRUE);
			break;
		}
		//lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
	}
	printf("Finish save lvx file.\n");
	lvx_file_handler.CloseLvxFile();
	//��ʾ��һ֡����
	Sleep(200);
	ClearTempFolder();
	thread tShowLastFrame(ShowLastFramePoint, strSaveFileName);
	tShowLastFrame.join();
	delete[] strSaveFileName;
	strSaveFileName = NULL;
	return 0;
}
//֡�ɼ���ť��Ӧ
void CLivoxFileSaveDlg::OnBnClickedBtnStartSampling()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(TRUE);
	if (m_WinThreadRealTime != NULL&&m_WinThreadRealTime->m_nThreadID != 0)
		m_WinThreadRealTime->SuspendThread();
	ClearTempFolder();
	GetDlgItem(IDC_BTN_START_SAMPLING)->EnableWindow(FALSE);
	m_WinThreadLastFrame = AfxBeginThread(StartSampling, NULL, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
	return;
}
//ѡ���ļ�����·����ť��Ӧ
void CLivoxFileSaveDlg::OnBnClickedBtnChooseSavePath()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	TCHAR szFolderPath[MAX_PATH] = { 0 };
	BROWSEINFO sInfo;
	::ZeroMemory(&sInfo, sizeof(BROWSEINFO));
	sInfo.pidlRoot = 0;
	sInfo.lpszTitle = _T("ѡ�����洢·��");
	sInfo.ulFlags = BIF_RETURNONLYFSDIRS | BIF_EDITBOX | BIF_DONTGOBELOWDOMAIN;
	sInfo.lpfn = NULL;
	// ��ʾ�ļ���ѡ��Ի���
	LPITEMIDLIST lpidlBrowse = ::SHBrowseForFolder(&sInfo);
	if (lpidlBrowse != NULL)
	{
		// ȡ���ļ�����
		if (::SHGetPathFromIDList(lpidlBrowse, szFolderPath))
			g_strSavePath = szFolderPath;
	}
	if (lpidlBrowse != NULL)
		::CoTaskMemFree(lpidlBrowse);
	m_strSavePath = g_strSavePath;
	UpdateData(FALSE);
}
//��ʾʾ��ͼ��ť��Ӧ
void CLivoxFileSaveDlg::OnBnClickedBtnShowTestData()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	char* strTestFileName = new char[80];
	strcpy(strTestFileName, "E:\\livox\\match\\txtfiles\\2020-09-04_18-07-42.txt");
	thread tShowLastFrame(ShowLastFramePoint, strTestFileName);
	tShowLastFrame.join();
	strTestFileName = NULL;
}
//�ؼ���С��Ӧ���ڳߴ�
void CLivoxFileSaveDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);
	// TODO: �ڴ˴������Ϣ����������
	if (nType == SIZE_RESTORED || nType == SIZE_MAXIMIZED)//�����С�����䶯���������
	{
		float fZoomFactor[2];
		POINT pNewDlgSize; //��ȡ���ڶԻ���Ĵ�С
		CRect rcNewRect;
		GetClientRect(rcNewRect); //ȡ�ͻ�����С   
		pNewDlgSize.x = rcNewRect.right - rcNewRect.left;
		pNewDlgSize.y = rcNewRect.bottom - rcNewRect.top;
		fZoomFactor[0] = (float)pNewDlgSize.x / pOldDlgSize.x;
		fZoomFactor[1] = (float)pNewDlgSize.y / pOldDlgSize.y;
		CRect Rect;
		int nCtrlID;//�ؼ�ID
		CPoint pOldTLPoint, pNewTLPoint; //���Ͻ�
		CPoint pOldBRPoint, pNewBRPoint; //���½�
		HWND hwndChild = ::GetWindow(m_hWnd, GW_CHILD); //�г����пؼ�   
		while (hwndChild)
		{
			nCtrlID = ::GetDlgCtrlID(hwndChild);//ȡ��ID
			GetDlgItem(nCtrlID)->GetWindowRect(Rect);
			ScreenToClient(Rect);
			pOldTLPoint = Rect.TopLeft();
			pNewTLPoint.x = long(pOldTLPoint.x*fZoomFactor[0]);
			pNewTLPoint.y = long(pOldTLPoint.y*fZoomFactor[1]);
			pOldBRPoint = Rect.BottomRight();
			pNewBRPoint.x = long(pOldBRPoint.x *fZoomFactor[0]);
			pNewBRPoint.y = long(pOldBRPoint.y *fZoomFactor[1]); //�߶Ȳ��ɶ��Ŀؼ���combBox),���ı��ֵ
			Rect.SetRect(pNewTLPoint, pNewBRPoint);
			GetDlgItem(nCtrlID)->MoveWindow(Rect, TRUE);
			hwndChild = ::GetWindow(hwndChild, GW_HWNDNEXT);
		}
		pOldDlgSize = pNewDlgSize;
	}
}

int CLivoxFileSaveDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialogEx::OnCreate(lpCreateStruct) == -1)
		return -1;
	// TODO:  �ڴ������ר�õĴ�������
	return 0;
}

void CLivoxFileSaveDlg::OnDestroy()
{
	if (m_spViewer3DRealTime.valid())
		m_spViewer3DRealTime->Destroy();
	m_spViewer3DRealTime = nullptr;
	if (m_spViewer3DLastFrame.valid())
		m_spViewer3DLastFrame->Destroy();
	m_spViewer3DLastFrame = nullptr;
	CDialogEx::OnDestroy();
}

void CLivoxFileSaveDlg::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	ClearTempFolder();
	Uninit();
	if (m_WinThreadRealTime != NULL && m_WinThreadRealTime->m_nThreadID != 0)
		m_WinThreadRealTime->SuspendThread();
	if (m_WinThreadLastFrame != NULL && m_WinThreadLastFrame->m_nThreadID != 0)
		m_WinThreadLastFrame->SuspendThread();
	CDialogEx::OnCancel();
	OnDestroy();
	exit(0);
}
//ʵʱ�����̻߳ص�����
static UINT ShowRealTimeData(LPVOID lpParam)
{
	ClearTempFolder();//��ջ����ļ���
	point_packet_list.clear();//��յ�ǰ�������ݻ���
	int i = 0;
	char* strWriteRealTimeFileName = new char[80];
	steady_clock::time_point last_time = steady_clock::now();
	for (i = 0; i < 24000; ++i) {//20 minutes
		std::list<LvxBasePackDetail> point_packet_list_temp;//ÿһ֡�½�һ��point packet��list temp
		{
			std::unique_lock<std::mutex> lock(mtx);//��Χ������������
			point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
			last_time = steady_clock::now();
			point_packet_list_temp.swap(point_packet_list);
			GetRealTimeFileName(strWriteRealTimeFileName);
			thread tWriteFile(WriteTXTFileRealTime, point_packet_list_temp, strWriteRealTimeFileName);//д��txt�ļ� 
			tWriteFile.join();
		}
		if (i % 20 == 0)
		{
			thread a(ShowRealTimePoint);
			a.join();
		}
		if (point_packet_list_temp.empty()) {
			printf("Point cloud packet is empty.\n");
			break;
		}
	}
	strWriteRealTimeFileName = NULL;
	return 0;
}
//����ʵʱ������ʾ
void CLivoxFileSaveDlg::OnBnClickedBtnRealTimeData()
{
	if (m_nBtnRealTimeClicked > 0)//�ڶ�ε��ʵʱ��ʾ��ť
	{
		DWORD dSusTime = m_WinThreadRealTime->SuspendThread();
		Sleep(10);
		if (dSusTime == -1)//�����Ƶȼ�
		{
			cout << "failed to suspend thread.\n";
			m_WinThreadRealTime = AfxBeginThread(ShowRealTimeData, NULL, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
		}
		else
		{
			for (unsigned int i = 0U; i <= dSusTime; i++)
				m_WinThreadRealTime->ResumeThread();
			GetDlgItem(IDC_BTN_START_SAMPLING)->EnableWindow(TRUE);
		}
	}
	else//���һ��ʵʱ��ʾ��ť
	{
		m_WinThreadRealTime = AfxBeginThread(ShowRealTimeData, NULL, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
		m_nBtnRealTimeClicked++;
	}
	return;
}

void CLivoxFileSaveDlg::OnStnClickedPctLastFrameData()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	GetDlgItem(IDC_PCT_LAST_FRAME_DATA)->SetFocus();
}

void CLivoxFileSaveDlg::OnStnClickedPctRealTimeData()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	GetDlgItem(IDC_PCT_REAL_TIME_DATA)->SetFocus();
}
//�޶�����ʱ���༭������Ϊ����
void CLivoxFileSaveDlg::OnEnChangeEdtSamplingDuration()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	CString strInputRule;
	GetDlgItem(IDC_EDT_SAMPLING_DURATION)->GetWindowText(strInputRule);
	int nStringLength = strInputRule.GetLength();;
	// ��������������������ַ�
	for (int nIndexm = 0; nIndexm < nStringLength; nIndexm++)
	{
		if ((strInputRule[nIndexm] > '9' || strInputRule[nIndexm] < '0'))
		{
			strInputRule = strInputRule.Left(nIndexm) + strInputRule.Right(strInputRule.GetLength() - nIndexm - 1);
			GetDlgItem(IDC_EDT_SAMPLING_DURATION)->SetWindowText(strInputRule);
			return;
		}
	}
	return;
}
