#ifndef _BIM_INTERFACE_PROTOCOL_H_
#define _BIM_INTERFACE_PROTOCOL_H_

#include <string>
#include <vector>

//风机箱
#define FJX_INFO_REQ							"njnjqrxj/api/v1/fjx"

//污水提升泵
#define WSTSB_INFO_REQ							"njnjqrxj/api/v1/wstsb"

//巡检机器人
#define	ROBOT_BASE_INFO_REQ						"njnjqrxj/api/v1/robot"
#define	ROBOT_ENV_INFO_REQ						"njnjqrxj/api/v1/environment"
#define	ROBOT_STATUS_INFO_REQ                   "njnjqrxj/api/v1/robot/status"
#define	ROBOT_POSE_INFO_REQ						"njnjqrxj/api/v1/robot/pose"
#define	ROBOT_LATEST_STATUS_INFO_REQ			"njnjqrxj/api/v1/robot/latest_status"
#define	ROBOT_PRESET_POSITION_INFO_REQ			"njnjqrxj/api/v1/robot/preset_position"
#define	ROBOT_PLANNED_ROUTE_INFO_REQ			"njnjqrxj/api/v1/robot/planned_route"

//摄像头
#define	CAMERA_PARAMETER_REQ					"njnjqrxj/api/v1/camera"

//机组图片
#define	DEV_IMAGE_PATH_REQ						"njnjqrxj/api/v1/device/image"
#define	DEV_UINT_IMAGE_PATH_REQ					"njnjqrxj/api/v1/device/unit_image"

typedef struct fjxFanStatus
{
	bool	current_status;
	std::string time;
}FjxFanStatus;

typedef struct realTimeData
{
	std::string time;
	std::string data;
}RealTimeData;

typedef struct fjxInfoStru
{
	std::string id;
	std::string dev_name;
	FjxFanStatus	bypass_status;
	FjxFanStatus	motor_loverload;
	FjxFanStatus	energy_saving;
	FjxFanStatus	energy_saving_mal;
	RealTimeData	ctrl_cab_temp;
	RealTimeData	cnds_pump_alert;
	RealTimeData	ph_volt;
	RealTimeData	l1_cur;
	RealTimeData	l2_cur;
	RealTimeData	l3_cur;
}FjxInfoStru;

class BimInterfaceProtocol
{
public:
	BimInterfaceProtocol(std::string ip = "192.168.1.12", int port = 8080);
	~BimInterfaceProtocol();

public:
	//风机箱
	std::string	fjx_info_query(std::string ip, int port, std::string device_id, std::string time_start, std::string time_end);

	//污水提升泵

	//巡检机器人

	//摄像头

	//机组图片

private:

};
#endif//_BIM_INTERFACE_PROTOCOL_H_
