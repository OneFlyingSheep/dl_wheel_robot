#ifndef __URL_DEF_H__
#define __URL_DEF_H__

#include "json/json.h"
#include <map>

/*--------------------------------function register-----------------------------------*/
//风机箱
#define FJX_INFO_REQ							"/njnjqrxjapi/api/v1/fjx"
#define ALL_FJX_LATEST_INFO_REQ					"/njnjqrxjapi/api/v1/fjx/latestall"

//污水提升泵
#define WSTSB_INFO_REQ							"/njnjqrxjapi/api/v1/wstsb"
#define ALL_WSTSB_LATEST_INFO_REQ				"/njnjqrxjapi/api/v1/wstsb/latestall"

//巡检机器人
#define	ROBOT_BASE_INFO_REQ						"/njnjqrxjapi/api/v1/robot"
#define	ROBOT_ENV_INFO_REQ						"/njnjqrxjapi/api/v1/environment"
#define	ROBOT_STATUS_INFO_REQ                   "/njnjqrxjapi/api/v1/robot/status"
#define	ROBOT_POSE_INFO_REQ						"/njnjqrxjapi/api/v1/robot/pose"
#define	ROBOT_LATEST_STATUS_INFO_REQ			"/njnjqrxjapi/api/v1/robot/latest_status"
#define	ROBOT_PRESET_POSITION_INFO_REQ			"/njnjqrxjapi/api/v1/robot/preset_position"
#define	ROBOT_PLANNED_ROUTE_INFO_REQ			"/njnjqrxjapi/api/v1/robot/planned_route"

//摄像头
#define	CAMERA_PARAMETER_REQ					"/njnjqrxjapi/api/v1/camera"

//机组图片
#define	DEV_IMAGE_PATH_REQ						"/njnjqrxjapi/api/v1/device/image"
#define	DEV_UINT_IMAGE_PATH_REQ					"/njnjqrxjapi/api/v1/device/unit_image"

/*--------------------------------struct define-----------------------------------*/
typedef struct realTimeData
{
	std::string time;
	std::string data;
}RealTimeData;

//风机箱相关信息 --- 开始
typedef struct fjxFanStatus
{
    std::string status_desc;
	std::string	current_status;
	std::string time;
}FjxFanStatus;

typedef struct fjxInfo
{
	std::string		id;
	std::string		dev_name;
    std::vector<FjxFanStatus> fan_status_vec;
	std::vector<RealTimeData> ctrl_cab_temp_vec;
	std::vector<RealTimeData> cnds_pump_alert_vec;
	std::vector<RealTimeData> ph_volt_vec;
	std::vector<RealTimeData> l1_cur_vec;
	std::vector<RealTimeData> l2_cur_vec;
	std::vector<RealTimeData> l3_cur_vec;
}FjxInfo;
//风机箱相关信息 --- 结束

//污水泵相关信息 --- 开始
typedef struct alertData
{
	std::string time;
	std::string status;
	std::string content;
}AlertData;

typedef struct wsbInfo
{
	std::string		id;
	std::vector<AlertData> alerts_vec;
}WsbInfo;
//污水泵相关信息 --- 结束

//机器人本体信息 --- 开始
typedef struct robotBaseInfo
{
	std::string		id;
	std::string		battery;
    std::string     battery_life;
    std::string     vel_speed;
}RobotBaseInfo;
//机器人本体信息 --- 结束

//环境信息 --- 开始
typedef struct envBaseData
{
	std::string time;
	std::string data;
}EnvBaseData;

typedef struct extinPressureData
{
	std::string time;
	std::string extin_id;
	std::string data;
}ExtinPressureData;

typedef struct pipeLeakData
{
	std::string time;
	std::string leak;
}PipeLeakData;

typedef struct robotEnvInfo
{
	std::string		id;
	std::vector<EnvBaseData>        temp_vec;
	std::vector<EnvBaseData>        hum_vec;
	std::vector<EnvBaseData>        co2_vec;
	std::vector<EnvBaseData>        pm25_vec;
	std::vector<EnvBaseData>        pm10_vec;
	std::vector<EnvBaseData>        noise_vec;
	std::vector<EnvBaseData>        tvoc_vec;
	std::vector<EnvBaseData>        ch2o_vec;
	std::vector<ExtinPressureData>  extin_pressure_vec;
	std::vector<PipeLeakData>       pipe_leak_vec;
}RobotEnvInfo;
//环境信息 --- 结束

//业务信息 --- 开始
typedef struct patrolInfo
{
    std::string status;
    std::string dev_id;
    std::string dev_name;
    std::string time;
}PatrolInfo;

typedef struct statusInfo
{
	std::string		id;
    std::vector<PatrolInfo> list_vec;
}StatusInfo;
//业务信息 --- 结束

//机器人实时位置 --- 开始
typedef struct poseInfo
{
	std::string		id;
    std::string      x;
    std::string      y;
    std::string      z;
    std::string      orientation_x;
    std::string      orientation_y;
    std::string      orientation_z;
    std::string      orientation_w;
    std::string     time;
}PoseInfo;
//机器人实时位置 --- 结束

//最新巡检状态 --- 开始
typedef struct latestPatrolInfo
{
	std::string		id;
    std::string     status;
    std::string     device_id;
    std::string     device_name;
    std::string     time;
}LatestPatrolInfo;
//最新巡检状态 --- 结束

//预设巡检设备 --- 开始
typedef struct presetPosInfo
{
    std::string seq;
    std::string enabled;
    std::string device_id;
    std::string description;
}PresetPosInfo;

typedef struct presetPatrolDevInfo
{
	std::string		id;
    std::vector<PresetPosInfo> preset_pos_vec;
}PresetPatrolDevInfo;
//预设巡检设备 --- 结束

//规划轨迹 --- 开始
typedef struct basePlannedRoute
{
    std::string   x;
    std::string   y;
    std::string   seq;
    std::string   preset_seq;
}BasePlannedRoute;

typedef struct plannedRoute
{
    std::string robot_id;
    std::vector<BasePlannedRoute>   planned_route_vec;
}PlannedRoute;
//规划轨迹 --- 结束

//摄像机参数 --- 开始
typedef struct cameraInfo
{
    std::string   name;
    std::string   ip;
    int           port;
    std::string   user;
    std::string   password;
    std::string   channel;
    std::string   subtype;
    std::string   type;
}CameraInfo;

typedef struct robotCamerasInfo
{
    std::string robot_id;
    std::vector<CameraInfo>   camera_vec;
}RobotCamerasInfo;
//摄像机参数 --- 结束

//图片获取 --- 开始
typedef struct imageInfo
{
    std::string   image_path;
    std::string   time;
}ImageInfo;

typedef struct devImagePathInfo
{
    std::string   robot_id;
    std::string   image_path;
    std::string   time;
    // std::vector<ImageInfo>   image_vec;
}DevImagePathInfo;
//图片获取 --- 结束

//机组图片获取 --- 开始
typedef struct sigleDevImageInfo
{
    std::string   name;
    std::string   image_path;
    std::string   time;
}SigleDevImageInfo;

typedef struct devsImageInfo
{
    std::string robot_id;
    std::vector<SigleDevImageInfo>   dev_image_vec;
}DevsImagePathInfo;
//机组图片获取 --- 结束

/*----------------------------------------------json content pack--------------------------*/
class JsonContent
{
public:
    JsonContent(){}
    ~JsonContent(){}

    template<typename T>
    void jsonAppendElement(std::string name, T t)
    {
        m_jsVal[name.c_str()] = t;
    }

//    template<typename T>
    void jsonAppendElements(const std::string &key, std::vector<JsonContent> elements)
    {
        for(int i = 0; i < elements.size(); ++i)
        {
            m_jsVal[key.c_str()][i] = elements[i].m_jsVal;
        }
    }
    template<typename T>
    void jsonAppendVector(const std::string &key, std::vector<T> elements)
    {
        for(int i = 0; i < elements.size(); ++i)
        {
            m_jsVal[key.c_str()][i] = elements[i];
        }
    }
    void jsonAppendJson(const std::string &key, JsonContent json)
    {

        m_jsVal[key.c_str()] = json.m_jsVal;

    }

    bool fromJsonStringToJsonVal(const std::string &msg)
    {
        try
        {
            Json::Reader read;
            if (read.parse(msg.c_str(), m_jsVal))
            {
                return true;
            }
            else
            {
                return false;
            }

        } catch (const Json::Exception &err)
        {
            printf("json err: %s", err.what());
            return false;
        }
    }

    std::string getJsonString()
    {
        Json::FastWriter writer;
        return writer.write(m_jsVal);
    }

    Json::Value &getJsonVal()
    {
        return m_jsVal;
    }

    void clear()
    {
        m_jsVal.clear();
    }

public:
    Json::Value m_jsVal;

};


#endif // __URL_DEF_H__
