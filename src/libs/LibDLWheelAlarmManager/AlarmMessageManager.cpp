#include "AlarmMessageManager.h"

AlarmMessageManager *AlarmMessageManager::m_pAlarmMsgManager = nullptr;
AlarmMessageManager *AlarmMessageManager::GetInstance()
{
    if (nullptr == m_pAlarmMsgManager)
    {
        m_pAlarmMsgManager = new AlarmMessageManager;
    }
    return m_pAlarmMsgManager;
}

void AlarmMessageManager::GetAlarmMessage(int iId, AlarmMessage &alarmMessage)
{
    std::map<int, AlarmMessage>::iterator it = m_mapID2AlarmMessage.find(iId);
    if (it != m_mapID2AlarmMessage.end())
    {
        alarmMessage.iID = it->second.iID;
        alarmMessage.strSpeakAlarmMessage = it->second.strSpeakAlarmMessage;
        alarmMessage.strDisplayAlarmMessage = it->second.strDisplayAlarmMessage;
        alarmMessage.strErrorCode = it->second.strErrorCode;
    }
}

AlarmMessageManager::~AlarmMessageManager()
{

}

void AlarmMessageManager::Init()
{

    //左前行走电机通讯异常 1101
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_COMMUNICATION, "左前轮系故障", "左前行走电机通讯异常");

    //左前行走电机断开连接 1102
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_POWER, "左前轮系故障", "左前行走电机断开连接");

    //左前行走电机状态异常 1103
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_STATUS, "左前轮系故障", "左前行走电机状态异常");

    //左前转向电机通讯异常 1104
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_COMMUNICATION, "左前轮系故障", "左前转向电机通讯异常");

    //左前转向电机断开连接 1105
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_POWER, "左前轮系故障", "左前转向电机断开连接");

    //左前转向电机状态异常 1106
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_STATUS, "左前轮系故障", "左前转向电机状态异常");

    //左后行走电机通讯异常  1201
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_COMMUNICATION, "左后轮系故障", "左后行走电机通讯异常");

    //左后行走电机断开连接  1202
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_POWER, "左后轮系故障", "左后行走电机断开连接");

    //左后行走电机状态异常 1203
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_STATUS, "左后轮系故障", "左后行走电机状态异常");

    //左后转向电机通讯异常 1204
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_COMMUNICATION, "左后轮系故障", "左后转向电机通讯异常");

    //左后转向电机断开连接 1205
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_POWER, "左后轮系故障", "左后转向电机断开连接");

    //左后转向电机状态异常 1206
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_STATUS, "左后轮系故障", "左后转向电机状态异常");

    //右前轮系故障
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_COMMUNICATION, "右前轮系故障", "右前行走电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_POWER, "右前轮系故障", "右前行走电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_STATUS, "右前轮系故障", "右前行走电机状态异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_COMMUNICATION, "右前轮系故障", "右前转向电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_POWER, "右前轮系故障", "右前转向电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_STATUS, "右前轮系故障", "右前转向电机状态异常");
    //右后轮系故障
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_COMMUNICATION, "右后轮系故障", "右后行走电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_POWER, "右后轮系故障", "右后行走电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_STATUS, "右后轮系故障", "右后行走电机状态异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_COMMUNICATION, "右后轮系故障", "右后转向电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_POWER, "右后轮系故障", "右后转向电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_STATUS, "右后轮系故障", "右后转向电机状态异常");
    // 前方探测到障碍物
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_SONAR, "前方探测到障碍物", "机器人左前方探测到障碍物");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_SONAR, "前方探测到障碍物", "机器人右前方探测到障碍物");
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_ANTI_DROP, "探测到台阶", "机器人左前方探测到台阶");
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_ANTI_DROP, "探测到台阶", "机器人左后方探测到台阶");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_ANTI_DROP, "探测到台阶", "机器人右前方探测到台阶");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_ANTI_DROP, "探测到台阶", "机器人右后方探测到台阶");
    InsertAlarmMessage(CLIENT_ALARM_LIDAR_COMMUNICATION, "激光雷达故障", "激光雷达通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_LIDAR_STATUS, "激光雷达故障", "激光雷达状态异常");
    // 控制器故障
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_BOARD_CONNECT, "控制器故障", "主控板断开连接");
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_BOARD_TEMPERATURE, "控制器温度过高", "控制器温度过高");
    InsertAlarmMessage(CLIENT_ALARM_ROBOT_COMMUNICATION, "机器人断开连接", "机器人断开连接");
    // 机器人电量过低
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_POWER, "机器人电量过低", "机器人电量过低");
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_VOLTAGE, "机器人电量过低", "机器人电压过低");
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_COMMUNICATION, "机器人电池异常", "机器人电池通讯异常");
    // 云台故障
    InsertAlarmMessage(CLIENT_ALARM_PTZ_CONNECT, "云台故障", "云台断开连接");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_COMMUNICATIION, "云台故障", "云台水平旋转电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_POWER, "云台故障", "云台水平旋转电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_STATUS, "云台故障", "云台水平旋转电机状态异常");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_COMMUNICATION, "云台故障", "云台俯仰电机通讯异常");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_POWER, "云台故障", "云台俯仰电机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_STATUS, "云台故障", "云台俯仰电机状态异常");
    // 可见光像机故障
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CONNECT, "可见光像机故障", "可见光相机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_SETFOCUS, "可见光像机故障", "可见光相机设置焦距或倍率失败");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_SETZOOM, "可见光像机故障", "可见光相机设置倍率失败");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CHECK_FOCUS_AND_ZOOM, "可见光像机故障", "可见光相机检查倍率和焦距失败");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CAPTURE, "可见光像机故障", "可见光相机拍照失败");
    // 红外相机故障
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_CONNECT, "红外相机故障", "红外相机断开连接");
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_FOCUS, "红外相机故障", "红外相机自动对焦失败");
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_CAPTURE, "红外相机故障", "红外相机拍照失败");
    // 地图信息丢失
    InsertAlarmMessage(CLIENT_ALARM_MAP_FILE_LOST, "地图信息丢失", "场地地图丢失");
    InsertAlarmMessage(CLIENT_ALARM_MAP_INFO_FILE_LOST, "地图信息丢失", "行走路线丢失");
    // 机器人丢失定位
    InsertAlarmMessage(CLIENT_ALARM_LOCATION_RELIABILITY, "机器人丢失定位", "机器人丢失定位");
    // 机器人偏离路线
    InsertAlarmMessage(CLIENT_ALARM_ROBOT_DEVIATE_PATH, "机器人偏离路线", "机器人偏离路线");

    //控制器 温度 异常
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_T_ANOMALY, "控制器 温度 异常", "控制器温度异常：");

    //控制器 湿度 异常
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_HUMIDTY_ANOMALY, "控制器 湿度 异常", "控制器湿度异常：");

	//	陌生人
	InsertAlarmMessage(CLIENT_CAP_STRANGER, "出现陌生人", "出现陌生人");

}

void AlarmMessageManager::InsertAlarmMessage(int iID, std::string strSpeakAlarmMessage, std::string strDisplayAlarmMessage, std::string strErrorCode)
{
    AlarmMessage stAlarmMessage;
    stAlarmMessage.iID = iID;
    stAlarmMessage.strSpeakAlarmMessage = strSpeakAlarmMessage;
    stAlarmMessage.strDisplayAlarmMessage = strDisplayAlarmMessage;
    stAlarmMessage.strErrorCode = strErrorCode;
    m_mapID2AlarmMessage.insert(std::make_pair(iID, stAlarmMessage));
}

AlarmMessageManager::AlarmMessageManager()
{
    Init();
}
