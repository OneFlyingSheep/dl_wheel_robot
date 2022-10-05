#ifndef __XML_PROTOCOL_MSG_H__
#define __XML_PROTOCOL_MSG_H__

#include <common/DLRobotCommonDef.h>
#include "tinyxml.h"
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>

//CTRL-TYPE
#define                     MSG_TYPE_ROBOT_BODY                     1
#define                     MSG_TYPE_ROBOT_BASE                     2
#define                     MSG_TYPE_ROBOT_PTZ                      3
#define                     MSG_TYPE_ROBOT_SUPPORT_DEV              4
#define                     MSG_TYPE_ROBOT_CAMERA                   21
#define                     MSG_TYPE_ROBOT_INFRARED                 22
#define                     MSG_TYPE_TASK_CTRL                      41
#define                     MSG_TYPE_MODEL_SYNC                     61
//Œ¥’“µΩ
#define                     MSG_TYPE_MODEL_SEND                     81

#define                     MSG_TYPE_TASK_ARRANGE                   101
#define                     MSG_TYPE_TASK_ATONCE                    102
//Œ¥’“µΩ 
#define                     MSG_TYPE_DATA_QUERY                     121
#define                     MSG_TYPE_SYSTEM_MSG                     251

//CODE
#define                     MSG_CODE_NEED_RESEND                    100
#define                     MSG_CODE_SUCCESS                        200
#define                     MSG_CODE_REJECTED                       400
#define                     MSG_CODE_ERROR                          500

// ctrl mode
enum ROBOT_CTRL_MODE
{
    ROBOT_MODE_UNKNOWN,
    ROBOT_MODE_TASK ,
    ROBOT_MODE_EMERGENCY_LOC,
    ROBOT_MODE_BACKGROUND_CTRL,
    ROBOT_MODE_JOY_CTRL,
};

struct tvu
{
    std::string type;
    std::string value;
    std::string value_unit;
    std::string unit;
};

struct robotStatus
    {
    std::string robot_name;
    std::string robot_code;
    std::string time;
    std::vector<tvu> data;
};

struct robotRunningStatus
{
    std::string robot_name;
    std::string robot_code;
    std::string time;
    std::vector<tvu> data;
};

struct robotCoordinate
{
    std::string robot_name;
    std::string file_path;
    std::string robot_code;
    std::string time;
    std::string coordinate_pixel;
    std::string coordinate_geography;
};

struct robotAlarmData
{
    std::string robot_name;
    std::string robot_code;
    std::string time;
    std::string content;
};

struct robotMapSelectPointResp
{
    int device_level;
    std::string device_list;
};

struct robotDeviceUploadType
{
    std::string device_name;
    std::string device_id;
    std::string value;
    std::string value_unit;
    std::string unit;
    std::string time;
    std::string recognition_type;
    std::string file_type;
    std::string file_path;
    std::string rectangle;
};

struct robotDeviceAlarmType
{
    std::string device_name;
    std::string device_id;
    std::string alarm_level;
    std::string alarm_type;
    std::string recognition_type;
    std::string value;
    std::string value_unit;
    std::string unit;
    std::string time;
    std::string content;
};

struct robotMicroWeatherData
{
    std::string robot_name;
    std::string robot_code;
    std::string time;
    std::vector<tvu> data;
};

struct robotTaskData
{
    std::string task_patrolled_id;
    std::string task_name;
    std::string task_code;
    std::string task_state;
    std::string plan_start_time;
    std::string start_time;
    std::string task_progress;
    std::string task_estimated_time;
    std::string description;
};

struct robotPatrolResult
{
    std::string robot_code;
    std::string task_name;
    std::string task_code;
    std::vector<robotDeviceUploadType> data;
    std::string task_patrolled_id;
};

struct robotDevAlarmData
{
    std::string robot_code;
    std::string task_name;
    std::string task_code;
    std::vector<robotDeviceAlarmType> data;
    std::string task_patrolled_id;
};

struct robotPatrolReportData
{
    std::string task_patrolled_id;
    std::string task_name;
    std::string task_code;
    std::string plan_start_time;
    std::string start_time;
    std::string end_time;
    int all_count;
    int normal_count;
    int alarm_count;
    int recognition_error_count;
    int miss_count;
    std::string temperature;
    std::string humidity;
    std::string wind_speed;
    std::string description;
    std::string road_file_path;
    std::string task_finish_state;
};

struct OriginalBaseMsg
{
    OriginalBaseMsg()
    {
        start_flag_[0] = 0xEB;
        start_flag_[1] = 0x90;
        end_flag_[0] = 0xEB;
        end_flag_[1] = 0x90;

        sequence_id_ = 0;
        xml_length_ = 0;
        xml_string_ = "";
    }
    uint8_t start_flag_[2];
    uint64_t sequence_id_;
    uint32_t xml_length_;
    std::string xml_string_;
    uint8_t end_flag_[2];
};

struct xml_deviceType
{
    int device_level;
    std::string device_id;
    std::string device_name;
    std::string voltage_level;
    std::string bay_id;
    std::string bay_name;
    std::string main_device_id;
    std::string main_device_name;
    std::string device_type;
    std::string meter_type;
    std::string appearance_type;
    std::string fever_type;
    std::string save_type_list;
    std::string recognition_type_list;
    std::string phase;
};

struct robotCfg
{
    std::string robot_name;
    std::string robot_code;
    std::string manufacturer;
    std::string istransport;
    std::string type;
};

struct alarmThresholdCfg
{
    std::string device_id;
    std::string device_name;
    std::string decision_rules;
    std::string warning_upper_limit;
    std::string warning_lower_limit;
    std::string warning_state;
    std::string common_upper_limit;
    std::string common_lower_limit;
    std::string common_state;
    std::string serious_upper_limit;
    std::string serious_lower_limit;
    std::string serious_state;
    std::string critical_upper_limit;
    std::string critical_lower_limit;
    std::string critical_state;
};

struct workAreaCfg
{
    std::string file_path;
    std::string start_time;
    std::string end_time;
    std::string coordinate_pixel;
    std::string coordinate_geography;
};

struct taskAssignWhole
{
    std::string type;
    std::string task_code;
    std::string task_name;
    std::string priority;
    std::string device_level;
    std::string device_list;
    std::string fixed_start_time;
    std::string cycle_month;
    std::string cycle_week;
    std::string cycle_execute_time;
    std::string cycle_start_time;
    std::string cycle_end_time;
    std::string interval_number;
    std::string interval_type;
    std::string interval_execute_time;
    std::string interval_start_time;
    std::string interval_end_time;
    std::string invalid_start_time;
    std::string invalid_end_time;
    std::string isenable;
    std::string creator;
    std::string create_time;
};

struct taskAssignAtOnce
{
    std::string task_code;
    std::string task_name;
    std::string priority;
    std::string device_level;
    std::string device_list;
};


class XmlProtocolMsg : public boost::enable_shared_from_this <XmlProtocolMsg>
{
public:
    XmlProtocolMsg();
    ~XmlProtocolMsg();

    void setOriginalMsg(uint8_t *buff, int length);
    void setOriginalMsg(std::vector<unsigned char> buff);
    OriginalBaseMsg *getOriginalMsg();
    std::string getString();

    std::string getSendCode();
    void setSendCode(std::string str);

    std::string getReceiveCode();
    void setReceiveCode(std::string str);

    int getType();
    void setType(int val);

    std::string getCode();
    void setCode(std::string val);

    int getCmd();
    void setCmd(int val);

    std::string getTime();
    void setTime(std::string str);

    void setItem(TiXmlElement *item);
    TiXmlElement *getItem();

    void setSequenceId(int seq);
    int getSequenceId();

    bool decode();

    bool encode(std::string &str);

    void generiateMsg(uint64_t id);
    void generiateMsg();
protected:
    std::string         SendCode_;
    std::string         ReceiveCode_;
    int                 Type_;
    std::string         Code_;
    int                 Command_;
    std::string         Time_;

    OriginalBaseMsg     *origMsg_;

    TiXmlDocument       *xmlDoc_;
    TiXmlElement        *xmlRobot_;
    TiXmlElement        *xmlItemData_;

    int                 sequence_id_;
};
#endif