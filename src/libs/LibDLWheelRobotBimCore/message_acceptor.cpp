#include "message_acceptor.h"
#include "common/DLRobotCommonDef.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include <QString>

using namespace std;

std::string splitEqual(const std::string& str)
{
	size_t start_pos;
	if (std::string::npos == (start_pos = str.find('=')))
	{
		return "";
	}
	return str.substr(start_pos + 1);
}

std::string eraseAnd(const std::string& str)
{
	size_t start_pos;
	if (std::string::npos == (start_pos = str.find('&')))
	{
		return str;
	}
	return str.substr(0, start_pos);
}

std::string getVaule(const std::string& input_str, const std::string &feild_name)
{
	size_t start_pos;
	if (std::string::npos == (start_pos = input_str.find(feild_name)))
	{
		return "";
	}
	return splitEqual(eraseAnd(input_str.substr(start_pos)));
}

#define DEBUG_FLAG  (0)

MessageAcceptor::MessageAcceptor()
{
	m_uploadRunning = true;
}

MessageAcceptor::~MessageAcceptor()
{
	m_uploadRunning = false;
	upload_file_thread_->join();
}

void MessageAcceptor::InitServer(const std::string &port)
{
	message_acceptor_ = boost::shared_ptr<HttpServer>(new HttpServer);
	message_acceptor_->Init(port);
	message_acceptor_->Start();

	ftp_client_ = boost::shared_ptr<FTPImpl>(new FTPImpl);
	upload_file_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MessageAcceptor::uploadPicLoop, this)));
}

void MessageAcceptor::RegistEventHandles()
{
	message_acceptor_->AddHandler(FJX_INFO_REQ, std::bind(&MessageAcceptor::handleFjxInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ALL_FJX_LATEST_INFO_REQ, std::bind(&MessageAcceptor::handleAllFjxLatestInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(WSTSB_INFO_REQ, std::bind(&MessageAcceptor::handleWsbInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ALL_WSTSB_LATEST_INFO_REQ, std::bind(&MessageAcceptor::handleAllWsbLatestInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_BASE_INFO_REQ, std::bind(&MessageAcceptor::handleRobotBaseInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_ENV_INFO_REQ, std::bind(&MessageAcceptor::handleRobotEnvInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_STATUS_INFO_REQ, std::bind(&MessageAcceptor::handleRobotStatusInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_POSE_INFO_REQ, std::bind(&MessageAcceptor::handleRobotPosInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_LATEST_STATUS_INFO_REQ, std::bind(&MessageAcceptor::handleLatestRobotStatusInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_PRESET_POSITION_INFO_REQ, std::bind(&MessageAcceptor::handleRobotPresetPositionInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(ROBOT_PLANNED_ROUTE_INFO_REQ, std::bind(&MessageAcceptor::handleRobotPlannedRouteInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(CAMERA_PARAMETER_REQ, std::bind(&MessageAcceptor::handleRobotCameraInfoQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(DEV_IMAGE_PATH_REQ, std::bind(&MessageAcceptor::handleDevImagePathQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	message_acceptor_->AddHandler(DEV_UINT_IMAGE_PATH_REQ, std::bind(&MessageAcceptor::handleDevUintImagePathQuery, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}
 
void MessageAcceptor::AppendInspectData(WheelInspectResultStruct inspect)
{
	boost::mutex::scoped_lock lock(robot_patrol_result_mutex_);
	m_patrolResults.push_back(inspect);
}

void MessageAcceptor::updateRobotAllInfo()
{
}

void MessageAcceptor::flushRobotInfo(int robot_id, std::string info)
{
		boost::mutex::scoped_lock _lock(robot_base_info_mutex_);
		m_robotBaseInfo[robot_id] = info;
}

void MessageAcceptor::flushRobotEnvInfo(int robot_id, std::string info)
{
	boost::mutex::scoped_lock lock(robot_env_info_mutex_);
	m_env_info[robot_id] = info;
}

void MessageAcceptor::flushRobotTaskTrajInfo(int robot_id, std::string info)
{
	boost::mutex::scoped_lock lock(robot_task_traj_mutex_);
	m_plannedRoute[robot_id] = info;
}

void MessageAcceptor::flushRobotPosInfo(int robot_id, std::string info)
{
	boost::mutex::scoped_lock lock(robot_pos_mutex_);
	m_robotPos[robot_id] = info;
}

void MessageAcceptor::flushRobotStatusInfo(int robot_id, std::string info)
{
	boost::mutex::scoped_lock lock(robot_patrol_status_mutex_);
	m_statusInfo[robot_id] = info;
}

bool MessageAcceptor::handleFjxInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("收到所有风机箱最新信息查询");
	std::string deviceid = getVaule(query_string, "deviceid");
	std::string timestart = getVaule(query_string, "timestart");
	std::string timeend = getVaule(query_string, "timeend");
	std::string robotid = getVaule(query_string, "robotid");

	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response;
	fjxInfo fjx_info;
	JsonContent item;

	//fan_status
	std::vector<JsonContent> fan_staus_items;
	for (int j = 0; j < fjx_info.fan_status_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement(fjx_info.fan_status_vec[j].status_desc, fjx_info.fan_status_vec[j].current_status);
		sub_item.jsonAppendElement("time", fjx_info.fan_status_vec[j].time);
		fan_staus_items.push_back(sub_item);
	}
	item.jsonAppendElement("id", fjx_info.id);
	item.jsonAppendElement("dev_name", fjx_info.dev_name);
	item.jsonAppendElements("fan_status", fan_staus_items);

	//ctrl_cab_temp
	std::vector<JsonContent> ctrl_cab_temp_items;
	for (int j = 0; j < fjx_info.ctrl_cab_temp_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.ctrl_cab_temp_vec[j].data);
		sub_item.jsonAppendElement("time", fjx_info.ctrl_cab_temp_vec[j].time);
		ctrl_cab_temp_items.push_back(sub_item);
	}
	item.jsonAppendElements("ctrl_cab_temp", ctrl_cab_temp_items);

	//cnds_pump_alert
	std::vector<JsonContent> cnds_pump_alert_items;
	for (int j = 0; j < fjx_info.cnds_pump_alert_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.cnds_pump_alert_vec[j].data);
		sub_item.jsonAppendElement("alert", fjx_info.cnds_pump_alert_vec[j].time);
		cnds_pump_alert_items.push_back(sub_item);
	}
	item.jsonAppendElements("ctrl_cab_temp", cnds_pump_alert_items);

	//ph_volt
	std::vector<JsonContent> ph_volt_items;
	for (int j = 0; j < fjx_info.ph_volt_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.ph_volt_vec[j].data);
		sub_item.jsonAppendElement("time", fjx_info.ph_volt_vec[j].time);
		ph_volt_items.push_back(sub_item);
	}
	item.jsonAppendElements("ctrl_cab_temp", ph_volt_items);

	//I1_cur
	std::vector<JsonContent> l1_cur_items;
	for (int j = 0; j < fjx_info.l1_cur_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.l1_cur_vec[j].data);
		sub_item.jsonAppendElement("time", fjx_info.l1_cur_vec[j].time);
		l1_cur_items.push_back(sub_item);
	}
	item.jsonAppendElements("l1_cur", l1_cur_items);

	//l2_cur
	std::vector<JsonContent> l2_cur_items;
	for (int j = 0; j < fjx_info.l2_cur_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.l2_cur_vec[j].data);
		sub_item.jsonAppendElement("time", fjx_info.l2_cur_vec[j].time);
		l2_cur_items.push_back(sub_item);
	}
	item.jsonAppendElements("l2_cur", l2_cur_items);

	//l3_cur
	std::vector<JsonContent> l3_cur_items;
	for (int j = 0; j < fjx_info.l3_cur_vec.size(); ++j)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("data", fjx_info.l3_cur_vec[j].data);
		sub_item.jsonAppendElement("time", fjx_info.l3_cur_vec[j].time);
		l3_cur_items.push_back(sub_item);
	}
	item.jsonAppendElements("l3_cur", l3_cur_items);

	response.jsonAppendJson("data", item);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleAllFjxLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response;
	std::vector<fjxInfo> fjx_info_vec;

	std::vector<JsonContent> json_fjx_infos;
	for (int i = 0; i < fjx_info_vec.size(); ++i)
	{
		JsonContent item;

		//fan_status
		std::vector<JsonContent> fan_staus_items;
		for (int j = 0; j < fjx_info_vec[i].fan_status_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement(fjx_info_vec[i].fan_status_vec[j].status_desc, fjx_info_vec[i].fan_status_vec[j].current_status);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].fan_status_vec[j].time);
			fan_staus_items.push_back(sub_item);
		}
		item.jsonAppendElement("id", fjx_info_vec[i].id); 
		item.jsonAppendElement("dev_name", fjx_info_vec[i].dev_name);
		item.jsonAppendElements("fan_status", fan_staus_items);

		//ctrl_cab_temp
		std::vector<JsonContent> ctrl_cab_temp_items;
		for (int j = 0; j < fjx_info_vec[i].ctrl_cab_temp_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].ctrl_cab_temp_vec[j].data);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].ctrl_cab_temp_vec[j].time);
			ctrl_cab_temp_items.push_back(sub_item);
		}
		item.jsonAppendElements("ctrl_cab_temp", ctrl_cab_temp_items);

		//cnds_pump_alert
		std::vector<JsonContent> cnds_pump_alert_items;
		for (int j = 0; j < fjx_info_vec[i].cnds_pump_alert_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].cnds_pump_alert_vec[j].data);
			sub_item.jsonAppendElement("alert", fjx_info_vec[i].cnds_pump_alert_vec[j].time);
			cnds_pump_alert_items.push_back(sub_item);
		}
		item.jsonAppendElements("ctrl_cab_temp", cnds_pump_alert_items);

		//ph_volt
		std::vector<JsonContent> ph_volt_items;
		for (int j = 0; j < fjx_info_vec[i].ph_volt_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].ph_volt_vec[j].data);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].ph_volt_vec[j].time);
			ph_volt_items.push_back(sub_item);
		}
		item.jsonAppendElements("ctrl_cab_temp", ph_volt_items);

		//I1_cur
		std::vector<JsonContent> l1_cur_items;
		for (int j = 0; j < fjx_info_vec[i].l1_cur_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].l1_cur_vec[j].data);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].l1_cur_vec[j].time);
			l1_cur_items.push_back(sub_item);
		}
		item.jsonAppendElements("l1_cur", l1_cur_items);

		//l2_cur
		std::vector<JsonContent> l2_cur_items;
		for (int j = 0; j < fjx_info_vec[i].l2_cur_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].l2_cur_vec[j].data);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].l2_cur_vec[j].time);
			l2_cur_items.push_back(sub_item);
		}
		item.jsonAppendElements("l2_cur", l2_cur_items);

		//l3_cur
		std::vector<JsonContent> l3_cur_items;
		for (int j = 0; j < fjx_info_vec[i].l3_cur_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("data", fjx_info_vec[i].l3_cur_vec[j].data);
			sub_item.jsonAppendElement("time", fjx_info_vec[i].l3_cur_vec[j].time);
			l3_cur_items.push_back(sub_item);
		}
		item.jsonAppendElements("l3_cur", l3_cur_items);

		json_fjx_infos.push_back(item);
	}
	response.jsonAppendElements("data", json_fjx_infos);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleWsbInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());
	std::string deviceid = getVaule(query_string, "deviceid");

	//获取数据
	JsonContent response;
	WsbInfo wsb_info;
	
	JsonContent item;
	std::vector<JsonContent> sub_items;
	for (int i = 0; i < wsb_info.alerts_vec.size(); ++i)
	{
		JsonContent sub_item;
		sub_item.jsonAppendElement("sta", wsb_info.alerts_vec[i].status);
		sub_item.jsonAppendElement("content", wsb_info.alerts_vec[i].content);
		sub_item.jsonAppendElement("time", wsb_info.alerts_vec[i].time);
		sub_items.push_back(sub_item);
	}

	item.jsonAppendElement("id", deviceid);
	item.jsonAppendElements("alert", sub_items);
	
	response.jsonAppendJson("data", item);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleAllWsbLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response;
	std::vector<WsbInfo> wsb_info_vec;

	std::vector<JsonContent> json_wsb_infos;
	for (int i = 0; i < wsb_info_vec.size(); ++i)
	{
		JsonContent item;
		std::vector<JsonContent> sub_items;

		for (int j = 0; j < wsb_info_vec[i].alerts_vec.size(); ++j)
		{
			JsonContent sub_item;
			sub_item.jsonAppendElement("sta", wsb_info_vec[i].alerts_vec[j].status);
			sub_item.jsonAppendElement("content", wsb_info_vec[i].alerts_vec[j].content);
			sub_item.jsonAppendElement("time", wsb_info_vec[i].alerts_vec[j].time);
			sub_items.push_back(sub_item);
		}

		item.jsonAppendElement("id", wsb_info_vec[i].id);
		item.jsonAppendElements("alert", sub_items);
		json_wsb_infos.push_back(item);
	}
	response.jsonAppendElements("data", json_wsb_infos);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleRobotBaseInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	JsonContent response, json_data;

	//获取数据
	{
		boost::mutex::scoped_lock _lock(robot_base_info_mutex_);
		std::string content = "";
		auto itor = m_robotBaseInfo.find(std::atoi(getVaule(query_string, "robotid").c_str()));
		if (itor != m_robotBaseInfo.end())
		{
			if (!json_data.fromJsonStringToJsonVal(itor->second)) {
				ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, itor->second.c_str());
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
			}
			else {
				response.jsonAppendJson("data", json_data);
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
			}
		}
		else {
			response.jsonAppendElement("success", true);
			response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		}
	}

	//返回json数据
	rsp_callback(c, response.getJsonString());

	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());
	return true;
}

bool MessageAcceptor::handleRobotEnvInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());
	std::string timestart = getVaule(query_string, "timestart");
	std::string timeend = getVaule(query_string, "timeend");

	//获取数据
	//std::string info;
	//{
	//	boost::mutex::scoped_lock lock(robot_env_info_mutex_);
	//	info = m_env_info;
	//}

	////std::string str = "{\"success\": true,\"message\" : \"成功\",\"data\" : {\"id\": \"3\",\"temp\" : [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 32.5},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 32.6}],\"hum\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 40},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 40.5}],\"CO2\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 578.0},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 578.0}],\"PM25\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 70.3},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 70.3}],\"PM10\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 40.5},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 40.5}],\"noise\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 53.3},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 53.3}],\"TVOC\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 2.7},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 2.7}],\"CH2O\": [{\"time\": \"2018-06-08 14:26:15\",\"data\" : 1.0},{\"time\": \"2018-06-08 14:26:17\",\"data\" : 1.0}],\"extin_pressure\": [{\"time\": \"2018-06-08 14:26:15\",\"extin_id\" : \"extin1\",\"data\" : 3.0},{\"time\": \"2018-06-08 14:26:15\",\"extin_id\" : \"extin2\",\"data\" : 3.0}],\"pipe_leak\": [{\"time\": \"2018-06-08 14:26:15\",\"leak\" : \"1\"},{\"time\": \"2018-05-08 14:26:15\",\"leak\" : \"1\"}]}}";

	//JsonContent response;
	//if (!response.fromJsonStringToJsonVal(info)) {
	//	ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, m_env_info.c_str());
	//	return false;
	//}

	JsonContent response;

	//获取数据
	{
		boost::mutex::scoped_lock _lock(robot_env_info_mutex_);
		std::string content = "";
		auto itor = m_env_info.find(std::atoi(getVaule(query_string, "robotid").c_str()));
		if (itor != m_env_info.end())
		{
			if (!response.fromJsonStringToJsonVal(itor->second)) {
				ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, itor->second.c_str());
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
			}
			else {
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
			}
		}
		else {
			response.jsonAppendElement("success", true);
			response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		}
	}


	response.getJsonVal().removeMember("robot_id");
	response.jsonAppendElement("id", getVaule(query_string, "robotid"));

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());
	
	std::string timestart = getVaule(query_string, "timestart");
	std::string timeend = getVaule(query_string, "timeend"); 
	std::string robotid = getVaule(query_string, "robotid");

	JsonContent response;
	if (robotid.empty() || timeend.empty() || timestart.empty()) 
	{
		response.jsonAppendElement("success", true);
		response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		rsp_callback(c, response.getJsonString());
		ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

		return true;
	}

	QList<WheelPatrolResultStruct> patrol_result;

	WHEEL_ROBOT_DB.getWheelPatrolResultDB(patrol_result, QString::fromStdString(timestart), QString::fromStdString(timeend));
	//获取数据
	JsonContent json_data;
	std::vector<JsonContent> result_list;

	QString device_deal_id, device_deal_name;
	for (auto itor = patrol_result.begin(); itor != patrol_result.end(); ++itor)
	{
		JsonContent patrol_info;
		patrol_info.jsonAppendElement("status", "GoalReached");
		
		WHEEL_ROBOT_DB.getDeviceSnForDeviceUUidDB(itor->device_uuid, device_deal_id, device_deal_name);

		patrol_info.jsonAppendElement("dev_id", device_deal_id.toStdString());

		patrol_info.jsonAppendElement("dev_name", WHEEL_DEVICE_CONFIG.getDeviceTypeNameForDeviceUUid(itor->device_uuid).toLocal8Bit().constData());
		patrol_info.jsonAppendElement("time", itor->inspect_time.toStdString());
		result_list.push_back(patrol_info);
	}
	
	json_data.jsonAppendElement("id", getVaule(query_string, "robotid"));
	json_data.jsonAppendElements("list", result_list);
	
	response.jsonAppendJson("data", json_data);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleRobotPosInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_INFO("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent json_data;
	JsonContent response;

	{
		boost::mutex::scoped_lock _lock(robot_pos_mutex_);
		std::string content = "";
		auto itor = m_robotPos.find(std::atoi(getVaule(query_string, "robotid").c_str()));
		if (itor != m_robotPos.end())
		{
			if (!json_data.fromJsonStringToJsonVal(itor->second)) {
				ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, itor->second.c_str());
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
			}
			else {
				response.jsonAppendJson("data", json_data);
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
			}
		}
		else {
			response.jsonAppendElement("success", true);
			response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		}
	}

	rsp_callback(c, response.getJsonString());
	ROS_INFO("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleLatestRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response;
	JsonContent json_data;

	{
		boost::mutex::scoped_lock _lock(robot_patrol_status_mutex_);
		std::string content = "";
		auto itor = m_statusInfo.find(std::atoi(getVaule(query_string, "robotid").c_str()));
		if (itor != m_statusInfo.end())
		{
			if (!json_data.fromJsonStringToJsonVal(itor->second)) {
				ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, itor->second.c_str());
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
			}
			else {
				response.jsonAppendJson("data", json_data);
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
			}
		}
		else {
			response.jsonAppendElement("success", true);
			response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		}
	}

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleRobotPresetPositionInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());
	std::string unit_image = getVaule(query_string, "unit_image");
	//获取数据
	JsonContent response, json_data;
	presetPatrolDevInfo presets_info;

	std::vector<JsonContent> json_presets_info;
	for (int i = 0; i < presets_info.preset_pos_vec.size(); ++i)
	{
		JsonContent item;
		item.jsonAppendElement("seq", presets_info.preset_pos_vec[i].seq);
		item.jsonAppendElement("enabled", presets_info.preset_pos_vec[i].enabled);
		item.jsonAppendElement("device_id", presets_info.preset_pos_vec[i].device_id);
		item.jsonAppendElement("description", presets_info.preset_pos_vec[i].description);
		json_presets_info.push_back(item);
	}
	json_data.jsonAppendElements("preset_pos", json_presets_info);
	json_data.jsonAppendElement("robotid", getVaule(query_string, "robotid"));

	response.jsonAppendJson("data", json_data);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

bool MessageAcceptor::handleRobotPlannedRouteInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	//序列化json数据
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent json_data;
	JsonContent response;

	{
		boost::mutex::scoped_lock _lock(robot_task_traj_mutex_);
		std::string content = "";
		auto itor = m_plannedRoute.find(std::atoi(getVaule(query_string, "robot_id").c_str()));
		if (itor != m_plannedRoute.end())
		{
			if (!json_data.fromJsonStringToJsonVal(itor->second)) {
				ROS_ERROR("MessageAcceptor::%s===>%s, str to json failed..", __FUNCTION__, itor->second.c_str());
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
			}
			else {
				response.jsonAppendJson("data", json_data);
				response.jsonAppendElement("success", true);
				response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
			}
		}
		else {
			response.jsonAppendElement("success", true);
			response.jsonAppendElement("message", QString("失败").toLocal8Bit().constData());
		}
	}

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());
	return true;
}

bool MessageAcceptor::handleRobotCameraInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response;
	JsonContent json_data;
	json_data.jsonAppendElement("id", getVaule(query_string, "robotid"));

	std::vector<JsonContent> camera_list;
	for (int i = 0; i < WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg().size(); ++i)
	{
		if (WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].robotName.toStdString() == getVaule(query_string, "robotid"))
		{
			JsonContent visible_item, infrared_item;
			visible_item.jsonAppendElement("name", QString("可见光").toLocal8Bit().constData());
			visible_item.jsonAppendElement("ip", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcIP.toStdString());
			visible_item.jsonAppendElement("port", std::to_string(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcCtrlPort));
			visible_item.jsonAppendElement("user", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcUserName.toStdString());
			visible_item.jsonAppendElement("password", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcPassword.toStdString());
			visible_item.jsonAppendElement("channel", "1");
			visible_item.jsonAppendElement("subtype", "main");
			visible_item.jsonAppendElement("type", QString("主可见光").toLocal8Bit().constData());
		
			infrared_item.jsonAppendElement("name", QString("热成像").toLocal8Bit().constData());
			infrared_item.jsonAppendElement("ip", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].infraredCameraIp.toStdString());
			infrared_item.jsonAppendElement("port", std::to_string(WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].infraredRtspPort));
			infrared_item.jsonAppendElement("user", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcUserName.toStdString());
			infrared_item.jsonAppendElement("password", WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg()[i].hcPassword.toStdString());
			infrared_item.jsonAppendElement("channel", "1");
			infrared_item.jsonAppendElement("subtype", "main");
			infrared_item.jsonAppendElement("type", QString("热成像").toLocal8Bit().constData());
			camera_list.push_back(visible_item);
			camera_list.push_back(infrared_item);
			json_data.jsonAppendElements("cameras", camera_list);
		}
	}
	response.jsonAppendJson("data", json_data);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	rsp_callback(c, response.getJsonString());
	return true;
}

bool MessageAcceptor::handleDevImagePathQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());

	//获取数据
	JsonContent response, image_info;
	SigleDevImageInfo image_path_info;

	image_info.jsonAppendElement("id", image_path_info.name);
	image_info.jsonAppendElement("image_path", image_path_info.image_path);
	image_info.jsonAppendElement("time", image_path_info.time);
	
	response.jsonAppendJson("data", image_info);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());
	return true;
}

bool MessageAcceptor::handleDevUintImagePathQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback)
{
	ROS_WARN("[%s::request]:%s", __FUNCTION__, query_string.c_str());
	//getVaule(query_string, "unit_image");

	std::string device_id = getVaule(query_string, "deviceid");

	//获取数据
	JsonContent response, json_data; 
	QString device_sn_name;
	QList<WheelPatrolResultStruct> wheel_patrol_result_list;
	QList<WheelPatrolResultStruct> result_wheel_patrol_result_list;

	//1.获取到设备的主设备名称
	WHEEL_ROBOT_DB.getDeviceSnNameForDeviceSn(QString::fromStdString(device_id), device_sn_name);

	//2.获取到主设备相关联的巡检设备
	QList<QString> devic_uuid_list;
	WHEEL_ROBOT_DB.getDeviceUuidForDeviceSn(QString::fromStdString(device_id), devic_uuid_list);

	//3.寻找近x填的巡检记录
	QDateTime current_date_time = QDateTime::currentDateTime();
	QString strCurrentTime = current_date_time.toString("yyyy-MM-dd hh:mm:ss");
	QString strThreeDaysBeforeTime = current_date_time.addDays( 0 - WHEEL_ROBOT_CORE_CONFIG.getCfg().recordPeriod).toString("yyyy-MM-dd hh:mm:ss");

	WHEEL_ROBOT_DB.getWheelPatrolResultDB(wheel_patrol_result_list, strThreeDaysBeforeTime, strCurrentTime);

	for (int i = 0; i < wheel_patrol_result_list.size(); ++i)
	{
		for (int j = 0; j < devic_uuid_list.size(); ++j)
		{
			if (0 == devic_uuid_list[j].compare(wheel_patrol_result_list[i].device_uuid))
			{
				result_wheel_patrol_result_list.append(wheel_patrol_result_list[i]);
			}
		}
	}

	//4.打包记录到数据中
	std::vector<JsonContent> dev_images;
	QString device_detail_name;
	QString device_full_name;
	QString image_path;
	JsonContent item;
	int record_length = 0;

	for(auto itor = result_wheel_patrol_result_list.begin(); itor != result_wheel_patrol_result_list.end(); ++itor)
	{
		item.clear();

		WHEEL_ROBOT_DB.getDeviceTypeNameForDeviceUUidDB(itor->device_uuid, device_detail_name);
		device_full_name = device_sn_name + "_" + device_detail_name;
		item.jsonAppendElement("name", device_full_name.toLocal8Bit().constData());

		//QString date_string = QDateTime::fromString(itor->inspect_time, "yyyy-MM-dd hh:mm:ss").toString("yyyy-MM-dd");

		image_path = "/" + QString::fromStdString(itor->inspect_time.toStdString().substr(0, 10)) + "/" +
					itor->task_uuid + "/" + itor->device_uuid + "/" + itor->device_uuid + "_result.jpg";
			
		item.jsonAppendElement("image_path", image_path.toStdString());
		item.jsonAppendElement("time", itor->inspect_time.toStdString());

		if (record_length++ < 10)
		{
			dev_images.push_back(item);
		}
	}

	json_data.jsonAppendElements("dev_images", dev_images);
	json_data.jsonAppendElement("id", device_id);

	response.jsonAppendJson("data", json_data);
	response.jsonAppendElement("success", true);
	response.jsonAppendElement("message", QString("成功").toLocal8Bit().constData());

	rsp_callback(c, response.getJsonString());
	ROS_WARN("[%s::response]:%s", __FUNCTION__, response.getJsonString().c_str());

	return true;
}

void MessageAcceptor::uploadPicLoop()
{
	Sleep(5000);
	bool is_empty = false;

	while (m_uploadRunning)
	{
		WheelInspectResultStruct inspect;
#if 1
		{
			boost::mutex::scoped_lock lock(robot_patrol_result_mutex_);
			is_empty = m_patrolResults.empty();
			if (!m_patrolResults.empty()) {
				ROS_WARN("MessageAcceptor::%s, m_patrolResults.size:%d", __FUNCTION__, m_patrolResults.size());
				inspect = m_patrolResults.front();
			}
		}

#else
		QList<WheelPatrolResultStruct> wheel_patrol_result_list;

		//3.寻找近x填的巡检记录
		QDateTime current_date_time = QDateTime::currentDateTime();
		QString strCurrentTime = current_date_time.toString("yyyy-MM-dd hh:mm:ss");
		QString strThreeDaysBeforeTime = current_date_time.addDays(-4).toString("yyyy-MM-dd hh:mm:ss");

		WHEEL_ROBOT_DB.getWheelPatrolResultDB(wheel_patrol_result_list, strThreeDaysBeforeTime, strCurrentTime);
		if (wheel_patrol_result_list.size() > 0)
		{
			inspect.device_uuid = wheel_patrol_result_list[0].device_uuid;
			inspect.inspect_time = wheel_patrol_result_list[0].inspect_time;
			inspect.task_uuid = wheel_patrol_result_list[0].task_uuid;
		}
#endif
		if(is_empty)
		{
			Sleep(500);
			continue;
		}


		QString local_file;
		QString remote_file;
		QString relative_path;
		QString date_string = QString::fromStdString(inspect.inspect_time.toStdString().substr(0, 10));

		QString root_path = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/task";
		{
			relative_path = "/" + inspect.task_uuid + "/" + inspect.device_uuid + "/" + inspect.device_uuid + "_result.jpg";

			local_file = root_path + relative_path;
			remote_file = "/" + date_string + relative_path;
		}

		ROS_INFO("ftp address===>%s:%d/%s/%", WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerIp.toStdString().c_str(),
			WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerPort,
			WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserName.toStdString().c_str(),
			WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserPasswd.toStdString().c_str()
		);

		//if (0 != ftp_client_->connect(WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserName.toStdString().c_str(),
		//	WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserPasswd.toStdString().c_str(),
		//	WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerIp.toStdString().c_str(),
		//	WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerPort))
		//{
		//	ROS_ERROR("MessageAcceptor::%s, connect ftp server failed...", __FUNCTION__);
		//}

		//if (0 != ftp_client_->upload(remote_file.toLocal8Bit().constData(), local_file.toLocal8Bit().constData())) 
		FTP_OPT ftp_option;
		ftp_option.url = (QString("ftp://%1:%2").arg(WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerIp.toStdString().c_str()).arg(WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpServerPort) + remote_file).toStdString();
		ftp_option.user_key = QString("%1:%2").arg(WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserName.toStdString().c_str()).arg(WHEEL_ROBOT_CORE_CONFIG.getCfg().ftpUserPasswd.toStdString().c_str()).toStdString();
		ftp_option.file = local_file.toStdString();

		if(FTP_UPLOAD_SUCCESS == ftp_client_->upload(ftp_option))
		{
			ROS_WARN("MessageAcceptor::%s, upload file:%s ftp server success...", __FUNCTION__, local_file.toLocal8Bit().constData());
#if 1
			boost::mutex::scoped_lock lock(robot_patrol_result_mutex_);
			m_patrolResults.pop_front();
#else
			is_empty = true;
#endif
		}
		else
		{
			ROS_ERROR("MessageAcceptor::%s, upload file:%s ftp server failed...", __FUNCTION__, local_file.toLocal8Bit().constData());
		}
		Sleep(1000);
	}
	//ftp_client_->close();
}
