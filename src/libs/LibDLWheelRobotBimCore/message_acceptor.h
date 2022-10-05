#ifndef __MESSAGE_ACCEPTOR_H__
#define __MESSAGE_ACCEPTOR_H__

#include <time.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include "http_server.h"
#include "url_def.h"
#include "common/DLWheelRobotGlobalDef.hpp"
#include "ftp_manager.h"
#include <map>

#define  SINGLE_TEST    (1)


class MessageAcceptor
{
public:
	MessageAcceptor();
	~MessageAcceptor();

public:
	void InitServer(const std::string&port);
	void RegistEventHandles();
	void AppendInspectData(WheelInspectResultStruct inspect);
	void updateRobotAllInfo();
	void flushRobotInfo(int robot_id, std::string info);
	void flushRobotEnvInfo(int robot_id, std::string info);
	void flushRobotTaskTrajInfo(int robot_id, std::string info);
	void flushRobotPosInfo(int robot_id, std::string info);
	void flushRobotStatusInfo(int robot_id, std::string info);

private:
	//风机箱信息
	bool handleFjxInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//风机箱最近信息
	bool handleAllFjxLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//污水提升泵信息
	bool handleWsbInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//污水提升泵最近信息
	bool handleAllWsbLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人本体信息
	bool handleRobotBaseInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人本体环境传感器信息
	bool handleRobotEnvInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人巡检数据信息
	bool handleRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人点位信息
	bool handleRobotPosInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人点位信息
	bool handleLatestRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人点位信息
	bool handleRobotPresetPositionInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//巡检机器人点位信息
	bool handleRobotPlannedRouteInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//机器人本体摄像头
	bool handleRobotCameraInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//机组图片
	bool handleDevImagePathQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//图片
	bool handleDevUintImagePathQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

private:
	void uploadPicLoop();

private:
	boost::mutex robot_patrol_result_mutex_;
	std::list<WheelInspectResultStruct> m_patrolResults;

	boost::mutex robot_patrol_status_mutex_;
	std::map<int, std::string> m_statusInfo;

	boost::mutex robot_base_info_mutex_;
	std::map<int, std::string> m_robotBaseInfo;

	boost::mutex robot_env_info_mutex_;
	std::map<int, std::string> m_env_info;

	boost::mutex robot_task_traj_mutex_;
	std::map<int, std::string> m_plannedRoute;

	boost::mutex robot_pos_mutex_;
	std::map<int, std::string> m_robotPos;


private:
	boost::shared_ptr<HttpServer> message_acceptor_;
	boost::shared_ptr<FTPImpl> ftp_client_;

	bool m_uploadRunning;
	boost::shared_ptr<boost::thread> upload_file_thread_;
};


#endif  //  __MESSAGE_ACCEPTOR_H__
