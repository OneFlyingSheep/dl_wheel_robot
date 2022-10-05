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
	//�������Ϣ
	bool handleFjxInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//����������Ϣ
	bool handleAllFjxLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//��ˮ��������Ϣ
	bool handleWsbInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//��ˮ�����������Ϣ
	bool handleAllWsbLatestInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˱�����Ϣ
	bool handleRobotBaseInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˱��廷����������Ϣ
	bool handleRobotEnvInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ�������Ѳ��������Ϣ
	bool handleRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˵�λ��Ϣ
	bool handleRobotPosInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˵�λ��Ϣ
	bool handleLatestRobotStatusInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˵�λ��Ϣ
	bool handleRobotPresetPositionInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//Ѳ������˵�λ��Ϣ
	bool handleRobotPlannedRouteInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//�����˱�������ͷ
	bool handleRobotCameraInfoQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//����ͼƬ
	bool handleDevImagePathQuery(std::string query_string, std::string body, mg_connection *c, OnRspCallback rsp_callback);

	//ͼƬ
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
