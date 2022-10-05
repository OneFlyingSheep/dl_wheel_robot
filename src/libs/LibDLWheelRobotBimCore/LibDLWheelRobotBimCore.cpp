#include "LibDLWheelRobotBimCore.h"
#include "message_acceptor.h"
#include "common/DLWheelRobotGlobalDef.hpp"


LibDLWheelRobotBimCore::LibDLWheelRobotBimCore()
{
}

LibDLWheelRobotBimCore::~LibDLWheelRobotBimCore()
{
}

void LibDLWheelRobotBimCore::RunMessageLoop(int port)
{
	//  ≥ı ºªØhttp_server
	msg_accptor_ = boost::shared_ptr<MessageAcceptor>(new MessageAcceptor);
	msg_accptor_->InitServer(std::to_string(port));
	msg_accptor_->RegistEventHandles();
	ROS_WARN("init http server:port[%d]", port);
}

void LibDLWheelRobotBimCore::slot_inspectResultNotify(WheelInspectResultStruct result)
{
	msg_accptor_->AppendInspectData(result);
}

void LibDLWheelRobotBimCore::slot_robotBodyInfo(int robot_id, std::string battery)
{

	msg_accptor_->flushRobotInfo(robot_id, battery);
}

void LibDLWheelRobotBimCore::slot_robotEnvInfo(int robot_id, std::string info)
{
	msg_accptor_->flushRobotEnvInfo(robot_id, info);
}

void LibDLWheelRobotBimCore::slot_robotTaskTrajInfo(int robot_id, std::string info)
{
	msg_accptor_->flushRobotTaskTrajInfo(robot_id, info);
}

void LibDLWheelRobotBimCore::slot_robotPosInfo(int robot_id, std::string info)
{
	msg_accptor_->flushRobotPosInfo(robot_id, info);
}

void LibDLWheelRobotBimCore::slot_robotStatusInfo(int robot_id, std::string info)
{
	msg_accptor_->flushRobotStatusInfo(robot_id, info);
}

