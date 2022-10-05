#ifndef __DL_WHEEL_ROBOT_BIM_CORE_H__
#define __DL_WHEEL_ROBOT_BIM_CORE_H__

#include <boost/shared_ptr.hpp>
#include "common/DLWheelRobotGlobalDef.hpp"
 
class MessageAcceptor;

class LibDLWheelRobotBimCore
{
public:
	LibDLWheelRobotBimCore();
    ~LibDLWheelRobotBimCore();

	void RunMessageLoop(int port = 10000);

	void slot_inspectResultNotify(WheelInspectResultStruct);
	void slot_robotBodyInfo(int robot_id, std::string info);
	void slot_robotEnvInfo(int robot_id, std::string info);
	void slot_robotTaskTrajInfo(int robot_id, std::string info);
	void slot_robotPosInfo(int robot_id, std::string info);
	void slot_robotStatusInfo(int robot_id, std::string info);

private:
	boost::shared_ptr<MessageAcceptor> msg_accptor_;

};


#endif