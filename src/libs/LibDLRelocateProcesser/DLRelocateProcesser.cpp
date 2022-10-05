#include "DLRelocateProcesser.h"
#include "LibDLSceneView/DLCustomScene.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <QDebug>



DLRelocateProcesser::DLRelocateProcesser(DLCustomScene *scene, double x, double y, double angle)
	: scene_(scene), pos_x_(x), pos_y_(y), pos_angle_(angle)
{

	isStop_ = false;
	old_execute_result_ = scene_->GetRelocateRetVal();
	WHEEL_BACK_TO_CORE_SOCKET.robot_control_reloc_req(pos_x_, pos_y_, pos_angle_);

}


void DLRelocateProcesser::run()
{
	
	int speed_time = 0;
	QTime timer;
	timer.start();
	int executeResult = -1;
	while (true)
	{
		Sleep(100);
		runningMutex_.lock();
		speed_time = timer.elapsed();
		
		//³¬Ê±´¦Àí
		if (speed_time > 10000) {
			emit signal_relocation_finished(-1);
			runningMutex_.unlock();
			break;
		}

		if(isStop_) {
			runningMutex_.unlock();
			break;
		}
        executeResult = scene_->GetRelocateRetVal();
	
		if(executeResult == old_execute_result_){
			runningMutex_.unlock();
			continue;
		}
		else{
			if (executeResult == 3){
				emit signal_relocation_finished(executeResult);	
			}
			else if(executeResult == 1){
				emit signal_relocation_finished(executeResult);	
				break;
			}
			old_execute_result_ = executeResult;
		}
		runningMutex_.unlock();
	}

}


void DLRelocateProcesser::stopThread()
{

	isStop_ = true;

}


void DLRelocateProcesser::slot_confirm_relocation()
{
	WHEEL_BACK_TO_CORE_SOCKET.robot_control_comfirmloc_req();
}

void DLRelocateProcesser::lockThread()
{
	
	runningMutex_.lock();

}


void DLRelocateProcesser::unlockThread()
{

	runningMutex_.unlock();

}


