#ifndef COMMON_DATA_DEF_H
#define COMMON_DATA_DEF_H

// ��Ź��õ����ݶ���;


enum RobotBodyControlType
{
	RobotBody_GoForward = 0,					// ǰ��;
	RobotBody_GoUp,								// ����;
	RobotBody_GoDown,							// �½�;
	RobotBody_GoBack,							// ����;
	RobotBody_Stretch,							// �쿪;
	RobotBody_Shrink,							// ����;
	RobotBody_Restoration						// ��λ;
};

enum RobotVideoControlType
{
	RobotVideo_Up = 0,							// ����;
	RobotBody_Levorotation,						// ����;
	RobotBody_Dextrorotation,					// ����;
	RobotBody_Down,								// ����;
	RobotBody_Lift_Up,							// ̧��;
	RobotBody_Lift_Down,						// ����;
	RobotVideo_Restoration						// ��λ;
};

#endif