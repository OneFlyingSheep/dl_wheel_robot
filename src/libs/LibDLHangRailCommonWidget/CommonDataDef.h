#ifndef COMMON_DATA_DEF_H
#define COMMON_DATA_DEF_H

// 存放公用的数据定义;


enum RobotBodyControlType
{
	RobotBody_GoForward = 0,					// 前进;
	RobotBody_GoUp,								// 上升;
	RobotBody_GoDown,							// 下降;
	RobotBody_GoBack,							// 后退;
	RobotBody_Stretch,							// 伸开;
	RobotBody_Shrink,							// 收缩;
	RobotBody_Restoration						// 复位;
};

enum RobotVideoControlType
{
	RobotVideo_Up = 0,							// 向上;
	RobotBody_Levorotation,						// 左旋;
	RobotBody_Dextrorotation,					// 右旋;
	RobotBody_Down,								// 向下;
	RobotBody_Lift_Up,							// 抬起;
	RobotBody_Lift_Down,						// 放下;
	RobotVideo_Restoration						// 复位;
};

#endif