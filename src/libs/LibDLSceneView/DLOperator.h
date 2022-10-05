#ifndef DLOPERATOR_ALEXWEI_H
#define DLOPERATOR_ALEXWEI_H

#include <QObject>

class QGraphicsScene;
class QUndoStack;
class DLUndoCommandInterface;
class QGraphicsSceneMouseEvent;



class DLOperator:public QObject
{
	
public:

	DLOperator(QGraphicsScene *scene);

	void load(std::string string);
	void clean();

	void RobotRecation(QPointF ptCurPos, double dAngle);  //机器人重定位

public:
	void set_type(int type);
	int type(){
		return type_;
	}
	QUndoStack* undo_stack();

	DLUndoCommandInterface* getCmd(){
		return cmd_;
	}

	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:

	QGraphicsScene *scene_;

public:
	
	static const int TYPE_MUTILPLE_SELECT=-1;
	static const int TYPE_SELECT=0;						//选择
	static const int TYPE_DEL=1;						//删除
	static const int TYPE_MOVE=2;
	static const int TYPE_ADD_LANDMARK=3;				//添加巡检点
	static const int TYPE_ADD_EDGE=4;					//添加巡检路径
	static const int TYPE_RELOCATION=5;					//重定位
	static const int TYPE_ADD_DEVICE=6;	
	static const int TYPE_ADD_ARC=7;	
	static const int TYPE_ADD_PICTURE=8; 
	static const int TYPE_ADD_AREA=9;					//添加高级区域
	static const int TYPE_ADD_LINE = 10;				//添加线
	static const int TYPE_ADD_VIRTUAL_LINE = 14;		//添加虚线
	static const int TYPE_ADD_FORBBIDON_LINE = 15;		//添加禁止线
	static const int TYPE_ADD_DEVAREA = 11;				//添加设备区域
	static const int TYPE_ADD_STATION = 12;				//添加站点区域
	static const int TYPE_COORDINATE_TRANSFORM = 13;	//坐标变换
	static const int TYPE_OVERFITTING_SELECT_LINE = 16;		//拟合线
	static const int TYPE_OVERFITTING_SELECT_POINT = 17;	//拟合点
private:
	void add_cmd(DLUndoCommandInterface **cmd);

	int type_;
	QUndoStack *undo_stack_; //保存操作的命令
	DLUndoCommandInterface *cmd_; //当前操作的指令
	

};

#endif