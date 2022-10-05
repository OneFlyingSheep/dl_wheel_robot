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

	void RobotRecation(QPointF ptCurPos, double dAngle);  //�������ض�λ

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
	static const int TYPE_SELECT=0;						//ѡ��
	static const int TYPE_DEL=1;						//ɾ��
	static const int TYPE_MOVE=2;
	static const int TYPE_ADD_LANDMARK=3;				//���Ѳ���
	static const int TYPE_ADD_EDGE=4;					//���Ѳ��·��
	static const int TYPE_RELOCATION=5;					//�ض�λ
	static const int TYPE_ADD_DEVICE=6;	
	static const int TYPE_ADD_ARC=7;	
	static const int TYPE_ADD_PICTURE=8; 
	static const int TYPE_ADD_AREA=9;					//��Ӹ߼�����
	static const int TYPE_ADD_LINE = 10;				//�����
	static const int TYPE_ADD_VIRTUAL_LINE = 14;		//�������
	static const int TYPE_ADD_FORBBIDON_LINE = 15;		//��ӽ�ֹ��
	static const int TYPE_ADD_DEVAREA = 11;				//����豸����
	static const int TYPE_ADD_STATION = 12;				//���վ������
	static const int TYPE_COORDINATE_TRANSFORM = 13;	//����任
	static const int TYPE_OVERFITTING_SELECT_LINE = 16;		//�����
	static const int TYPE_OVERFITTING_SELECT_POINT = 17;	//��ϵ�
private:
	void add_cmd(DLUndoCommandInterface **cmd);

	int type_;
	QUndoStack *undo_stack_; //�������������
	DLUndoCommandInterface *cmd_; //��ǰ������ָ��
	

};

#endif