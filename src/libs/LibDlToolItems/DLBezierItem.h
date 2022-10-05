#ifndef DLBEZIERITEM_ALEXWEI_20180420_H
#define DLBEZIERITEM_ALEXWEI_20180420_H

#pragma execution_character_set("utf-8")

#include "DLShapeItem.h"
#include <string>
#include <QDialog>

class DLBezierItem;
class DLShapeItem;
class BezierPropertyWidget;
class QLabel;
class QPushButton;
class QLineEdit;
class QComboBox;
class QCheckBox;
class QGroupBox;

enum LinePassingState
{
	AllowPassing,			//����ͨ��
	StartToStopPassing,		//��㵽�յ�ɹ�
	StopToStartPassing,		//�յ㵽���ɹ�
	ForbidPassing,			//��ֹͨ��
	Unnormal,
};

class BezierPropertyWidget : public QDialog
{
	Q_OBJECT
public:
	BezierPropertyWidget(QWidget *parent = NULL);
	~BezierPropertyWidget();

	void keyPressEvent(QKeyEvent *event);
	void setBezierProperty();
	void setParent(DLBezierItem *item);

	//��������Ľӿ�
	void SetBatchSet(bool bIsBatchSet);										//�Ƿ�����������
	void SetBatchSetParents(QList<DLBezierItem *> lstParentItems);			//������������ĸ��ڵ�
	void InitBatchProperty();												//��ʼ���������������ֵ

protected:
	virtual void showEvent(QShowEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);

private slots:
	void slot_on_set_bezier_info();
	void DefaultValBtnSlot();						//Ĭ��ֵ��ť�Ĳۺ���
	void MedianValBtnSlot();						//�м�ֵ��ť�Ĳۺ���
	void MinValBtnSlot();							//��Сֵ��ť�Ĳۺ���

	void PositiveStateChangeSlot(int iState);		//״̬�����ı�
	void NagetiveStateChangeSlot(int iState);

private:
	void ClearContent();							//��ս��������

private:
	QGroupBox * bezier_groupBox_;

	QGroupBox * bezier_base_pro_groupBox_;
	QLabel * start_index_id_label_;								//���ID
	QLineEdit *start_index_id_lineEdit_;						//���ID�༭��
	QLabel * end_index_id_label_;								//�յ�ID
	QLineEdit *end_index_id_lineEdit_;							//�յ�ID�༭��

	QLabel* direction_type_label_;								//����
	QComboBox *direction_type_combobox_;						//����combobox
	QLabel* widget_label_;										//Ȩ��
	QLineEdit *weight_lineEdit_;								//Ȩ��edit

	QCheckBox *path_passing_checkBox_;							//�Ƿ�ͨ��
	QCheckBox *m_rotate_checkBox;								//�Ƿ������ת

	QCheckBox *m_pUltrasonicChx{nullptr};								//������
	QCheckBox *m_pFallArrestChx{nullptr};								//������

	QLineEdit *LineEditObstacleRange;

	QPushButton *m_pDefaultValBtn;									//Ĭ��ֵ��ť
	QPushButton *m_pMedianValBtn;									//�м�ֵ��ť
	QPushButton *m_pMinValBtn;									//��Сֵ��ť

	
	QGroupBox *positive_direction_gropBox_;						//ȥ
	QLineEdit *positive_max_speed_lineEdit_;
	QLineEdit *positive_max_acc_speed_lineEdit_;
	QLineEdit *positive_max_angular_speed_lineEdit_;
	QLineEdit *positive_max_acc_angular_speed_lineEdit_;
	QLineEdit *positive_block_distance_lineEdit_;

	QGroupBox *nagetive_direction_gropBox_;
	QLineEdit *nagetive_max_speed_lineEdit_;
	QLineEdit *nagetive_max_acc_speed_lineEdit_;
	QLineEdit *nagetive_max_angular_speed_lineEdit_;
	QLineEdit *nagetive_max_acc_angular_speed_lineEdit_;
	QLineEdit *nagetive_block_distance_lineEdit_;

	QPushButton *save_info_buttom_;

private:
	DLBezierItem * parent_;
	bool m_bIsBatchSet;						//�Ƿ�����������
	bool m_bIsPressed;
	QPoint m_ptStartPos;
	QList<DLBezierItem *> m_lstParentItems;

	QCheckBox *m_pPositiveCheckBox;  //�����checkbox
	QCheckBox *m_pNagetiveCheckBox;  //�����checkbox
};





class DLBezierItem:public DLShapeItem
{
	Q_OBJECT
	
	
public:
	enum { Type = HShape::BEZIERITEM };
	enum
	{
		WEIGHT_PRO = 0,
		MAXSPEED_PRO,
		MAXACC_PRO,
		MAXROT_PRO,
		MAXROTACC_PRO,
		BLOCKDIST_PRO = 5,
		PASSING_PRO,
		ALLOWSPIN_PRO,				//�Ƿ�ͬ��
		ULTRASONIC,					//������
		FALL_ARREST,					//������
		ADVANCE_ARE_PRO,				//�߼�����
	};
	int type() const
	{
		return Type;
	}
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className);
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className,std::string instanceName);
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,int id=0,std::string name="");
	~DLBezierItem();

	void create_item(DLShapeItem *start,DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string name);		//����Ѳ��㴴������������
	double path_weight();							
	void setPassing(bool isPassing = true);
	void SetIsAdvanceArea(bool bIsAdvanceArea = false); //�����Ƿ��Ǹ߼�����

protected:
	virtual QRectF boundingRect() const;
	virtual QPainterPath shape() const;
	virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Bezier pro);							//�ⲿ��������

signals:
	void sig_pro(HShape::Bezier pro);							//���Ա仯֪ͨ�ⲿ
	void SelectItemChangedSignal(QGraphicsItem *pItem);				

public:
	void update_bezier();
	void update_shape();										//����item����
	void set_control_point_visible(bool visible);				//���ÿ��Ƶ��Ƿ���ʾ
	void SetShowAuxLine(bool bIsShowAuxLine);					//�����Ƿ���ʾ����
	void hide_control_point();									//���ؿ��Ƶ�
	void set_child_selectable(bool state);						
	LinePassingState getLinePassingState();
	QPointF start_pos();										//��ȡÿ�����Ƶ������
	QPointF c1_pos();
	QPointF c2_pos();
	QPointF end_pos();

	void set_dir_pro(bool state){
		double_dir_ = state;
	}

	bool dir_pro() {
		return double_dir_;
	}

	bool isPassing() {
		return m_bIsPassing;
	}

	int id()
	{//��ȡid
		return id_;
	}

	std::string className()
	{//"BezierPath"
		return className_;
	}

	std::string instanceName(){
		return instanceName_;
	}

	int start_id(){
		return start_landmark_->id();
	}

	int end_id(){
		return end_landmark_->id();
	}
	void setPropertyList(std::vector<Property> PropertyVec);


	std::vector<Property>& getPropertyList(){
		return m_vGoPropertys;
	}

	void setNagetivePropertyList(std::vector<Property> PropertyVec);
	

	std::vector<Property>& getNagetivePropertyList() {
		return m_vBackPropertys;
	}


	DLShapeItem* start_item(){
		return start_landmark_;
	}

	DLShapeItem* end_item(){
		return end_landmark_;
	}

	DLShapeItem* control_item1(){
		return c1_item_;
	}
	
	DLShapeItem* control_item2(){
		return c2_item_;
	}

	HShape::Bezier get_pro(){
		return pro_;
	}


private:
	//�ӿڲ�����
	void set_bezier_pro(HShape::Bezier pro);		//
	void set_control_pro(HShape::Bezier pro);

private:
	HShape::Bezier cal_pro();						//���Ƶ���ƶ�������״�ı仯

private:
	HShape::Bezier pro_;							//��¼���������ߵ�����
	DLShapeItem *start_item_;						//�ĸ����Ƶ�
	DLShapeItem *c1_item_;
	DLShapeItem *c2_item_;
	DLShapeItem *end_item_;

	DLShapeItem *start_landmark_;					//��ʼ��Ѳ���
	DLShapeItem *end_landmark_;						//������Ѳ���

private:
	bool m_bIsPassing;								//�Ƿ�ͨ��
	bool m_bIsAdvanceArea{false};							//�Ƿ��Ǹ߼�����
	bool m_bIsShowAuxLine;							//�Ƿ�������ߵ�
	bool double_dir_;
	bool childe_selectable_;
	int id_;										//���������ߵ�id
	std::string className_;
	std::string instanceName_;
	int start_id_;									//���������߿�ʼ��id
	int end_id_;									//���������߽�����id
	std::vector<Property> m_vGoPropertys;			//ȥ�����б�
	std::vector<Property> m_vBackPropertys;	//�ص������б�
};


#endif