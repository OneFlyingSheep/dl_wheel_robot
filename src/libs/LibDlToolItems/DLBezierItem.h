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
	AllowPassing,			//允许通过
	StartToStopPassing,		//起点到终点可过
	StopToStartPassing,		//终点到起点可过
	ForbidPassing,			//禁止通过
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

	//批量处理的接口
	void SetBatchSet(bool bIsBatchSet);										//是否是批量处理
	void SetBatchSetParents(QList<DLBezierItem *> lstParentItems);			//设置批量处理的父节点
	void InitBatchProperty();												//初始化批量处理的属性值

protected:
	virtual void showEvent(QShowEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);

private slots:
	void slot_on_set_bezier_info();
	void DefaultValBtnSlot();						//默认值按钮的槽函数
	void MedianValBtnSlot();						//中间值按钮的槽函数
	void MinValBtnSlot();							//最小值按钮的槽函数

	void PositiveStateChangeSlot(int iState);		//状态发生改变
	void NagetiveStateChangeSlot(int iState);

private:
	void ClearContent();							//清空界面的内容

private:
	QGroupBox * bezier_groupBox_;

	QGroupBox * bezier_base_pro_groupBox_;
	QLabel * start_index_id_label_;								//起点ID
	QLineEdit *start_index_id_lineEdit_;						//起点ID编辑框
	QLabel * end_index_id_label_;								//终点ID
	QLineEdit *end_index_id_lineEdit_;							//终点ID编辑框

	QLabel* direction_type_label_;								//方向
	QComboBox *direction_type_combobox_;						//方向combobox
	QLabel* widget_label_;										//权重
	QLineEdit *weight_lineEdit_;								//权重edit

	QCheckBox *path_passing_checkBox_;							//是否通行
	QCheckBox *m_rotate_checkBox;								//是否可以旋转

	QCheckBox *m_pUltrasonicChx{nullptr};								//超声波
	QCheckBox *m_pFallArrestChx{nullptr};								//防跌落

	QLineEdit *LineEditObstacleRange;

	QPushButton *m_pDefaultValBtn;									//默认值按钮
	QPushButton *m_pMedianValBtn;									//中间值按钮
	QPushButton *m_pMinValBtn;									//最小值按钮

	
	QGroupBox *positive_direction_gropBox_;						//去
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
	bool m_bIsBatchSet;						//是否是批量处理
	bool m_bIsPressed;
	QPoint m_ptStartPos;
	QList<DLBezierItem *> m_lstParentItems;

	QCheckBox *m_pPositiveCheckBox;  //正向的checkbox
	QCheckBox *m_pNagetiveCheckBox;  //逆向的checkbox
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
		ALLOWSPIN_PRO,				//是否同行
		ULTRASONIC,					//超声波
		FALL_ARREST,					//防跌落
		ADVANCE_ARE_PRO,				//高级区域
	};
	int type() const
	{
		return Type;
	}
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className);
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className,std::string instanceName);
	DLBezierItem(DLShapeItem *start, DLShapeItem *end,int id=0,std::string name="");
	~DLBezierItem();

	void create_item(DLShapeItem *start,DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string name);		//根据巡检点创建贝塞尔曲线
	double path_weight();							
	void setPassing(bool isPassing = true);
	void SetIsAdvanceArea(bool bIsAdvanceArea = false); //设置是否是高级区域

protected:
	virtual QRectF boundingRect() const;
	virtual QPainterPath shape() const;
	virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Bezier pro);							//外部设置属性

signals:
	void sig_pro(HShape::Bezier pro);							//属性变化通知外部
	void SelectItemChangedSignal(QGraphicsItem *pItem);				

public:
	void update_bezier();
	void update_shape();										//供子item调用
	void set_control_point_visible(bool visible);				//设置控制点是否显示
	void SetShowAuxLine(bool bIsShowAuxLine);					//设置是否显示虚线
	void hide_control_point();									//隐藏控制点
	void set_child_selectable(bool state);						
	LinePassingState getLinePassingState();
	QPointF start_pos();										//获取每个控制点的坐标
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
	{//获取id
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
	//接口不开放
	void set_bezier_pro(HShape::Bezier pro);		//
	void set_control_pro(HShape::Bezier pro);

private:
	HShape::Bezier cal_pro();						//控制点的移动导致形状的变化

private:
	HShape::Bezier pro_;							//记录贝塞尔曲线的属性
	DLShapeItem *start_item_;						//四个控制点
	DLShapeItem *c1_item_;
	DLShapeItem *c2_item_;
	DLShapeItem *end_item_;

	DLShapeItem *start_landmark_;					//开始的巡检点
	DLShapeItem *end_landmark_;						//结束的巡检点

private:
	bool m_bIsPassing;								//是否通行
	bool m_bIsAdvanceArea{false};							//是否是高级区域
	bool m_bIsShowAuxLine;							//是否绘制虚线的
	bool double_dir_;
	bool childe_selectable_;
	int id_;										//贝塞尔曲线的id
	std::string className_;
	std::string instanceName_;
	int start_id_;									//贝塞尔曲线开始的id
	int end_id_;									//贝塞尔曲线结束的id
	std::vector<Property> m_vGoPropertys;			//去属性列表
	std::vector<Property> m_vBackPropertys;	//回的属性列表
};


#endif