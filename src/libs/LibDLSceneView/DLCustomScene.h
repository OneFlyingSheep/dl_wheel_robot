#ifndef DLCUSTOM_SCENE_H
#define DLCUSTOM_SCENE_H

/*************************************************************
*类名：DLCustomScene
*功能:该类用于显示地图的scene
**************************************************************/

#include <QGraphicsScene>
#include "LibMapReader/MapData.h"
#include "LibDlToolItems/DLShapeItem.h"
#include <QWidget>
#include <QLineF>
#include <QMutex>
#include <QDialog>
#include "common/DLWheelRobotGlobalDef.hpp"

class DLOperator;
class QUndoStack;
class DLMultiPointItem;
class DLCoordinateItem;
class QGraphicsLineItem;
class MapReader;
class QLabel;
class DLGridItem;
class MapReaderInfoWidget;
class DLRobotItem;
class DLPointItem;
class DLPathItem;
class DLLaserItem;
class LandMarkPropertyWidget;
class DeviceAreaPropertyWidget;
class BezierPropertyWidget;
class AdvancedAreaPropertyWidget;
class DLPlatformItem;
class DLDeviceViewer;
class DLPointInfoWidget;
class DLViewGrabDlg;
class QGraphicsPixmapItem;
class DLPixmapItem;
class QListWidget;
class DLOverfittingDlg;				//拟合窗口
class ReadBgMapThread;
struct AreaInfo;
class QTime;
class LoadPixmapItem;

enum CUSTOM_SCENE_TYPE 
{
	DL_BROWSE_TYPE,							//客户端的地图
	DL_COLLECT_MAP_TYPE,					//采集控制的地图
	DL_COLLECT_EDIT_TYPE,					//采集地图
	DL_SCENE_NUM
};

class CheckDeviceDlg :public QDialog
{
	Q_OBJECT
public:
	CheckDeviceDlg(QWidget *parent = NULL);
	~CheckDeviceDlg();

	void LoadData(const QStringList &lstEquipmentInfo);

private:
	QListWidget *m_pContextListWgt;
};



class DLCustomScene : public QGraphicsScene
{
	Q_OBJECT
public:
	DLCustomScene(int iSceneType = DL_BROWSE_TYPE, QObject *parent = 0);
	~DLCustomScene();

	bool									GetItemIsChanged();
	void									AddPatrolPoint();													//定制需求，添加巡检点
	void									TaskBegin(WheelRobotTaskBegin task_begin);
	void									TaskEnd(QString task_uuid, QString task_time, int ret_code);
	void									SetCurrentPathIndex(int path_index);
	void									FlushCarItemPropertry(double pos_x, double pos_y, double angle);
	QRectF									GetPathRect();														//获取path的rect
	void									SetOperateType(int type);											//设置操作类型
	QUndoStack							*	UndoStack();														//
	DLOperator							*	GetOperate();														//操作
	QPointF									GetCenter();														//获取中心点
	//////////////////////////////////////////////////////////////////////////
	void									add_normalPoint(QGraphicsItem *item);								//添加普通点
	void									add_normalLine(QGraphicsItem *item);								//添加普通线
	void									add_advancedArea(QGraphicsItem *item);								//添加area
	void									add_advancedPoint(QGraphicsItem *item);								//添加巡检点
	void									add_deviceArea(QGraphicsItem *item);
	void									add_advancedCurve(QGraphicsItem *item);								//添加贝塞尔曲线
	//////////////////////////////////////////////////////////////////////////
	int										GetRelocateRetVal();
	DLRobotItem							*	RobotItem();														//获取机器人节点	
	void									SetRobotItemPropertry(double pos_x, double pos_y, double angle);
	//////////////////////////////////////////////////////////////////////////
// 	void									SetMutilPointItem(std::map<int, QPointF> );							//设置多个普通点
// 	void									GetNormalPosList(std::map<int, QPointF> &point);					//获取多个普通点

	//void									CalcMutilPoint(const QRectF &rect);
//	void									SetMutilSelectRect(const QRectF &rect);								//设置选中的矩形
	QGraphicsItem						*	FindItem(QPointF pos, int type);									
	int										GetLandmarkId();
	int										GetBezierId();
	int										GetAdvancedAreaId();
	//////////////////////////////////////////////////////////////////////////
	void									remove_advancedpoint(int id);
	void									remove_advancedcurve(int id);
	void									removeall(bool isOnlyLoadLm = false);
	//////////////////////////////////////////////////////////////////////////加载地图文件和设备文件
	void									save_json_map(QString file);											//保存smap地图文件数据
	void									load_json_map(QString file, bool isOnlyLoadLm = false);					//读取smap地图数据
	void									connect_backstage_map_path();
	void									SetCutOutState(bool bIsCutoutState);									//设置剪切状态
	void									SetBgPixmap(const QString &strBgPixmap);
	void									SetIsShowBgPixmap(bool bIsShowPixmap = false);							//设置是否显示背景图片

	void									ShowOverfittingDlg();    //显示拟合窗口
	void									CancelOverfitting();		//取消拟合
	void									StartOverfitting();			//开始拟合
	void									ClearItemFocus();			//清空节点的focus

	void									UpdateNormalBackground();

	void									ShowMessage(const QString &strMessage);
	void									HideMessage();
	qreal									GetResolution();

	void									AddAdvanceAreaItemMap(QGraphicsItem *pAdvanceAreaItem, QGraphicsItem *pItem);   //添加到高级区域map中

signals:
	void									sig_scene_edit();
	void									sig_sceneRect_changed(const QRectF &rect);
	void									ChangeTypeSignal(const QString &strMessage, int iSelectType);				//类型发生变化的信号
	void									SMAPChangedSignal();
	//void									ReadFinishedSignal(bool bIsRunning);						//读写smap文件结束，检查地图是否需要上传
	void									ExitDrawStateSignal();										//退出绘制状态
	//////////////////////////////////////////////////////////////////////////
	void									SigFlushPlateform(WheelRobotRealtimeStatus status);
	void									SigFlushPath(WheelRobotTaskCurrentPointStatus status);
	void									signalRobotModeStateCallBack(WheelRobotSwitchRunningStatus);
	//////////////////////////////////////////////////////////////////////////
	void									sig_flush_viewPort(QPointF);



private slots:
	void									ReadFinishedSlot(int, bool isOnlyLoadLm);
    void									slot_on_robot_move();
    void									UpdateRobotItem();
    void									update_laserItem();
	void									slot_on_deviceAre_moveable();
	void									slot_on_deviceArea_dismoveable();
	void									slot_on_deviceAre_loadMap();
	void									slot_on_hide_auxLine();
	void									slot_on_show_auxLine();
	void									slot_on_back_home();
	void									DrawStateChangedSlot(bool);			//绘制状态改变的槽函数
	void									RelacationActionSlot();				//重定位槽函数
	//////////////////////////////////////////////////////////////////////////
	void									slot_on_map_reset(bool);
	void									FlushRobotSlot(WheelRobotTaskCurrentPointStatus currentPointStatus);			//客户端，更新机器人的状态
	void									FlushPlatformSlot(WheelRobotRealtimeStatus status);								//客户端，更新平台的状态
	//////////////////////////////////////////////////////////////////////////
	void									slot_on_add_patrol_point();
	void									slot_on_view_detail_info();

	void									CreatePatrolTaskSlot();				//生成巡检任务
	void									ReadViewDeviceStateSlot();				//查看巡检设备
	//void									SendAreaPointsSlot(QPointF ptMinPoint, QPointF ptMaxPoint);

protected:
	void									mousePressEvent(QGraphicsSceneMouseEvent *event);
	void									mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void									mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void									mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event);
	void									keyPressEvent(QKeyEvent *event);
	void									contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent);

private:
	void									_save_json_map();
	void									_load_json_map(bool isOnlyLoadLm);
	void									setRelocateRetVal(int retVal);
	void									create_laserItem();														//地图里item的创建、设置
	//void									hide_laserItem();
	//void									show_laserItem();
	void									set_laserItem_propertry(std::vector<QPointF> pointList);
	//DLLaserItem							*	laserItem();
	void									CreateRobotItem();														//创建机器人节点
	//void									hide_carItem();
	//void									show_carItem();
	void									create_pathItem();
	//void									hide_pathItem();
	//void									show_pathItem();
	//void									set_pathItem_propertry(int id);
	//DLPathItem							*	pathItem();
	QPointF									CalcTransform(double pos_x, double pos_y);								//计算旋转后的坐标
	QPoint									calculate_window_point(QSize size);
	QGraphicsItem						*	find_advancedCurve(int start_id, int end_id);
	QGraphicsItem						*	find_advancedpoint(int id);
	//////////////////////////////////////////////////////////////////////////
	//加入地图数据的item
	void									add_advancedLine(QGraphicsItem *item);								//添加线
	void									add_no_normalPoint(QGraphicsItem *item);
	void									add_no_normalLine(QGraphicsItem *item);
	void									add_no_advancedPoint(QGraphicsItem *item);
	void									add_no_advancedLine(QGraphicsItem *item);
	void									add_no_advancedCurve(QGraphicsItem *item);
	void									add_no_advancedArea(QGraphicsItem *item);

	void									add_no_deviceArea(QGraphicsItem *item);
	QPoint									cal_bezier_index(AdvancedCurve bezier);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void									InitLandmarkId();
	void									InitBezierId();
	void									InitAdvancedAreaId();

	//////////////////////////////////////////////////////////////////////////
	void									cal_nearset_line(QGraphicsItem *item, int &index, int &part, int &index_v, int &part_v);
	void									onHideItemControlPoint();											//隐藏每个节点的控制点
	void									onCatchItempoistion(QGraphicsItem *item, QPointF point);
	bool									check_smap(int &error_code, QString &err_desc);							//0成功, 1警告, 2错误
	bool									getBezier_curvature(const QPainterPath &bezier_path, int pointCount);
	//////////////////////////////////////////////////////////////////////////
	void									SetWorkPath(std::vector<QPoint> work_paths, std::vector<int> patrol_points, bool state);
	void									create_platformItem();
	double									cal_car_angle(QPointF startPos, QPointF c1Pos, QPointF c2Pos, QPointF endPos, double percentage);
	void									SetPlatformItemPropertry(double pos_x, double pos_y, double angle);
	
	QGraphicsItem						*	find_advancedCurve(int start_id, int end_id, bool &is_positive);		//检索地图里的item
	///////////////////////////////////////////////////////////////////////////////
	bool									IsBorderPoint(QGraphicsItem *pItem);									//判断是否是相邻端点
	void									DrawCurPath();															//绘制选择的路径点
	void									DrawCurPath(QPointF ptMovePos);											//同上，在mousemoveevent里面使用
	QPointF									CalcNeedPos(const QLineF line, QPointF &ptCurPos);						//根据点击的点计算需要的点
	void									UpdateLandmarkItemState(QPointF ptMovePos);								//更新鼠标移动中节点的状态
	bool									MessageBox();															//消息弹框
	void									SendData2Core();														//发送数据到core
	void									InitAllArea();															//初始化区域
	QGraphicsItem						*	find_bezier(int start_id, int end_id);
	QLineF									cal_bezier_path(QGraphicsItem *bezier_item, int start_index, int end_index);
	void									ConnectSock();															//连接sock

	void									SetLandmarkItemMoved(bool bIsMoved);			//设置巡检点是否可移动

	bool									IsExistOtherAdvanceArea(QGraphicsItem *pAdvaceAreaItem, QGraphicsItem *pItem);		//是否存在于别的高级区域中


private:
	//////////////////////////////////////////////////////////////////////////地图数据的相关属性		每一个map对应不同的类型的节点
	Header									m_header;
	std::list<AdvancedDefine>				m_lstAdvancedObjectDefines;
	std::map<int, QPointF>					m_mapNormalPos;																//普通点  地图背景
	std::map<int, QGraphicsItem*>			m_mapNormalLines;
	std::map<int, QGraphicsItem*>			m_mapAdvancedPoints;														//记录巡检点节点
	std::map<int, QGraphicsItem*>			m_mapAdvancedLines;
	std::map<int, QGraphicsItem*>			m_mapAdvancedCurves;														//记录巡检路径节点
	std::map<int, QGraphicsItem*>			m_mapAdvancedAreas;															//高级区域节点
	std::map<QGraphicsItem *, QVector<QGraphicsItem *>> m_mapAdvanceAreaItem2BesizerItems;								//高级区域及对应的item
	std::map<int, QGraphicsItem*>			m_mapDeviceAreas;
//	DLMultiPointItem					*	m_pMutilNormalPointItem;													//背景地图节点
	std::map<int, QPointF>					m_mapSelectNormalPos;														//选择的普通节点

	/////////////////////////////////////////////////////////////////绘制地图的网格线的item
	DLLaserItem							*	m_pLaserItem;																//实时变换的范围的节点
	DLRobotItem							*	m_pRobotItem;																//机器人节点
	DLCoordinateItem					*	m_pWorldCoordinateItem;														//绘制坐标的方向
	QGraphicsLineItem					*	m_pHLineItem;
	QGraphicsLineItem					*	m_pVLineItem;
	DLGridItem							*	m_pGridItem;																//网格节点
	DLPathItem							*	m_pPathItem;
	std::vector<QPoint>						m_vBackstagemapPaths;														//后台地图路径连接图
	std::map<int, QLineF>					m_mapTopHLines;
	std::map<int, QLineF>					m_mapBottomHLines;
	std::map<int, QLineF>					m_mapLeftVLines;
	std::map<int, QLineF>					m_mapRightVLines;
	std::map<int, QLineF>					m_mapLandmarkHLines;
	std::map<int, QLineF>					m_mapLandmarkVLines;
	//////////////////////////////////////////////////////////////////////////
	QGraphicsItem						*	m_pCurrentItem;
	double									m_dCoordinateAngle;																//记录背景地图选装角度
	QPointF									m_ptTransformPoint;																//偏移坐标
	int										m_iButtonType;																	//记录是拖拽还是旋转
	bool									m_bIsPress;
	int										m_iLandmarkId;
	int										m_iBezierId;
	int										m_iAdvancedAreaId;
	QLineF									m_lineStart;																	//开始的直线
	QLineF									m_lineEnd;																		//结束的直线
	DLOperator							*	m_pOperator;
	//////////////////////////////////////////////////////////////////////////
    QMutex									m_mutexLaser;
    std::vector<QPointF>					m_vLaserDatas;
    QMutex									m_mutexLocation;
	double									m_dRobotCurrentX;																//机器人坐标x
	double									m_dRobotCurrentY;																//机器人坐标y
	double									m_dRobotCurrentAngle;															//机器人转角

	//////////////////////////////////////////////////////////////////////////
	MapReader							*	m_pReaderThread;																//读写map的线程
	MapReaderInfoWidget					*	m_pReaderInfoWidget;															//弹窗
	LandMarkPropertyWidget				*	m_pLandmarkPropertyWidget;
	DeviceAreaPropertyWidget			*	m_pDeviceAreaPropertyWidget;
	BezierPropertyWidget				*	m_pBezierPropertyWidget;
	AdvancedAreaPropertyWidget			*	m_pAdvancedAreaPropertyWidget;

	//////////////////////////////////////////////////////////////////////////
	QGraphicsItem						*	m_pDrawPathItem;																//记录绘制的节点
	QList<QPointF>							m_lstPaths;																		//记录绘制的路劲
	bool									m_bIsDrawState;																	//是否是绘制状态
	std::list< AdvancedPosition >			m_lstMapAdvancedPointList;
	QList<QPainterPath>						m_lstPainterPaths;																//区域范围，用于判断是否是吸附区域
	QList<QLineF>							m_lstLines;																		//记录每个区域的开始点和结束点
	QList<QGraphicsItem *>					m_lstDrawAreaItems;																//画绘制区域的item
	QGraphicsItem						*	m_pCurSelectItem;
	CheckDeviceDlg						*	m_pCheckDeviceDlg;																//查看巡检设备
	//////////////////////////////////////////////////////////////////////////
	QList<int>								m_lstSelectIndexs;																//选中的巡检点的索引号
	DLDeviceViewer						*	m_pDeviceViewer;																//巡检设备
	QPointF									m_ptStartPos;
	QGraphicsRectItem					*	m_pSelectItem;																	//选中节点
	DLPlatformItem						*	m_pPlatformItem;																//平台节点，扫描
	bool									m_bPressDown;
	QMutex									m_mutexPatrolPoint;
	int										m_iTaskPatrolPoint;
	QString									m_strCurrentTaskUuid;
	bool									m_bIsStart;
	int										m_iCustomSceneType;																//scene的类型
	//////////////////////////////////////////////////////////////////////////
	DLPointInfoWidget					*	m_pPointInfoViewer;
	QString									m_strCurrentLandmarkId;
	bool									m_bIsCutoutState;
	QPointF									m_ptStartPoint;
	QPointF									m_ptEndPoint;
	//QString									m_strBgPixmap;
	//DLPixmapItem						*	m_pBgPixmapItem;

	//QRectF									m_rectMutilArea;
	DLViewGrabDlg						*	m_pViewGrabDlg;

	qreal									m_rResolution;																	//记录地图的分辨率
	QPointF									m_ptBgCenterPoint;																//背景图的中心点
	qreal									m_rBgWidth;
	qreal									m_rBgHeight;
	QString									m_strLowImagePath;					//缩略图
	QString									m_strMediumImagePath;						//原图
	QString									m_strHighImagePath;					//高清图
	//QString									m_strBgImagePath;																//背景图片路径

	DLOverfittingDlg *m_pOverFittingDlg;																					//拟合弹窗
	QGraphicsLineItem *m_pOverFittingLineItem;

	QVector<QGraphicsItem *> m_vOverFittingLineItem;   //拟合的线
	QVector<QGraphicsItem *> m_vOverFittingPointsItems;  //拟合的点

	//ReadBgMapThread *m_pReadBgMapThread;
	QTime m_time;

	LoadPixmapItem *m_pLoadPixmapItem;
};

#endif //DLCUSTOM_SCENE_H
