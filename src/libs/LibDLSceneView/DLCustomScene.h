#ifndef DLCUSTOM_SCENE_H
#define DLCUSTOM_SCENE_H

/*************************************************************
*������DLCustomScene
*����:����������ʾ��ͼ��scene
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
class DLOverfittingDlg;				//��ϴ���
class ReadBgMapThread;
struct AreaInfo;
class QTime;
class LoadPixmapItem;

enum CUSTOM_SCENE_TYPE 
{
	DL_BROWSE_TYPE,							//�ͻ��˵ĵ�ͼ
	DL_COLLECT_MAP_TYPE,					//�ɼ����Ƶĵ�ͼ
	DL_COLLECT_EDIT_TYPE,					//�ɼ���ͼ
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
	void									AddPatrolPoint();													//�����������Ѳ���
	void									TaskBegin(WheelRobotTaskBegin task_begin);
	void									TaskEnd(QString task_uuid, QString task_time, int ret_code);
	void									SetCurrentPathIndex(int path_index);
	void									FlushCarItemPropertry(double pos_x, double pos_y, double angle);
	QRectF									GetPathRect();														//��ȡpath��rect
	void									SetOperateType(int type);											//���ò�������
	QUndoStack							*	UndoStack();														//
	DLOperator							*	GetOperate();														//����
	QPointF									GetCenter();														//��ȡ���ĵ�
	//////////////////////////////////////////////////////////////////////////
	void									add_normalPoint(QGraphicsItem *item);								//�����ͨ��
	void									add_normalLine(QGraphicsItem *item);								//�����ͨ��
	void									add_advancedArea(QGraphicsItem *item);								//���area
	void									add_advancedPoint(QGraphicsItem *item);								//���Ѳ���
	void									add_deviceArea(QGraphicsItem *item);
	void									add_advancedCurve(QGraphicsItem *item);								//��ӱ���������
	//////////////////////////////////////////////////////////////////////////
	int										GetRelocateRetVal();
	DLRobotItem							*	RobotItem();														//��ȡ�����˽ڵ�	
	void									SetRobotItemPropertry(double pos_x, double pos_y, double angle);
	//////////////////////////////////////////////////////////////////////////
// 	void									SetMutilPointItem(std::map<int, QPointF> );							//���ö����ͨ��
// 	void									GetNormalPosList(std::map<int, QPointF> &point);					//��ȡ�����ͨ��

	//void									CalcMutilPoint(const QRectF &rect);
//	void									SetMutilSelectRect(const QRectF &rect);								//����ѡ�еľ���
	QGraphicsItem						*	FindItem(QPointF pos, int type);									
	int										GetLandmarkId();
	int										GetBezierId();
	int										GetAdvancedAreaId();
	//////////////////////////////////////////////////////////////////////////
	void									remove_advancedpoint(int id);
	void									remove_advancedcurve(int id);
	void									removeall(bool isOnlyLoadLm = false);
	//////////////////////////////////////////////////////////////////////////���ص�ͼ�ļ����豸�ļ�
	void									save_json_map(QString file);											//����smap��ͼ�ļ�����
	void									load_json_map(QString file, bool isOnlyLoadLm = false);					//��ȡsmap��ͼ����
	void									connect_backstage_map_path();
	void									SetCutOutState(bool bIsCutoutState);									//���ü���״̬
	void									SetBgPixmap(const QString &strBgPixmap);
	void									SetIsShowBgPixmap(bool bIsShowPixmap = false);							//�����Ƿ���ʾ����ͼƬ

	void									ShowOverfittingDlg();    //��ʾ��ϴ���
	void									CancelOverfitting();		//ȡ�����
	void									StartOverfitting();			//��ʼ���
	void									ClearItemFocus();			//��սڵ��focus

	void									UpdateNormalBackground();

	void									ShowMessage(const QString &strMessage);
	void									HideMessage();
	qreal									GetResolution();

	void									AddAdvanceAreaItemMap(QGraphicsItem *pAdvanceAreaItem, QGraphicsItem *pItem);   //��ӵ��߼�����map��

signals:
	void									sig_scene_edit();
	void									sig_sceneRect_changed(const QRectF &rect);
	void									ChangeTypeSignal(const QString &strMessage, int iSelectType);				//���ͷ����仯���ź�
	void									SMAPChangedSignal();
	//void									ReadFinishedSignal(bool bIsRunning);						//��дsmap�ļ�����������ͼ�Ƿ���Ҫ�ϴ�
	void									ExitDrawStateSignal();										//�˳�����״̬
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
	void									DrawStateChangedSlot(bool);			//����״̬�ı�Ĳۺ���
	void									RelacationActionSlot();				//�ض�λ�ۺ���
	//////////////////////////////////////////////////////////////////////////
	void									slot_on_map_reset(bool);
	void									FlushRobotSlot(WheelRobotTaskCurrentPointStatus currentPointStatus);			//�ͻ��ˣ����»����˵�״̬
	void									FlushPlatformSlot(WheelRobotRealtimeStatus status);								//�ͻ��ˣ�����ƽ̨��״̬
	//////////////////////////////////////////////////////////////////////////
	void									slot_on_add_patrol_point();
	void									slot_on_view_detail_info();

	void									CreatePatrolTaskSlot();				//����Ѳ������
	void									ReadViewDeviceStateSlot();				//�鿴Ѳ���豸
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
	void									create_laserItem();														//��ͼ��item�Ĵ���������
	//void									hide_laserItem();
	//void									show_laserItem();
	void									set_laserItem_propertry(std::vector<QPointF> pointList);
	//DLLaserItem							*	laserItem();
	void									CreateRobotItem();														//���������˽ڵ�
	//void									hide_carItem();
	//void									show_carItem();
	void									create_pathItem();
	//void									hide_pathItem();
	//void									show_pathItem();
	//void									set_pathItem_propertry(int id);
	//DLPathItem							*	pathItem();
	QPointF									CalcTransform(double pos_x, double pos_y);								//������ת�������
	QPoint									calculate_window_point(QSize size);
	QGraphicsItem						*	find_advancedCurve(int start_id, int end_id);
	QGraphicsItem						*	find_advancedpoint(int id);
	//////////////////////////////////////////////////////////////////////////
	//�����ͼ���ݵ�item
	void									add_advancedLine(QGraphicsItem *item);								//�����
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
	void									onHideItemControlPoint();											//����ÿ���ڵ�Ŀ��Ƶ�
	void									onCatchItempoistion(QGraphicsItem *item, QPointF point);
	bool									check_smap(int &error_code, QString &err_desc);							//0�ɹ�, 1����, 2����
	bool									getBezier_curvature(const QPainterPath &bezier_path, int pointCount);
	//////////////////////////////////////////////////////////////////////////
	void									SetWorkPath(std::vector<QPoint> work_paths, std::vector<int> patrol_points, bool state);
	void									create_platformItem();
	double									cal_car_angle(QPointF startPos, QPointF c1Pos, QPointF c2Pos, QPointF endPos, double percentage);
	void									SetPlatformItemPropertry(double pos_x, double pos_y, double angle);
	
	QGraphicsItem						*	find_advancedCurve(int start_id, int end_id, bool &is_positive);		//������ͼ���item
	///////////////////////////////////////////////////////////////////////////////
	bool									IsBorderPoint(QGraphicsItem *pItem);									//�ж��Ƿ������ڶ˵�
	void									DrawCurPath();															//����ѡ���·����
	void									DrawCurPath(QPointF ptMovePos);											//ͬ�ϣ���mousemoveevent����ʹ��
	QPointF									CalcNeedPos(const QLineF line, QPointF &ptCurPos);						//���ݵ���ĵ������Ҫ�ĵ�
	void									UpdateLandmarkItemState(QPointF ptMovePos);								//��������ƶ��нڵ��״̬
	bool									MessageBox();															//��Ϣ����
	void									SendData2Core();														//�������ݵ�core
	void									InitAllArea();															//��ʼ������
	QGraphicsItem						*	find_bezier(int start_id, int end_id);
	QLineF									cal_bezier_path(QGraphicsItem *bezier_item, int start_index, int end_index);
	void									ConnectSock();															//����sock

	void									SetLandmarkItemMoved(bool bIsMoved);			//����Ѳ����Ƿ���ƶ�

	bool									IsExistOtherAdvanceArea(QGraphicsItem *pAdvaceAreaItem, QGraphicsItem *pItem);		//�Ƿ�����ڱ�ĸ߼�������


private:
	//////////////////////////////////////////////////////////////////////////��ͼ���ݵ��������		ÿһ��map��Ӧ��ͬ�����͵Ľڵ�
	Header									m_header;
	std::list<AdvancedDefine>				m_lstAdvancedObjectDefines;
	std::map<int, QPointF>					m_mapNormalPos;																//��ͨ��  ��ͼ����
	std::map<int, QGraphicsItem*>			m_mapNormalLines;
	std::map<int, QGraphicsItem*>			m_mapAdvancedPoints;														//��¼Ѳ���ڵ�
	std::map<int, QGraphicsItem*>			m_mapAdvancedLines;
	std::map<int, QGraphicsItem*>			m_mapAdvancedCurves;														//��¼Ѳ��·���ڵ�
	std::map<int, QGraphicsItem*>			m_mapAdvancedAreas;															//�߼�����ڵ�
	std::map<QGraphicsItem *, QVector<QGraphicsItem *>> m_mapAdvanceAreaItem2BesizerItems;								//�߼����򼰶�Ӧ��item
	std::map<int, QGraphicsItem*>			m_mapDeviceAreas;
//	DLMultiPointItem					*	m_pMutilNormalPointItem;													//������ͼ�ڵ�
	std::map<int, QPointF>					m_mapSelectNormalPos;														//ѡ�����ͨ�ڵ�

	/////////////////////////////////////////////////////////////////���Ƶ�ͼ�������ߵ�item
	DLLaserItem							*	m_pLaserItem;																//ʵʱ�任�ķ�Χ�Ľڵ�
	DLRobotItem							*	m_pRobotItem;																//�����˽ڵ�
	DLCoordinateItem					*	m_pWorldCoordinateItem;														//��������ķ���
	QGraphicsLineItem					*	m_pHLineItem;
	QGraphicsLineItem					*	m_pVLineItem;
	DLGridItem							*	m_pGridItem;																//����ڵ�
	DLPathItem							*	m_pPathItem;
	std::vector<QPoint>						m_vBackstagemapPaths;														//��̨��ͼ·������ͼ
	std::map<int, QLineF>					m_mapTopHLines;
	std::map<int, QLineF>					m_mapBottomHLines;
	std::map<int, QLineF>					m_mapLeftVLines;
	std::map<int, QLineF>					m_mapRightVLines;
	std::map<int, QLineF>					m_mapLandmarkHLines;
	std::map<int, QLineF>					m_mapLandmarkVLines;
	//////////////////////////////////////////////////////////////////////////
	QGraphicsItem						*	m_pCurrentItem;
	double									m_dCoordinateAngle;																//��¼������ͼѡװ�Ƕ�
	QPointF									m_ptTransformPoint;																//ƫ������
	int										m_iButtonType;																	//��¼����ק������ת
	bool									m_bIsPress;
	int										m_iLandmarkId;
	int										m_iBezierId;
	int										m_iAdvancedAreaId;
	QLineF									m_lineStart;																	//��ʼ��ֱ��
	QLineF									m_lineEnd;																		//������ֱ��
	DLOperator							*	m_pOperator;
	//////////////////////////////////////////////////////////////////////////
    QMutex									m_mutexLaser;
    std::vector<QPointF>					m_vLaserDatas;
    QMutex									m_mutexLocation;
	double									m_dRobotCurrentX;																//����������x
	double									m_dRobotCurrentY;																//����������y
	double									m_dRobotCurrentAngle;															//������ת��

	//////////////////////////////////////////////////////////////////////////
	MapReader							*	m_pReaderThread;																//��дmap���߳�
	MapReaderInfoWidget					*	m_pReaderInfoWidget;															//����
	LandMarkPropertyWidget				*	m_pLandmarkPropertyWidget;
	DeviceAreaPropertyWidget			*	m_pDeviceAreaPropertyWidget;
	BezierPropertyWidget				*	m_pBezierPropertyWidget;
	AdvancedAreaPropertyWidget			*	m_pAdvancedAreaPropertyWidget;

	//////////////////////////////////////////////////////////////////////////
	QGraphicsItem						*	m_pDrawPathItem;																//��¼���ƵĽڵ�
	QList<QPointF>							m_lstPaths;																		//��¼���Ƶ�·��
	bool									m_bIsDrawState;																	//�Ƿ��ǻ���״̬
	std::list< AdvancedPosition >			m_lstMapAdvancedPointList;
	QList<QPainterPath>						m_lstPainterPaths;																//����Χ�������ж��Ƿ�����������
	QList<QLineF>							m_lstLines;																		//��¼ÿ������Ŀ�ʼ��ͽ�����
	QList<QGraphicsItem *>					m_lstDrawAreaItems;																//�����������item
	QGraphicsItem						*	m_pCurSelectItem;
	CheckDeviceDlg						*	m_pCheckDeviceDlg;																//�鿴Ѳ���豸
	//////////////////////////////////////////////////////////////////////////
	QList<int>								m_lstSelectIndexs;																//ѡ�е�Ѳ����������
	DLDeviceViewer						*	m_pDeviceViewer;																//Ѳ���豸
	QPointF									m_ptStartPos;
	QGraphicsRectItem					*	m_pSelectItem;																	//ѡ�нڵ�
	DLPlatformItem						*	m_pPlatformItem;																//ƽ̨�ڵ㣬ɨ��
	bool									m_bPressDown;
	QMutex									m_mutexPatrolPoint;
	int										m_iTaskPatrolPoint;
	QString									m_strCurrentTaskUuid;
	bool									m_bIsStart;
	int										m_iCustomSceneType;																//scene������
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

	qreal									m_rResolution;																	//��¼��ͼ�ķֱ���
	QPointF									m_ptBgCenterPoint;																//����ͼ�����ĵ�
	qreal									m_rBgWidth;
	qreal									m_rBgHeight;
	QString									m_strLowImagePath;					//����ͼ
	QString									m_strMediumImagePath;						//ԭͼ
	QString									m_strHighImagePath;					//����ͼ
	//QString									m_strBgImagePath;																//����ͼƬ·��

	DLOverfittingDlg *m_pOverFittingDlg;																					//��ϵ���
	QGraphicsLineItem *m_pOverFittingLineItem;

	QVector<QGraphicsItem *> m_vOverFittingLineItem;   //��ϵ���
	QVector<QGraphicsItem *> m_vOverFittingPointsItems;  //��ϵĵ�

	//ReadBgMapThread *m_pReadBgMapThread;
	QTime m_time;

	LoadPixmapItem *m_pLoadPixmapItem;
};

#endif //DLCUSTOM_SCENE_H
