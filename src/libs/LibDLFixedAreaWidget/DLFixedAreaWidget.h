#ifndef DLFIXEDAREAWIDGET_ALEXWEI_20181009
#define DLFIXEDAREAWIDGET_ALEXWEI_20181009

#include <QGraphicsView>
#include <QFrame>
#include <QGraphicsScene>
#include "LibMapReader/MapData.h"
#include "LibDlToolItems/DLShapeItem.h"
#include <QWidget>
#include <QLineF>



class QWidget;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QLabel;
class QPushButton;
class QComboBox;
//class DLFixedAreaScene;
class DLMultiPointItem;
class DLCoordinateItem;
class QGraphicsLineItem;
class MapReader;
class QLabel;
class DLGridItem;
class MapReaderInfoWidget;
class DLPointItem;
class LandMarkPropertyWidget;
class DeviceAreaPropertyWidget;
class BezierPropertyWidget;
class AdvancedAreaPropertyWidget;
class DLFixedAreaView;
//class DLFixedAreaScene;
class QGraphicsRectItem;
class DataTransfer;
class DLCustomScene;


struct Fixed_ScaleInfo {
	QPointF point_;
	bool flag_;	// 1代表5的倍数，2代表10的倍数
};



class DLFixedAreaWidget : public QFrame
{
	Q_OBJECT
public:
	DLFixedAreaWidget();
	~DLFixedAreaWidget();

private:
	DLFixedAreaView * view_;
	//DLFixedAreaScene *scene_;
	DLCustomScene *m_pScene;
};


//////////////



class DLFixedAreaView : public QGraphicsView
{
	Q_OBJECT
public:
	DLFixedAreaView(QWidget *parent = 0);
	void initView();
	std::vector<Fixed_ScaleInfo> get_horizon();
	std::vector<Fixed_ScaleInfo> get_vertical();

protected:
	void keyPressEvent(QKeyEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void drawBackground(QPainter *painter, const QRectF &rect);

signals:
	void sig_changed();
	void sig_viewRect_changed(const QRectF&);
	void sig_smap_upload_end(bool, QString);


private:
	QPushButton * zoom_in_pushbutton_;
	QPushButton *zoom_out_pushbutton_;
	QPushButton *center_pushbutton_;

    QPushButton *open_map_pushbutton_;
    QPushButton *upload_map_pushbutton_;
	QPushButton *m_pAddAdvanceAreaBtn;
    QComboBox *operate_type_comboBox_;


	///////////////////////////////////////////
public:
	//平移速度
	void setTranslateSpeed(double speed);
	double translateSpeed();

	//缩放增量
	void  setZoomDelta(double delta);
	double zoomDelta();
	void smapUploadCompleted(bool retcode, QString desc);


public slots:
    void slot_on_operate_type(int);
	void AddAdvanceAreaSlot();
    void slot_on_open_map();
    void slot_on_save_map();
    void slot_on_upload_map();
    void slot_on_transfer_finished(int command_type, int execCode, QString exeParam);

	void zoomIn();  // 放大
	void zoomOut();  // 缩小
	void zoom(float scaleFactor); // 缩放 - scaleFactor：缩放的比例因子
	void translate(QPointF delta);  // 平移
	void slot_on_center();

private:
	Qt::MouseButton translate_button_;  // 平移按钮
	qreal translate_speed_;  // 平移速度
	qreal zoom_delta_;  // 缩放的增量
	bool bMouse_translate_;  // 平移标识
	QPoint last_mousePos_;  // 鼠标最后按下的位置
	qreal scale_;  // 缩放值
   


private:
	std::vector<Fixed_ScaleInfo> horizon_vec_;		//记录水平的点
	std::vector<Fixed_ScaleInfo> vertical_vec_;		//记录垂直的点
    DataTransfer *m_pDataTransfer;					//线程用来上传smap到机器人
	QString m_strOpenDir;
};



///////////////////////////////////



class DLFixedAreaScene : public QGraphicsScene
{
	Q_OBJECT
public:
	DLFixedAreaScene(QObject *parent = 0);
	~DLFixedAreaScene();

	double rad2angle(double rad)
	{
		//（-π，π）转换到（0， 2π）
		double angle = 0;
		if (rad > 0) {
			angle = rad * 180 / 3.14159;
		}
		else {
			angle = ((2 * 3.14159 + rad) * 180) / 3.14159;
		}

		//逆时针转换到顺时针
		return (360 - angle);
	}

	double angle2rad(double angle)
	{
		double rad = 0;
		if (angle > 0 && angle < 180) {
			rad = (0 - angle) * 3.14159 / 180;
		}
		else {
			rad = (360 - angle) * 3.14159 / 180;
		}

		return rad;
	}


public:
	int button();
    void set_type(int operate_type);
    int get_type();
	QPoint cal_bezier_index(AdvancedCurve bezier);
	void cal_bezier_pro(QGraphicsItem *item);
	void cal_fixed_path();

public:
	//加入地图数据的item
	void add_normalPoint(QGraphicsItem *item);
	void add_normalLine(QGraphicsItem *item);
	void add_advancedPoint(QGraphicsItem *item);
	void add_advancedLine(QGraphicsItem *item);
	void add_advancedCurve(QGraphicsItem *item);
	void add_advancedArea(QGraphicsItem *item);

	void add_no_normalPoint(QGraphicsItem *item);
	void add_no_normalLine(QGraphicsItem *item);
	void add_no_advancedPoint(QGraphicsItem *item);
	void add_no_advancedLine(QGraphicsItem *item);
	void add_no_advancedCurve(QGraphicsItem *item);
	void add_no_advancedArea(QGraphicsItem *item);


public:
	void add_deviceArea(QGraphicsItem *item);
	void add_no_deviceArea(QGraphicsItem *item);


public:
	void set_mutilPoint_item(std::map<int, QPointF>);
	void get_normalPos_list(std::map<int, QPointF> &point);
	QGraphicsItem* find_item(QPointF pos, int type);
	QGraphicsItem* find_advancedCurve(int start_id, int end_id);
	QGraphicsItem* find_advancedpoint(int id);
	QList<QGraphicsItem*> getDeviceList() const;
	int get_landmark_id();
	int get_bezier_id();
    int get_advancedArea_id();


private:
	void set_landmark_id();
	void set_bezier_id();
    void set_advancedArea_id();


public:
	void remove_advancedpoint(int id);
	void remove_advancedcurve(int id);
	void remove_point(int id);
	void removeall(bool isOnlyLoadLm = false);

public:
	//加载地图文件和设备文件
	void save_json_map(QString file, bool is_upload = false);
	void load_json_map(QString file, bool isOnlyLoadLm = false);
	void _save_json_map();
	void _load_json_map(bool isOnlyLoadLm);

public:
	QPointF cal_transform(double pos_x, double pos_y);
	QPoint calculate_window_point(QSize size);

signals:
	void sig_scene_edit();
	void sig_sceneRect_changed(const QRectF &);
	void sig_smap_save_end();


public slots:
	void slot_on_read_finished(int, bool isOnlyLoadLm);
	void slot_on_uploadMap(bool exeRet, QString mesg);
	void slot_on_close_mapreaderWidget();

protected:
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	void contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent);



private:
	void onHideItemControlPoint();


private:
	//地图数据的相关属性
	Header header_;
	std::list<AdvancedDefine> advancedObjectDefine_list_;
	std::map<int, QPointF> normalPos_map_;
	std::map<int, QGraphicsItem*> normalLine_map_;
	std::map<int, QGraphicsItem*> advancedPoint_map_;
	std::map<int, QGraphicsItem*> advancedLine_map_;
	std::map<int, QGraphicsItem*> advancedCurve_map_;
	std::map<int, QGraphicsItem*> advancedArea_map_;
	std::map<int, QGraphicsItem*> device_area_map_;
	DLMultiPointItem *mutil_normal_point_item_;

	std::map<int, QPointF> select_normalPos_map_;


private:
    QGraphicsRectItem * virtual_item_;
	DLCoordinateItem *world_coordinate_item_;
	DLGridItem * grid_item_;

private:
	double coordinate_angle_;
	QPointF transform_point_;
	int button_type_;
	bool isPress_;
	int landmark_id_;
	int bezier_id_;
    int advancedArea_id_;
    QPointF start_pos_;
    QPointF end_pos_;
    int operate_type_; // 0 平移, 1 编辑, 2 添加
	bool is_upload_;

private:
	MapReader * reader_;
	MapReaderInfoWidget *reader_info_widget_;
	LandMarkPropertyWidget *landmark_property_widget_;
	DeviceAreaPropertyWidget *deviceArea_property_widget_;
	BezierPropertyWidget *bezier_property_widget_;
	AdvancedAreaPropertyWidget *advancedArea_property_idget_;
};



#endif // !DLFIXEDAREAWIDGET_ALEXWEI_20181009