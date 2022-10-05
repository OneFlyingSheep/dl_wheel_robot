#include <QtWidgets>
#include "DLFixedAreaWidget.h"
#include <QDebug>
#include <fstream>
#include <iostream>
#include <QTime>
#include "LibMapReader/MapReader.h"
#include "LibDlToolItems/DLDeviceItem.h"
#include "LibDlToolItems/DLPixmapItem.h"
#include "LibDlToolItems/DLBezierItem.h"
#include "LibDlToolItems/DLLandmarkItem.h"
#include "LibDlToolItems/DLCoordinateItem.h"
#include "LibDlToolItems/DLMultiPointItem.h"
#include "LibDlToolItems/DLSegmentItem.h"
#include "LibDlToolItems/DLPolygonItem.h"
#include "LibDLToolItems/DLAdvancedAreaItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLDeviceAreaItem.h"
#include "LibDlToolItems/DLGridItem.h"
#include "LibMapReaderInfoWidget/MapReaderInfoWidget.h"
#include "LibDataTransfer/DataTransfer.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <boost/filesystem.hpp>
#include "LibDLSceneView/DLCustomScene.h"

#include "LibDLSceneView/DLOperator.h"


#define HLINECOUNT (500+1)
#define FIXEDAREA_VLINECOUNT (500+1)

#define FIXEDAREA_VIEW_CENTER (viewport()->rect().center())
#define FIXEDAREA_VIEW_WIDTH  (viewport()->rect().width())
#define FIXEDAREA_VIEW_HEIGHT (viewport()->rect().height())
#define FIXEDAREA_VIEW_MARGIN	(30)




DLFixedAreaWidget::DLFixedAreaWidget()
{
	view_ = new DLFixedAreaView;
	//scene_ = new DLFixedAreaScene;
	m_pScene = new DLCustomScene(DL_COLLECT_EDIT_TYPE, this);
	view_->setStyleSheet("background-color:rgb(229,231,218);");
	view_->setSceneRect(-20000, -20000, 40000, 40000);
	view_->setScene(m_pScene);


	QHBoxLayout *layout = new QHBoxLayout;
	layout->setContentsMargins(FIXEDAREA_VIEW_MARGIN, FIXEDAREA_VIEW_MARGIN, 0, 0);
	layout->addWidget(view_);
	this->setLayout(layout);
	connect(view_, SIGNAL(sig_changed()), this, SLOT(update()));
	//connect(view_, SIGNAL(sig_viewRect_changed(const QRectF&)), scene_, SIGNAL(sig_sceneRect_changed(const QRectF &)));
	//connect(scene_, SIGNAL(sig_smap_save_end()), view_, SLOT(slot_on_save_map()));
	//connect(view_, SIGNAL(sig_smap_upload_end(bool, QString)), scene_, SLOT(slot_on_uploadMap(bool, QString)));
}


DLFixedAreaWidget::~DLFixedAreaWidget()
{

}


//////////////////////////////////////////////////////////////



DLFixedAreaView::DLFixedAreaView(QWidget *parent)
	: QGraphicsView(parent),
	translate_button_(Qt::MidButton),
	scale_(1.0),
	zoom_delta_(0.1),
	translate_speed_(1.0),
	bMouse_translate_(false)
	, m_strOpenDir("")
{
  
	initView();
	setFocusPolicy(Qt::StrongFocus);
	setCacheMode(CacheBackground);
	setViewportUpdateMode(BoundingRectViewportUpdate);
	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	//setMatrix(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0), true);//镜像变换																   // 去掉滚动条
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	centerOn(0, 0);
    m_pDataTransfer = new DataTransfer(this);
    connect(m_pDataTransfer, SIGNAL(sig_finished(int, int, QString)), this, SLOT(slot_on_transfer_finished(int, int, QString)));
}


void DLFixedAreaView::initView()
{
	zoom_in_pushbutton_ = new QPushButton;
	zoom_out_pushbutton_ = new QPushButton;
	center_pushbutton_ = new QPushButton;
    open_map_pushbutton_ = new QPushButton;
    upload_map_pushbutton_ = new QPushButton;
	m_pAddAdvanceAreaBtn = new QPushButton;
	m_pAddAdvanceAreaBtn->setText("添加");

//     operate_type_comboBox_ = new QComboBox;
//     operate_type_comboBox_->addItem("平移");
//     operate_type_comboBox_->addItem("编辑");
//     operate_type_comboBox_->addItem("添加");


	zoom_in_pushbutton_->setFixedSize(40, 40);
	zoom_out_pushbutton_->setFixedSize(40, 40);
	center_pushbutton_->setFixedSize(40, 40);
    open_map_pushbutton_->setFixedSize(40, 40);
    upload_map_pushbutton_->setFixedSize(40, 40);
	m_pAddAdvanceAreaBtn->setFixedSize(40, 40);
    //operate_type_comboBox_->setFixedSize(50, 30);


	open_map_pushbutton_->setToolTip("加载地图");
	upload_map_pushbutton_->setToolTip("同步地图");


	QPixmap pic_zoom_in, pic_zoom_out, pic_center, pic_open, pic_upload;
	pic_zoom_in.load(":/Resources/Common/image/zoom_in.png");
	pic_zoom_out.load(":/Resources/Common/image/zoom_out.png");
	pic_center.load(":/Resources/Common/image/center.png");
	pic_open.load(":/Resources/Common/image/open.png");
	pic_upload.load(":/Resources/Common/image/upload.png");
	pic_zoom_in.scaled(40, 40);
	pic_zoom_out.scaled(40, 40);
	pic_center.scaled(40, 40);
	pic_open.scaled(40, 40);
	pic_upload.scaled(40, 40);


	zoom_in_pushbutton_->setIcon(QIcon(pic_zoom_in));
	zoom_out_pushbutton_->setIcon(QIcon(pic_zoom_out));
	center_pushbutton_->setIcon(QIcon(pic_center));
	open_map_pushbutton_->setIcon(QIcon(pic_open));
	upload_map_pushbutton_->setIcon(QIcon(pic_upload));


	QHBoxLayout *main_layout = new QHBoxLayout;
	QVBoxLayout *bottom_layout = new QVBoxLayout;
   // bottom_layout->addWidget(operate_type_comboBox_);
	bottom_layout->addWidget(m_pAddAdvanceAreaBtn);
	bottom_layout->addWidget(open_map_pushbutton_);
    bottom_layout->addWidget(upload_map_pushbutton_);
	bottom_layout->addStretch();
	bottom_layout->addWidget(zoom_in_pushbutton_);
	bottom_layout->addWidget(zoom_out_pushbutton_);
	bottom_layout->addWidget(center_pushbutton_);

	main_layout->addStretch();
	main_layout->addLayout(bottom_layout);
	this->setLayout(main_layout);


    connect(open_map_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_open_map()));
    connect(upload_map_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_upload_map()));
	connect(m_pAddAdvanceAreaBtn, SIGNAL(clicked()), this, SLOT(AddAdvanceAreaSlot()));
    //connect(operate_type_comboBox_, SIGNAL(currentIndexChanged(int)), this, SLOT(slot_on_operate_type(int)));
   

	connect(zoom_in_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomIn()));
	connect(zoom_out_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomOut()));
	connect(center_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_center()));

	//更新地图上传状态,旧方案
	//WHEEL_BACK_TO_CORE_SOCKET.wheelUploadMap2Robot.connect(boost::bind(&DLFixedAreaView::smapUploadCompleted, this, _1, _2));
}


void DLFixedAreaView::smapUploadCompleted(bool retcode, QString desc)
{
	emit sig_smap_upload_end(retcode, desc);
}


void DLFixedAreaView::slot_on_center()
{

	centerOn(0, 0);
	
}


void DLFixedAreaView::drawBackground(QPainter *painter, const QRectF &exposed_rect)
{
	horizon_vec_.clear();
	vertical_vec_.clear();


	QPoint p1(this->rect().topLeft().x(), this->rect().topLeft().y());
	QPoint p2(this->rect().bottomRight().x(), this->rect().bottomRight().y());
	QPoint p3(this->rect().topRight().x(), this->rect().topRight().y());
	QPoint p4(this->rect().bottomLeft().x(), this->rect().bottomLeft().y());

	QPointF topLeft = mapToScene(p1);
	QPointF bottomRight = mapToScene(p2);
	QPointF topRight = mapToScene(p3);
	QPointF bottomLeft = mapToScene(p4);

	double exposed_min_horizon = topLeft.x();
	double exposed_max_horizon = bottomRight.x();
	double exposed_min_vertical = bottomRight.y();
	double exposed_max_vertical = topLeft.y();

	QRectF rect = sceneRect();
	QPointF center_pos = rect.center();
	QPointF top_left = rect.topLeft();
	QPointF bottom_left = rect.bottomLeft();
	QPointF top_right = rect.topRight();
	QPointF bottom_right = rect.bottomRight();

	double h_offset = abs((top_left - top_right).x()) / (HLINECOUNT - 1);
	double v_offset = abs((bottom_left - top_left).y()) / (FIXEDAREA_VLINECOUNT - 1);

	//计算水平线
	for (int i = 0; i < HLINECOUNT; ++i)
	{
		double temp = top_left.y() + i * v_offset;
		QPointF left_pos(top_left.x(), temp);
		QPointF right_pos(top_right.x(), temp);
		QLineF line(left_pos, right_pos);


		if (temp > exposed_min_vertical && temp < exposed_max_vertical) {

			bool isSpecial = false;
			if ((i % 5) == 0) {
				isSpecial = true;
			}

			Fixed_ScaleInfo info;
			info.point_ = QPointF(exposed_min_horizon, temp);
			info.flag_ = isSpecial;
			vertical_vec_.push_back(info);
		}
	}

	//计算垂直线
	for (int i = 0; i < FIXEDAREA_VLINECOUNT; ++i)
	{
		double temp = top_left.x() + i * h_offset;
		QPointF top_pos(temp, top_left.y());
		QPointF bottom_pos(temp, bottom_left.y());
		QLineF line(top_pos, bottom_pos);

		if (temp > exposed_min_horizon && temp < exposed_max_horizon) {

			bool isSpecial = false;
			if ((i % 5) == 0) {
				isSpecial = true;
			}

			Fixed_ScaleInfo info;
			info.point_ = QPointF(temp, exposed_max_vertical);
			info.flag_ = isSpecial;
			horizon_vec_.push_back(info);
		}
	}

	emit sig_changed();
}


std::vector<Fixed_ScaleInfo> DLFixedAreaView::get_horizon()
{
	return horizon_vec_;
}


std::vector<Fixed_ScaleInfo> DLFixedAreaView::get_vertical()
{
	return vertical_vec_;
}


void DLFixedAreaView::setTranslateSpeed(double speed)
{
	// 建议速度范围
	Q_ASSERT_X(speed >= 0.0 && speed <= 2.0,
		"InteractiveView::setTranslateSpeed", "Speed should be in range [0.0, 2.0].");
	translate_speed_ = speed;
}


double DLFixedAreaView::translateSpeed()
{
	return translate_speed_;
}


void DLFixedAreaView::setZoomDelta(double delta)
{
	// 建议增量范围
	Q_ASSERT_X(delta >= 0.0 && delta <= 1.0,
		"InteractiveView::setZoomDelta", "Delta should be in range [0.0, 1.0].");
	zoom_delta_ = delta;
}


double DLFixedAreaView::zoomDelta()
{
	return zoom_delta_;
}


void DLFixedAreaView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
	case Qt::Key_Up:
		translate(QPointF(0, -20));  // 上移
		break;
	case Qt::Key_Down:
		translate(QPointF(0, 20));  // 下移
		break;
	case Qt::Key_Left:
		translate(QPointF(-20, 0));  // 左移
		break;
	case Qt::Key_Right:
		translate(QPointF(20, 0));  // 右移
		break;
	case Qt::Key_Plus:  // 放大
		zoomIn();
		break;
	case Qt::Key_Minus:  // 缩小
		zoomOut();
		break;
	default:
		QGraphicsView::keyPressEvent(event);
	}
}


void DLFixedAreaView::mouseMoveEvent(QMouseEvent *event)
{
	if (bMouse_translate_) 
	{
		QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mousePos_);
		translate(mouseDelta);
		//emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
	}

	last_mousePos_ = event->pos();
	return QGraphicsView::mouseMoveEvent(event);
}


void DLFixedAreaView::mousePressEvent(QMouseEvent *event)
{
	// 当光标底下没有 item 时，才能移动
	if (event->button() == translate_button_) {
		
		QPointF point = mapToScene(event->pos());
		//if (scene()->itemAt(point, transform()) == NULL)
		{
			setCursor(Qt::SizeAllCursor);
			bMouse_translate_ = true;
			last_mousePos_ = event->pos();
		}
	}

	QGraphicsView::mousePressEvent(event);
}


void DLFixedAreaView::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) {
		bMouse_translate_ = false;
		setCursor(Qt::ArrowCursor);
	}

	QGraphicsView::mouseReleaseEvent(event);
}


void DLFixedAreaView::wheelEvent(QWheelEvent *event)
{
	// 滚轮的滚动量
	QPoint scrollAmount = event->angleDelta();
	// 正值表示滚轮远离使用者（放大），负值表示朝向使用者（缩小）
	scrollAmount.y() > 0 ? zoomIn() : zoomOut();
	emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
}


void DLFixedAreaView::slot_on_operate_type(int state)
{
    //((DLFixedAreaScene*)scene())->set_type(state);
}


void DLFixedAreaView::AddAdvanceAreaSlot()
{
	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (nullptr != pScene)
	{
		pScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
	}
}

void DLFixedAreaView::slot_on_transfer_finished(int command_type, int execCode, QString fileName)
{
    switch (command_type)
    {
    case DataTransfer::DOWNLOAD_FILE:
    {
        if (!execCode) {
            qDebug() << fileName + " 下载完毕";
        }
        else {
            qDebug() << fileName + "下载出错";
        }
        break;
    }
    case DataTransfer::UPLOAD_FILE:
    {
        if (!execCode) {
            //下载完毕给core回一条包含下载文件的消息
			WHEEL_BACK_TO_CORE_SOCKET.robot_config_uploadmap_req(fileName);
            qDebug() << fileName + "上传载完毕";
        }
        else {
            qDebug() << fileName + "上传出错";
        }
        break;
    }
    default:
        break;
    }

}


void DLFixedAreaView::slot_on_open_map()
{
// 	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), qApp->applicationDirPath(), tr("Maps (*.smap)"));
// 	if (fileName.isEmpty() || fileName.isNull()) {
// 		return;
// 	}
// 	((DLFixedAreaScene*)scene())->removeall();
// 	((DLFixedAreaScene*)scene())->load_json_map(fileName);

	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		m_strOpenDir = QFileDialog::getExistingDirectory(this, tr("选择地图"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		pScene->removeall();
		pScene->load_json_map(m_strOpenDir);
	}
}


void DLFixedAreaView::slot_on_save_map()
{
	QString file = QCoreApplication::applicationDirPath() + "/temp.smap";
	if (!QFile(file).exists()) {
		return;
	}
	emit sig_smap_upload_end(0, "");

	//向core上传smap地图到巡检车
	//if (NULL == dataTransfer_ || dataTransfer_->isRunning()) {
	//	return;
	//}

//	TransferPro file_pro;
//	file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
//	file_pro.file_name_ = boost::filesystem::path(file.toLocal8Bit().data()).filename().string();
//	file_pro.dst_relative_path_ = "smap";
//	file_pro.src_file_path_ = file.toLocal8Bit();
//
//	dataTransfer_->set_transfer_info(file_pro);
//	dataTransfer_->start();
}


void DLFixedAreaView::slot_on_upload_map()
{
// 	QString file = QCoreApplication::applicationDirPath() + "/temp.smap";
// 	((DLFixedAreaScene*)scene())->save_json_map(file, true);
// 	((DLFixedAreaScene*)scene())->cal_fixed_path();

	//先把点边信息文件保存到本地，再上传
	if (m_strOpenDir.isEmpty())
	{
		m_strOpenDir = qApp->applicationDirPath();
	}

	QDir dir(m_strOpenDir);
	//smap文件
	QStringList nameFilters;
	nameFilters << "*.mapinfo";
	QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
	QString strFileName = "";
	if (files.size() > 0)
	{//默认取第一个
		strFileName = files.at(0);
		if (strFileName.isEmpty())
		{
			strFileName = QFileDialog::getSaveFileName(this, "保存", m_strOpenDir, tr("*.mapinfo"));
		}
	}
	else
	{
		strFileName = QFileDialog::getSaveFileName(this, "保存", m_strOpenDir, tr("*.mapinfo"));
	}

	QString strFilePath = m_strOpenDir + "/"+ strFileName;

	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (nullptr != pScene)
	{
		pScene->save_json_map(strFilePath);
	}

	//向core上传smap地图到巡检车
	if (NULL == m_pDataTransfer || m_pDataTransfer->isRunning())
	{
		return;
	}

	//WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	//QString strDir = stMapInfo.rootPath + "/map/";
	
	//QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), m_strOpenDir, tr("Maps (*.mapinfo)"));

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
	file_pro.file_name_ = boost::filesystem::path(strFilePath.toLocal8Bit().data()).filename().string();
	file_pro.dst_relative_path_ = "smap";
	file_pro.src_file_path_ = strFilePath.toLocal8Bit();

	if (QFile::exists(strFilePath))
	{
		m_pDataTransfer->set_transfer_info(file_pro);
		m_pDataTransfer->start();
	}
	
}


void DLFixedAreaView::zoomIn()
{

	zoom(1 + zoom_delta_);

}


void DLFixedAreaView::zoomOut()
{
	zoom(1 - zoom_delta_);
}


void DLFixedAreaView::zoom(float scaleFactor)
{
	// 防止过小或过大
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
	scale_ *= scaleFactor;
}


void DLFixedAreaView::translate(QPointF delta)
{

	// 根据当前 zoom 缩放平移数
	delta *= scale_;
	delta *= translate_speed_;

	// view 根据鼠标下的点作为锚点来定位 scene
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	QPoint newCenter(FIXEDAREA_VIEW_WIDTH / 2 - delta.x(), FIXEDAREA_VIEW_HEIGHT / 2 - delta.y());
	centerOn(mapToScene(newCenter));

	// scene 在 view 的中心点作为锚点
	setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}


/////////////////////////////////////////////



DLFixedAreaScene::DLFixedAreaScene(QObject *parent) : QGraphicsScene(parent)
{
    virtual_item_ = new QGraphicsRectItem;
	virtual_item_->setZValue(100);
    virtual_item_->setRect(QRect(0,0,0,0));
    virtual_item_->setPen(Qt::NoPen);
    virtual_item_->setOpacity(0.5);
    virtual_item_->setBrush(QColor(133, 177, 222));
    addItem(virtual_item_);

	mutil_normal_point_item_ = NULL;
	landmark_property_widget_ = NULL;
	deviceArea_property_widget_ = NULL;

    operate_type_ = 0;
	isPress_ = false;
    advancedArea_id_ = -1;
	landmark_id_ = -1;
	bezier_id_ = -1;
	coordinate_angle_ = 0;
	transform_point_.setX(0);
	transform_point_.setY(0);

	world_coordinate_item_ = new DLCoordinateItem;
	grid_item_ = new DLGridItem(QRect(-20000, -20000, 40000, 40000));
	this->addItem(grid_item_);

	this->addItem(world_coordinate_item_);
	world_coordinate_item_->setZValue(1);

	reader_ = new MapReader(this);
	reader_info_widget_ = new MapReaderInfoWidget();
	landmark_property_widget_ = new LandMarkPropertyWidget;
	deviceArea_property_widget_ = new DeviceAreaPropertyWidget;
	bezier_property_widget_ = new BezierPropertyWidget;
	advancedArea_property_idget_ = new AdvancedAreaPropertyWidget;

	setItemIndexMethod(QGraphicsScene::NoIndex);
	connect(reader_, SIGNAL(sig_finished(int, bool)), this, SLOT(slot_on_read_finished(int, bool)));

}


DLFixedAreaScene::~DLFixedAreaScene()
{
	if (reader_info_widget_ != NULL)
	{
		delete reader_info_widget_;
		reader_info_widget_ = NULL;
	}
	if (landmark_property_widget_ != NULL)
	{
		delete landmark_property_widget_;
		landmark_property_widget_ = NULL;
	}
	if (deviceArea_property_widget_ != NULL)
	{
		delete deviceArea_property_widget_;
		deviceArea_property_widget_ = NULL;
	}
	if (bezier_property_widget_ != NULL)
	{
		delete bezier_property_widget_;
		bezier_property_widget_ = NULL;
	}
	if (advancedArea_property_idget_ != NULL)
	{
		delete advancedArea_property_idget_;
		advancedArea_property_idget_ = NULL;
	}
}


QGraphicsItem* DLFixedAreaScene::find_advancedCurve(int start_id, int end_id)
{
	{
		int id = (start_id << 16) + end_id;
		std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.find(id);
		if (it != advancedPoint_map_.end()) {
			return it->second;
		}
		else {
			return NULL;
		}
	}

	{
		int id = (end_id << 16) + start_id;
		std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.find(id);
		if (it != advancedPoint_map_.end()) {
			return it->second;
		}
		else {
			return NULL;
		}

	}
}


QGraphicsItem* DLFixedAreaScene::find_item(QPointF pos, int type)
{
	QGraphicsItem *node = NULL;
	QList<QGraphicsItem*> item_list = items(pos);
	foreach(QGraphicsItem *item, item_list) {
		if (item->type() == type)
		{
			node = item;
			break;
		}
	}
	return node;
}


QGraphicsItem* DLFixedAreaScene::find_advancedpoint(int id)
{
	QGraphicsItem* item = NULL;
	std::map<int, QGraphicsItem*>::iterator it = advancedPoint_map_.find(id);
	item = it->second;
	return item;
}


QList<QGraphicsItem*> DLFixedAreaScene::getDeviceList() const
{
	QList<QGraphicsItem*> item_list;
	return item_list;
}


int DLFixedAreaScene::get_landmark_id()
{
	return (landmark_id_++);
}


int DLFixedAreaScene::get_bezier_id()
{
	return (++bezier_id_);
}


int DLFixedAreaScene::get_advancedArea_id()
{
    return (++bezier_id_);
}


void DLFixedAreaScene::set_landmark_id()
{
	std::map<int, QGraphicsItem*>::iterator it = advancedPoint_map_.begin();

	int id = -1;
	while (it != advancedPoint_map_.end()) {
		id = it->first;
		it++;
	}
	landmark_id_ = id;
}


void DLFixedAreaScene::set_bezier_id()
{
	std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.begin();

	int id = -1;
	while (it != advancedCurve_map_.end()) {
		id = it->first;
		it++;
	}
	bezier_id_ = id;
}


void DLFixedAreaScene::set_advancedArea_id()
{
    std::map<int, QGraphicsItem*>::iterator it = advancedArea_map_.begin();

    int id = -1;
    while (it != advancedArea_map_.end()) {
        id = it->first;
        it++;
    }
    advancedArea_id_ = id;
}


void DLFixedAreaScene::remove_advancedpoint(int id)
{
	std::map<int, QGraphicsItem*>::iterator it = advancedPoint_map_.find(id);
	if (it != advancedPoint_map_.end()) {
		removeItem(it->second);
		update();
		advancedPoint_map_.erase(it);
	}
}


void DLFixedAreaScene::remove_advancedcurve(int id)
{
	std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.find(id);
	if (it != advancedCurve_map_.end()) {
		removeItem(it->second);
		update();
		advancedCurve_map_.erase(it);
	}
}


void DLFixedAreaScene::remove_point(int id)
{
}


void DLFixedAreaScene::removeall(bool isOnlyLoadLm)
{

	coordinate_angle_ = 0;
	transform_point_.setX(0);
	transform_point_.setY(0);

	landmark_id_ = -1;
	bezier_id_ = -1;
	advancedObjectDefine_list_.clear();
	normalPos_map_.clear();
	normalLine_map_.clear();
	advancedPoint_map_.clear();
	advancedLine_map_.clear();
	advancedCurve_map_.clear();
	advancedArea_map_.clear();

	if (isOnlyLoadLm == false) {
		device_area_map_.clear();
		foreach(QGraphicsItem* item, this->items()) {
			if ( (DLCoordinateItem*)item == world_coordinate_item_  || (DLGridItem*)item == grid_item_ || (QGraphicsRectItem*)item == virtual_item_) {
				continue;
			}
			else {
				this->removeItem(item);
			}
		}
	}
	else {
		foreach(QGraphicsItem* item, this->items()) {
			if ((DLCoordinateItem*)item == world_coordinate_item_ || (DLGridItem*)item == grid_item_ || (QGraphicsRectItem*)item == virtual_item_) {
				continue;
			}
			else {
				this->removeItem(item);
			}
		}
	}
	mutil_normal_point_item_ = NULL;

}


void DLFixedAreaScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent)
{

}

void DLFixedAreaScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
	QTransform tran;
	QGraphicsItem *item = NULL;
	item = itemAt(event->scenePos(), tran);

	if (item != NULL && HShape::LANDMARKITEM == item->type())
	{
		landmark_property_widget_->setParent((DLLandmarkItem*)item);
		landmark_property_widget_->show();
		landmark_property_widget_->move(calculate_window_point(landmark_property_widget_->size()));
		landmark_property_widget_->setLandMarkProperty();

	}
	if (item != NULL && HShape::DEVICEAREAITEM == item->type())
	{
		deviceArea_property_widget_->setParent((DLDeviceAreaItem*)item);
		deviceArea_property_widget_->move(calculate_window_point(deviceArea_property_widget_->size()));
		deviceArea_property_widget_->setDeviceAreaProperty();
		deviceArea_property_widget_->show();
	}
	if (item != NULL && HShape::BEZIERITEM == item->type())
	{
		bezier_property_widget_->setParent((DLBezierItem*)item);
		bezier_property_widget_->move(calculate_window_point(bezier_property_widget_->size()));
		bezier_property_widget_->setBezierProperty();
		bezier_property_widget_->show();
	}
	if (item != NULL && HShape::ADVANCEDAREA == item->type())
	{
		advancedArea_property_idget_->setParent((DLAdvancedAreaItem*)item);
		advancedArea_property_idget_->move(calculate_window_point(landmark_property_widget_->size()));
		advancedArea_property_idget_->setAdvancedAreaProperty();
		advancedArea_property_idget_->show();
	}
}


void DLFixedAreaScene::onHideItemControlPoint()
{
	for (std::map<int, QGraphicsItem*>::iterator it = normalLine_map_.begin(); it != normalLine_map_.end(); ++it)
	{
		((DLSegmentItem*)it->second)->set_control_point_visible(false);
	}
	for (std::map<int, QGraphicsItem*>::iterator it = advancedArea_map_.begin(); it != advancedArea_map_.end(); ++it)
	{
		((DLAdvancedAreaItem*)it->second)->set_control_point_visible(false);
	}
	for (std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.begin(); it != advancedCurve_map_.end(); ++it)
	{
		((DLBezierItem*)it->second)->set_control_point_visible(false);
	}
	for (std::map<int, QGraphicsItem*>::iterator it = device_area_map_.begin(); it != device_area_map_.end(); ++it)
	{
		((DLDeviceAreaItem*)it->second)->set_control_point_visible(false);
	}
	for (std::map<int, QGraphicsItem*>::iterator it = advancedPoint_map_.begin(); it != advancedPoint_map_.end(); ++it)
	{
		//((DLLandmarkItem*)it->second)->set_control_point_visible(false);
	}
}



int DLFixedAreaScene::button()
{
	return button_type_;
}


void DLFixedAreaScene::set_type(int operate_type)
{
    operate_type_ = operate_type;
}


int DLFixedAreaScene::get_type()
{
    return operate_type_;
}


void DLFixedAreaScene::get_normalPos_list(std::map<int, QPointF> &point_map)
{
	point_map = normalPos_map_;
}


QPoint DLFixedAreaScene::cal_bezier_index(AdvancedCurve bezier)
{
	std::string className = bezier.className;
	std::string instanceName = bezier.instanceName;

	std::string startPosID = bezier.startPos.instanceName;
	std::string endPosID = bezier.endPos.instanceName;
	std::string tempStr_start;
	std::string tempStr_end;

	for (unsigned int i = 0; i < startPosID.size(); i++)
	{
		if (startPosID[i] > 0x2F && startPosID[i] < 0x3A) {
			tempStr_start.push_back(startPosID[i]);
		}
	}
	for (unsigned int i = 0; i < endPosID.size(); i++)
	{
		if (endPosID[i] > 0x2F && endPosID[i] <= 0x3A) {
			tempStr_end.push_back(endPosID[i]);
		}
	}

	int start_id = atoi(tempStr_start.c_str());
	int end_id = atoi(tempStr_end.c_str());

	return QPoint(start_id, end_id);

}


void DLFixedAreaScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	isPress_ = true;
    start_pos_ = event->scenePos();
	if (event->button() & Qt::RightButton)
	{
		onHideItemControlPoint();
		QGraphicsScene::mousePressEvent(event);
	}
	else if (event->button() & Qt::LeftButton) {
        if (operate_type_ != 0 && itemAt(event->scenePos(), QTransform()) == NULL) {
            virtual_item_->setRect(QRect(0, 0, 0, 0));
			virtual_item_->setBrush(QColor(133, 177, 222));
            virtual_item_->setVisible(true);
        }
		QGraphicsScene::mousePressEvent(event);
	}
	else {
		QGraphicsScene::mousePressEvent(event);
	}

}


void DLFixedAreaScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	if (event->button() == Qt::RightButton) {
		QGraphicsScene::mouseReleaseEvent(event);
	}
	else if (event->button() == Qt::LeftButton) {
        if (operate_type_ == 2) {
            QLineF line(start_pos_, end_pos_);
            if (line.length() > 20) {
                QPointF offset = start_pos_ - end_pos_;
                HShape::Rectangle pro;
                pro.center_.x_ = (end_pos_.x() + start_pos_.x()) / 2;
                pro.center_.y_ = (end_pos_.y() + start_pos_.y()) / 2;
                pro.width_ = abs(offset.x());
                pro.height_ = abs(offset.y());

                int id = get_advancedArea_id();
				DLDeviceAreaItem *item = new DLDeviceAreaItem(pro,id);
				item->set_solid(true);
				item->loadPixMap(QCoreApplication::applicationDirPath()+"/FixedArea.png");
                addItem(item);
				device_area_map_[id] = item;

				cal_bezier_pro(item);
            }
        }
		else {

		}
        QGraphicsScene::mouseReleaseEvent(event);

	}
	else {
		QGraphicsScene::mouseReleaseEvent(event);
	}

	virtual_item_->setVisible(false);
	isPress_ = false;
    update();
}


void DLFixedAreaScene::cal_bezier_pro(QGraphicsItem *item)
{
	for (auto itor = advancedCurve_map_.begin(); itor != advancedCurve_map_.end(); ++itor) {
		bool ret = ((DLBezierItem*)itor->second)->collidesWithItem(item);
		if (ret) {
			((DLBezierItem*)itor->second)->setPassing(false);
		}
	}
}


void DLFixedAreaScene::cal_fixed_path()
{
	QVector<WheelRobotPoint> listPoints;
	for (auto itor = advancedCurve_map_.begin(); itor != advancedCurve_map_.end(); ++itor) {
		bool ret = ((DLBezierItem*)itor->second)->isPassing();
		if (!ret) {
			WheelRobotPoint point;
			point.x = ((DLBezierItem*)itor->second)->start_id();
			point.y = ((DLBezierItem*)itor->second)->end_id();
			listPoints.push_back(point);
		}
	}
	WHEEL_BACK_TO_CORE_SOCKET.robot_config_listing_area_req(listPoints);
}



void DLFixedAreaScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    end_pos_ = event->scenePos();

	if (isPress_ && operate_type_ != 0) {
		virtual_item_->setRect(QRectF(start_pos_, end_pos_));
		QGraphicsScene::mouseMoveEvent(event);
	}
	else {
		QGraphicsScene::mouseMoveEvent(event);
	}

}


void DLFixedAreaScene::keyPressEvent(QKeyEvent *event)
{
	QGraphicsScene::keyPressEvent(event);
}


void DLFixedAreaScene::keyReleaseEvent(QKeyEvent *event)
{

	QGraphicsScene::keyReleaseEvent(event);
}


void DLFixedAreaScene::add_deviceArea(QGraphicsItem *item)
{
	add_no_deviceArea(item);
	update();
}


void DLFixedAreaScene::add_no_deviceArea(QGraphicsItem *item)
{
	if (item == NULL) {
		return;
	}
	item->setZValue(2);
	int id = ((DLDeviceAreaItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = device_area_map_.find(id);
	if (it != device_area_map_.end()) {
		it = device_area_map_.end();
		it--;
		id = ((DLDeviceAreaItem*)it->second)->id();
		device_area_map_[id + 1] = item;
		((DLDeviceAreaItem*)item)->set_id(id + 1);
		addItem(item);
	}
	else {
		device_area_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::add_normalPoint(QGraphicsItem *item)
{
	add_no_normalPoint(item);
	update();
}


void DLFixedAreaScene::add_no_normalPoint(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	addItem(item);
}


void DLFixedAreaScene::add_normalLine(QGraphicsItem *item)
{
	add_no_normalLine(item);
	update();
}


void DLFixedAreaScene::add_no_normalLine(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	int id = ((DLSegmentItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = normalLine_map_.find(id);
	if (it != normalLine_map_.end()) {
		it = normalLine_map_.end();
		it--;
		id = it->first;
		normalLine_map_[id + 1] = item;
		addItem(item);
	}
	else
	{
		normalLine_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::add_advancedPoint(QGraphicsItem *item)
{
	add_no_advancedPoint(item);
	update();
}


void DLFixedAreaScene::add_no_advancedPoint(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	int id = ((DLLandmarkItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = advancedPoint_map_.find(id);
	if (it != advancedPoint_map_.end()) {
		it = advancedPoint_map_.end();
		it--;
		id = ((DLLandmarkItem*)it->second)->id();
		advancedPoint_map_[id + 1] = item;
		addItem(item);
	}
	else
	{
		advancedPoint_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::add_advancedLine(QGraphicsItem *item)
{
	add_no_advancedLine(item);
	update();
}


void DLFixedAreaScene::add_no_advancedLine(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	int id = ((DLSegmentItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = advancedLine_map_.find(id);
	if (it != advancedLine_map_.end()) {
		it = advancedLine_map_.end();
		it--;
		id = ((DLSegmentItem*)it->second)->id();
		advancedLine_map_[id + 1] = item;
		addItem(item);
	}
	else {
		advancedLine_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::add_advancedCurve(QGraphicsItem *item)
{
	add_no_advancedCurve(item);
	update();
}


void DLFixedAreaScene::add_no_advancedCurve(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	item->setZValue(5);
	int id = ((DLBezierItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = advancedCurve_map_.find(id);
	if (it != advancedCurve_map_.end()) {
		it = advancedCurve_map_.end();
		it--;
		id = ((DLBezierItem*)it->second)->id();
		advancedCurve_map_[id + 1] = item;
		addItem(item);
	}
	else
	{
		advancedCurve_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::add_advancedArea(QGraphicsItem *item)
{
	add_no_advancedArea(item);
	update();
}


void DLFixedAreaScene::add_no_advancedArea(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	int id = ((DLAdvancedAreaItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = advancedArea_map_.find(id);
	if (it != advancedArea_map_.end()) {
		it = advancedArea_map_.end();
		it--;
		id = ((DLAdvancedAreaItem*)it->second)->id();
		advancedArea_map_[id + 1] = item;
		addItem(item);
	}
	else
	{
		advancedArea_map_[id] = item;
		addItem(item);
	}

}


void DLFixedAreaScene::set_mutilPoint_item(std::map<int, QPointF> points_map)
{
	select_normalPos_map_.clear();
	select_normalPos_map_ = points_map;
}


QPointF DLFixedAreaScene::cal_transform(double pos_x, double pos_y)
{
	if (mutil_normal_point_item_ == NULL) {
		return QPointF(pos_x, pos_y);
	}
	QPointF point;
	double angle = (coordinate_angle_ * 3.14159) / 180;

	//绕item中心点旋转后的坐标
	QPointF start_point = mutil_normal_point_item_->scenePos() - transform_point_;

	double rotate_x = (pos_x - start_point.x())*cos(angle) - (pos_y - start_point.y())*sin(angle) + start_point.x();
	double rotate_y = (pos_x - start_point.x())*sin(angle) + (pos_y - start_point.y())*cos(angle) + start_point.y();

	//平移后的坐标
	double translate_x = rotate_x + transform_point_.x();
	double translate_y = rotate_y + transform_point_.y();
	point.setX(translate_x);
	point.setY(translate_y);

	return point;
}


QPoint DLFixedAreaScene::calculate_window_point(QSize size)
{
	int screen_width = QApplication::desktop()->width();
	int screen_height = QApplication::desktop()->height();
	QPoint point = QCursor::pos();

	if (screen_width - point.x() >= size.width() && screen_height - point.y() >= size.height())
		point = QPoint(point);
	else if (screen_width - point.x() < size.width() && screen_height - point.y() >= size.height())
		point = QPoint(point.x() - size.width(), point.y());
	else if (screen_width - point.x() < size.width() && screen_height - point.y() < size.height())
		point = QPoint(point.x() - size.width(), point.y() - size.height());
	else if (screen_width - point.x() >= size.width() && screen_height - point.y() < size.height())
		point = QPoint(point.x(), point.y() - size.height());

	return point;
}


void DLFixedAreaScene::slot_on_read_finished(int type, bool isOnlyLoadLm)
{
	switch (type)
	{
	case 0:
		_load_json_map(isOnlyLoadLm);
		break;
	case 2:
		_save_json_map();
		break;
	default:
		break;
	}
}


void  DLFixedAreaScene::_save_json_map()
{
	if (!is_upload_) {
		reader_info_widget_->hide();
	}
	else {
		emit sig_smap_save_end();
	}
	reader_info_widget_->hide();
}


void DLFixedAreaScene::slot_on_uploadMap(bool exeRet, QString mesg)
{
	if (exeRet) {
		reader_info_widget_->set_info(MapHandleInfo(0, "检修区域设置成功!"));
		QTimer::singleShot(1000, this, SLOT(slot_on_close_mapreaderWidget()));
		//reader_info_widget_->hide();
	}
	else {
		reader_info_widget_->show();
		reader_info_widget_->set_info(MapHandleInfo(0, mesg));
	}
}


void DLFixedAreaScene::slot_on_close_mapreaderWidget()
{
	reader_info_widget_->hide();
}


void  DLFixedAreaScene::_load_json_map(bool isOnlyLoadLm)
{

	MapData map_data = reader_->GetData();

	std::list<NormalPosition>::iterator NPIt;
	std::list<NormalLine>::iterator NLIt;
	std::list<AdvancedDefine>::iterator ADIt;
	std::list<AdvancedPosition>::iterator APIt;
	std::list<AdvancedLine>::iterator ALIt;
	std::list<AdvancedCurve>::iterator ACIt;
	std::list<AdvancedArea>::iterator AAIt;

	if (isOnlyLoadLm) {
		//------advanvedPoint-------
		for (APIt = map_data.MapAdvancedPointList.begin(); APIt != map_data.MapAdvancedPointList.end(); ++APIt)
		{
			std::string instanceName = APIt->instanceName;
			std::string className = APIt->className;
			std::string temp = instanceName;
			std::string tempStr;
			for (unsigned int i = 0; i < temp.size(); i++)
			{
				if (temp[i] > 0x2F && temp[i] < 0x3A) {
					tempStr.push_back(temp[i]);
				}
			}
			int id = atoi(tempStr.c_str());
			double angle = rad2angle(APIt->pos.pos_dir);
			double x = APIt->pos.pos_x;
			double y = APIt->pos.pos_y;
			DLLandmarkItem *item = new DLLandmarkItem(QPointF(100 * x, 100 * y), id, angle, instanceName, className);
			item->setPropertyList(APIt->PropertyList);
			item->setIsFixedList(APIt->ISFixedList);
			this->add_advancedPoint(item);
		}


		for (ACIt = map_data.MapAdvancedCurveList.begin(); ACIt != map_data.MapAdvancedCurveList.end(); ACIt++)
		{
			std::string className = ACIt->className;
			std::string instanceName = ACIt->instanceName;
			QPointF controlPos1(100 * ACIt->controlPos1.pos_x, 100 * ACIt->controlPos1.pos_y);
			QPointF controlPos2(100 * ACIt->controlPos2.pos_x, 100 * ACIt->controlPos2.pos_y);

			std::string startPosID = ACIt->startPos.instanceName;
			std::string endPosID = ACIt->endPos.instanceName;
			std::string tempStr_start;
			std::string tempStr_end;

			for (unsigned int i = 0; i < startPosID.size(); i++)
			{
				if (startPosID[i] > 0x2F && startPosID[i] < 0x3A) {
					tempStr_start.push_back(startPosID[i]);
				}
			}
			for (unsigned int i = 0; i < endPosID.size(); i++)
			{
				if (endPosID[i] > 0x2F && endPosID[i] <= 0x3A) {
					tempStr_end.push_back(endPosID[i]);
				}
			}

			int start_id = atoi(tempStr_start.c_str());
			int end_id = atoi(tempStr_end.c_str());

			int id = (start_id << 16) + end_id;

			DLLandmarkItem *start_item = (DLLandmarkItem*)this->find_advancedpoint(start_id);
			DLLandmarkItem *end_item = (DLLandmarkItem*)this->find_advancedpoint(end_id);

			if (start_item != NULL && end_item != NULL)
			{
				DLBezierItem *item = new DLBezierItem(start_item, end_item, controlPos1, controlPos2, id, className, instanceName);
				item->setPropertyList(ACIt->PropertyList);
				this->add_advancedCurve(item);
				QPoint index(start_id, end_id);
			}
			ACIt++;
		}
	}
	else {
		//----------header----------
		header_.mapName = map_data.header.mapName;
		header_.mapType = map_data.header.mapType;
		header_.minPos.pos_x = map_data.header.minPos.pos_x * 100;
		header_.minPos.pos_y = map_data.header.minPos.pos_y * 100;
		header_.maxPos.pos_x = map_data.header.maxPos.pos_x * 100;
		header_.maxPos.pos_y = map_data.header.maxPos.pos_y * 100;
		header_.resolution = map_data.header.resolution;

		double width = abs(header_.minPos.pos_x - header_.maxPos.pos_x);
		double height = abs(header_.minPos.pos_y - header_.maxPos.pos_y);
		QRectF map_boundRect_(QPointF(header_.minPos.pos_x, header_.minPos.pos_y), QSizeF(width, height));


		//---------advancedObjectDefine----------
		advancedObjectDefine_list_.clear();
		for (ADIt = map_data.MapAdvancedObjectDefineList.begin(); ADIt != map_data.MapAdvancedObjectDefineList.end(); ADIt++)
		{
			AdvancedDefine advancedDefine;
			advancedDefine.className = ADIt->className;
			advancedDefine.type = ADIt->type;
			advancedObjectDefine_list_.push_back(advancedDefine);
		}


		//-------normalPoslist---------
		int normalPointID = 0;
		QVector<QPointF> point_list;
		normalPos_map_.clear();
		for (NPIt = map_data.MapNormalPosList.begin(); NPIt != map_data.MapNormalPosList.end(); ++NPIt)
		{
			QPointF point(100 * NPIt->pos_x, 100 * NPIt->pos_y);
			point_list.push_back(point);
			normalPos_map_[normalPointID] = point;
			normalPointID++;
		}
	//	mutil_normal_point_item_ = new DLMultiPointItem(point_list, map_boundRect_);
		mutil_normal_point_item_ = new DLMultiPointItem;
		this->add_normalPoint(mutil_normal_point_item_);
// 		if (mutil_normal_point_item_ != NULL) {
// 			connect(this, SIGNAL(sig_sceneRect_changed(const QRectF &)), mutil_normal_point_item_, SLOT(slot_pro(const QRectF &)));
// 		}

		//------normalLineList----------
		int normalLineID = 0;
		normalLine_map_.clear();
		for (NLIt = map_data.MapNormalLineList.begin(); NLIt != map_data.MapNormalLineList.end(); ++NLIt)
		{
			HShape::Segment pro;
			pro.start_.x_ = 100 * NLIt->startPos.pos_x;
			pro.start_.y_ = 100 * NLIt->startPos.pos_y;
			pro.end_.x_ = 100 * NLIt->endPos.pos_x;
			pro.end_.y_ = 100 * NLIt->endPos.pos_y;

			DLSegmentItem *item = new DLSegmentItem(pro, normalLineID, DLSegmentItem::NormalLine);
			this->add_normalLine(item);
			normalLineID++;
		}


		//------advanvedPoint-------
		for (APIt = map_data.MapAdvancedPointList.begin(); APIt != map_data.MapAdvancedPointList.end(); ++APIt)
		{
			std::string instanceName = APIt->instanceName;
			std::string className = APIt->className;
			std::string temp = instanceName;
			std::string tempStr;
			for (unsigned int i = 0; i < temp.size(); i++)
			{
				if (temp[i] > 0x2F && temp[i] < 0x3A) {
					tempStr.push_back(temp[i]);
				}
			}
			int id = atoi(tempStr.c_str());
			double angle = rad2angle(APIt->pos.pos_dir);
			double x = APIt->pos.pos_x;
			double y = APIt->pos.pos_y;
			DLLandmarkItem *item = new DLLandmarkItem(QPointF(100 * x, 100 * y), id, angle, instanceName, className);
			item->setPropertyList(APIt->PropertyList);
			item->setIsFixedList(APIt->ISFixedList);
			this->add_advancedPoint(item);
		}


		//--------advancedLine--------
		int advancedLineID = 0;
		for (ALIt = map_data.MapAdvancedLineList.begin(); ALIt != map_data.MapAdvancedLineList.end(); ++ALIt)
		{
			HShape::Segment pro;
			pro.start_.x_ = 100 * ALIt->normalLine.startPos.pos_x;
			pro.start_.y_ = 100 * ALIt->normalLine.startPos.pos_y;
			pro.end_.x_ = 100 * ALIt->normalLine.endPos.pos_x;
			pro.end_.y_ = 100 * ALIt->normalLine.endPos.pos_y;

			if (ALIt->className == "ForbiddenLine")
			{
				DLSegmentItem *item = new DLSegmentItem(pro, advancedLineID, DLSegmentItem::ForbiddenLine);
				this->add_advancedLine(item);
			}
			else if (ALIt->className == "VirtualLine")
			{
				DLSegmentItem *item = new DLSegmentItem(pro, advancedLineID, DLSegmentItem::VirtualLine);
				this->add_advancedLine(item);
			}
			advancedLineID++;
		}


		//--------advancedCurve-------
		for (ACIt = map_data.MapAdvancedCurveList.begin(); ACIt != map_data.MapAdvancedCurveList.end(); ACIt++)
		{
			QPointF controlPos1_1(100 * ACIt->controlPos1.pos_x, 100 * ACIt->controlPos1.pos_y);
			QPointF controlPos2_1(100 * ACIt->controlPos2.pos_x, 100 * ACIt->controlPos2.pos_y);
			std::string className_1 = ACIt->className;
			std::string instanceName_1 = ACIt->instanceName;

			QPoint bezier_index = cal_bezier_index(*ACIt);
			int start_id = bezier_index.x();
			int end_id = bezier_index.y();

			int id = (start_id << 16) + end_id;

			DLLandmarkItem *start_item = (DLLandmarkItem*)this->find_advancedpoint(start_id);
			DLLandmarkItem *end_item = (DLLandmarkItem*)this->find_advancedpoint(end_id);

			DLBezierItem *item_1 = NULL;
			if (start_item != NULL && end_item != NULL)
			{
				item_1 = new DLBezierItem(start_item, end_item, controlPos1_1, controlPos2_1, id, className_1, instanceName_1);
				this->add_advancedCurve(item_1);

				if (ACIt->PropertyList.size() != 0) 
				{
					std::string temp_value = ACIt->PropertyList[0].value;
					ACIt->PropertyList[0].value = std::to_string(std::stod(temp_value) * 100);

					if (ACIt->PropertyList[6].value == "false")
					{
						item_1->setPassing(false);
					}
					else 
					{
						item_1->setPassing(true);
					}
					item_1->setPropertyList(ACIt->PropertyList);
				}
			}

			ACIt++;

			if (ACIt != map_data.MapAdvancedCurveList.end())
			{
				QPoint second_bezier_index = cal_bezier_index(*ACIt);
				int second_start_id = second_bezier_index.x();
				int second_end_id = second_bezier_index.y();

				if (second_end_id == start_id && second_start_id == end_id) {
					item_1->set_dir_pro(true);

					std::string temp_value = ACIt->PropertyList[0].value;
					ACIt->PropertyList[0].value = std::to_string(std::stod(temp_value) * 100);
					item_1->setNagetivePropertyList(ACIt->PropertyList);
				}
				else {
					item_1->set_dir_pro(false);
					ACIt--;
				}
			}
			else {
				item_1->set_dir_pro(false);
				ACIt--;
			}
		}



		//--------advancedArea-------
		for (AAIt = map_data.MapAdvanceAreaList.begin(); AAIt != map_data.MapAdvanceAreaList.end(); ++AAIt)
		{
			std::string className = AAIt->className;
			std::string instanceName = AAIt->instanceName;
			if (instanceName.empty()) {
				instanceName = "1";
			}

			QPointF bottom_left(AAIt->posGroup[0].pos_x * 100, AAIt->posGroup[0].pos_y * 100);
			QPointF top_right(AAIt->posGroup[1].pos_x * 100, AAIt->posGroup[1].pos_y * 100);

			HShape::Rectangle rect_pro;
			rect_pro.center_.x_ = (bottom_left.x() + top_right.x()) / 2;
			rect_pro.center_.y_ = (bottom_left.y() + top_right.y()) / 2;
			rect_pro.width_ = abs(bottom_left.x() - top_right.x());
			rect_pro.height_ = abs(bottom_left.y() - top_right.y());

			DLAdvancedAreaItem *item = new DLAdvancedAreaItem(rect_pro, std::stoi(instanceName), className, instanceName, DLAdvancedAreaItem::Forbidden);
			item->setPropertyList(AAIt->PropertyList);
			this->add_advancedArea(item);
		}
	}

	set_landmark_id();
	set_bezier_id();
    set_advancedArea_id();
	reader_info_widget_->hide();
}


void DLFixedAreaScene::save_json_map(QString file, bool is_upload)
{
	MapData map_data;
	map_data.header.mapType = header_.mapType;
	map_data.header.mapName = header_.mapName;
	QPointF __minPos = cal_transform(header_.minPos.pos_x, header_.minPos.pos_y);
	QPointF __maxPos = cal_transform(header_.maxPos.pos_x, header_.maxPos.pos_y);
	map_data.header.minPos.pos_x = __minPos.x() / 100;
	map_data.header.minPos.pos_y = __minPos.y() / 100;
	map_data.header.maxPos.pos_x = __maxPos.x() / 100;
	map_data.header.maxPos.pos_y = __maxPos.y() / 100;

	map_data.header.resolution = header_.resolution;
	map_data.header.version = header_.version;

	std::map<int, QPointF>::iterator MapIt;
	MapIt = normalPos_map_.begin();
	QPointF __normalPos;
	while (MapIt != normalPos_map_.end())
	{
		NormalPosition normalPosition;
		__normalPos = cal_transform(MapIt->second.x(), MapIt->second.y());
		normalPosition.pos_x = __normalPos.x() / 100;
		normalPosition.pos_y = __normalPos.y() / 100;
		map_data.MapNormalPosList.push_back(normalPosition);
		MapIt++;
	}


	std::map<int, QGraphicsItem*>::iterator NlIt = normalLine_map_.begin();
	while (NlIt != normalLine_map_.end())
	{
		NormalLine normalLine;
		normalLine.startPos.pos_x = ((DLSegmentItem*)(NlIt->second))->get_pro().start_.x_ / 100;
		normalLine.startPos.pos_y = ((DLSegmentItem*)(NlIt->second))->get_pro().start_.y_ / 100;
		normalLine.endPos.pos_x = ((DLSegmentItem*)(NlIt->second))->get_pro().end_.x_ / 100;
		normalLine.endPos.pos_y = ((DLSegmentItem*)(NlIt->second))->get_pro().end_.y_ / 100;
		map_data.MapNormalLineList.push_back(normalLine);
		NlIt++;
	}



	std::list<AdvancedDefine>::iterator AODIt = advancedObjectDefine_list_.begin();
	while (AODIt != advancedObjectDefine_list_.end())
	{
		AdvancedDefine advancedDefine;
		advancedDefine.className = AODIt->className;
		advancedDefine.type = AODIt->type;
		map_data.MapAdvancedObjectDefineList.push_back(advancedDefine);
		AODIt++;
	}



	std::map<int, QGraphicsItem*>::iterator APIt = advancedPoint_map_.begin();
	std::vector<Property>::iterator AP_ProIt;
	std::vector<Add_IsFixed>::iterator AP_IsFixed;
	while (APIt != advancedPoint_map_.end())
	{
		AdvancedPosition advancedPosition;
		double angle = ((DLLandmarkItem*)(APIt->second))->get_pro().angle_;

		advancedPosition.className = ((DLLandmarkItem*)(APIt->second))->className();
		advancedPosition.instanceName = ((DLLandmarkItem*)(APIt->second))->instanceName();
		advancedPosition.pos.pos_x = ((DLLandmarkItem*)(APIt->second))->get_pro().center_.x_ / 100;
		advancedPosition.pos.pos_y = ((DLLandmarkItem*)(APIt->second))->get_pro().center_.y_ / 100;
		advancedPosition.pos.pos_dir = angle2rad(((DLLandmarkItem*)(APIt->second))->get_pro().angle_);

		std::vector<Property> propertyVec = ((DLLandmarkItem*)(APIt->second))->getPropertyList();
		for (AP_ProIt = propertyVec.begin(); AP_ProIt != propertyVec.end(); AP_ProIt++)
		{
			Property property;
			property.key = AP_ProIt->key;
			property.type = AP_ProIt->type;
			property.value = AP_ProIt->value;
			advancedPosition.PropertyList.push_back(property);
		}
		std::vector<Add_IsFixed> isFixedVec = ((DLLandmarkItem*)(APIt->second))->getIsFixedList();
		for (AP_IsFixed = isFixedVec.begin(); AP_IsFixed != isFixedVec.end(); AP_IsFixed++)
		{
			Add_IsFixed isFixedValue;
			isFixedValue.key = AP_IsFixed->key;
			isFixedValue.type = AP_IsFixed->type;
			isFixedValue.value = AP_IsFixed->value;
			advancedPosition.ISFixedList.push_back(isFixedValue);
		}
		map_data.MapAdvancedPointList.push_back(advancedPosition);
		APIt++;
	}


	std::map<int, QGraphicsItem*>::iterator ALIt = advancedLine_map_.begin();
	while (ALIt != advancedLine_map_.end())
	{
		AdvancedLine advancedLine;
		advancedLine.className = ((DLSegmentItem*)(ALIt->second))->className();
		advancedLine.instanceName = ((DLSegmentItem*)(ALIt->second))->instanceName();
		advancedLine.normalLine.startPos.pos_x = ((DLSegmentItem*)(ALIt->second))->get_pro().start_.x_ / 100;
		advancedLine.normalLine.startPos.pos_y = ((DLSegmentItem*)(ALIt->second))->get_pro().start_.x_ / 100;
		advancedLine.normalLine.endPos.pos_x = ((DLSegmentItem*)(ALIt->second))->get_pro().end_.x_ / 100;
		advancedLine.normalLine.endPos.pos_y = ((DLSegmentItem*)(ALIt->second))->get_pro().end_.y_ / 100;
		map_data.MapAdvancedLineList.push_back(advancedLine);
		ALIt++;
	}

	std::map<int, QGraphicsItem*>::iterator AAIt = advancedArea_map_.begin();
	while (AAIt != advancedArea_map_.end())
	{
		std::vector<Property>::iterator AA_ProIt = ((DLAdvancedAreaItem*)(AAIt->second))->getPropertyList().begin();
		AdvancedArea advancedArea;
		advancedArea.className = ((DLAdvancedAreaItem*)(AAIt->second))->className();
		advancedArea.instanceName = ((DLAdvancedAreaItem*)(AAIt->second))->instanceName();

		HShape::Rectangle pro = ((DLAdvancedAreaItem*)(AAIt->second))->get_pro();

		NormalPosition normalPosition_1, normalPosition_2;
		normalPosition_1.pos_x = (pro.center_.x_ - pro.width_ / 2) / 100;
		normalPosition_1.pos_y = (pro.center_.y_ - pro.height_ / 2) / 100;
		normalPosition_2.pos_x = (pro.center_.x_ + pro.width_ / 2) / 100;
		normalPosition_2.pos_y = (pro.center_.y_ + pro.height_ / 2) / 100;
		advancedArea.posGroup.push_back(normalPosition_1);
		advancedArea.posGroup.push_back(normalPosition_2);


		while (AA_ProIt != ((DLAdvancedAreaItem*)(AAIt->second))->getPropertyList().end())
		{
			Property property;
			property.key = AA_ProIt->key;
			property.type = AA_ProIt->type;
			property.value = AA_ProIt->value;
			advancedArea.PropertyList.push_back(property);
			AA_ProIt++;
		}
		map_data.MapAdvanceAreaList.push_back(advancedArea);
		AAIt++;
	}


	std::map<int, QGraphicsItem*>::iterator ACIt = advancedCurve_map_.begin();
	std::vector<Property>::iterator AC_ProIt;
	while (ACIt != advancedCurve_map_.end())
	{
		DLLandmarkItem *start_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->start_item();
		DLLandmarkItem *end_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->end_item();
		bool isDoubleDir = ((DLBezierItem*)(ACIt->second))->dir_pro();

		AdvancedCurve advancedCurve;
		advancedCurve.className = ((DLBezierItem*)(ACIt->second))->className();
		advancedCurve.instanceName = ((DLBezierItem*)(ACIt->second))->instanceName();
		advancedCurve.startPos.className = start_item->className();
		advancedCurve.startPos.instanceName = start_item->instanceName();
		advancedCurve.startPos.pos_x = start_item->get_pro().center_.x_ / 100;
		advancedCurve.startPos.pos_y = start_item->get_pro().center_.y_ / 100;
		advancedCurve.endPos.className = end_item->className();
		advancedCurve.endPos.instanceName = end_item->instanceName();
		advancedCurve.endPos.pos_x = end_item->get_pro().center_.x_ / 100;
		advancedCurve.endPos.pos_y = end_item->get_pro().center_.y_ / 100;
		advancedCurve.controlPos1.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.x_ / 100;
		advancedCurve.controlPos1.pos_y = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.y_ / 100;
		advancedCurve.controlPos2.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.x_ / 100;
		advancedCurve.controlPos2.pos_y = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.y_ / 100;

		AdvancedCurve advancedCurve1;
		advancedCurve1.className = ((DLBezierItem*)(ACIt->second))->className();
		advancedCurve1.instanceName = ((DLBezierItem*)(ACIt->second))->instanceName();
		advancedCurve1.startPos.className = end_item->className();
		advancedCurve1.startPos.instanceName = end_item->instanceName();
		advancedCurve1.startPos.pos_x = end_item->get_pro().center_.x_ / 100;
		advancedCurve1.startPos.pos_y = end_item->get_pro().center_.y_ / 100;
		advancedCurve1.endPos.className = start_item->className();
		advancedCurve1.endPos.instanceName = start_item->instanceName();
		advancedCurve1.endPos.pos_x = start_item->get_pro().center_.x_ / 100;
		advancedCurve1.endPos.pos_y = start_item->get_pro().center_.y_ / 100;
		advancedCurve1.controlPos1.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.x_ / 100;
		advancedCurve1.controlPos1.pos_y = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.y_ / 100;
		advancedCurve1.controlPos2.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.x_ / 100;
		advancedCurve1.controlPos2.pos_y = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.y_ / 100;

		for (AC_ProIt = ((DLBezierItem*)(ACIt->second))->getPropertyList().begin(); AC_ProIt != ((DLBezierItem*)(ACIt->second))->getPropertyList().end(); ++AC_ProIt)
		{
			Property property;
			property.key = AC_ProIt->key;
			property.type = AC_ProIt->type;
			if (property.key == "weight") {
				property.value = std::to_string(std::stod(AC_ProIt->value) / 100);
			}
			else {
				property.value = AC_ProIt->value;
			}
			advancedCurve.PropertyList.push_back(property);
		}

		for (AC_ProIt = ((DLBezierItem*)(ACIt->second))->getNagetivePropertyList().begin(); AC_ProIt != ((DLBezierItem*)(ACIt->second))->getNagetivePropertyList().end(); ++AC_ProIt)
		{
			Property property;
			property.key = AC_ProIt->key;
			property.type = AC_ProIt->type;
			if (property.key == "weight") {
				property.value = std::to_string(std::stod(AC_ProIt->value) / 100);
			}
			else {
				property.value = AC_ProIt->value;
			}
			advancedCurve1.PropertyList.push_back(property);
		}

		if (isDoubleDir) {
			map_data.MapAdvancedCurveList.push_back(advancedCurve);
			map_data.MapAdvancedCurveList.push_back(advancedCurve1);
		}
		else {
			map_data.MapAdvancedCurveList.push_back(advancedCurve);
		}
		ACIt++;
	}

	if (is_upload) 
	{
		reader_info_widget_->show();
		reader_info_widget_->set_info(MapHandleInfo(0, "正在与机器人同步数据，请耐心等待。"));
	}
	else 
	{
		reader_info_widget_->show();
		reader_info_widget_->set_info(MapHandleInfo(0, "正在保存smap地图，请耐心等待。"));
	}
	is_upload_ = is_upload;

	std::string filename_new = file.toLocal8Bit();
	reader_->SetMapData(filename_new, map_data);
	reader_->start();

}


void DLFixedAreaScene::load_json_map(QString file, bool isOnlyLoadLm)
{
	reader_info_widget_->show();
	reader_info_widget_->set_info(MapHandleInfo(0, "正在打开smap地图，请耐心等待。"));
	std::string file_name = file.toLocal8Bit();
	reader_->SetReadPro(file_name, 0, isOnlyLoadLm);
	reader_->start();
}

