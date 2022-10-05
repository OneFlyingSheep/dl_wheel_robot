#include <QtWidgets>
#include <QButtonGroup>
#include <QGroupBox>
#include <boost/filesystem.hpp>
#include "DLMapEditorWidget.h"
#include "LibDLSceneView/DLOperator.h"
#include "LibDLSceneView/DLCustomScene.h"
#include "LibDLSceneView/DLView.h"
#include "LibDataTransfer/DataTransfer.h"
#include "LibDLStatusBar/DLStatusBar.h"
#include "LibLaserParamWidget/LaserParamWidget.h"
#include "LibSmapBuilder/SmapBuilder.h"
#include <QDebug>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"

#include "LibDlToolItems/DLBezierItem.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

#include "LibDLWheelCollectControlWidget\DLMapListWidget.h"

#include "LibDLCollectMap/DLCollectMap.h"



///////////////////////////////////////



DLMapEditorWidget::DLMapEditorWidget(QWidget *parent)
	:m_bSMAPIsChanged(false)
	, m_strFileName("")
	, m_bIsUploadSmap(false)
{
	qRegisterMetaType<WheelRobotSwitchRunningStatus>("WheelRobotSwitchRunningStatus");
	initStatusBar();
	initSceneView();
	initWidget();
	initCoreFunction();
	m_pThreadDataTransfer = new DataTransfer(this);
    //mapListWidget_ = new DLMapListWidget;
	laserParamWidget_ = new LaserParamWidget;
	smapBuilder_ = new SmapBuilder(this);

	connect(laserParamWidget_, SIGNAL(sig_param_ok()), this, SLOT(slot_on_build_map()));
	connect(smapBuilder_, SIGNAL(sig_process(float)), this, SLOT(slot_on_build_process(float)));
	connect(smapBuilder_, SIGNAL(sig_finished()), this, SLOT(slot_on_build_finished()));
    connect(m_pThreadDataTransfer, SIGNAL(sig_finished(int, int, QString)), this, SLOT(slot_on_transfer_finished(int, int, QString)));
    //connect(mapListWidget_, SIGNAL(sig_select_file(QString, QString, int)), this, SLOT(slot_on_choose_map(QString, QString, int)));

	m_pMapListWgt = new DLMapListWidget(this);
	connect(m_pMapListWgt, SIGNAL(sig_select_file(QString, QString, int)), this, SLOT(slot_on_choose_map(QString, QString, int)));

}


DLMapEditorWidget::~DLMapEditorWidget()
{
	
	if (laserParamWidget_ != NULL)
	{
		delete laserParamWidget_;
		laserParamWidget_ = NULL;
	}
}


void DLMapEditorWidget::initSceneView()
{
	m_pScene = new DLCustomScene(DL_COLLECT_EDIT_TYPE, this);
	m_pScene->setSceneRect(-20000, -20000, 40000, 40000);
	connect(m_pScene, SIGNAL(ChangeTypeSignal(const QString &)), this, SLOT(ChangeTypeSlot(const QString &)));
	connect(m_pScene, SIGNAL(SMAPChangedSignal()), this, SLOT(SMAPChangedSlot()));
	connect(m_pScene, SIGNAL(ReadFinishedSignal(bool)), this, SLOT(ReadFinishedSlot(bool)));
	//connect(scene_, SIGNAL(selectionChanged()), this, SLOT(SelectedChangedSlot()));
// 	DLView *view = new DLView;
// 	view->setScene(m_pScene);
// 	view->setStyleSheet("background-color:rgb(229,231,218);");
// 	m_pView = new DLFrameView;
// 	m_pView->setScene(m_pScene);
//	connect(view, SIGNAL(sig_viewRect_changed(const QRectF&)), m_pScene, SIGNAL(sig_sceneRect_changed(const QRectF&)));
	m_pView = new DLCollectMapView(this);
	m_pView->setScene(m_pScene);

	connect(m_pView, SIGNAL(sig_add_patrolPoint()), m_pScene, SLOT(slot_on_add_patrol_point()));
	connect(m_pView, SIGNAL(sig_add_patrolPoint()), this, SIGNAL(signalCollectPatrolPoint()));
	connect(m_pScene, SIGNAL(sig_flush_viewPort(QPointF)), m_pView, SLOT(slot_on_flush_viewPort(QPointF)));
	connect(m_pView, SIGNAL(sig_viewRect_changed(const QRectF&)), m_pScene, SIGNAL(sig_sceneRect_changed(const QRectF&)));
}


void DLMapEditorWidget::initStatusBar()
{
	m_pStatusBar = new DLStatusBar;
}


void DLMapEditorWidget::initCoreFunction()
{

	//更新地图上传状态
	WHEEL_BACK_TO_CORE_SOCKET.wheelUploadMap2Robot.connect(boost::bind(&DLMapEditorWidget::smapUploadCompleted, this, _1, _2));
	

    // 更新当前机器人模式状态;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotSwitchRunningStatusSignal.connect(boost::bind(&DLMapEditorWidget::signalRobotModeStateCallBack, this, _1));

    connect(this, &DLMapEditorWidget::signalRobotModeStateCallBack, this, [=](WheelRobotSwitchRunningStatus status) {
        switch (status)
        {
        case WHEEL_ROBOT_SWITCH_AUTORUNNING:
            m_pCurrentModeLabel->setShowValue("任务模式");
            break;
        case WHEEL_ROBOT_SWITCH_REMOTE_CTRL:
            m_pCurrentModeLabel->setShowValue("键盘模式");
            break;
        case WHEEL_ROBOT_SWITCH_JOY_CTRL:
            m_pCurrentModeLabel->setShowValue("手柄模式");
            break;
        case WHEEL_ROBOT_SWITCH_EMERGENCY_LOC:
            m_pCurrentModeLabel->setShowValue("定位模式");
            break;
        default:
            break;
        }
    });

}


void DLMapEditorWidget::SaveSMAPFile()
{
	m_pStatusBar->setOperate("保存地图");
	if (m_strFileName.isEmpty())
	{
		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";

		m_strFileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.smap)"));
	}
	if (m_strFileName.isEmpty() || m_strFileName.isNull()) {
		return;
	}

	m_pScene->save_json_map(m_strFileName);
	QString file = m_strFileName.split("/").last();
	m_pStatusBar->setOperate(file + "保存成功。");

	m_bSMAPIsChanged = false;
}

void DLMapEditorWidget::SaveAsSMAPFile(QString &strSaveAsFile)
{
	m_pStatusBar->setOperate("另存为地图");
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	strSaveAsFile = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.smap)"));
	if (strSaveAsFile.isEmpty() || strSaveAsFile.isNull()) {
		return;
	}

	m_pScene->save_json_map(strSaveAsFile);
	QString file = strSaveAsFile.split("/").last();
	m_pStatusBar->setOperate(file + "另存为成功。");

	m_bSMAPIsChanged = false;
}

void DLMapEditorWidget::SaveBSMAPFile(QString strFilePath)
{
	if (strFilePath.isEmpty())
	{
		QString strFile = m_strFileName;
		//m_pScene->save_backstage_map(strFile.replace(".smap", ".bsmap"));
		QString file = strFile.split("/").last().replace(".smap", ".bsmap");
		m_pStatusBar->setOperate(tr("加载") + file);
	}
	else
	{
		//m_pScene->save_backstage_map(strFilePath.replace(".smap", ".bsmap"));
		QString file = strFilePath.split("/").last().replace(".smap", ".bsmap");
		m_pStatusBar->setOperate(tr("加载") + file);
	}

}

void DLMapEditorWidget::UploadSMAPFile()
{
	//向core上传smap地图到巡检车
	if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
		return;
	}

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
	file_pro.file_name_ = boost::filesystem::path(m_strFileName.toLocal8Bit().data()).filename().string();
	file_pro.dst_relative_path_ = "smap";
	file_pro.src_file_path_ = m_strFileName.toLocal8Bit();

	if (QFile::exists(m_strFileName))
	{
		m_pThreadDataTransfer->set_transfer_info(file_pro);
		m_pThreadDataTransfer->start();
		m_pStatusBar->setOperate(tr("正在上传smap地图"));
	}

}

QPushButton * DLMapEditorWidget::CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath)
{
	QPushButton *pBtn = new QPushButton(QIcon(strIconPath), "", parentWgt);
	pBtn->setToolTip(strName);
	pBtn->setFixedSize(50, 50);
	pBtn->setCheckable(true);
	pBtn->setIconSize(QSize(40, 40));
	pBtn->setObjectName(strObjectName);
	return pBtn;
}

void DLMapEditorWidget::ClearMap()
{
	if(!m_bSMAPIsChanged)
	{
		m_bSMAPIsChanged = m_pScene->GetItemIsChanged();
	}

	if (m_bSMAPIsChanged)
	{//是否有修改，如果有，需要保存为smap地图，保存dsmap地图，上传smap地图三个功能
		QMessageBox msgBox;
		msgBox.setWindowTitle("提示");
		msgBox.setText("用户smap已修改，尚未保存？");
		//msgBox.setInformativeText("Do you want to save your changes?");
		msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::SaveAll | QMessageBox::Cancel);
		msgBox.setButtonText(QMessageBox::Save, "更新地图");
		msgBox.setButtonText(QMessageBox::SaveAll, "另存地图");
		msgBox.setButtonText(QMessageBox::Cancel, "放弃修改");
		msgBox.setDefaultButton(QMessageBox::Save);
		int ret = msgBox.exec();
		if (QMessageBox::Save == ret)
		{//保存地图
			SaveSMAPFile();					//保存smap文件
			SaveBSMAPFile();				//保存bsmap文件
			//UploadSMAPFile();				//上传smap
			m_bIsUploadSmap = true;
		}
		else if (QMessageBox::SaveAll == ret)
		{//另存为
			QString strSaveAsFile = "";
			SaveAsSMAPFile(strSaveAsFile);				//另存为smap文件,在另存为函数中获取文件路径
			SaveBSMAPFile(strSaveAsFile);				//保存bsmap文件
		}
		else
		{
			m_pScene->removeall();
			m_bSMAPIsChanged = false;
			m_strFileName = "";
		}
	}
}


void DLMapEditorWidget::initWidget()
{
//     m_pCollectStartSlamBtn = new QPushButton(tr("开始扫描"));
//     m_pCollectEndSlamBtn = new QPushButton(tr("结束扫描"));
//     m_pDownload2DBtn = new QPushButton(tr("下载2D地图"));
// 	m_pDownloadSMAPBtn = new QPushButton("下载smap地图");
//     m_pUploadSMAPBtn = new QPushButton(tr("上传smap"));
//     m_pCreateSMAPBtn = new QPushButton(tr("构建smap地图"));
// 
//     QGroupBox *collect_groupBox = new QGroupBox(tr("地图采集"));
//     QGridLayout *collect_layout = new QGridLayout;
//     collect_layout->addWidget(m_pCollectStartSlamBtn, 0, 0);
//     collect_layout->addWidget(m_pCollectEndSlamBtn, 0, 1);
//     collect_layout->addWidget(m_pDownload2DBtn, 1, 0);
//     collect_layout->addWidget(m_pUploadSMAPBtn, 1, 1);
// 	collect_layout->addWidget(m_pDownloadSMAPBtn, 2, 0);
//     collect_layout->addWidget(m_pCreateSMAPBtn, 2, 1);
//     collect_groupBox->setLayout(collect_layout);
// 
//     connect(m_pCollectStartSlamBtn, SIGNAL(clicked()), this, SLOT(slot_on_start_slam()));
//     connect(m_pCollectEndSlamBtn, SIGNAL(clicked()), this, SLOT(slot_on_end_slam()));
//     connect(m_pDownload2DBtn, SIGNAL(clicked()), this, SLOT(slot_on_download_2dmap()));
//     connect(m_pUploadSMAPBtn, SIGNAL(clicked()), this, SLOT(slot_on_upload_smap()));
// 	connect(m_pDownloadSMAPBtn, SIGNAL(clicked()), this, SLOT(slot_on_download_smap()));
//     connect(m_pCreateSMAPBtn, SIGNAL(clicked()), this, SLOT(slot_on_set_build_map_param()));
// 
// ////////////////////////////////////////////////////////////////////
// 
// 	m_pOpenSMAPBtn = new QPushButton(tr("打开smap地图"));
// 	m_pSaveSMAPBtn = new QPushButton(tr("保存smap地图"));
// 
// 
// 
// 
// 	m_pUnloadMapBtn = new QPushButton(tr("卸载地图"));
// 	m_pQuitBtn = new QPushButton(tr("退出"));
// 
// 	QGroupBox *file_groupBox = new QGroupBox(tr("地图文件"));
// 	//QVBoxLayout *file_groupBox_layout = new QVBoxLayout;
// 	QGridLayout *file_layout = new QGridLayout;
// 	file_layout->addWidget(m_pOpenSMAPBtn, 0, 0);
// 	file_layout->addWidget(m_pSaveSMAPBtn, 0, 1);
// 	file_layout->addWidget(m_pUnloadMapBtn, 1, 0);
//     file_layout->addWidget(m_pQuitBtn, 1, 1);
// 
// 	file_groupBox->setLayout(file_layout);
// 
// 
// 
// 	connect(m_pOpenSMAPBtn, SIGNAL(clicked()), this, SLOT(slot_on_load_smap()));
// 	connect(m_pSaveSMAPBtn, SIGNAL(clicked()), this, SLOT(slot_on_save_smap()));
// 	connect(m_pUnloadMapBtn, SIGNAL(clicked()), this, SLOT(slot_on_unload_smap()));
// 	connect(m_pQuitBtn, SIGNAL(clicked()), this, SLOT(slot_on_quit()));
// 
// 	////////////////////////////////////
// 	edit_undo_button_ = new QPushButton(tr("撤销"));
// 	edit_redo_button_ = new QPushButton(tr("重做"));
// 	edit_cut_button_ = new QPushButton(tr("剪切"));
// 	edit_copy_button_ = new QPushButton(tr("拷贝"));
// 	edit_paste_button_ = new QPushButton(tr("粘贴"));
// 	edit_delete_button_ = new QPushButton(tr("删除"));
// 	edit_selectall_button_ = new QPushButton(tr("全选"));
// 	edit_find_button_ = new QPushButton(tr("查找"));
// 	m_pCutOutAreaBtn = new QPushButton(tr("截取"));
// 	m_pSetBgPixmapBtn = new QPushButton(tr("设置背景图片"));
// 
// 
// 
// 	QGroupBox *edit_groupBox = new QGroupBox(tr("地图操作"));
// 	QGridLayout *edit_layout = new QGridLayout;
// 	edit_layout->addWidget(edit_undo_button_, 0, 0);
// 	edit_layout->addWidget(edit_redo_button_, 0, 1);
// 	edit_layout->addWidget(edit_cut_button_, 1, 0);
// 	edit_layout->addWidget(edit_copy_button_, 1, 1);
// 	edit_layout->addWidget(edit_paste_button_, 2, 0);
// 	edit_layout->addWidget(edit_delete_button_, 2, 1);
// 	edit_layout->addWidget(edit_selectall_button_, 3, 0);
// 	edit_layout->addWidget(edit_find_button_, 3, 1);
// 	edit_layout->addWidget(m_pCutOutAreaBtn, 4, 0);
// 	edit_layout->addWidget(m_pSetBgPixmapBtn, 4, 1);
// 	edit_groupBox->setLayout(edit_layout);
// 	
// 	connect(edit_undo_button_, SIGNAL(clicked()), this, SLOT(slot_on_undo()));
// 	connect(edit_redo_button_, SIGNAL(clicked()), this, SLOT(slot_on_redo()));
// 	connect(edit_cut_button_, SIGNAL(clicked()), this, SLOT(slot_on_cut()));
// 	connect(edit_copy_button_, SIGNAL(clicked()), this, SLOT(slot_on_copy()));
// 	connect(edit_paste_button_, SIGNAL(clicked()), this, SLOT(slot_on_paste()));
// 	connect(edit_delete_button_, SIGNAL(clicked()), this, SLOT(slot_on_delete()));
// 	connect(edit_selectall_button_, SIGNAL(clicked()), this, SLOT(slot_on_selectall()));
// 	connect(edit_find_button_, SIGNAL(clicked()), this, SLOT(slot_on_find()));
// 	connect(m_pCutOutAreaBtn, SIGNAL(clicked()), this, SLOT(CutOutAreaBtnSlot()));
// 	connect(m_pSetBgPixmapBtn, SIGNAL(clicked()), this, SLOT(SetBgPixmapBtnSlot()));
// 
// 	////////////////////////////////////////////////////
// 
// 	operate_select_button_ = new QPushButton;
// 	operate_coordinate_transform_button_ = new QPushButton;
// 	operate_landmark_button_ = new QPushButton;
// 	operate_bezier_button_ = new QPushButton;
// 	operate_advanced_area_button_ = new QPushButton;
// 	operate_normal_line_button_ = new QPushButton;
// 	operate_virtual_line_button_ = new QPushButton;
// 	operate_forbidden_line_button_ = new QPushButton;
// 	operate_picture_button_ = new QPushButton;
// 	operate_device_area_button_ = new QPushButton;
// 	operate_station_button_ = new QPushButton;
// 	operate_relocation_button_ = new QPushButton;
// 	//operate_add_relocation_button_ = new QPushButton("巡检点");
// 
// 	operate_select_button_->setToolTip("选择");
// 	operate_coordinate_transform_button_->setToolTip("坐标变换");
// 	operate_landmark_button_->setToolTip("巡检点");
// 	operate_bezier_button_->setToolTip("巡检路径");
// 	operate_advanced_area_button_->setToolTip("高级区域");
// 	operate_normal_line_button_->setToolTip("普通线");
// 	operate_virtual_line_button_->setToolTip("虚线");
// 	operate_forbidden_line_button_->setToolTip("禁止线");
// 	operate_picture_button_->setToolTip("图片");
// 	operate_device_area_button_->setToolTip("设备区域");
// 	operate_station_button_->setToolTip("电站区域");
// 	operate_relocation_button_->setToolTip("重定向");
// 
// 	
// 	operate_select_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/select.png").scaled(40, 40)));
// 	operate_coordinate_transform_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/coordinate_transfer.png").scaled(40, 40)));
// 	operate_landmark_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_advanced_point.png").scaled(40, 40)));
// 	operate_bezier_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_patrol_path.png").scaled(40, 40)));
// 	operate_advanced_area_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_advanced_area.png").scaled(40, 40)));
// 	operate_normal_line_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/line.png").scaled(40, 40)));
// 	operate_virtual_line_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/virtual_line.png").scaled(40, 40)));
// 	operate_forbidden_line_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/advanced_line.png").scaled(40, 40)));
// 	operate_picture_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_pixmap.png").scaled(40, 40)));
// 	operate_device_area_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_device_area.png").scaled(40, 40)));
// 	operate_station_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/add_station_area.png").scaled(40, 40)));
// 	operate_relocation_button_->setIcon(QIcon(QPixmap(":/Resources/map_edite/relocation.png").scaled(40, 40)));
// 
// 	operate_select_button_->setIconSize(QSize(40, 40));
// 	//operate_add_relocation_button_->setIconSize(QSize(40, 40));
// 	operate_coordinate_transform_button_->setIconSize(QSize(40, 40));
// 	operate_select_button_->setIconSize(QSize(40, 40));
// 	operate_coordinate_transform_button_->setIconSize(QSize(40, 40));
// 	operate_landmark_button_->setIconSize(QSize(40, 40));
// 	operate_bezier_button_->setIconSize(QSize(40, 40));
// 	operate_advanced_area_button_->setIconSize(QSize(40, 40));
// 	operate_normal_line_button_->setIconSize(QSize(40, 40));
// 	operate_virtual_line_button_->setIconSize(QSize(40, 40));
// 	operate_forbidden_line_button_->setIconSize(QSize(40, 40));
// 	operate_picture_button_->setIconSize(QSize(40, 40));
// 	operate_device_area_button_->setIconSize(QSize(40, 40));
// 	operate_station_button_->setIconSize(QSize(40, 40));
// 	operate_relocation_button_->setIconSize(QSize(40, 40));
// 
// 
// 
// 	QGroupBox *operate_groupBox = new QGroupBox(tr("地图编辑"));
// 	QGridLayout *operate_layout = new QGridLayout;
// 	operate_layout->addWidget(operate_select_button_, 0, 0);
// 	operate_layout->addWidget(operate_coordinate_transform_button_, 0, 1);
//     operate_layout->addWidget(operate_landmark_button_, 1, 0);
// 	operate_layout->addWidget(operate_bezier_button_, 1, 1);
// 	operate_layout->addWidget(operate_advanced_area_button_, 2, 0);
// 	operate_layout->addWidget(operate_normal_line_button_, 2, 1);
// 	operate_layout->addWidget(operate_virtual_line_button_, 3, 0);
// 	operate_layout->addWidget(operate_forbidden_line_button_, 3, 1);
// 	operate_layout->addWidget(operate_picture_button_, 4, 0);
// 	operate_layout->addWidget(operate_device_area_button_, 4, 1);
// 	operate_layout->addWidget(operate_station_button_, 5, 0);
// 	operate_layout->addWidget(operate_relocation_button_, 5, 1);
// 	//operate_layout->addWidget(operate_add_relocation_button_, 6, 0);
// 	operate_groupBox->setLayout(operate_layout);
// 
 	// 当前模式;
    m_pCurrentModeLabel = new InputWidget(InputWidgetType::WheelValueShowWidget);
    m_pCurrentModeLabel->setTipText("当前模式");
    m_pCurrentModeLabel->setFixedSize(QSize(160, 20));
	// 模式选择;
	QStringList modeList = QStringList() << ("任务模式") << ("键盘模式") << ("手柄模式") << ("定位模式");
	InputWidget *modeChangeWidget = new InputWidget(InputWidgetType::ComboBox);
	modeChangeWidget->setTipText("模式选择:");
	modeChangeWidget->setComboBoxContent(modeList);

	connect(modeChangeWidget, &InputWidget::signalComboBoxIndexActivated, this, [=](int index) {
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(index));
	});
// 
// 	connect(operate_coordinate_transform_button_, SIGNAL(clicked()), this, SLOT(slot_on_rotate()));
// 	connect(operate_select_button_, SIGNAL(clicked()), this, SLOT(slot_on_select()));
// 	connect(operate_landmark_button_, SIGNAL(clicked()), this, SLOT(slot_on_landmark()));
// 	connect(operate_bezier_button_, SIGNAL(clicked()), this, SLOT(slot_on_bezier()));
// 	connect(operate_advanced_area_button_, SIGNAL(clicked()), this, SLOT(slot_on_advanced_area()));
// 	connect(operate_normal_line_button_, SIGNAL(clicked()), this, SLOT(slot_on_line()));
// 	connect(operate_virtual_line_button_, SIGNAL(clicked()), this, SLOT(slot_on_virtual_line()));
// 	connect(operate_forbidden_line_button_, SIGNAL(clicked()), this, SLOT(slot_on_forbbidon_line()));
// 	connect(operate_picture_button_, SIGNAL(clicked()), this, SLOT(slot_on_picture()));
// 	connect(operate_device_area_button_, SIGNAL(clicked()), this, SLOT(slot_on_device_area()));
// 	connect(operate_station_button_, SIGNAL(clicked()), this, SLOT(slot_on_station_area()));
// 	connect(operate_relocation_button_, SIGNAL(clicked()), this, SLOT(slot_on_relocation()));
// 	//connect(operate_add_relocation_button_, SIGNAL(clicked()), this, SLOT(slot_on_add_landmark()));

	///////////////////////////////////





	QVBoxLayout *pVLayout = new QVBoxLayout;
	pVLayout->setMargin(0);
	pVLayout->setSpacing(20);
//pMapToolLayout->addLayout(pVLayout);

	QWidget *pToolBtnWgt = new QWidget;
	pToolBtnWgt->setLayout(pVLayout);

	m_pFileGroupBox = new QGroupBox(tr("地图文件"), pToolBtnWgt);
	m_pOperatorGroupBox = new QGroupBox(tr("地图操作"), pToolBtnWgt);
	pVLayout->addWidget(m_pFileGroupBox);
	pVLayout->addWidget(m_pOperatorGroupBox);


	m_pFileBtnGroup = new QButtonGroup(this);
	connect(m_pFileBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));

	m_pOperatorBtnGroup = new QButtonGroup(this);
	connect(m_pOperatorBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));


// 	m_pModelBtnGroup = new QButtonGroup(this);
// 	connect(m_pModelBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(BtnClickeSlot(int)));


	int iButtonID = 0;
	QGridLayout *pFileLayout = new QGridLayout;
	//pVLayout->addLayout(pFileLayout);
	pFileLayout->setSpacing(20);
	pFileLayout->setMargin(10);

	m_pFileGroupBox->setLayout(pFileLayout);

	//打开点边信息文件openMapBtn
	QPushButton *pBtn = CreateToolBtn(pToolBtnWgt, "打开点边信息文件", "operatorBtn", ":/Resources/Common/image/open.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 0, 0);

	//保存点边信息文件saveMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "保存点边信息文件", "operatorBtn", ":/Resources/Common/image/save.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 0, 1);

	//另存点边信息文件saveAsMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "另存点边信息文件", "operatorBtn", ":/Resources/Common/image/saveAs.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 1, 0);

	//下载2D地图download2DBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "下载2D地图", "operatorBtn", ":/Resources/Common/image/Download-2d.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 1, 1);

	//上传SmapuploadMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "上传Smap", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 2, 0);

	//下载Smap downloadMapBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "下载Smap", "operatorBtn", ":/Resources/Common/image/Download-smap.png");
	//上传mapinfo
	pBtn = CreateToolBtn(pToolBtnWgt, "上传点边信息文件", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 2, 1);

	QGridLayout *pOperatorLayout = new QGridLayout;
	//pVLayout->addLayout(pOperatorLayout);
	pOperatorLayout->setSpacing(20);
	pOperatorLayout->setMargin(10);

	m_pOperatorGroupBox->setLayout(pOperatorLayout);


	//选择工具selectBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "选择工具", "operatorBtn", ":/Resources/Common/image/select.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 0, 0);
	pBtn->setChecked(true);

	//手动加点addPointManulBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "手动加点", "operatorBtn", ":/Resources/Common/image/addPoint-manul.png");
	// 	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	// 	pOperatorLayout->addWidget(pBtn);

	// 	QPushButton *pOverFittingBtn = new QPushButton("拟合");
	// 	pOverFittingBtn->setObjectName("overfittingBtn");
	// 	connect(pOverFittingBtn, SIGNAL(clicked()), this, SLOT(OverFittingBtnSlot()));
	pBtn = CreateToolBtn(pToolBtnWgt, "拟合", "overfittingBtn", ":/Resources/Common/image/Fitting.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 0, 1);

	//添加路径slideBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "添加路径", "operatorBtn", ":/Resources/Common/image/Slide.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 1, 0);
	//添加高级区域advanceAreaBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "添加检修区域", "operatorBtn", ":/Resources/Common/image/AdvancedArea.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 1, 1);
	//重定位relocationBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "重定位", "operatorBtn", ":/Resources/Common/image/reLocation.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 2, 0);

	//扫描地图scanBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "扫描地图", "operatorBtn", ":/Resources/Common/image/Scann.png");
	//m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 2, 1);
	connect(pBtn, SIGNAL(clicked()), this, SLOT(ScannMapBtnSlot()));


// 	QWidget *pBottomWgt = new QWidget(this);
// 	//pMapToolLayout->addWidget(pBottomWgt);
// 
// 	QGridLayout *pControlLayout = new QGridLayout(pBottomWgt);
// 	pControlLayout->setHorizontalSpacing(20);
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("键盘");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pBtn->setChecked(true);
// 	pControlLayout->addWidget(pBtn, 0, 0);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("任务");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pControlLayout->addWidget(pBtn, 0, 1);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("手柄");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pControlLayout->addWidget(pBtn, 1, 0);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("紧急\n定位");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pControlLayout->addWidget(pBtn, 1, 1);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);

	QVBoxLayout *view_layout = new QVBoxLayout;
	//view_layout->addWidget(pBottomWgt);
	view_layout->addWidget(pToolBtnWgt);
	view_layout->addStretch();

	view_layout->addWidget(m_pCurrentModeLabel);	
	view_layout->addWidget(modeChangeWidget);
	//view_layout->addStretch();
	QWidget *right_widget = new QWidget;
	right_widget->setLayout(view_layout);
	right_widget->setFixedWidth(200);

	QHBoxLayout *top_layout = new QHBoxLayout;
	top_layout->addWidget(right_widget);
	top_layout->addWidget(m_pView);

	QVBoxLayout *main_layout = new QVBoxLayout;
	main_layout->addLayout(top_layout);
	main_layout->addWidget(m_pStatusBar);
	this->setLayout(main_layout);


}


void DLMapEditorWidget::slot_on_relocation()
{
	m_pScene->SetOperateType(DLOperator::TYPE_RELOCATION);
	m_pStatusBar->setOperate(tr("重定位"));
}


void DLMapEditorWidget::slot_on_picture()
{
	m_pStatusBar->setOperate(tr("添加图片"));
}


void DLMapEditorWidget::slot_on_undo()
{
	m_pScene->UndoStack();
	m_pStatusBar->setOperate(tr("撤销"));
}


void DLMapEditorWidget::slot_on_redo()
{

	m_pStatusBar->setOperate(tr("重做"));
}


void DLMapEditorWidget::slot_on_cut()
{
	m_pStatusBar->setOperate(tr("剪切"));
}


void DLMapEditorWidget::slot_on_copy()
{
	m_pStatusBar->setOperate(tr("拷贝"));
}


void DLMapEditorWidget::slot_on_paste()
{
	m_pStatusBar->setOperate(tr("粘贴"));
}


void DLMapEditorWidget::slot_on_delete()
{
	m_pScene->SetOperateType(DLOperator::TYPE_DEL);
	m_pStatusBar->setOperate(tr("删除"));
}


void DLMapEditorWidget::slot_on_selectall()
{
	m_pStatusBar->setOperate(tr("全选"));
}


void DLMapEditorWidget::slot_on_find()
{
	m_pStatusBar->setOperate(tr("查找"));
}


void DLMapEditorWidget::CutOutAreaBtnSlot()
{
	m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
	m_pScene->SetCutOutState(true);
	m_pStatusBar->setOperate(tr("截取"));
}

void DLMapEditorWidget::SetBgPixmapBtnSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("打开图片"), "D:/", tr("Images (*.png *.xpm *.jpg)"));
	m_pScene->SetBgPixmap(fileName);
}

void DLMapEditorWidget::slot_on_rotate()
{
	m_pScene->SetOperateType(DLOperator::TYPE_COORDINATE_TRANSFORM);
	m_pStatusBar->setOperate(tr("坐标变换"));
}


void DLMapEditorWidget::slot_on_quit()
{
	ClearMap();
	qApp->quit();
}


void DLMapEditorWidget::slot_on_select()
{
	m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
	m_pStatusBar->setOperate(tr("选择"));
}


void DLMapEditorWidget::slot_on_landmark()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_LANDMARK);
	m_pStatusBar->setOperate(tr("添加巡检点"));
}


void DLMapEditorWidget::slot_on_add_landmark()
{
	m_pScene->AddPatrolPoint();
	m_pStatusBar->setOperate(tr("添加巡检点"));
}


void DLMapEditorWidget::slot_on_bezier()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_EDGE);
	m_pStatusBar->setOperate(tr("添加巡检路径"));
}


void DLMapEditorWidget::slot_on_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_LINE);
	m_pStatusBar->setOperate(tr("添加线"));
}


void DLMapEditorWidget::slot_on_virtual_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_VIRTUAL_LINE);
	m_pStatusBar->setOperate(tr("添加虚线"));
}


void DLMapEditorWidget::slot_on_forbbidon_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_FORBBIDON_LINE);
	m_pStatusBar->setOperate(tr("添加禁止线"));
}


void DLMapEditorWidget::slot_on_advanced_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
	m_pStatusBar->setOperate(tr("添加高级区域"));
}


void DLMapEditorWidget::slot_on_device_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_DEVAREA);
	m_pStatusBar->setOperate(tr("添加设备区域"));
}


void DLMapEditorWidget::slot_on_station_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_STATION);
	m_pStatusBar->setOperate(tr("添加站点区域"));
}


void DLMapEditorWidget::slot_on_start_slam()
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_slam_req();
	m_pStatusBar->setOperate(tr("开始扫描地图"));
}


void DLMapEditorWidget::slot_on_end_slam()
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_endslam_req();
	m_pStatusBar->setOperate(tr("结束扫描地图"));
}


void DLMapEditorWidget::SelectedChangedSlot()
{
	QList<QGraphicsItem *> lstTemps = m_pScene->selectedItems();
	foreach(QGraphicsItem *pSelectedItem, m_lstSelectedsItems)
	{
		if (lstTemps.contains(pSelectedItem)) continue;
		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(pSelectedItem);
		if (NULL != pBezierItem)
		{
			//pBezierItem->set_control_point_visible(true);
			//pBezierItem->set_control_point_visible(false);
		}
	}
	m_lstSelectedsItems.clear();

	m_lstSelectedsItems = lstTemps;			//获取选中的节点
	foreach(QGraphicsItem *pSelectedItem, m_lstSelectedsItems)
	{
		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(pSelectedItem);
		if (NULL != pBezierItem)
		{
			//pBezierItem->set_control_point_visible(true);
			//pBezierItem->set_control_point_visible(true);
		}
	}
	
}

void DLMapEditorWidget::ChangeTypeSlot(const QString &strMessage)
{
	m_pStatusBar->setOperate(strMessage);
}

void DLMapEditorWidget::SMAPChangedSlot()
{
	if (!m_bSMAPIsChanged) 
	{
		m_bSMAPIsChanged = true;
	}
}

void DLMapEditorWidget::ReadFinishedSlot(bool bIsRunning)
{
	if (!bIsRunning && m_bIsUploadSmap)
	{
		m_bIsUploadSmap = false;
		UploadSMAPFile();
		//qDebug() << "file name:" << m_strFileName << __LINE__;
		emit OpenSmapFileSignal(m_strFileName);
	}
	m_pScene->removeall();
	m_bSMAPIsChanged = false;
	m_strFileName = "";
}

void DLMapEditorWidget::BtnClickeSlot(int iID)
{
	switch (iID)
	{
	case OPEN_SMAP_TYPE:					//打开点边信息文件
	{
		QString strMapDir = QFileDialog::getExistingDirectory(this, tr("选择地图"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		m_pScene->removeall();
		m_pScene->load_json_map(strMapDir);

		QDir dir(strMapDir);
		//smap文件
		QStringList nameFilters;
		nameFilters.clear();
		nameFilters << "*.mapinfo";
		QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
		if (files.size() > 0)
		{//默认取第一个
			m_strOpenMapPath = QString("%1/%2").arg(strMapDir).arg(files.at(0));
		}
		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVE_SMAP_TYPE:					//保存点边信息文件
	{
		m_pStatusBar->setOperate("保存地图");
		QString file = m_strOpenMapPath.split("/").last();
		if (m_strOpenMapPath.isEmpty() || m_strOpenMapPath.isNull())
		{//保存文件不存在，则另存为
			//m_pStatusBar->setOperate(file + "保存失败。");
			BtnClickeSlot(SAVEAS_SMAP_TYPE);
			return;
		}

		m_pScene->save_json_map(m_strOpenMapPath);
		m_pStatusBar->setOperate(file + "保存成功。");

		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVEAS_SMAP_TYPE:					//另存点边信息文件		
	{
		m_pStatusBar->setOperate("地图另存为");
		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.mapinfo)"));
		QString file = fileName.split("/").last();
		if (fileName.isEmpty() || fileName.isNull()) {
			m_pStatusBar->setOperate(file + "保存失败。");
			return;
		}

		m_pScene->save_json_map(fileName);
		m_pStatusBar->setOperate(file + "保存成功。");

		//m_bSMAPIsChanged = false;

	}break;
	case DOWNLOAD_2D_MAP_TYPE:				//下载2D地图
	{
		//获取2dmap列表 
		m_pMapListWgt->set_type(DataTransfer::FILE_2D);
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_2d_map_query_req();
		m_pMapListWgt->show();

		m_pStatusBar->setOperate(("正在下载2dmap地图"));
	}break;
	case UPLOAD_SMAP_TYPE:					//上传Smap
	{
		//向core上传smap地图到巡检车
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), strDir, tr("Maps (*.smap)"));

		TransferPro file_pro;
		file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
		//file_pro.file_name_ = boost::filesystem::path(m_strFileName.toLocal8Bit().data()).filename().string();
		file_pro.file_name_ = boost::filesystem::path(fileName.toLocal8Bit().data()).filename().string();
		file_pro.dst_relative_path_ = "smap";
		//file_pro.src_file_path_ = m_strFileName.toLocal8Bit();
		file_pro.src_file_path_ = fileName.toLocal8Bit();

		//if (QFile::exists(m_strFileName))
		if (QFile::exists(fileName))
		{
			m_pThreadDataTransfer->set_transfer_info(file_pro);
			m_pThreadDataTransfer->start();
			m_pStatusBar->setOperate(tr("正在上传smap地图"));
		}
	}break;
	// 	case DOWNLOAD_SMAP_TYPE:				//下载Smap
	// 	{
	// 		// 获取smap列表
	// 		m_pMapListWgt->set_type(DataTransfer::FILE_SMAP);
	// 		WHEEL_BACK_TO_CORE_SOCKET.robot_config_smap_query_req();
	// 		m_pMapListWgt->show();
	// 
	// 
	// 		m_pStatusBar->setOperate(("正在下载smap地图"));
	// 	}break;
	case UPLOAD_MAPINFO_TYPE:				//上传点边信息文件
	{
		//向core上传smap地图到巡检车
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), strDir, tr("Maps (*.mapinfo)"));

		TransferPro file_pro;
		file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
		file_pro.file_name_ = boost::filesystem::path(fileName.toLocal8Bit().data()).filename().string();
		file_pro.dst_relative_path_ = "smap";
		file_pro.src_file_path_ = fileName.toLocal8Bit();

		if (QFile::exists(fileName))
		{
			m_pThreadDataTransfer->set_transfer_info(file_pro);
			m_pThreadDataTransfer->start();
			m_pStatusBar->setOperate(tr("正在上传点边信息文件"));
		}
	}break;
	case SELECT_TYPE:						//选择
	{
		m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
		m_pStatusBar->setOperate(tr("选择"));
	}break;
	case OVERFITTING_TYPE:					//拟合
	{
		if (NULL != m_pScene)
		{
			m_pScene->ShowOverfittingDlg();
		}
	}break;
	case ADD_PATH_TYPE:						//添加路径
	{
		m_pScene->SetOperateType(DLOperator::TYPE_ADD_EDGE);
		m_pStatusBar->setOperate(tr("添加巡检路径"));
	}break;
	case ADD_ADVANCED_AREA_TYPE:			//高级区域
	{
		m_pScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
		m_pStatusBar->setOperate(tr("添加高级区域"));
	}break;
	case RELOCATION_TYPE:					//重定位
	{
		m_pScene->SetOperateType(DLOperator::TYPE_RELOCATION);
		m_pStatusBar->setOperate(tr("重定位"));
	}break;
	case KEY_MODE_TYPE:						//键盘模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_REMOTE_CTRL));
	}break;
	case TASK_MODE_TYPE:					//任务模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_AUTORUNNING));
	}break;
	case HAND_MODE_TYPE:					//手柄模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_JOY_CTRL));
	}break;
	case URGENCY_RELOCATION_TYPE:			//紧急定位模式
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_EMERGENCY_LOC));
	}break;
	default:break;
	}
}

void DLMapEditorWidget::slot_on_open_smap(QString file)
{
    m_pScene->removeall();
    m_pScene->load_json_map(file);
}


void DLMapEditorWidget::slot_on_load_smap()
{
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), strDir, tr("Maps (*.smap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	m_strFileName = fileName;
	m_pScene->removeall();
	m_pScene->load_json_map(fileName);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(tr("加载") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_save_smap()
{
	m_pStatusBar->setOperate("保存地图");
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.smap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	
	m_pScene->save_json_map(fileName);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(file + "保存成功。");

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_load_backstagelandmark()
{
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Template Map"), strDir, tr("Maps (*.smap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	m_pScene->removeall(true);
	m_pScene->load_json_map(fileName, true);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(tr("加载") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_connect_backstagelandmark()
{
	m_pScene->connect_backstage_map_path();
	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_load_backstage_map()
{
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), strDir, tr("Maps (*.bsmap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	m_pScene->removeall();
	//m_pScene->load_backstage_map(fileName);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(tr("加载") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_save_backstage_map()
{
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.bsmap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	//m_pScene->save_backstage_map(fileName);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(tr("加载") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_unload_smap()
{
	m_pScene->removeall();
	m_pStatusBar->setOperate(tr("卸载地图"));

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_set_build_map_param()
{
	//激光参数设置接口
	laserParamWidget_->show();
	m_pStatusBar->setOperate(("正在设置构建smap地图参数"), 0, false);

}


void DLMapEditorWidget::slot_on_build_map()
{
	BuildSmapParam parm = laserParamWidget_->getLaserParam();
	smapBuilder_->set_map_param(parm);
	smapBuilder_->start();
	smapBuilder_->start_timer();
	m_pStatusBar->setOperate(("正在构建smap"), 0, true);
}


void DLMapEditorWidget::slot_on_download_2dmap()
{
	//获取2dmap列表 
	//mapListWidget_->set_type(DataTransfer::FILE_2D);
    WHEEL_BACK_TO_CORE_SOCKET.robot_config_2d_map_query_req();
	//mapListWidget_->show();

	m_pStatusBar->setOperate(("正在下载2dmap地图"));
}


void DLMapEditorWidget::slot_on_download_smap()
{
	//获取smap列表 
	//mapListWidget_->set_type(DataTransfer::FILE_SMAP);
	WHEEL_BACK_TO_CORE_SOCKET.robot_config_smap_query_req();
	//mapListWidget_->show();


	m_pStatusBar->setOperate(("正在下载smap地图"));
}


void DLMapEditorWidget::slot_on_upload_smap()
{
	QString filepath = QFileDialog::getOpenFileName(this, tr("选择上传的smap"), tr("D:/RCF_Server/smap"), tr("Maps (*.smap)"));
	if (filepath.isEmpty() || filepath.isNull()) {
		return;
	}

	//向core上传smap地图到巡检车
	if ( NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning() ) {
		return;
	}

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
	file_pro.file_name_ = boost::filesystem::path(filepath.toLocal8Bit().data()).filename().string();
	file_pro.dst_relative_path_ = "smap";
	file_pro.src_file_path_ = filepath.toLocal8Bit();

	m_pThreadDataTransfer->set_transfer_info(file_pro);
	m_pThreadDataTransfer->start();
	m_pStatusBar->setOperate(tr("正在上传smap地图"));
}


void DLMapEditorWidget::slot_on_build_process(float progress)
{
	m_pStatusBar->setOperate(tr("正在构建smap,"), progress, true);
}


void DLMapEditorWidget::slot_on_build_finished()
{
	smapBuilder_->stop_timer();
    m_pStatusBar->setOperate(tr("smap地图构建完成"), 100, true);
}


void DLMapEditorWidget::slot_on_choose_map(QString download_path, QString filename, int file_type)
{
    //mapListWidget_->hide();
    if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
        return;
    }

	TransferPro file_pro;
	file_pro.cmd_type_ = DataTransfer::DOWNLOAD_FILE;
	file_pro.file_name_ = filename.toLocal8Bit();
	file_pro.download_path_ = download_path.toLocal8Bit();

	if (file_type == DataTransfer::FILE_SMAP) {
		file_pro.dst_relative_path_ = "smap";
	}
	else if(file_type == DataTransfer::FILE_2D){
		file_pro.dst_relative_path_ = "2dmap";
	}

	m_pThreadDataTransfer->set_transfer_info(file_pro);
	m_pThreadDataTransfer->start();
	m_pStatusBar->setOperate(tr("正在下载地图"));

}


void DLMapEditorWidget::slot_on_transfer_finished(int command_type, int execCode, QString fileName)
{
	switch (command_type)
	{
    case DataTransfer::DOWNLOAD_FILE:
	{
		if (!execCode) {
			m_pStatusBar->setOperate(fileName + " 下载完毕");
		}
		else {
			m_pStatusBar->setOperate(fileName + "下载出错");
		}
		break;
	}
    case DataTransfer::UPLOAD_FILE:
	{
		if (!execCode) 
		{
            //下载完毕给core回一条包含下载的好文件名的消息
			WHEEL_BACK_TO_CORE_SOCKET.robot_config_uploadmap_req(fileName);
			m_pStatusBar->setOperate(fileName + "上传成功");
		}
		else {
			m_pStatusBar->setOperate(fileName + "上传出错");
		}
		break;
	}
	default:
		break;
	}

}


void DLMapEditorWidget::smapUploadCompleted(bool retcode, QString desc)
{
	if (retcode) {
		m_pStatusBar->setOperate("smap上传同步完毕");
	}
	else {
		m_pStatusBar->setOperate("smap上传同步失败");
	}
	ROS_ERROR("smapUploadCompleted err:", desc.toLocal8Bit().constData());
}

void DLMapEditorWidget::closeEvent(QCloseEvent *event)
{
	if (m_bSMAPIsChanged)
	{

	}
}

