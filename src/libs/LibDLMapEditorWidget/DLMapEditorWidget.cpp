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

	//���µ�ͼ�ϴ�״̬
	WHEEL_BACK_TO_CORE_SOCKET.wheelUploadMap2Robot.connect(boost::bind(&DLMapEditorWidget::smapUploadCompleted, this, _1, _2));
	

    // ���µ�ǰ������ģʽ״̬;
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotSwitchRunningStatusSignal.connect(boost::bind(&DLMapEditorWidget::signalRobotModeStateCallBack, this, _1));

    connect(this, &DLMapEditorWidget::signalRobotModeStateCallBack, this, [=](WheelRobotSwitchRunningStatus status) {
        switch (status)
        {
        case WHEEL_ROBOT_SWITCH_AUTORUNNING:
            m_pCurrentModeLabel->setShowValue("����ģʽ");
            break;
        case WHEEL_ROBOT_SWITCH_REMOTE_CTRL:
            m_pCurrentModeLabel->setShowValue("����ģʽ");
            break;
        case WHEEL_ROBOT_SWITCH_JOY_CTRL:
            m_pCurrentModeLabel->setShowValue("�ֱ�ģʽ");
            break;
        case WHEEL_ROBOT_SWITCH_EMERGENCY_LOC:
            m_pCurrentModeLabel->setShowValue("��λģʽ");
            break;
        default:
            break;
        }
    });

}


void DLMapEditorWidget::SaveSMAPFile()
{
	m_pStatusBar->setOperate("�����ͼ");
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
	m_pStatusBar->setOperate(file + "����ɹ���");

	m_bSMAPIsChanged = false;
}

void DLMapEditorWidget::SaveAsSMAPFile(QString &strSaveAsFile)
{
	m_pStatusBar->setOperate("���Ϊ��ͼ");
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	strSaveAsFile = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.smap)"));
	if (strSaveAsFile.isEmpty() || strSaveAsFile.isNull()) {
		return;
	}

	m_pScene->save_json_map(strSaveAsFile);
	QString file = strSaveAsFile.split("/").last();
	m_pStatusBar->setOperate(file + "���Ϊ�ɹ���");

	m_bSMAPIsChanged = false;
}

void DLMapEditorWidget::SaveBSMAPFile(QString strFilePath)
{
	if (strFilePath.isEmpty())
	{
		QString strFile = m_strFileName;
		//m_pScene->save_backstage_map(strFile.replace(".smap", ".bsmap"));
		QString file = strFile.split("/").last().replace(".smap", ".bsmap");
		m_pStatusBar->setOperate(tr("����") + file);
	}
	else
	{
		//m_pScene->save_backstage_map(strFilePath.replace(".smap", ".bsmap"));
		QString file = strFilePath.split("/").last().replace(".smap", ".bsmap");
		m_pStatusBar->setOperate(tr("����") + file);
	}

}

void DLMapEditorWidget::UploadSMAPFile()
{
	//��core�ϴ�smap��ͼ��Ѳ�쳵
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
		m_pStatusBar->setOperate(tr("�����ϴ�smap��ͼ"));
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
	{//�Ƿ����޸ģ�����У���Ҫ����Ϊsmap��ͼ������dsmap��ͼ���ϴ�smap��ͼ��������
		QMessageBox msgBox;
		msgBox.setWindowTitle("��ʾ");
		msgBox.setText("�û�smap���޸ģ���δ���棿");
		//msgBox.setInformativeText("Do you want to save your changes?");
		msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::SaveAll | QMessageBox::Cancel);
		msgBox.setButtonText(QMessageBox::Save, "���µ�ͼ");
		msgBox.setButtonText(QMessageBox::SaveAll, "����ͼ");
		msgBox.setButtonText(QMessageBox::Cancel, "�����޸�");
		msgBox.setDefaultButton(QMessageBox::Save);
		int ret = msgBox.exec();
		if (QMessageBox::Save == ret)
		{//�����ͼ
			SaveSMAPFile();					//����smap�ļ�
			SaveBSMAPFile();				//����bsmap�ļ�
			//UploadSMAPFile();				//�ϴ�smap
			m_bIsUploadSmap = true;
		}
		else if (QMessageBox::SaveAll == ret)
		{//���Ϊ
			QString strSaveAsFile = "";
			SaveAsSMAPFile(strSaveAsFile);				//���Ϊsmap�ļ�,�����Ϊ�����л�ȡ�ļ�·��
			SaveBSMAPFile(strSaveAsFile);				//����bsmap�ļ�
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
//     m_pCollectStartSlamBtn = new QPushButton(tr("��ʼɨ��"));
//     m_pCollectEndSlamBtn = new QPushButton(tr("����ɨ��"));
//     m_pDownload2DBtn = new QPushButton(tr("����2D��ͼ"));
// 	m_pDownloadSMAPBtn = new QPushButton("����smap��ͼ");
//     m_pUploadSMAPBtn = new QPushButton(tr("�ϴ�smap"));
//     m_pCreateSMAPBtn = new QPushButton(tr("����smap��ͼ"));
// 
//     QGroupBox *collect_groupBox = new QGroupBox(tr("��ͼ�ɼ�"));
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
// 	m_pOpenSMAPBtn = new QPushButton(tr("��smap��ͼ"));
// 	m_pSaveSMAPBtn = new QPushButton(tr("����smap��ͼ"));
// 
// 
// 
// 
// 	m_pUnloadMapBtn = new QPushButton(tr("ж�ص�ͼ"));
// 	m_pQuitBtn = new QPushButton(tr("�˳�"));
// 
// 	QGroupBox *file_groupBox = new QGroupBox(tr("��ͼ�ļ�"));
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
// 	edit_undo_button_ = new QPushButton(tr("����"));
// 	edit_redo_button_ = new QPushButton(tr("����"));
// 	edit_cut_button_ = new QPushButton(tr("����"));
// 	edit_copy_button_ = new QPushButton(tr("����"));
// 	edit_paste_button_ = new QPushButton(tr("ճ��"));
// 	edit_delete_button_ = new QPushButton(tr("ɾ��"));
// 	edit_selectall_button_ = new QPushButton(tr("ȫѡ"));
// 	edit_find_button_ = new QPushButton(tr("����"));
// 	m_pCutOutAreaBtn = new QPushButton(tr("��ȡ"));
// 	m_pSetBgPixmapBtn = new QPushButton(tr("���ñ���ͼƬ"));
// 
// 
// 
// 	QGroupBox *edit_groupBox = new QGroupBox(tr("��ͼ����"));
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
// 	//operate_add_relocation_button_ = new QPushButton("Ѳ���");
// 
// 	operate_select_button_->setToolTip("ѡ��");
// 	operate_coordinate_transform_button_->setToolTip("����任");
// 	operate_landmark_button_->setToolTip("Ѳ���");
// 	operate_bezier_button_->setToolTip("Ѳ��·��");
// 	operate_advanced_area_button_->setToolTip("�߼�����");
// 	operate_normal_line_button_->setToolTip("��ͨ��");
// 	operate_virtual_line_button_->setToolTip("����");
// 	operate_forbidden_line_button_->setToolTip("��ֹ��");
// 	operate_picture_button_->setToolTip("ͼƬ");
// 	operate_device_area_button_->setToolTip("�豸����");
// 	operate_station_button_->setToolTip("��վ����");
// 	operate_relocation_button_->setToolTip("�ض���");
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
// 	QGroupBox *operate_groupBox = new QGroupBox(tr("��ͼ�༭"));
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
 	// ��ǰģʽ;
    m_pCurrentModeLabel = new InputWidget(InputWidgetType::WheelValueShowWidget);
    m_pCurrentModeLabel->setTipText("��ǰģʽ");
    m_pCurrentModeLabel->setFixedSize(QSize(160, 20));
	// ģʽѡ��;
	QStringList modeList = QStringList() << ("����ģʽ") << ("����ģʽ") << ("�ֱ�ģʽ") << ("��λģʽ");
	InputWidget *modeChangeWidget = new InputWidget(InputWidgetType::ComboBox);
	modeChangeWidget->setTipText("ģʽѡ��:");
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

	m_pFileGroupBox = new QGroupBox(tr("��ͼ�ļ�"), pToolBtnWgt);
	m_pOperatorGroupBox = new QGroupBox(tr("��ͼ����"), pToolBtnWgt);
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

	//�򿪵����Ϣ�ļ�openMapBtn
	QPushButton *pBtn = CreateToolBtn(pToolBtnWgt, "�򿪵����Ϣ�ļ�", "operatorBtn", ":/Resources/Common/image/open.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 0, 0);

	//��������Ϣ�ļ�saveMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "��������Ϣ�ļ�", "operatorBtn", ":/Resources/Common/image/save.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 0, 1);

	//�������Ϣ�ļ�saveAsMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "�������Ϣ�ļ�", "operatorBtn", ":/Resources/Common/image/saveAs.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 1, 0);

	//����2D��ͼdownload2DBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "����2D��ͼ", "operatorBtn", ":/Resources/Common/image/Download-2d.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 1, 1);

	//�ϴ�SmapuploadMapBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "�ϴ�Smap", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 2, 0);

	//����Smap downloadMapBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "����Smap", "operatorBtn", ":/Resources/Common/image/Download-smap.png");
	//�ϴ�mapinfo
	pBtn = CreateToolBtn(pToolBtnWgt, "�ϴ������Ϣ�ļ�", "operatorBtn", ":/Resources/Common/image/upload.png");
	pBtn->setCheckable(false);
	m_pFileBtnGroup->addButton(pBtn, iButtonID++);
	pFileLayout->addWidget(pBtn, 2, 1);

	QGridLayout *pOperatorLayout = new QGridLayout;
	//pVLayout->addLayout(pOperatorLayout);
	pOperatorLayout->setSpacing(20);
	pOperatorLayout->setMargin(10);

	m_pOperatorGroupBox->setLayout(pOperatorLayout);


	//ѡ�񹤾�selectBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "ѡ�񹤾�", "operatorBtn", ":/Resources/Common/image/select.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 0, 0);
	pBtn->setChecked(true);

	//�ֶ��ӵ�addPointManulBtn
	//pBtn = CreateToolBtn(pToolBtnWgt, "�ֶ��ӵ�", "operatorBtn", ":/Resources/Common/image/addPoint-manul.png");
	// 	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	// 	pOperatorLayout->addWidget(pBtn);

	// 	QPushButton *pOverFittingBtn = new QPushButton("���");
	// 	pOverFittingBtn->setObjectName("overfittingBtn");
	// 	connect(pOverFittingBtn, SIGNAL(clicked()), this, SLOT(OverFittingBtnSlot()));
	pBtn = CreateToolBtn(pToolBtnWgt, "���", "overfittingBtn", ":/Resources/Common/image/Fitting.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 0, 1);

	//���·��slideBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "���·��", "operatorBtn", ":/Resources/Common/image/Slide.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 1, 0);
	//��Ӹ߼�����advanceAreaBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "��Ӽ�������", "operatorBtn", ":/Resources/Common/image/AdvancedArea.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 1, 1);
	//�ض�λrelocationBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "�ض�λ", "operatorBtn", ":/Resources/Common/image/reLocation.png");
	m_pOperatorBtnGroup->addButton(pBtn, iButtonID++);
	pOperatorLayout->addWidget(pBtn, 2, 0);

	//ɨ���ͼscanBtn
	pBtn = CreateToolBtn(pToolBtnWgt, "ɨ���ͼ", "operatorBtn", ":/Resources/Common/image/Scann.png");
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
// 	pBtn->setText("����");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pBtn->setChecked(true);
// 	pControlLayout->addWidget(pBtn, 0, 0);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("����");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pControlLayout->addWidget(pBtn, 0, 1);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("�ֱ�");
// 	pBtn->setFixedSize(60, 50);
// 	pBtn->setCheckable(true);
// 	pControlLayout->addWidget(pBtn, 1, 0);
// 	m_pModelBtnGroup->addButton(pBtn, iButtonID++);
// 
// 	pBtn = new QPushButton(pBottomWgt);
// 	pBtn->setObjectName("redBtn");
// 	pBtn->setText("����\n��λ");
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
	m_pStatusBar->setOperate(tr("�ض�λ"));
}


void DLMapEditorWidget::slot_on_picture()
{
	m_pStatusBar->setOperate(tr("���ͼƬ"));
}


void DLMapEditorWidget::slot_on_undo()
{
	m_pScene->UndoStack();
	m_pStatusBar->setOperate(tr("����"));
}


void DLMapEditorWidget::slot_on_redo()
{

	m_pStatusBar->setOperate(tr("����"));
}


void DLMapEditorWidget::slot_on_cut()
{
	m_pStatusBar->setOperate(tr("����"));
}


void DLMapEditorWidget::slot_on_copy()
{
	m_pStatusBar->setOperate(tr("����"));
}


void DLMapEditorWidget::slot_on_paste()
{
	m_pStatusBar->setOperate(tr("ճ��"));
}


void DLMapEditorWidget::slot_on_delete()
{
	m_pScene->SetOperateType(DLOperator::TYPE_DEL);
	m_pStatusBar->setOperate(tr("ɾ��"));
}


void DLMapEditorWidget::slot_on_selectall()
{
	m_pStatusBar->setOperate(tr("ȫѡ"));
}


void DLMapEditorWidget::slot_on_find()
{
	m_pStatusBar->setOperate(tr("����"));
}


void DLMapEditorWidget::CutOutAreaBtnSlot()
{
	m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
	m_pScene->SetCutOutState(true);
	m_pStatusBar->setOperate(tr("��ȡ"));
}

void DLMapEditorWidget::SetBgPixmapBtnSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("��ͼƬ"), "D:/", tr("Images (*.png *.xpm *.jpg)"));
	m_pScene->SetBgPixmap(fileName);
}

void DLMapEditorWidget::slot_on_rotate()
{
	m_pScene->SetOperateType(DLOperator::TYPE_COORDINATE_TRANSFORM);
	m_pStatusBar->setOperate(tr("����任"));
}


void DLMapEditorWidget::slot_on_quit()
{
	ClearMap();
	qApp->quit();
}


void DLMapEditorWidget::slot_on_select()
{
	m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
	m_pStatusBar->setOperate(tr("ѡ��"));
}


void DLMapEditorWidget::slot_on_landmark()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_LANDMARK);
	m_pStatusBar->setOperate(tr("���Ѳ���"));
}


void DLMapEditorWidget::slot_on_add_landmark()
{
	m_pScene->AddPatrolPoint();
	m_pStatusBar->setOperate(tr("���Ѳ���"));
}


void DLMapEditorWidget::slot_on_bezier()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_EDGE);
	m_pStatusBar->setOperate(tr("���Ѳ��·��"));
}


void DLMapEditorWidget::slot_on_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_LINE);
	m_pStatusBar->setOperate(tr("�����"));
}


void DLMapEditorWidget::slot_on_virtual_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_VIRTUAL_LINE);
	m_pStatusBar->setOperate(tr("�������"));
}


void DLMapEditorWidget::slot_on_forbbidon_line()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_FORBBIDON_LINE);
	m_pStatusBar->setOperate(tr("��ӽ�ֹ��"));
}


void DLMapEditorWidget::slot_on_advanced_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
	m_pStatusBar->setOperate(tr("��Ӹ߼�����"));
}


void DLMapEditorWidget::slot_on_device_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_DEVAREA);
	m_pStatusBar->setOperate(tr("����豸����"));
}


void DLMapEditorWidget::slot_on_station_area()
{
	m_pScene->SetOperateType(DLOperator::TYPE_ADD_STATION);
	m_pStatusBar->setOperate(tr("���վ������"));
}


void DLMapEditorWidget::slot_on_start_slam()
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_slam_req();
	m_pStatusBar->setOperate(tr("��ʼɨ���ͼ"));
}


void DLMapEditorWidget::slot_on_end_slam()
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_endslam_req();
	m_pStatusBar->setOperate(tr("����ɨ���ͼ"));
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

	m_lstSelectedsItems = lstTemps;			//��ȡѡ�еĽڵ�
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
	case OPEN_SMAP_TYPE:					//�򿪵����Ϣ�ļ�
	{
		QString strMapDir = QFileDialog::getExistingDirectory(this, tr("ѡ���ͼ"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		m_pScene->removeall();
		m_pScene->load_json_map(strMapDir);

		QDir dir(strMapDir);
		//smap�ļ�
		QStringList nameFilters;
		nameFilters.clear();
		nameFilters << "*.mapinfo";
		QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
		if (files.size() > 0)
		{//Ĭ��ȡ��һ��
			m_strOpenMapPath = QString("%1/%2").arg(strMapDir).arg(files.at(0));
		}
		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVE_SMAP_TYPE:					//��������Ϣ�ļ�
	{
		m_pStatusBar->setOperate("�����ͼ");
		QString file = m_strOpenMapPath.split("/").last();
		if (m_strOpenMapPath.isEmpty() || m_strOpenMapPath.isNull())
		{//�����ļ������ڣ������Ϊ
			//m_pStatusBar->setOperate(file + "����ʧ�ܡ�");
			BtnClickeSlot(SAVEAS_SMAP_TYPE);
			return;
		}

		m_pScene->save_json_map(m_strOpenMapPath);
		m_pStatusBar->setOperate(file + "����ɹ���");

		m_bSMAPIsChanged = false;
		m_pFileBtnGroup->button(SAVE_SMAP_TYPE)->setEnabled(false);

	}break;
	case SAVEAS_SMAP_TYPE:					//�������Ϣ�ļ�		
	{
		m_pStatusBar->setOperate("��ͼ���Ϊ");
		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.mapinfo)"));
		QString file = fileName.split("/").last();
		if (fileName.isEmpty() || fileName.isNull()) {
			m_pStatusBar->setOperate(file + "����ʧ�ܡ�");
			return;
		}

		m_pScene->save_json_map(fileName);
		m_pStatusBar->setOperate(file + "����ɹ���");

		//m_bSMAPIsChanged = false;

	}break;
	case DOWNLOAD_2D_MAP_TYPE:				//����2D��ͼ
	{
		//��ȡ2dmap�б� 
		m_pMapListWgt->set_type(DataTransfer::FILE_2D);
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_2d_map_query_req();
		m_pMapListWgt->show();

		m_pStatusBar->setOperate(("��������2dmap��ͼ"));
	}break;
	case UPLOAD_SMAP_TYPE:					//�ϴ�Smap
	{
		//��core�ϴ�smap��ͼ��Ѳ�쳵
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("ѡ���ļ�"), strDir, tr("Maps (*.smap)"));

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
			m_pStatusBar->setOperate(tr("�����ϴ�smap��ͼ"));
		}
	}break;
	// 	case DOWNLOAD_SMAP_TYPE:				//����Smap
	// 	{
	// 		// ��ȡsmap�б�
	// 		m_pMapListWgt->set_type(DataTransfer::FILE_SMAP);
	// 		WHEEL_BACK_TO_CORE_SOCKET.robot_config_smap_query_req();
	// 		m_pMapListWgt->show();
	// 
	// 
	// 		m_pStatusBar->setOperate(("��������smap��ͼ"));
	// 	}break;
	case UPLOAD_MAPINFO_TYPE:				//�ϴ������Ϣ�ļ�
	{
		//��core�ϴ�smap��ͼ��Ѳ�쳵
		if (NULL == m_pThreadDataTransfer || m_pThreadDataTransfer->isRunning()) {
			return;
		}

		WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
		QString strDir = stMapInfo.rootPath + "/map/";
		QString fileName = QFileDialog::getOpenFileName(this, tr("ѡ���ļ�"), strDir, tr("Maps (*.mapinfo)"));

		TransferPro file_pro;
		file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
		file_pro.file_name_ = boost::filesystem::path(fileName.toLocal8Bit().data()).filename().string();
		file_pro.dst_relative_path_ = "smap";
		file_pro.src_file_path_ = fileName.toLocal8Bit();

		if (QFile::exists(fileName))
		{
			m_pThreadDataTransfer->set_transfer_info(file_pro);
			m_pThreadDataTransfer->start();
			m_pStatusBar->setOperate(tr("�����ϴ������Ϣ�ļ�"));
		}
	}break;
	case SELECT_TYPE:						//ѡ��
	{
		m_pScene->SetOperateType(DLOperator::TYPE_SELECT);
		m_pStatusBar->setOperate(tr("ѡ��"));
	}break;
	case OVERFITTING_TYPE:					//���
	{
		if (NULL != m_pScene)
		{
			m_pScene->ShowOverfittingDlg();
		}
	}break;
	case ADD_PATH_TYPE:						//���·��
	{
		m_pScene->SetOperateType(DLOperator::TYPE_ADD_EDGE);
		m_pStatusBar->setOperate(tr("���Ѳ��·��"));
	}break;
	case ADD_ADVANCED_AREA_TYPE:			//�߼�����
	{
		m_pScene->SetOperateType(DLOperator::TYPE_ADD_AREA);
		m_pStatusBar->setOperate(tr("��Ӹ߼�����"));
	}break;
	case RELOCATION_TYPE:					//�ض�λ
	{
		m_pScene->SetOperateType(DLOperator::TYPE_RELOCATION);
		m_pStatusBar->setOperate(tr("�ض�λ"));
	}break;
	case KEY_MODE_TYPE:						//����ģʽ
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_REMOTE_CTRL));
	}break;
	case TASK_MODE_TYPE:					//����ģʽ
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_AUTORUNNING));
	}break;
	case HAND_MODE_TYPE:					//�ֱ�ģʽ
	{
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(WHEEL_ROBOT_SWITCH_JOY_CTRL));
	}break;
	case URGENCY_RELOCATION_TYPE:			//������λģʽ
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
	m_pStatusBar->setOperate(tr("����") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_save_smap()
{
	m_pStatusBar->setOperate("�����ͼ");
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
	QString strDir = stMapInfo.rootPath + "/map/";
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), strDir, tr("Maps (*.smap)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	
	m_pScene->save_json_map(fileName);
	QString file = fileName.split("/").last();
	m_pStatusBar->setOperate(file + "����ɹ���");

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
	m_pStatusBar->setOperate(tr("����") + file);

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
	m_pStatusBar->setOperate(tr("����") + file);

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
	m_pStatusBar->setOperate(tr("����") + file);

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_unload_smap()
{
	m_pScene->removeall();
	m_pStatusBar->setOperate(tr("ж�ص�ͼ"));

	m_bSMAPIsChanged = false;
}


void DLMapEditorWidget::slot_on_set_build_map_param()
{
	//����������ýӿ�
	laserParamWidget_->show();
	m_pStatusBar->setOperate(("�������ù���smap��ͼ����"), 0, false);

}


void DLMapEditorWidget::slot_on_build_map()
{
	BuildSmapParam parm = laserParamWidget_->getLaserParam();
	smapBuilder_->set_map_param(parm);
	smapBuilder_->start();
	smapBuilder_->start_timer();
	m_pStatusBar->setOperate(("���ڹ���smap"), 0, true);
}


void DLMapEditorWidget::slot_on_download_2dmap()
{
	//��ȡ2dmap�б� 
	//mapListWidget_->set_type(DataTransfer::FILE_2D);
    WHEEL_BACK_TO_CORE_SOCKET.robot_config_2d_map_query_req();
	//mapListWidget_->show();

	m_pStatusBar->setOperate(("��������2dmap��ͼ"));
}


void DLMapEditorWidget::slot_on_download_smap()
{
	//��ȡsmap�б� 
	//mapListWidget_->set_type(DataTransfer::FILE_SMAP);
	WHEEL_BACK_TO_CORE_SOCKET.robot_config_smap_query_req();
	//mapListWidget_->show();


	m_pStatusBar->setOperate(("��������smap��ͼ"));
}


void DLMapEditorWidget::slot_on_upload_smap()
{
	QString filepath = QFileDialog::getOpenFileName(this, tr("ѡ���ϴ���smap"), tr("D:/RCF_Server/smap"), tr("Maps (*.smap)"));
	if (filepath.isEmpty() || filepath.isNull()) {
		return;
	}

	//��core�ϴ�smap��ͼ��Ѳ�쳵
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
	m_pStatusBar->setOperate(tr("�����ϴ�smap��ͼ"));
}


void DLMapEditorWidget::slot_on_build_process(float progress)
{
	m_pStatusBar->setOperate(tr("���ڹ���smap,"), progress, true);
}


void DLMapEditorWidget::slot_on_build_finished()
{
	smapBuilder_->stop_timer();
    m_pStatusBar->setOperate(tr("smap��ͼ�������"), 100, true);
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
	m_pStatusBar->setOperate(tr("�������ص�ͼ"));

}


void DLMapEditorWidget::slot_on_transfer_finished(int command_type, int execCode, QString fileName)
{
	switch (command_type)
	{
    case DataTransfer::DOWNLOAD_FILE:
	{
		if (!execCode) {
			m_pStatusBar->setOperate(fileName + " �������");
		}
		else {
			m_pStatusBar->setOperate(fileName + "���س���");
		}
		break;
	}
    case DataTransfer::UPLOAD_FILE:
	{
		if (!execCode) 
		{
            //������ϸ�core��һ���������صĺ��ļ�������Ϣ
			WHEEL_BACK_TO_CORE_SOCKET.robot_config_uploadmap_req(fileName);
			m_pStatusBar->setOperate(fileName + "�ϴ��ɹ�");
		}
		else {
			m_pStatusBar->setOperate(fileName + "�ϴ�����");
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
		m_pStatusBar->setOperate("smap�ϴ�ͬ�����");
	}
	else {
		m_pStatusBar->setOperate("smap�ϴ�ͬ��ʧ��");
	}
	ROS_ERROR("smapUploadCompleted err:", desc.toLocal8Bit().constData());
}

void DLMapEditorWidget::closeEvent(QCloseEvent *event)
{
	if (m_bSMAPIsChanged)
	{

	}
}

