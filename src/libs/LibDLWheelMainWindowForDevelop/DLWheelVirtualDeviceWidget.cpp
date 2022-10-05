#include "DLWheelVirtualDeviceWidget.h"
#include <QHBoxLayout>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <QFileDialog>

#pragma execution_character_set("utf-8")

DLWheelVirtualDeviceWidget::DLWheelVirtualDeviceWidget(QWidget *parent /* = NULL */)
    : QWidget(parent)
{
	initVirtualWidget();
    initUpgradeWidget();
    initShowWidget();
	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#CommonButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#CommonButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}\
						QPushButton#CommonButton:disabled{background:gray;}");		

    // 删除设备结果返回函数绑定;
    WHEEL_BACK_TO_CORE_SOCKET.wheelAutoRelevanceDevice.connect(boost::bind(&DLWheelVirtualDeviceWidget::signalAutoCombineCallBack, this, _1, _2));
    connect(this, &DLWheelVirtualDeviceWidget::signalAutoCombineCallBack, this, [=](bool isSuccess, QString strError) {
        m_blockMessageBox->hide();
        if (isSuccess)
        {
            emit signalSendOperateMsg("虚拟设备关联成功");
        }
        else
        {
            emit signalSendOperateMsg("虚拟设备关联失败 : " + strError);
        }
    });

    m_pThreadDataTransfer = new DataTransfer;
    connect(m_pThreadDataTransfer, &DataTransfer::sig_finished, this, &DLWheelVirtualDeviceWidget::slot_finished_upload);
}

DLWheelVirtualDeviceWidget::~DLWheelVirtualDeviceWidget()
{

}

void DLWheelVirtualDeviceWidget::initVirtualWidget()
{
    m_virtualDevieBaseWidget = new BaseWidget(NULL, BaseWidgetType::CommonWidget);
    m_virtualDevieBaseWidget->setTitleContent("虚拟设备");
    m_virtualDevieBaseWidget->setDrawBorderLine();
    m_virtualDevieBaseWidget->setFixedWidth(350);

    m_blockMessageBox = new BlockMessageBox;
    m_blockMessageBox->setBlockTime(30);

    m_pButtonAutoCombine = new QPushButton("自动关联");
    m_pButtonAutoCombine->setObjectName("CommonButton");
    m_pButtonAutoCombine->setFixedSize(QSize(75, 25));

    connect(m_pButtonAutoCombine, &QPushButton::clicked, this, [=] {
        WHEEL_BACK_TO_CORE_SOCKET.robot_auto_relevance_device_req();
        m_blockMessageBox->show();
    });

    QHBoxLayout* hButtonLayout = new QHBoxLayout;
    hButtonLayout->addWidget(m_pButtonAutoCombine);
    hButtonLayout->addStretch();
    hButtonLayout->setMargin(0);

    QVBoxLayout* m_virtualButtonLayout = new QVBoxLayout(m_virtualDevieBaseWidget->getCenterWidget());
    m_virtualButtonLayout->addLayout(hButtonLayout);
    m_virtualButtonLayout->addStretch();
    m_virtualButtonLayout->setMargin(25);
}

void DLWheelVirtualDeviceWidget::initUpgradeWidget()
{
    m_upgradeDeviceBaseWidget = new BaseWidget(NULL, BaseWidgetType::CommonWidget);
    m_upgradeDeviceBaseWidget->setTitleContent("固件升级");
    m_upgradeDeviceBaseWidget->setDrawBorderLine();
    m_upgradeDeviceBaseWidget->setFixedWidth(350);

    m_pButtonCloudUpgrade = new QPushButton("CCU升级");
    m_pButtonCloudUpgrade->setObjectName("CommonButton");
    m_pButtonCloudUpgrade->setFixedSize(QSize(75, 25));

    connect(m_pButtonCloudUpgrade, &QPushButton::clicked, this, [=] {
        sendRemoteUpgrade(REMOTE_UPGRADE_CLOUD);
    });

    QLabel *pCCUVerLabel = new QLabel("CCU当前软件版本:");
    pCCUVerLabel->setStyleSheet("color:rgb(105, 105, 105);font-size:15px;font-weight:bold;");
    
    QLabel *pPCUVerLabel = new QLabel("PCU当前软件版本:");
    pPCUVerLabel->setStyleSheet("color:rgb(105, 105, 105);font-size:15px;font-weight:bold;");

    QLabel *pFileHeadLabel = new QLabel("升级文件");
    pFileHeadLabel->setStyleSheet("color:rgb(105, 105, 105);font-size:15px;font-weight:bold;");

    QLineEdit *pFileLine = new QLineEdit;
    pFileLine->setStyleSheet("QLineEdit{;padding-left:5px;border:1px solid rgb(170, 230, 200);}");

    QPushButton *pOpenButton = new QPushButton("浏览");
    pOpenButton->setObjectName("CommonButton");
    pOpenButton->setFixedSize(QSize(40, 25));
    connect(pOpenButton, &QPushButton::clicked, [=] {
        QString fileName = QFileDialog::getOpenFileName(this, "", "", "bin(*.bin)");
        pFileLine->setText(fileName);
        m_openFileName = fileName;
        m_upgradeFileName = "upgrade/" + fileName.split("/").last();
    });

    QHBoxLayout* hHBCCUPCULayout = new QHBoxLayout;
    hHBCCUPCULayout->addWidget(pFileHeadLabel);
    hHBCCUPCULayout->addWidget(pFileLine);
    hHBCCUPCULayout->addWidget(pOpenButton);

    ////////////////////////////////////////////////////////////////////////////
    m_pButtonBodyUpgrade = new QPushButton("PCU升级");
    m_pButtonBodyUpgrade->setObjectName("CommonButton");
    m_pButtonBodyUpgrade->setFixedSize(QSize(75, 25));

    connect(m_pButtonBodyUpgrade, &QPushButton::clicked, this, [=] {
        sendRemoteUpgrade(REMOTE_UPGRADE_BODY);
    });

    QHBoxLayout* CCUPCULayout = new QHBoxLayout;
    CCUPCULayout->addWidget(m_pButtonCloudUpgrade);
    CCUPCULayout->addWidget(m_pButtonBodyUpgrade);

    QVBoxLayout* hButtonLayout = new QVBoxLayout;
    hButtonLayout->addStretch();
    hButtonLayout->addWidget(pCCUVerLabel);
    hButtonLayout->addWidget(pPCUVerLabel);
    hButtonLayout->addLayout(hHBCCUPCULayout);
    hButtonLayout->addLayout(CCUPCULayout);
    hButtonLayout->addStretch();
    hButtonLayout->setMargin(0);

    QVBoxLayout* m_upgradeButtonLayout = new QVBoxLayout(m_upgradeDeviceBaseWidget->getCenterWidget());
    m_upgradeButtonLayout->addLayout(hButtonLayout);
    m_upgradeButtonLayout->addStretch(10);
    m_upgradeButtonLayout->setMargin(25);
}

void DLWheelVirtualDeviceWidget::initShowWidget()
{
    QHBoxLayout* hMainLayout = new QHBoxLayout(this);
    hMainLayout->addWidget(m_virtualDevieBaseWidget);
    hMainLayout->addWidget(m_upgradeDeviceBaseWidget);
    hMainLayout->addStretch();
    hMainLayout->setSpacing(10);
    hMainLayout->setMargin(10);
}

void DLWheelVirtualDeviceWidget::sendRemoteUpgrade(WheelRemoteUpgradeType type)
{
#if 0
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_remote_upgrade_req(type, m_upgradeFileName);

#else
    if (NULL == m_pThreadDataTransfer) return;
    m_upgradeType = type;

    TransferPro file_pro;
    QString src_upfile = "upgrade";
    file_pro.cmd_type_ = DataTransfer::UPLOAD_FILE;
    file_pro.src_file_path_ = m_openFileName.toLocal8Bit().data();
    file_pro.dst_relative_path_ = src_upfile.toLocal8Bit().data();
    m_pThreadDataTransfer->set_transfer_info(file_pro);
    m_pThreadDataTransfer->start();
#endif
}

void DLWheelVirtualDeviceWidget::slot_finished_upload(int action_type, int execCode, QString exeParam)
{
    WHEEL_BACK_TO_CORE_SOCKET.robot_control_remote_upgrade_req(m_upgradeType, m_upgradeFileName);
}