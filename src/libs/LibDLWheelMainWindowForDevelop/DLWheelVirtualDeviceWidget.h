#ifndef __DL_WHEEL_ROBOT_VIRTUAL_DEVICE_H__
#define __DL_WHEEL_ROBOT_VIRTUAL_DEVICE_H__

#include <QWidget>
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDataTransfer/DataTransfer.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"

class DLWheelVirtualDeviceWidget : public QWidget
{
	Q_OBJECT

public:
    DLWheelVirtualDeviceWidget(QWidget *parent = NULL);
	~DLWheelVirtualDeviceWidget();

private:
	void initVirtualWidget();
    void initUpgradeWidget();
    void initShowWidget();
    void sendRemoteUpgrade(WheelRemoteUpgradeType type);

signals:
    // core回调;
    void signalAutoCombineCallBack(bool, QString);
    // 发送操作信息到主窗口;
    void signalSendOperateMsg(QString);

public slots:
    void slot_finished_upload(int action_type, int execCode, QString exeParam);

private:
    BaseWidget * m_virtualDevieBaseWidget;
    BlockMessageBox* m_blockMessageBox;
    QPushButton* m_pButtonAutoCombine;

//    QVBoxLayout* m_virtualButtonLayout;

    BaseWidget *m_upgradeDeviceBaseWidget;
    QPushButton* m_pButtonCloudUpgrade;
    QPushButton* m_pButtonBodyUpgrade;
//    QVBoxLayout* m_upgradeButtonLayout;

    DataTransfer *m_pThreadDataTransfer;

    WheelRemoteUpgradeType m_upgradeType;
    QString m_upgradeFileName;
    QString m_openFileName;
};

#endif