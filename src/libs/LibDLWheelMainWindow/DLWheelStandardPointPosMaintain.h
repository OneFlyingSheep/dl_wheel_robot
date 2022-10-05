#ifndef DL_WHEEL_STANDARD_POINT_POS_MAINTAIN_H
#define DL_WHEEL_STANDARD_POINT_POS_MAINTAIN_H

#include <QWidget>
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include <QPushButton>
#include <QHBoxLayout>
#include <QUuid>
#include <QTimer>
#include <QListWidget>
#include <QProgressBar>
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"

#pragma execution_character_set("utf-8")
#define  OVER_TIME_LENGTH 5000

/**********����������ؼ�**********/

class ProgressBarWidget : public BaseWidget
{
    Q_OBJECT
public:
    ProgressBarWidget(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
        , m_current(0)
        , m_total(1)
    {
        initWidget();

        this->setTitleContent("�������");
        this->setFixedWidth(350);
        this->setStyleSheet("QPushButton{font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);border-radius:3px;}\
                                QPushButton:pressed{padding-left:3px;padding-top:3px;}");
        this->setWindowModality(Qt::ApplicationModal);
    }

    // ���õ�ǰ����;
    void setCurrentProgress(int currentProgress)
    {
        m_current = currentProgress;
        if (m_current > m_total)
        {
            m_current = m_total;
        }
        int percent = 1.0 * m_current / m_total * 100;
        m_progressBar->setValue(percent);
    }

    // �����ܵĴ�С;
    void setTotelSize(int totalSize)
    {
        m_total = totalSize;
    }

    // ��Ӵ�����Ϣ;
    void addErrorMsg(QString errorMsg)
    {
        m_errorMsgListWidget->addItem(errorMsg);
    }

    // ���ý�����;
    void resetProgress()
    {
        m_current = 0;
        m_total = 1;
        m_errorMsgListWidget->clear();
        m_progressBar->setValue(0);
    }

private:
    // ��ʼ���ؼ�;
    void initWidget()
    {
        m_errorMsgListWidget = new QListWidget;

        m_progressBar = new QProgressBar;
        m_progressBar->setOrientation(Qt::Horizontal);

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            emit signalStopImportExcelData();
            this->hide();
            resetProgress();
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(0);

        QVBoxLayout* vMainLayout = new QVBoxLayout;
        vMainLayout->addWidget(m_errorMsgListWidget);
        vMainLayout->addWidget(m_progressBar);
        vMainLayout->addLayout(hButtonLayout);
        vMainLayout->setSpacing(15);
        vMainLayout->setMargin(0);

        QHBoxLayout* hMainLayout = new QHBoxLayout(this->getCenterWidget());
        hMainLayout->addStretch();
        hMainLayout->addLayout(vMainLayout);
        hMainLayout->addStretch();
        hMainLayout->setContentsMargins(0, 15, 0, 15);

    }

signals:
    // ֹͣ����Excel����;
    void signalStopImportExcelData();

private:
    QProgressBar* m_progressBar;
    QListWidget* m_errorMsgListWidget;

    int m_current;
    int m_total;
};

/*******���ѡ�񴰿�**********/

class SheetChooseWidget : public BaseWidget
{
    Q_OBJECT
public:
    SheetChooseWidget(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
    {
        initWidget();
        this->setTitleContent("��׼��λ�⵼��");
        this->setFixedWidth(350);
        this->setStyleSheet("QPushButton{font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);border-radius:3px;}\
                                QPushButton:pressed{padding-left:3px;padding-top:3px;}");
        this->setWindowModality(Qt::ApplicationModal);
    }

    // ���ñ����������;
    void setSheetListData(QStringList sheetNameList)
    {
        m_sheetListWidget->clear();
        m_sheetListWidget->addItems(sheetNameList);
    }

signals:
    // ѡ����������;
    void signalChooseSheetName(QString sheetName);

private:
    // ��ʼ���ؼ�;
    void initWidget()
    {
        m_sheetListWidget = new QListWidget;

        QPushButton* pButtonOk = new QPushButton("ȷ��");
        pButtonOk->setFixedSize(QSize(60, 25));
        connect(pButtonOk, &QPushButton::clicked, this, [=] {
            QListWidgetItem* item = m_sheetListWidget->currentItem();
            if (item != NULL)
            {
                QString sheetName = item->text();
                emit signalChooseSheetName(sheetName);
                this->hide();
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "��ʾ", "��ѡ��һ��sheet", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            this->hide();
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonOk);
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(0);

        QVBoxLayout* vMainLayout = new QVBoxLayout;
        vMainLayout->addWidget(m_sheetListWidget);
        vMainLayout->addLayout(hButtonLayout);
        vMainLayout->setSpacing(15);
        vMainLayout->setMargin(0);

        QHBoxLayout* hMainLayout = new QHBoxLayout(this->getCenterWidget());
        hMainLayout->addStretch();
        hMainLayout->addLayout(vMainLayout);
        hMainLayout->addStretch();
        hMainLayout->setContentsMargins(0, 15, 0, 15);
    }

private:
    QListWidget* m_sheetListWidget;
};

/***********��ӵ�λ����*************/

class AddPointPosWidget : public BaseWidget
{
    Q_OBJECT
public:
    AddPointPosWidget(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
        , m_isModifyData(false)
    {
        initWidget();
        initData();

        this->setFixedWidth(350);
        this->setStyleSheet("QPushButton{font-weight:bold;border:1px solid rgb(121,134,203);background:rgb(175, 191, 255);border-radius:3px;}\
                                QPushButton:pressed{padding-left:3px;padding-top:3px;}");
        this->setWindowModality(Qt::ApplicationModal);

        WHEEL_BACK_TO_CORE_SOCKET.wheelRobotUpdateStandardStatus.connect(boost::bind(&AddPointPosWidget::signalSaveTaskCheckData, this, _1, _2));
        WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInsertStandardStatus.connect(boost::bind(&AddPointPosWidget::signalSaveTaskCheckData, this, _1, _2));

        connect(this, &AddPointPosWidget::signalSaveTaskCheckData, this, [=](bool isSuccess, QString strMsg) {
            m_overTimeTimer.stop();
            if (isSuccess)
            {
                this->hide();
                // ֪ͨ����ˢ������;
                emit signalSaveDataSuccess(m_isModifyData, m_data);
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "����", "����ʧ��:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            this->setDisabled(false);
            WHEEL_DEVICE_CONFIG.refreshData();
        });
    }

	void insertData(int device_type_id, int sub_device_type_id, QString device_point_name, int recognition_type, int meter_type, QString device_sn, int save_type)
	{
		initData();

		WheelStandardPatrolVindicateStruct data;

		data.m_device_type_name = m_deviceTypeWidget->getComboBoxContentByIndex(device_type_id);
		data.m_sub_device_name = m_subDeviceTypeWidget->getComboBoxContentByIndex(sub_device_type_id);
		data.m_device_point_name = device_point_name;
		data.m_recognition_type_name = m_recognitionTypeWidget->getComboBoxContentByIndex(recognition_type);
		data.m_meter_type_name = m_meterTypeWidget->getComboBoxContentByIndex(meter_type);
		data.m_fever_type_name = device_sn;
		data.m_save_type_name = m_saveTypeWidget->getComboBoxContentByIndex(save_type);

		setCurrentData(data);
		onSaveButtonClicked();
		m_overTimeTimer.stop();
	}

    // �޸�ʱ��Ҫ�����ò���;
    void setCurrentData(WheelStandardPatrolVindicateStruct data)
    {
        m_isModifyData = true;
        m_data = data;

        m_deviceTypeWidget->setComboBoxCurrentContent(m_data.m_device_type_name);

        m_subDeviceTypeWidget->setComboBoxCurrentContent(m_data.m_sub_device_name);

        m_devicePointNameWidget->setComboBoxCurrentContent(m_data.m_device_point_name);

        m_recognitionTypeWidget->setComboBoxCurrentContent(m_data.m_recognition_type_name);

        m_meterTypeWidget->setComboBoxCurrentContent(m_data.m_meter_type_name);

        m_feverTypeWidget->setComboBoxCurrentContent(m_data.m_fever_type_name);

        m_saveTypeWidget->setComboBoxCurrentContent(m_data.m_save_type_name);
    }

    // ��ʼ������;
    void initData()
    {
        m_isModifyData = false;

        m_deviceTypeWidget->setComboBoxContent(WHEEL_DEVICE_CONFIG.getWheelDeviceTypeNameQList());

        m_subDeviceTypeWidget->setComboBoxContent(WHEEL_DEVICE_CONFIG.getWheelSubDeviceNameQList());

        m_devicePointNameWidget->setComboBoxContent(WHEEL_DEVICE_CONFIG.getWheelDevicePointNameQList());

        m_recognitionTypeWidget->setComboBoxContent(QStringList() << "" << WHEEL_DEVICE_CONFIG.getWheelRecognitionTypeQList());

        m_meterTypeWidget->setComboBoxContent(QStringList() << "" << WHEEL_DEVICE_CONFIG.getWheelMeterTypeNameQList());

        m_feverTypeWidget->setComboBoxContent(QStringList() << "" << WHEEL_DEVICE_CONFIG.getWheelFeverTypeNameQList());

        m_saveTypeWidget->setComboBoxContent(QStringList() << "" << WHEEL_DEVICE_CONFIG.getWheelSavaTypeDataQList());
    }

private:
    // ��ʼ���ؼ�;
    void initWidget()
    {
        this->setTitleContent("��λ��ά��");

        m_deviceTypeWidget = new InputWidget(WheelComboBox);
        m_deviceTypeWidget->setTipText("�豸����");
        m_deviceTypeWidget->getComboBoxWidget()->setEditable(true);
        m_deviceTypeWidget->setFixedWidth(300);
        m_deviceTypeWidget->getComboBoxWidget()->setFixedWidth(220);

        m_subDeviceTypeWidget = new InputWidget(WheelComboBox);
        m_subDeviceTypeWidget->setTipText("С���豸");
        m_subDeviceTypeWidget->getComboBoxWidget()->setEditable(true);
        m_subDeviceTypeWidget->setFixedWidth(300);
        QComboBox* deviceTypeComboBox = m_subDeviceTypeWidget->getComboBoxWidget();
        deviceTypeComboBox->view()->setFixedWidth(240);
        deviceTypeComboBox->setFixedWidth(220);

        m_devicePointNameWidget = new InputWidget(WheelComboBox);
        m_devicePointNameWidget->setTipText("��λ����");
        m_devicePointNameWidget->getComboBoxWidget()->setEditable(true);
        m_devicePointNameWidget->setFixedWidth(300);
        QComboBox* devicePointNameComboBox = m_devicePointNameWidget->getComboBoxWidget();
        devicePointNameComboBox->view()->setFixedWidth(240);
        devicePointNameComboBox->setFixedWidth(220);

        m_recognitionTypeWidget = new InputWidget(WheelComboBox);
        m_recognitionTypeWidget->setTipText("ʶ������");
        m_recognitionTypeWidget->setFixedWidth(300);
        m_recognitionTypeWidget->getComboBoxWidget()->setFixedWidth(220);

        m_meterTypeWidget = new InputWidget(WheelComboBox);
        m_meterTypeWidget->setTipText("�������");
        m_meterTypeWidget->setFixedWidth(300);
        m_meterTypeWidget->getComboBoxWidget()->setFixedWidth(220);

        m_feverTypeWidget = new InputWidget(WheelComboBox);
        m_feverTypeWidget->setTipText("��������");
        m_feverTypeWidget->setFixedWidth(300);
        m_feverTypeWidget->getComboBoxWidget()->setFixedWidth(220);

        m_saveTypeWidget = new InputWidget(WheelComboBox);
        m_saveTypeWidget->setTipText("��������");
        m_saveTypeWidget->setFixedWidth(300);
        m_saveTypeWidget->getComboBoxWidget()->setFixedWidth(220);

        QPushButton* pButtonSave = new QPushButton("����");
        pButtonSave->setFixedSize(QSize(60, 25));
        connect(pButtonSave, &QPushButton::clicked, this, &AddPointPosWidget::onSaveButtonClicked);

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            this->hide();
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonSave);
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(15);
        hButtonLayout->setMargin(0);


        QVBoxLayout* vLayout = new QVBoxLayout();
        vLayout->addWidget(m_deviceTypeWidget);
        vLayout->addWidget(m_subDeviceTypeWidget);
        vLayout->addWidget(m_devicePointNameWidget);
        vLayout->addWidget(m_recognitionTypeWidget);
        vLayout->addWidget(m_meterTypeWidget);
        vLayout->addWidget(m_feverTypeWidget);
        vLayout->addWidget(m_saveTypeWidget);
        vLayout->addLayout(hButtonLayout);
        vLayout->setSpacing(20);
        vLayout->setMargin(0);

        QHBoxLayout* hMainLayout = new QHBoxLayout(this->getCenterWidget());
        hMainLayout->addStretch();
        hMainLayout->addLayout(vLayout);
        hMainLayout->addStretch();
        hMainLayout->setContentsMargins(0, 15, 0, 15);
    }

    // ��ʼ����ʱʱ��;
    void initOverTimeTimer()
    {
        m_overTimeTimer.setInterval(OVER_TIME_LENGTH);
        connect(&m_overTimeTimer, &QTimer::timeout, this, [=] {
            DLMessageBox::showDLMessageBox(NULL, "����", "������ʱ", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            this->setDisabled(false);
        });
    }

signals:
    // ���ݱ���ɹ�;
    void signalSaveDataSuccess(bool isModify, WheelStandardPatrolVindicateStruct data);

    // ���淵�ػص�;
    void signalSaveTaskCheckData(bool, QString);

private slots:
    // ���水ť���;
    void onSaveButtonClicked()
    {
        m_data.m_device_type_name = m_deviceTypeWidget->getComboBoxCurrentContent();

        m_data.m_sub_device_name =  m_subDeviceTypeWidget->getComboBoxCurrentContent();

        m_data.m_device_point_name = m_devicePointNameWidget->getComboBoxCurrentContent();

        m_data.m_recognition_type_name = m_recognitionTypeWidget->getComboBoxCurrentContent();

        m_data.m_meter_type_name = m_meterTypeWidget->getComboBoxCurrentContent();

        m_data.m_fever_type_name = m_feverTypeWidget->getComboBoxCurrentContent();

        m_data.m_save_type_name = m_saveTypeWidget->getComboBoxCurrentContent();

        if (!m_isModifyData)
        {
            m_data.m_device_point_uuid = QUuid::createUuid().toString().remove("{").remove("}").remove("-");
            WHEEL_BACK_TO_CORE_SOCKET.robot_insert_standard_patrol_vindicate_req(m_data);
        }
        else
        {
            WHEEL_BACK_TO_CORE_SOCKET.robot_update_standard_patrol_vindicate_req(m_data);
        }
        this->setDisabled(true);
        m_overTimeTimer.start();
    }

private:
    InputWidget* m_deviceTypeWidget;
    InputWidget* m_subDeviceTypeWidget;
    InputWidget* m_devicePointNameWidget;
    InputWidget* m_recognitionTypeWidget;
    InputWidget* m_meterTypeWidget;
    InputWidget* m_feverTypeWidget;
    InputWidget* m_saveTypeWidget;

    // ��ǰ�Ƿ����޸�;
    bool m_isModifyData;

    WheelStandardPatrolVindicateStruct m_data;

    // ��ʱʱ��;
    QTimer m_overTimeTimer;
};

/*********������ť�ؼ�*********/

class OperateButtonWidget : public QWidget
{
    Q_OBJECT
public:
    OperateButtonWidget(QWidget* parent = NULL)
        : QWidget(parent)
    {
        initWidget();

        this->setStyleSheet("QPushButton{border-radius:3px;border:1px solid gray;}\
                                QPushButton:pressed{padding-left:3px;padding-top:3px;}");
    }

    // ���õ��ڰ�ť����;
    void setButtonRow(int buttonRow)
    {
        m_buttonRow = buttonRow;
    }

    // ����ɾ����ť;
    void resetDeleteButton()
    {
        m_pButtonDelete->setDisabled(false);
    }

signals:
    // �޸İ�ť���;
    void signalModifyButtonClicked(int buttonRow);
    // ɾ����ť���;
    void signalDeleteButtonClicked(int buttonRow);

private:
    // ��ʼ���ؼ�;
    void initWidget()
    {
        m_pButtonModify = new QPushButton("�޸�");
        m_pButtonModify->setFixedSize(QSize(50, 25));
        connect(m_pButtonModify, &QPushButton::clicked, this, [=] {
            emit signalModifyButtonClicked(m_buttonRow);
        });

        m_pButtonDelete = new QPushButton("ɾ��");
        m_pButtonDelete->setFixedSize(QSize(50, 25));
        connect(m_pButtonDelete, &QPushButton::clicked, this, [=] {
            m_pButtonDelete->setDisabled(true);
            m_overTimeTimer.start();
            emit signalDeleteButtonClicked(m_buttonRow);
        });

        QHBoxLayout* hLayout = new QHBoxLayout(this);
        hLayout->addStretch();
        hLayout->addWidget(m_pButtonModify);
        hLayout->addWidget(m_pButtonDelete);
        hLayout->addStretch();
        hLayout->setSpacing(5);
        hLayout->setMargin(0);

        m_overTimeTimer.setInterval(OVER_TIME_LENGTH);
        connect(&m_overTimeTimer, &QTimer::timeout, this, [=] {
            DLMessageBox::showDLMessageBox(NULL, "����", "������ʱ", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            m_pButtonDelete->setDisabled(false);
            m_overTimeTimer.stop();
        });
    }

private:
    QPushButton* m_pButtonModify;
    QPushButton* m_pButtonDelete;

    int m_buttonRow;

    QTimer m_overTimeTimer;
};

class SearchRecordCreateExcel;
class CustomButtonListWidget;
class CustomTableWidget;

/************��׼��λ��ά��ҳ��************/

class DLWheelStandardPointPosMaintain : public QWidget
{
	Q_OBJECT

public:
	DLWheelStandardPointPosMaintain();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // ���ڸ����ؼ���ʼ��;
	void initCheckBoxWidget();
	void initButtonListWidget();
	void initTopWidget();
	void initTableWidget();
    void initTableData();

    // ɾ��ĳһ��;
    void resetDeleteButton(QString strUuid);

    // ����;
    void exportTable();

    // ��׼��λ�⵼��;
    void standardPointPosLibImport();

signals:
    // ɾ�����ػص�;
    void signalDeleteCallBack(QString, bool, QString); 

    // excel���ݵ���ص�;
    void signalExcelImportCallBack(bool, QString);

private slots:
    // ������ť���;
    void onButtonClicked(int buttonId);
    // �޸İ�ť���;
    void onModifyButtonClicked(int buttonRow);
    // ɾ����ť���;
    void onDeleteButtonClicked(int buttonRow);
    // ��� / �޸� ��λˢ��table;
    void onRefreshTable(bool isModify, WheelStandardPatrolVindicateStruct data);
    // ��׼��λ�⵼��;
    void onStandardPointPosLibImport(QStringList excelData);
	// �ļ�����;
	void file_import_patrol_list();

private:
	QWidget* m_topBackWidget;
	
    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	CustomButtonListWidget* m_buttonListWidget;

	CustomTableWidget* m_customTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable������;
    QList<WheelStandardPatrolVindicateStruct> m_tableDataList;

    // ��ǰѡ�е�checkBox��idList;
    QStringList m_device_type_uuid;

    // ��� / �޸� ��λ����;
    AddPointPosWidget* m_addPointPosWidget;

    // ��׼��λ����sheetѡ�񴰿�;
    SheetChooseWidget* m_sheetChooseWidget;

    // ��ǰ������table������;
    int m_currentTableRow;

    // ��׼��λ������;
    QList<QStringList> m_standardPointPosLibDataList;
    // ���浱ǰ���͵ĵ�λ������index;
    int m_sendPointPosLibDataIndex;

    // Excel����;
    SearchRecordCreateExcel m_searchRecordCreateExcel;

    // Excel������ȴ���;
    ProgressBarWidget* m_progressBarWidget;

    // �Ƿ�ֹͣ����;
    bool m_isStopImportExcelData;
};

#endif // DL_WHEEL_TASK_CHECK_H
