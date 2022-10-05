#ifndef DL_WHEEL_PATROL_RESULT_BROWER_H
#define DL_WHEEL_PATROL_RESULT_BROWER_H

#include <QWidget>
#include <QDateEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QListWidget>
#include <QStackedWidget>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelCustomWidget/TurnPageWidget.h"
#include "LibProtoClient/ProtoClient.h"


class RCFImageDownload;
class DLWheelTaskCheck;
class DLMessageBox;

/*********��ʾ�豸ͼƬ�ؼ�**********/

class ImageWidget : public QWidget
{
public:
	ImageWidget()
	{
		m_imageLabel = new QLabel;
		m_imageLabel->setObjectName("ImageLabel");

		m_textLabel = new QLabel;
		m_textLabel->setFixedHeight(20);

		QVBoxLayout* vLayout = new QVBoxLayout(this);
		vLayout->addWidget(m_imageLabel);
		vLayout->addWidget(m_textLabel);
		vLayout->setSpacing(0);
		vLayout->setMargin(0);

		this->setStyleSheet("QLabel#ImageLabel{background:rgb(159,168,218);}");
	}

    // ��������;
	void setText(const QString& text)
	{
		m_textLabel->setText(text);
		m_textLabel->setScaledContents(true);
	}

    // ����ͼƬ;
	void setImage(QString imagePath)
	{
		m_imageLabel->setPixmap(QPixmap(imagePath).scaled(m_imageLabel->size()));
	}

private:
	QLabel* m_imageLabel;
	QLabel* m_textLabel;
};

/********Ѳ�������ҳ��********/

class DLWheelPatrolResultBrowse : public QWidget
{
	Q_OBJECT

public:
	DLWheelPatrolResultBrowse();

    ~DLWheelPatrolResultBrowse();

	// ���ڿؼ���ʼ��;
	void initWidget();

    // Ѳ����������ύ�ص�;
    void onPatrolResultCheckPeopleCallBack(bool isSuccess, QString strTaskId, QString strMsg);

private:
	// ��ʼ���豸��;
	void initDeviceTreeWidget();
    // ��ʼ�������б�;
    void initTaskList();
    // ��ʼ���󲿿ؼ�;
    void initLeftWidget();
	// ��ʼ����ť�б�;
	void initButtonListWidget();
	// ��ʼ����������б�;
	void initTableWidget();
	// ��ʼ��ͼƬ��ʾ;
	void initImageShowWidget();

    // ����table��ʾ;
    void updateTabelData(QString strTaskId);

private slots:
	void onButtonClicked(int buttonId);
    // �������б���в���;
    void onTaskButtonClicked(int buttonId);
    // �����б���;
    void onTaskListClciked(QListWidgetItem* item);
    // ͼƬ�������;
    void onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath);

	//һ����˽����ź�
	void AKeyAuditFinishSlot(bool bIsSuccess, QString strErrorMessage);


private:
    // �Զ������ؼ�;
	CustomTreeWidget * m_deviceTreeWidget;

    // �����б�;
    QListWidget* m_pTaskListWidget;
    QWidget* m_taskBackWidget;

    QStackedWidget* m_leftBackWidget;

	CustomButtonListWidget* m_pCustomButtonListWidget;
	QDateEdit* m_startTimeEdit;
	QDateEdit* m_endTimeEdit;

	CustomTableWidget* m_customTableWidget;

    QList<ImageWidget*> m_lstImageWgts;
	QWidget* m_pImageShowBackWidget;
	TurnPageWidget* m_imageTurnPageWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ����table��ǰҳ;
    int m_currentPageIndex;
    // ��ǰ���������id;
    QString m_strCurrentTaskId;

    QLineEdit* m_checkPeopleLineEdit;

    QDateEdit* m_checkSuggestTextEdit;

    // ����table����;
    QList<DeviceAlarmSearchStruct> m_lstTableData;

    // �������;
    DLWheelTaskCheck* m_taskCheckWidget;

    // ͼƬ���� �߳�;
    RCFImageDownload* m_pThreadRCFImageDownload;

	RCFImageDownload* m_pTaskThreadRCFImageDownload;			//�������������߳�
};

#endif
