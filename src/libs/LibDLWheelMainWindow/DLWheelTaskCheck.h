#ifndef DL_WHEEL_TASK_CHECK_H
#define DL_WHEEL_TASK_CHECK_H

#include <QWidget>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QRadioButton>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <QTimer>
#include <QScrollArea>
#include "LibDLHangRailCommonWidget/VideoPlayer.h"
#include <QKeyEvent>
#include <QTableWidget>
#include <QHeaderView>
#include <QTableWidgetItem>
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

enum TaskCheckType
{
	DeviceAlarmInfo_Check,      // �豸�澯��Ϣ���;
	DeviceAlarmSearchAffirm,    // �豸�澯��ѯȷ��;
	PatrolResultBrowse,         // Ѳ�������;
};

class TitleWidget : public QWidget
{
public:
	TitleWidget(bool isInfrared = false, QWidget* parent = NULL)
		: QWidget(parent)
		, m_pImageLabel(NULL)
		, m_isInfrared(isInfrared)
	{
		initWidget();
		this->setStyleSheet("QWidget#TitleBackWidget{background:rgb(159,168,218);}\
								QWidget#CenterBackWidget{background:white;}");
	//	setImageWidget();
	}

	bool eventFilter(QObject *obj, QEvent *event)
	{
		if (obj == m_centerBackWidget) {
			if (event->type() == QMouseEvent::MouseButtonDblClick) {
				if (m_centerBackWidget->isFullScreen()) {
					m_centerBackWidget->setWindowFlags(Qt::Window & Qt::WindowMinMaxButtonsHint & Qt::WindowCloseButtonHint);
					m_centerBackWidget->showNormal();
					//	update();m_wight = windowWidth;
					//m_height = windowHeight;
					m_pImageLabel->setFixedSize(QSize(m_wight, m_height));
					QLabel *pPixmapLbl = dynamic_cast<QLabel *>(m_pImageLabel);
					if (NULL != pPixmapLbl)
					{
						pPixmapLbl->setPixmap(QPixmap(m_strImagePath).scaled(m_pImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
					}
				}
				else {
					m_centerBackWidget->setWindowFlag(Qt::Window);
					m_centerBackWidget->showFullScreen();
					m_pImageLabel->setFixedSize(m_centerBackWidget->size());
					//static_cast<QLabel*>(m_imageLabel)->setPixmap(QPixmap(m_strImagePath).scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
					QLabel *pPixmapLbl = dynamic_cast<QLabel *>(m_pImageLabel);
					if (NULL != pPixmapLbl)
					{
						pPixmapLbl->setPixmap(QPixmap(m_strImagePath).scaled(m_pImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
					}
				}
				return true;
			}
			else if (event->type() == QKeyEvent::KeyRelease) {
				QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
				if (m_centerBackWidget->isFullScreen() && Qt::Key_Escape == keyEvent->key()) {
					//label_1->setWindowFlag(Qt::SubWindow);������仰�ᴥ���ӿؼ����븸�ؼ�
					m_centerBackWidget->setWindowFlags(Qt::Window & Qt::WindowMinMaxButtonsHint & Qt::WindowCloseButtonHint);
					m_centerBackWidget->showNormal();
				}
				m_pImageLabel->setFixedSize(m_centerBackWidget->size());
				QLabel *pPixmapLbl = dynamic_cast<QLabel *>(m_pImageLabel);
				if (NULL != pPixmapLbl)
				{
					pPixmapLbl->setPixmap(QPixmap(m_strImagePath).scaled(m_pImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
				}
				//static_cast<QLabel*>(m_imageLabel)->setPixmap(QPixmap(m_strImagePath).scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
				return true;
			}
			else {
				return false;
			}
		}
		else {
			// pass the event on to the parent class
			return TitleWidget::eventFilter(obj, event);
		}
	}

	// ���ñ���;
	void setTitleText(const QString& text)
	{
		m_titleLabel->setText(text);
	}

	// ��ȡ����widget���Ա��ڶԴ�widget����;
	QWidget* getCenterWidget()
	{
		return m_centerBackWidget;
	}

    void setIsWidget(bool isWidget)
    {
        m_isWidget = isWidget;
    }

	void setImageWidget()
	{

        if (m_isWidget)
        {
            m_pImageLabel = new QTableWidget;
            QStringList sListHeader;
            sListHeader << "�澯�ȼ�" << "�澯��ֵ";
            static_cast<QTableWidget*>(m_pImageLabel)->setColumnCount(2);
            static_cast<QTableWidget*>(m_pImageLabel)->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
            static_cast<QTableWidget*>(m_pImageLabel)->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
            static_cast<QTableWidget*>(m_pImageLabel)->setHorizontalHeaderLabels(sListHeader);
            static_cast<QTableWidget*>(m_pImageLabel)->setSelectionBehavior(QAbstractItemView::SelectItems);
            static_cast<QTableWidget*>(m_pImageLabel)->setEditTriggers(QAbstractItemView::NoEditTriggers);
            static_cast<QTableWidget*>(m_pImageLabel)->horizontalHeader()->setHighlightSections(false);

            QHBoxLayout* hLayout = new QHBoxLayout(m_centerBackWidget);
            hLayout->addWidget(m_pImageLabel);
            hLayout->setMargin(0);
        }
        else
        {
            m_scrollImageWidget = new QScrollArea;
            m_scrollImageWidget->setStyleSheet("QScrollArea{border:none;background:white;}");
            QHBoxLayout* hLayout = new QHBoxLayout(m_centerBackWidget);
            hLayout->addWidget(m_scrollImageWidget);
            hLayout->setMargin(0);

            if (m_isInfrared)
            {
                m_pImageLabel = new InfraredImageWidget;
            }
            else
            {
                m_pImageLabel = new QLabel;
            }
            m_scrollImageWidget->setWidget(m_pImageLabel);
        }
	}

    void setTableValue(QList<QStringList> listValue)
    {
        static_cast<QTableWidget*>(m_pImageLabel)->setRowCount(0);
        static_cast<QTableWidget*>(m_pImageLabel)->clearContents();
        for (int i = 0; i < listValue.size(); i++)
        {
            QTableWidgetItem *item = new QTableWidgetItem(listValue[i][0]);
            item->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
            int index = static_cast<QTableWidget*>(m_pImageLabel)->rowCount();
            static_cast<QTableWidget*>(m_pImageLabel)->setRowCount(index + 1);
            static_cast<QTableWidget*>(m_pImageLabel)->setItem(index, 0, item);
            static_cast<QTableWidget*>(m_pImageLabel)->setItem(index, 1, new QTableWidgetItem(listValue[i][1]));
        }
    }

	void setImage(QString strImagePath)
	{
		m_strImagePath = strImagePath;
		//	if (m_imageLabel != NULL && !strImagePath.isEmpty())
	//	QString imagePath = "D:\\RCF_Server_Root\\task\\fa344e5c33744fd2a31c7ba83944543d\\6a5c71478503493e875d0c0fb008384b\\6a5c71478503493e875d0c0fb008384b.jpg";
 
		if (m_pImageLabel != NULL)
		{
			int windowWidth = m_scrollImageWidget->width() - 20;

			QSize imageSize = QPixmap(m_strImagePath).size();
			m_scrollImageWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

			int windowHeight = 1.0 * windowWidth / imageSize.width() * imageSize.height();
			m_wight = windowWidth;
			m_height = windowHeight;
			m_pImageLabel->setFixedSize(windowWidth, windowHeight);
			if (m_isInfrared)
			{
				static_cast<InfraredImageWidget*>(m_pImageLabel)->setInfraredImagePath(m_strImagePath, "");
			}
			else
			{
				static_cast<QLabel*>(m_pImageLabel)->setScaledContents(true);
				static_cast<QLabel*>(m_pImageLabel)->setPixmap(QPixmap(m_strImagePath).scaled(m_pImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
			}
		}
	}

	void resizeImage() {}

private:
	void initWidget()
	{
		QWidget* titleBackWidget = new QWidget;
		titleBackWidget->setObjectName("TitleBackWidget");
		titleBackWidget->setFixedHeight(30);
		titleBackWidget->setStyleSheet("QWidget{background:rgb(159,168,218);}");
		m_titleLabel = new QLabel;
		m_titleLabel->setStyleSheet("font-weight:bold;");
		QHBoxLayout* hTitleLayout = new QHBoxLayout(titleBackWidget);
		hTitleLayout->addWidget(m_titleLabel);
		hTitleLayout->addStretch();
		hTitleLayout->setContentsMargins(5, 0, 0, 0);

		m_centerBackWidget = new QWidget;
		m_centerBackWidget->setObjectName("CenterBackWidget");
		QVBoxLayout* vMainLayout = new QVBoxLayout(this);
		vMainLayout->addWidget(titleBackWidget);
		vMainLayout->addWidget(m_centerBackWidget);
		vMainLayout->setSpacing(0);
		vMainLayout->setMargin(0);
		m_centerBackWidget->installEventFilter(this);
	}

private:
	// �Ƿ��Ǻ���;
	bool m_isInfrared;
	QLabel* m_titleLabel;
	QWidget* m_centerBackWidget;

	QScrollArea* m_scrollImageWidget;
	QWidget* m_pImageLabel;
	QString m_strImagePath;

	int m_wight = 0;
	int m_height = 0;

    bool m_isWidget = false;
};

class RCFImageDownload;

class DLWheelTaskCheck : public QWidget
{
	Q_OBJECT

public:
	DLWheelTaskCheck(QWidget* parent = NULL);

	~DLWheelTaskCheck();

	// ��������;
	void setData(DeviceAlarmSearchStruct data);

    void thresholdAnalysis(QString deviceUUid);
    // ��ȡ��ǰ����;
	DeviceAlarmSearchStruct getData()
	{
		return m_data;
	}

	// ���ö�ҳ����;
	void setDataList(QList<DeviceAlarmSearchStruct> dataList);

	// �Ƿ��ǵ���;
	void setIsSingleTask(bool isSingleTask);

	void setTaskCheckType(TaskCheckType taskCheckType);

	private slots:
	void onSaveTaskCheckData();

	// ͼƬ�������;
	void onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath);

signals:
	// �����������;
	void signalSaveTaskCheckData(int chooseType, bool isSuccess, QString strMsg);

	// ֪ͨˢ��table;
	void signalRefreshTable();

private:
	// ���ڿؼ���ʼ��;
	void initTopWidget();
	void initLastPageWidget();
	void initVisibleImageWidget();
	void initFraredImageWidget();
	void initAudioFileWidget();
	void initThresholdValueWidget();
	void initLeftWidget();
	void initRightWidget();
	void initCenterWidget();
	void initWidget();

	// ���泬ʱ;
	void initTimeout();

	// ���ڱ��������¼�;
	void paintEvent(QPaintEvent *event);

private:
	QWidget * m_topBackWidget;
	QWidget* m_leftBackWidget;
	QWidget* m_rightBackWidget;
	QWidget* m_centerBackWidget;

    QLabel * m_resultShow;

	TitleWidget* m_visibleImageWidget;
	TitleWidget* m_fraredImageWidget;
	TitleWidget* m_audioImageWidget;
	TitleWidget* m_ThresholdValueWidget;

	QToolButton* m_pButtonClose;
	QToolButton* m_pButtonSave;
	QToolButton* m_pButtonCancel;

	// �����������;
	// ��������;
	DeviceAlarmSearchStruct m_data;

	QList<DeviceAlarmSearchStruct> m_dataList;

	// ��ǰ�������inidex;
	int m_taskCheckDataIndex;

	int m_taskCheckDataCount;

	// ��ǰ�Ѿ���˵�ҳ��;
	int m_currentCheckedIndex;

	// ��һҳ����һҳ;
	QLabel* m_currentTaskCheckIndexLabel;
	QToolButton* m_pButtonLastPage;
	QToolButton* m_pButtonNextPage;

	// ��λ��Ϣ�����;
	QLineEdit* m_pointInfoLineEdit;
	QLabel* m_timeLabel;

	QRadioButton* m_checkBoxNormal;
	QRadioButton* m_checkBoxAbnormal;

	QComboBox* m_identifyComboBox;
	QComboBox* m_alarmLavelComBoBox;

	QRadioButton* m_identifyRight;
	QRadioButton* m_identifyError;

	// ʵ��ֵ;
	QLineEdit* m_realValueLineEdit;

	// �澯Ƶ��;
	QLabel* m_alarmFrequencyLabel;
	// �澯����;
	QLabel* m_alarmTypeLabel;
	// �澯��ֵ;
	QLabel* m_alarmThresholdLabel;

	// ���泬ʱʱ��;
	QTimer m_saveTimeoutTimer;

	// ��ǰ�Ƿ������;
	bool m_isSingleTask;

	// ��ǰ���񵯳�ҳ������;
	TaskCheckType m_taskCheckType;

	// ͼƬ����;
	RCFImageDownload* m_RCFImageDownload;
};

#endif // DL_WHEEL_TASK_CHECK_H