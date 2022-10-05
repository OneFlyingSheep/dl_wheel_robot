#ifndef BASE_WIDGET_H
#define BASE_WIDGET_H

#include <QDialog>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QButtonGroup>
#include <QTreeView>
#include <QStandardItemModel>
#include <QComboBox>
#include <QTimer>

#pragma execution_character_set("utf-8") 

enum BaseWidgetType
{
	CommonWidget = 0,			// һ�㴰��;
	VideoSwitchWidget,			// ��Ƶ�����Ƶ�л�����(��checkBox);
	PatrolWidget,				// ����Ѳ�촰��;
	StackedWidget,				// �������;
	RobotControlWidget,			// �����˿���;
	TreeViewWindow,				// ����Ѳ�����ؼ�����;
	PopupWindow,				// ����ʽ����;
	EnvironmentStateWidget,		// ����״̬����ͼ����;
	TableWidgetWithTrack,		// �����ٵ�tableWidget;
};

enum IntelligentPatrolButtonType
{
	PatrolOverAll,							// ȫ��Ѳ��;
	PatrolOrienteering,						// ����Ѳ��;
	PatrolSpecific,							// ����Ѳ��;
	PatrolPauseTask,						// ��ͣ����;
	PatrolStopTask							// ֹͣ����;
};

enum EnvironmentStateType
{
	TemperatureState,						// �¶�;
	TumidityState,							// ʪ��;
	SF6State,								// SF6;
	O3State,								// O3;
};

class BaseWidget : public QDialog
{
	Q_OBJECT

public:
	BaseWidget(QWidget *parent = NULL, BaseWidgetType baseWidgetType = CommonWidget);
	~BaseWidget();

	// ���ñ�������;
	void setTitleContent(QString strContent);
	// ���ñ���������ɫ;
	void setTitleColor(QColor titleColor);
	// ���ñ��ⱳ��ɫ;
	void setTitleBackColor(QColor titleBackColor);
	// ��ȡ���Ĳ���ָ��;
	QWidget* getCenterWidget();
	// ��centerWidget����widget,�����stackedWidget���͵Ĵ���;
	void insertWidget(int buttonId, QString buttonText, QWidget* widget);
	// �������,�������;
	void insertWidgetDone();
	// �����Ƿ���Ʊ߿���;
	void setDrawBorderLine(bool isDrawBorderLine = true);

	// ���õ���ʽ���������ؼ���model;
	void setTreeViewModel(QStandardItemModel* model);

	// ��ȡ����Ѳ����checkBox״̬;
	bool getPatrolWidgetCheckBoxState();
	
	// �������ؼ�;
	QTreeView* getPatrolTreeWidget();

	// �����������;
	void setTaskProcess(QString taskProcess);

    // �����´����񵹼�ʱʱ��;
    void setNextTaskCountDown(int iCountDown);

	// ��������Ѳ����ͣ����ť״̬;
	void setStopTaskButtonState(bool isStopTaskClicked);

	// ��ȡ����Ѳ����ͣ����ť״̬;
	bool getStopTaskButtonState();

	// StackedWidget���ʹ�����ת��ĳһҳ;
	void setCurrentStackedWidgetPage(int pageIndex);

	// �����Ƿ���ʾ���Ͻǹرհ�ť;
	void setShowCloseButton(bool isShow = true);

	// ��ȡtilteWidget(�����Զ���������Ͽؼ�);
	QWidget* getTitleBackWidget();

	// ��ȡ��ǰStackedWidget��pageIndex;
	int getCurrentPageIndex();

    // ���ô����Ƿ�����ƶ�;
    void setMovable(bool isMovable);

    void setTitleHide();
signals:
	// ��Ƶ�л�״̬�ı�;
	void signalCheckBoxStateChanged(bool isChecked);

	// �����˿���-ȫ��Ӧ�ð�ť���;
	void signalRobotControlAllApplyClicked();

	// ����Ѳ���������ť���;
	void signalPatrolButtonClicked(int);

	// ֪ͨ����������;
	void signalSendNewCustomTask(QString taskName, QStringList taskIdList);

	// ����״̬����ͼѡ��״̬;
	void signalEnvironmentStateChanged(int index);

	// ����ʽ���ڹرհ�ť���;
	void signalCloseButtonClicked();

	// StackedWidget�����л�;
	void signalStackedWidgetChanged(int button);

public slots:
	// ��Ƶ�����л�;
	void onCheckBoxStateChanged(int state);
	// stackedWidgetҳ���л�;
	void onStackedButtonClicked(int buttonId);
private:
	// ���ڿؼ���ʼ��;
	void initControl();
	// ��ʼ��һ�㴰��;
	void initCommonWidget();
	// ��ʼ����Ƶ�л�����;
	void initVideoSwitchWidget();
	// ����Ѳ�촰��;
	void initPatrolWidget();
	// �����˿���;
	void initRobotControlWidget();
	// stackedWidget;
	void initStackedWidget();
	// ��ʼ������ʽ����;
	void initTreeViewWindow();
	// ����ʽ����;;
	void initPopupWindow();
	// ����״̬����ͼ����;
	void initEnvironmentStateWidget();
	// �����ٵ�tableWidget;
	void initTableWidgetWithTrack();

	// �����¼�;
	void paintEvent(QPaintEvent *event);

	// �����¼�;
	void keyPressEvent(QKeyEvent *event);

	// �϶������¼�;
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

protected:
	QLabel * m_titleLabel;
	QWidget* m_titleBackWidget;
	QWidget* m_centerWidget;
	// ��Ƶ�л�����;
	QCheckBox* m_checkeBox;

	// ����Ѳ��;
	QPushButton* m_pButtonPatrolOverAll;
	QPushButton* m_pButtonPatrolOrienteering;
	QPushButton* m_pButtonPatrolSpecific;
	QPushButton* m_pButtonPatrolPauseTask;
	QPushButton* m_pButtonPatrolStopTask;
	QCheckBox* m_chechBoxTrack;
	QLabel* m_taskProcessLabel;
    QLabel* m_nextTaskCountDownLabel;

	// �����˿���;
	// ȫ��Ӧ��;
	QPushButton* m_pButtonAllApply;

	// stackedWidget;
	QStackedWidget* m_centerStackedWidget;
	QButtonGroup* m_stackedButtonGroup;

	// ��������;
	BaseWidgetType m_baseWidgetType;

	// ����ʽ���ڿؼ�;
	QTreeView* m_pupUpTreeView;
	QPushButton* m_pButtonOk;
	QPushButton* m_pButtonCancel;

	// ����״̬����ͼ������������б�ѡ��ĳ������;
	QComboBox* m_environmentStateComboBox;

	// �Ƿ���Ʊ߿���;
	bool m_isDrawBorderLine;

	// �Ƿ��ǵ���ʽ����;
	bool m_isPopupWindow;

	// ��ǰ�Ƿ���ͣ����(����Ѳ��);
	bool m_isStopTaskClicked;

	// �������Ƿ���,��������϶�����;
	bool m_isPressed;
	QPoint m_startMovePos;

    // �رհ�ť;
	QPushButton* m_pButtonClose;
};

enum MessageButtonType
{
	BUTTON_OK = 0,					// ֻ��ȷ����ť;
	BUTTON_OK_AND_CANCEL,			// ȷ����ȡ����ť;
	BUTTON_CLOSE,					// �رհ�ť;
    BUTTON_NONE                     // û�а�ť;
};

// ��Ϣ��ʾ��;
class DLMessageBox : public BaseWidget
{
	Q_OBJECT

public:
	DLMessageBox(QWidget* parent = NULL);
	~DLMessageBox();

	// ����MessageBox����;
	void setMessageContent(QString strContent);

    // �Ƿ���ʾOK��ť;
    void setButtonOKVisible(bool isVisible);

	// �Ƿ���ʾcancel��ť;
	void setButtonCancelVisible(bool isVisible);

	void setButtonType(MessageButtonType buttonType);

	int static showDLMessageBox(QWidget* parent, const QString &title, const QString &contentText, MessageButtonType messageButtonType, bool isModelWindow = false, QSize windowSize = QSize(300, 180));

signals:
	void signalButtonOkClicked();

private:
	void initWidget();

protected:
	QLabel* m_tipTextLabel;
	QLabel* m_tipIconLabel;
};

// ����Core��������������������;
class BlockMessageBox : public DLMessageBox
{
    Q_OBJECT
public:
    BlockMessageBox(QWidget* parent = NULL)
        : DLMessageBox(parent)
    {
        this->setWindowTitle("��ʾ");
        this->setWindowModality(Qt::ApplicationModal);
        this->setFixedSize(QSize(250, 160));
        this->setMovable(false);
        initTimer();
        connect(m_pButtonClose, &QPushButton::clicked, this, &BlockMessageBox::hide);
        m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;}");

    }

    // ��λ��(��Ҫ��show֮ǰ����);
    void setBlockTime(int timeCount)
    {
        m_operateTimer.setInterval(timeCount * 1000);
    }

    void show()
    {
        this->setMessageContent("����������");
        this->setButtonType(BUTTON_NONE);
        m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;color:black;}");
        m_operateTimer.start();
        m_showTimer.start();
    }

    void hide()
    {
        m_showTimer.stop();
        m_operateTimer.stop();
        QWidget::hide();
    }

private:
    void initTimer()
    {
        m_operateTimer.setInterval(5 * 1000);
        connect(&m_operateTimer, &QTimer::timeout, this, [=] {
            this->setMessageContent("������ʱ");
            m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;color:red;}");
            this->setButtonType(BUTTON_CLOSE);
        });

        m_showTimer.setInterval(300);
        connect(&m_showTimer, &QTimer::timeout, this, [=] {
            QWidget::show();
        });
    }

private:
    QTimer m_operateTimer;
    QTimer m_showTimer;
};

#endif // BASE_WIDGET_H