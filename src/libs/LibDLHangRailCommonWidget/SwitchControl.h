#ifndef SWITCH_CONTROL
#define SWITCH_CONTROL

#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <QHBoxLayout>
#include <QPainter>

#pragma execution_character_set("utf-8")

class SwitchControl : public QWidget
{
	Q_OBJECT

public:
	explicit SwitchControl(QWidget *parent = 0);

	// ���ؿ���״̬ - �򿪣�true �رգ�false;
	bool isToggled() const;

	// ���ÿ���״̬;
	void setToggle(bool checked);

	// ���ñ�����ɫ;
	void setBackgroundColor(QColor color);

	// ����ѡ����ɫ;
	void setCheckedColor(QColor color);

	// ���ò�������ɫ;
	void setDisbaledColor(QColor color);

	// ���û���ʱ��;
	void setSlideTime(int slideTime);

protected:
	// ���ƿ���;
	void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

	// ��갴���¼�;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

	// ����ͷ��¼� - �л�����״̬������toggled()�ź�;
	void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

	// ��С�ı��¼�;
	void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;

	// ȱʡ��С;
	QSize sizeHint() const Q_DECL_OVERRIDE;
	QSize minimumSizeHint() const Q_DECL_OVERRIDE;

signals:
	// ״̬�ı�ʱ�������ź�;
	void toggled(bool checked);

	private slots:
	// ״̬�л�ʱ�����ڲ�������Ч��;
	void onTimeout();

private:
	bool m_bChecked;         // �Ƿ�ѡ��;
	QColor m_background;     // ������ɫ;
	QColor m_checkedColor;   // ѡ����ɫ;
	QColor m_disabledColor;  // ��������ɫ;;
	QColor m_thumbColor;     // Ĵָ��ɫ
	qreal m_radius;          // Բ��;
	qreal m_nX;              // x������;
	qint16 m_nHeight;        // �߶�;
	qint16 m_nMargin;        // ��߾�;
	QTimer m_timer;          // ��ʱ��;
	int m_slideTime;		 // ����ʱ��;
};

class SwitchWidget : public QWidget
{
	Q_OBJECT

public:
	SwitchWidget(bool isWheelRobot = false) 
		: m_isWheelRobot(isWheelRobot)
	{
		initControl();
	};

	~SwitchWidget() {};

	// ���ÿ�������;
	void setSwitchName(QString switchName)
	{
		m_switchNamelabel->setText(switchName + ":");
	};

	// ���ÿ���״̬;
	void setSwitchState(bool bChecked)
	{
		m_pSwitchControl->setToggle(bChecked);
	}
	// ��ȡ����״̬;
	bool getSwitchState()
	{
		return m_pSwitchControl->isToggled();
	}

	// ���ÿ��ش�С;
	void setSwitchSize(QSize size)
	{
		m_pSwitchControl->setFixedSize(size);
	}

    // ���ÿ������ִ�С;
    void setTextSize(int poinSize)
    {
        m_switchNamelabel->setStyleSheet(QString("font-size:%1px").arg(poinSize));
    }

	// ���ð�ť��ɫ;
	void setSwitchCheckColor(QColor color)
	{
		m_pSwitchControl->setCheckedColor(color);
	}

	// ���û���ʱ��;
	void setSlideTime(int slideTime)
	{
		m_pSwitchControl->setSlideTime(slideTime);
	}

private:
	void initControl()
	{
		m_pSwitchControl = new SwitchControl(this);
		m_pSwitchControl->setFixedSize(QSize(50, 20));
		// ����״̬����ʽ;
		m_pSwitchControl->setCheckedColor(QColor(0, 160, 230));

		m_switchNamelabel = new QLabel();

		QFont labelFont;
		labelFont.setPointSize(11);
		m_switchNamelabel->setFont(labelFont);

		QLabel* labelOPen = new QLabel("��");
		labelOPen->setStyleSheet("color:gray;");
		QLabel* labelClose = new QLabel("��");
		labelClose->setStyleSheet("color:gray;");

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addStretch();
		hLayout->addWidget(m_switchNamelabel);
        hLayout->addWidget(labelClose);
		hLayout->addWidget(m_pSwitchControl);
        hLayout->addWidget(labelOPen);
		hLayout->addStretch();
		hLayout->setSpacing(12);
		hLayout->setMargin(0);

		// �����źŲ�;
		connect(m_pSwitchControl, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));

		if (m_isWheelRobot)
		{
			m_switchNamelabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");
			m_switchNamelabel->setFixedWidth(100);
			this->setSwitchSize(QSize(150, 24));
			this->setSwitchCheckColor(QColor(80, 215, 105));
			this->setSlideTime(2);
		}
		else
		{
			labelOPen->setVisible(false);
			labelClose->setVisible(false);
		}
	};

    void paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        if (m_isWheelRobot)
        {
            painter.setPen(QPen(Qt::gray));
            painter.drawRect(this->rect().adjusted(0, 0, -1, -1));
        }
    }

signals:
	// ״̬�ı�ʱ�������ź�;
	void toggled(bool checked);

private:
	SwitchControl * m_pSwitchControl;
	QLabel* m_switchNamelabel;
	bool m_isWheelRobot;
};

#endif // SWITCH_CONTROL