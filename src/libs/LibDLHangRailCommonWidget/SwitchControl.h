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

	// 返回开关状态 - 打开：true 关闭：false;
	bool isToggled() const;

	// 设置开关状态;
	void setToggle(bool checked);

	// 设置背景颜色;
	void setBackgroundColor(QColor color);

	// 设置选中颜色;
	void setCheckedColor(QColor color);

	// 设置不可用颜色;
	void setDisbaledColor(QColor color);

	// 设置滑动时间;
	void setSlideTime(int slideTime);

protected:
	// 绘制开关;
	void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

	// 鼠标按下事件;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

	// 鼠标释放事件 - 切换开关状态、发射toggled()信号;
	void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

	// 大小改变事件;
	void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;

	// 缺省大小;
	QSize sizeHint() const Q_DECL_OVERRIDE;
	QSize minimumSizeHint() const Q_DECL_OVERRIDE;

signals:
	// 状态改变时，发射信号;
	void toggled(bool checked);

	private slots:
	// 状态切换时，用于产生滑动效果;
	void onTimeout();

private:
	bool m_bChecked;         // 是否选中;
	QColor m_background;     // 背景颜色;
	QColor m_checkedColor;   // 选中颜色;
	QColor m_disabledColor;  // 不可用颜色;;
	QColor m_thumbColor;     // 拇指颜色
	qreal m_radius;          // 圆角;
	qreal m_nX;              // x点坐标;
	qint16 m_nHeight;        // 高度;
	qint16 m_nMargin;        // 外边距;
	QTimer m_timer;          // 定时器;
	int m_slideTime;		 // 滑动时间;
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

	// 设置开关名称;
	void setSwitchName(QString switchName)
	{
		m_switchNamelabel->setText(switchName + ":");
	};

	// 设置开关状态;
	void setSwitchState(bool bChecked)
	{
		m_pSwitchControl->setToggle(bChecked);
	}
	// 获取开关状态;
	bool getSwitchState()
	{
		return m_pSwitchControl->isToggled();
	}

	// 设置开关大小;
	void setSwitchSize(QSize size)
	{
		m_pSwitchControl->setFixedSize(size);
	}

    // 设置开关文字大小;
    void setTextSize(int poinSize)
    {
        m_switchNamelabel->setStyleSheet(QString("font-size:%1px").arg(poinSize));
    }

	// 设置按钮颜色;
	void setSwitchCheckColor(QColor color)
	{
		m_pSwitchControl->setCheckedColor(color);
	}

	// 设置滑动时间;
	void setSlideTime(int slideTime)
	{
		m_pSwitchControl->setSlideTime(slideTime);
	}

private:
	void initControl()
	{
		m_pSwitchControl = new SwitchControl(this);
		m_pSwitchControl->setFixedSize(QSize(50, 20));
		// 设置状态、样式;
		m_pSwitchControl->setCheckedColor(QColor(0, 160, 230));

		m_switchNamelabel = new QLabel();

		QFont labelFont;
		labelFont.setPointSize(11);
		m_switchNamelabel->setFont(labelFont);

		QLabel* labelOPen = new QLabel("开");
		labelOPen->setStyleSheet("color:gray;");
		QLabel* labelClose = new QLabel("关");
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

		// 连接信号槽;
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
	// 状态改变时，发射信号;
	void toggled(bool checked);

private:
	SwitchControl * m_pSwitchControl;
	QLabel* m_switchNamelabel;
	bool m_isWheelRobot;
};

#endif // SWITCH_CONTROL