#ifndef INPUT_WIDGET_H
#define INPUT_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimeEdit>
#include <QDateTimeEdit>
#include <QPainter>
#include "common/DLWheelRobotGlobalDef.hpp"

class BatteryWidget : public QWidget
{
public:
	BatteryWidget(QWidget* parent = NULL)
		: QWidget(parent)
		, m_alarmValue(30)
		, m_currentEnergy(100)
	{
		this->setAttribute(Qt::WA_TranslucentBackground);
		this->setFixedSize(QSize(70, 25));
	}

	void setCurrentBatteryEnergy(int currentEnergy)
	{
		m_currentEnergy = currentEnergy;
		update();
	}

	void setAlarmValue(int alarmValue)
	{
		m_alarmValue = alarmValue;
	}

private:
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.setRenderHint(QPainter::Antialiasing);
		QColor batteryColor;
		if (m_currentEnergy > m_alarmValue)
		{
			batteryColor = QColor(0, 255, 0);
		}
		else
		{
			batteryColor = QColor(255, 0, 0);
		}

		// 电池外框;
		painter.setPen(QPen(batteryColor, 3));
		painter.drawRoundedRect(QRect(2, 2, this->width() - 10, this->height() - 4), 2, 2);
		painter.setBrush(batteryColor);

		// 电池电量;
		int batteryEnergyWidth = this->width() - 20;
		int enmergyWidth = 1.0 * m_currentEnergy / 100 * batteryEnergyWidth;
		if (enmergyWidth != 0)
		{
			painter.drawRoundedRect(QRect(7, 7, enmergyWidth, this->height() - 14), 0.5, 0.5);
		}

		// 电池触点;
		painter.fillRect(QRect(this->width() - 10, this->height() / 2 - 5, 8, 10), batteryColor);
	}

private:
	int m_currentEnergy;
	int m_alarmValue;
};



enum InputWidgetType
{
	LineEdit = 0,					// 输入框;
	LineEditWithButton,				// 输入框加按钮组合;
	SpinBox,						// 选值框;
	ComboBox,						// 下拉框;
	TimeEdit,						// 时间编辑框;
	IntervalGroup,					// 时间间隔组合(Combobox + 输入框);
	DateTimeEdit,					// 日期编辑框;
	ConnectStatusIcon,				// 连接状态图标;
	SliderWidget,					// 滑动条widget;
	FineAdjustWidget,				// 微调加减控件;
	ValueShowWidget,				// 数值显示widget;
	WheelValueShowWidget,			// 轮式机器人数值显示widget;
	WheelValueShowWidget_Green,		// 轮式机器人数值显示widget(绿色字体);
	WheelLineEdit,					// 轮式机器人LineEdit;
	WheelLineEdit_Green,			// 轮式机器人LineEdit(绿色字体);
	WheelComboBox,					// 轮式机器人ComboBox;
	WheelComboBox_Green,			// 轮式机器人ComboBox(绿色字体);
	WheelSignalWidget,				// 轮式机器人通信信号widget;
    WheelLongValueShowWidget,
};

class InputWidget : public QWidget
{
	Q_OBJECT
public:
	InputWidget(InputWidgetType type = LineEdit);
	~InputWidget();

	// 设置Label文字;
	void setTipText(QString text);
	// 设置Label文字的宽度;
	void setTipLabelWidth(int width);

	// 设置文字大小;
	void setTipTextSize(int pointSize);

	// 设置文字是否加粗;
	void setTipTextBold(bool isBold);

	// 设置lineEdti内容;
	void setLineEditText(QString text);
	// 设置lineEdit的宽度;
	void setLineEditWidth(int width);
	// 设置lineEdit是否只读;
	void setLineEditReadOnly(bool isReadOnly);
	// 设置按钮文字;
	void setButtonText(QString text);

	// 设置comboBox内容;
	void setComboBoxContent(QStringList strList);
	// 设置当前ComboBox的index;
	void setComboBoxCurrentIndex(int currentIndex);
	// 设置comboBox当前显示内容;
	void setComboBoxCurrentContent(QString strCurrentContent);
	// 添加 comboBox item;
	void addComboBoxItem(QString itemText, QString itemData);
	// 获取ComboBox对象;
    QComboBox* getComboBoxWidget();
    // 设置comboBox宽度;
    void setComboBoxWidth(int width);

	// 获取lineEdit内容;
	QString getLineEditText();
	// 获取spinBox内容;
	QString getSpinBoxText();
	// 设置spinBox内容;
	void setSpinBoxValue(int value);
	// 获取comboBox当前选项index;
	int getComboBoxCurrentIndex();
	// 获取comboBox当前内容;
	QString getComboBoxCurrentContent();
	// 获取指定index的comboBox内容
	QString getComboBoxContentByIndex(int index);

	// 获取TimeEdit的时间;
	QString getTimeEditContent();
	// 获取DateTimeEdit的时间;
	QString getDateTimeEditCotnent();

	// 设置SpinBox范围;
	void setSpinBoxRange(int minimum, int maximum);

	// 设置SpinBox宽度;
	void setSpinBoxWidth(int width);

	// 设置连接状态，修改图标;
	void setConnectStatus(bool isConnect);

	void setLinkingStatus(int iLink);

	// 设置滑动条初始值;
	void setSliderInitValue(int initValue);

	// 设置滑动条范围;
	void setSliderValueRange(int minValue, int maxValue);

	// 获取当前滑动条值;
	int getSliderValue();

	// 清除输入内容;
	void clearContent();

	// 设置焦点进入LineEdit;
	void setFocusOnLineEdit();

	// 设置采集控制中的数值显示;
	void setShowValue(QString strValue);

	// 获取采集控制中的数值显示;
	QString getShowValue();

	// 设置通信信号状态，修改图标;
	void setSignalStatus(bool isConnect);

	// 给输入框设置正则;
	void setLineEditValidator(const QValidator *v);

    void setButtonStyleSheet(WheelButtonStyleSheet type);
    
signals:
	// 按钮点击;
	void signalButtonClicked();
	// comboBox选项改变;
	void signalComboBoxIndexChanged(int);
	// comboBox选项改变;
	void signalComboBoxIndexActivated(int);
	// slider值改变;
	void signalSliderValueChanged();

	// 微调按钮点击;
	void signalFileAdjustClicked(bool isAdd);

private:
	// 初始化控件;
	void initLineEdit();
	void initLineEditWithButton();
	void initSpinBox();
	void initComboBox();
	void initTimeEdit();
	void initDateTimeEdit();
	void initConnectStatusIcon();
	void initSliderWidget();
	void initFineAdjustWidget();
	void initValueShowWidget();
    void initLongValueShowWidget();
    void initWheelValueShowWidget();
	void initWheelComboBox();
	void initWheelLineEdit();
	void initWheelLineEdit_Green();
	void initWheelValueShowWidget_Green();
	void initWheelComboBox_Green();
	void initWheelSignalWidget();
	// 初始化时间间隔组合控件;
	void initIntervalGroup();

private:
	QLabel * m_tipLabel;
	QLineEdit* m_lineEdit;
	QSpinBox* m_spinBox;
	QComboBox* m_comboBox;
	QPushButton* m_pButton;
	QTimeEdit* m_timeEdit;
	QDateTimeEdit* m_dateTimeEdit;
	QLabel* m_statusIconLabel;
	QSlider* m_sliderWidget;

	// 微调加减按钮;
	QPushButton* m_pButtonAdd;
	QPushButton* m_pButtonSub;
	// 数值显示label;
	QLabel* m_valueShowLabel;
	// widget类型;
	InputWidgetType m_inputWidgetType;
	// 保存鼠标是否点击了滑块而不是拖动滑块;
	bool m_isSliderPageStepMove;
	// 通信信号Icon;
	QLabel* m_signalIconLabel;

    // 信号widget当前是否连接;
    bool m_isConnected;
};

#endif // INPUT_WIDGET_H