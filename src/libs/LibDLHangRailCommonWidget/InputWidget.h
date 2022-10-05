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

		// ������;
		painter.setPen(QPen(batteryColor, 3));
		painter.drawRoundedRect(QRect(2, 2, this->width() - 10, this->height() - 4), 2, 2);
		painter.setBrush(batteryColor);

		// ��ص���;
		int batteryEnergyWidth = this->width() - 20;
		int enmergyWidth = 1.0 * m_currentEnergy / 100 * batteryEnergyWidth;
		if (enmergyWidth != 0)
		{
			painter.drawRoundedRect(QRect(7, 7, enmergyWidth, this->height() - 14), 0.5, 0.5);
		}

		// ��ش���;
		painter.fillRect(QRect(this->width() - 10, this->height() / 2 - 5, 8, 10), batteryColor);
	}

private:
	int m_currentEnergy;
	int m_alarmValue;
};



enum InputWidgetType
{
	LineEdit = 0,					// �����;
	LineEditWithButton,				// �����Ӱ�ť���;
	SpinBox,						// ѡֵ��;
	ComboBox,						// ������;
	TimeEdit,						// ʱ��༭��;
	IntervalGroup,					// ʱ�������(Combobox + �����);
	DateTimeEdit,					// ���ڱ༭��;
	ConnectStatusIcon,				// ����״̬ͼ��;
	SliderWidget,					// ������widget;
	FineAdjustWidget,				// ΢���Ӽ��ؼ�;
	ValueShowWidget,				// ��ֵ��ʾwidget;
	WheelValueShowWidget,			// ��ʽ��������ֵ��ʾwidget;
	WheelValueShowWidget_Green,		// ��ʽ��������ֵ��ʾwidget(��ɫ����);
	WheelLineEdit,					// ��ʽ������LineEdit;
	WheelLineEdit_Green,			// ��ʽ������LineEdit(��ɫ����);
	WheelComboBox,					// ��ʽ������ComboBox;
	WheelComboBox_Green,			// ��ʽ������ComboBox(��ɫ����);
	WheelSignalWidget,				// ��ʽ������ͨ���ź�widget;
    WheelLongValueShowWidget,
};

class InputWidget : public QWidget
{
	Q_OBJECT
public:
	InputWidget(InputWidgetType type = LineEdit);
	~InputWidget();

	// ����Label����;
	void setTipText(QString text);
	// ����Label���ֵĿ��;
	void setTipLabelWidth(int width);

	// �������ִ�С;
	void setTipTextSize(int pointSize);

	// ���������Ƿ�Ӵ�;
	void setTipTextBold(bool isBold);

	// ����lineEdti����;
	void setLineEditText(QString text);
	// ����lineEdit�Ŀ��;
	void setLineEditWidth(int width);
	// ����lineEdit�Ƿ�ֻ��;
	void setLineEditReadOnly(bool isReadOnly);
	// ���ð�ť����;
	void setButtonText(QString text);

	// ����comboBox����;
	void setComboBoxContent(QStringList strList);
	// ���õ�ǰComboBox��index;
	void setComboBoxCurrentIndex(int currentIndex);
	// ����comboBox��ǰ��ʾ����;
	void setComboBoxCurrentContent(QString strCurrentContent);
	// ��� comboBox item;
	void addComboBoxItem(QString itemText, QString itemData);
	// ��ȡComboBox����;
    QComboBox* getComboBoxWidget();
    // ����comboBox���;
    void setComboBoxWidth(int width);

	// ��ȡlineEdit����;
	QString getLineEditText();
	// ��ȡspinBox����;
	QString getSpinBoxText();
	// ����spinBox����;
	void setSpinBoxValue(int value);
	// ��ȡcomboBox��ǰѡ��index;
	int getComboBoxCurrentIndex();
	// ��ȡcomboBox��ǰ����;
	QString getComboBoxCurrentContent();
	// ��ȡָ��index��comboBox����
	QString getComboBoxContentByIndex(int index);

	// ��ȡTimeEdit��ʱ��;
	QString getTimeEditContent();
	// ��ȡDateTimeEdit��ʱ��;
	QString getDateTimeEditCotnent();

	// ����SpinBox��Χ;
	void setSpinBoxRange(int minimum, int maximum);

	// ����SpinBox���;
	void setSpinBoxWidth(int width);

	// ��������״̬���޸�ͼ��;
	void setConnectStatus(bool isConnect);

	void setLinkingStatus(int iLink);

	// ���û�������ʼֵ;
	void setSliderInitValue(int initValue);

	// ���û�������Χ;
	void setSliderValueRange(int minValue, int maxValue);

	// ��ȡ��ǰ������ֵ;
	int getSliderValue();

	// �����������;
	void clearContent();

	// ���ý������LineEdit;
	void setFocusOnLineEdit();

	// ���òɼ������е���ֵ��ʾ;
	void setShowValue(QString strValue);

	// ��ȡ�ɼ������е���ֵ��ʾ;
	QString getShowValue();

	// ����ͨ���ź�״̬���޸�ͼ��;
	void setSignalStatus(bool isConnect);

	// ���������������;
	void setLineEditValidator(const QValidator *v);

    void setButtonStyleSheet(WheelButtonStyleSheet type);
    
signals:
	// ��ť���;
	void signalButtonClicked();
	// comboBoxѡ��ı�;
	void signalComboBoxIndexChanged(int);
	// comboBoxѡ��ı�;
	void signalComboBoxIndexActivated(int);
	// sliderֵ�ı�;
	void signalSliderValueChanged();

	// ΢����ť���;
	void signalFileAdjustClicked(bool isAdd);

private:
	// ��ʼ���ؼ�;
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
	// ��ʼ��ʱ������Ͽؼ�;
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

	// ΢���Ӽ���ť;
	QPushButton* m_pButtonAdd;
	QPushButton* m_pButtonSub;
	// ��ֵ��ʾlabel;
	QLabel* m_valueShowLabel;
	// widget����;
	InputWidgetType m_inputWidgetType;
	// ��������Ƿ����˻���������϶�����;
	bool m_isSliderPageStepMove;
	// ͨ���ź�Icon;
	QLabel* m_signalIconLabel;

    // �ź�widget��ǰ�Ƿ�����;
    bool m_isConnected;
};

#endif // INPUT_WIDGET_H