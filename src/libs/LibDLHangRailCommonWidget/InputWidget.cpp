#include "InputWidget.h"
#include <QCalendarWidget>
#include <QDebug>

InputWidget::InputWidget(InputWidgetType type)
	: QWidget(NULL)
	, m_tipLabel(NULL)
	, m_inputWidgetType(type)
	, m_lineEdit(NULL)
	, m_spinBox(NULL)
	, m_comboBox(NULL)
	, m_pButton(NULL)
	, m_timeEdit(NULL)
	, m_dateTimeEdit(NULL)
	, m_statusIconLabel(NULL)
	, m_sliderWidget(NULL)
	, m_isSliderPageStepMove(false)
	, m_pButtonAdd(NULL)
	, m_pButtonSub(NULL)
	, m_valueShowLabel(NULL)
	, m_signalIconLabel(NULL)
    , m_isConnected(false)
{
	this->setFixedSize(QSize(190, 30));
	switch (m_inputWidgetType)
	{
	case LineEdit:
		initLineEdit();
		break;
	case LineEditWithButton:
		initLineEditWithButton();
		break;
	case SpinBox:
		initSpinBox();
		break;
	case ComboBox:
		initComboBox();
		break;
	case TimeEdit:
		initTimeEdit();
		break;
	case IntervalGroup:
		initIntervalGroup();
		break;
	case DateTimeEdit:
		initDateTimeEdit();
		break;
	case ConnectStatusIcon:
		initConnectStatusIcon();
		break;
	case SliderWidget:
		initSliderWidget();
		break;
	case FineAdjustWidget:
		initFineAdjustWidget();
		break;
	case ValueShowWidget:
		initValueShowWidget();
		break;
	case WheelValueShowWidget:
		initWheelValueShowWidget();
		break;
	case WheelComboBox:
		initWheelComboBox();
		break;
	case WheelLineEdit:
		initWheelLineEdit();
		break;
	case WheelLineEdit_Green:
		initWheelLineEdit_Green();
		break;
	case WheelValueShowWidget_Green:
		initWheelValueShowWidget_Green();
		break;
	case WheelComboBox_Green:
		initWheelComboBox_Green();
		break;
	case WheelSignalWidget:
		initWheelSignalWidget();
		break;
    case WheelLongValueShowWidget:
        initLongValueShowWidget();
	default:
		break;
	}
	this->setStyleSheet("QLabel{color:rgb(144, 164, 173);}");
}

InputWidget::~InputWidget()
{	
}

void InputWidget::initLineEdit()
{
	m_tipLabel = new QLabel;
	m_lineEdit = new QLineEdit;
	m_lineEdit->setFixedWidth(120);
	m_lineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_lineEdit);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);
}

void InputWidget::initLineEditWithButton()
{
	m_tipLabel = new QLabel;
	m_lineEdit = new QLineEdit;
	m_lineEdit->setFixedWidth(120);
	m_lineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");
	m_pButton = new QPushButton;
	m_pButton->setFixedSize(QSize(80, 25));
	m_pButton->setStyleSheet("QPushButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");

	connect(m_pButton, SIGNAL(clicked()), this, SIGNAL(signalButtonClicked()));

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_lineEdit);
	hLayout->addWidget(m_pButton);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(285, 30));
}

void InputWidget::initSpinBox()
{
	m_tipLabel = new QLabel;
	m_spinBox = new QSpinBox;
	m_spinBox->setFixedWidth(120);
	m_spinBox->setStyleSheet("QSpinBox{color:black;}\
								QSpinBox::down-arrow{height:10px;width:13px;border-image:url(:/Resources/arrow_Down.png)}\
								QSpinBox::up-arrow{height:10px;width:13px;border-image:url(:/Resources/arrow_Up.png);}\
								QSpinBox::up-button{border:none;}QSpinBox::up-button:pressed{padding-left:2px;padding-top:2px;}\
								QSpinBox::down-button{margin-left:1px;border:none;}QSpinBox::down-button:pressed{padding-left:2px;padding-top:2px;}");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_spinBox);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);
}

void InputWidget::initComboBox()
{
	m_tipLabel = new QLabel;
	m_comboBox = new QComboBox;
	m_comboBox->setFocusPolicy(Qt::NoFocus);
	m_comboBox->setFixedWidth(120);
	m_comboBox->setStyleSheet("QComboBox{color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QComboBox:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QComboBox QAbstractItemView::item{height:30px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/arrow_Down.png);height:10px;width:13px;}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/arrow_Up.png);height:10px;width:13px;}\
								QComboBox::drop-down{width:20px;background:transparent;padding-right:5px;}");

	connect(m_comboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(signalComboBoxIndexChanged(int)));
	connect(m_comboBox, SIGNAL(activated(int)), this, SIGNAL(signalComboBoxIndexActivated(int)));

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_comboBox);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);
}

void InputWidget::initTimeEdit()
{
	m_tipLabel = new QLabel;
	m_timeEdit = new QTimeEdit;
	m_timeEdit->setFixedWidth(120);
	m_timeEdit->setStyleSheet("QTimeEdit{color:black;}\
								QTimeEdit::down-arrow{height:10px;width:13px;border-image:url(:/Resources/arrow_Down.png)}\
								QTimeEdit::up-arrow{height:10px;width:13px;border-image:url(:/Resources/arrow_Up.png);}\
								QTimeEdit::up-button{border:none;}QSpinBox::up-button:pressed{padding-left:2px;padding-top:2px;}\
								QTimeEdit::down-button{margin-left:1px;border:none;}QSpinBox::down-button:pressed{padding-left:2px;padding-top:2px;}");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_timeEdit);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);
}

void InputWidget::initIntervalGroup()
{
	QStringList timeList;
	timeList << QStringLiteral("间隔日") << QStringLiteral("间隔时") << QStringLiteral("间隔分");
	m_comboBox = new QComboBox;
	m_comboBox->setFixedWidth(80);
	m_comboBox->addItems(timeList);
	m_comboBox->setStyleSheet("QComboBox{color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QComboBox:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QComboBox QAbstractItemView::item{height:30px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/arrow_Down.png);height:10px;width:13px;}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/arrow_Up.png);height:10px;width:13px;}\
								QComboBox::drop-down{width:20px;background:transparent;padding-right:5px;}");

	m_lineEdit = new QLineEdit;
	m_lineEdit->setFixedWidth(80);
	m_lineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_comboBox);
	hLayout->addWidget(m_lineEdit);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);
}

void InputWidget::initDateTimeEdit()
{
	m_tipLabel = new QLabel;
	m_dateTimeEdit = new QDateTimeEdit;
	m_dateTimeEdit->setFixedWidth(120);
	m_dateTimeEdit->setStyleSheet("QDateTimeEdit{color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QDateTimeEdit:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QDateTimeEdit QAbstractItemView::item{height:30px;}\
								QDateTimeEdit::down-arrow{border-image:url(:/Resources/arrow_Down.png);height:10px;width:13px;}\
								QDateTimeEdit::down-arrow:on{border-image:url(:/Resources/arrow_Up.png);height:10px;width:13px;}\
								QDateTimeEdit::drop-down{width:20px;background:transparent;padding-right:5px;}");

	m_dateTimeEdit->setCalendarPopup(true);

	QCalendarWidget* calendarWidget = new QCalendarWidget;
	m_dateTimeEdit->setCalendarWidget(calendarWidget);
	m_dateTimeEdit->setDate(QDate::currentDate());

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_dateTimeEdit);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(175, 30));
}

void InputWidget::initConnectStatusIcon()
{
	m_tipLabel = new QLabel;
	m_statusIconLabel = new QLabel;
	m_statusIconLabel->setFixedSize(QSize(35, 35));

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_statusIconLabel);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);

	this->setFixedSize(120, 40);
}

void InputWidget::initSliderWidget()
{
	m_tipLabel = new QLabel;
	m_sliderWidget = new QSlider;
	m_sliderWidget->setOrientation(Qt::Horizontal);
	m_sliderWidget->setFixedWidth(120);

	connect(m_sliderWidget, &QSlider::sliderReleased, this, &InputWidget::signalSliderValueChanged);
	connect(m_sliderWidget, &QSlider::actionTriggered, this, [=](int actionId) {
		// 说明是鼠标点击了滑块，发送信号;
		if (actionId == QSlider::SliderPageStepAdd || actionId == QSlider::SliderPageStepSub)
		{
			m_isSliderPageStepMove = true;
		}
		else
		{
			m_isSliderPageStepMove = false;
		}
	});

	connect(m_sliderWidget, &QSlider::valueChanged, this, [=](int value) {
		if (m_isSliderPageStepMove)
		{
			emit signalSliderValueChanged();
		}
	});


	m_lineEdit = new QLineEdit;
	m_lineEdit->setFixedWidth(80);
	m_lineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");
	connect(m_lineEdit, &QLineEdit::editingFinished, this, [=] {
		if (!m_lineEdit->text().isEmpty())
		{
			m_sliderWidget->setPageStep(m_lineEdit->text().toInt());
		}
	});

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_sliderWidget);
	hLayout->addWidget(m_lineEdit);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(285, 30));
}

void InputWidget::initFineAdjustWidget()
{
	m_tipLabel = new QLabel;
	m_pButtonAdd = new QPushButton;
	m_pButtonAdd->setIcon(QIcon(":/Resources/CommonWidget/AddButton.png"));
	m_pButtonAdd->setIconSize(QSize(30, 30));
	m_pButtonAdd->setStyleSheet("QPushButton{border:none}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
	connect(m_pButtonAdd, &QPushButton::clicked, this, [=] {
		emit signalFileAdjustClicked(true);
	});

	m_pButtonSub = new QPushButton;
	m_pButtonSub->setIcon(QIcon(":/Resources/CommonWidget/SubButton.png"));
	m_pButtonSub->setIconSize(QSize(30, 30));
	m_pButtonSub->setStyleSheet("QPushButton{border:none}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
	connect(m_pButtonSub, &QPushButton::clicked, this, [=] {
		emit signalFileAdjustClicked(false);
	});

	m_lineEdit = new QLineEdit;
	m_lineEdit->setFixedWidth(80);
	m_lineEdit->setStyleSheet("QLineEdit{border:1px solid gray;border-radius:2px;color:black;}\
								QLineEdit:read-only{background: lightgray;}");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addStretch();
	hLayout->addWidget(m_pButtonSub);
	hLayout->addWidget(m_pButtonAdd);
	hLayout->addStretch();
	hLayout->addWidget(m_lineEdit);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(285, 30));
}

void InputWidget::initValueShowWidget()
{
	m_tipLabel = new QLabel;
	m_tipLabel->setFixedWidth(70);
	
	m_valueShowLabel = new QLabel;
	m_valueShowLabel->setFixedWidth(60);
	m_valueShowLabel->setStyleSheet("color:rgb(70, 80, 90);");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_valueShowLabel);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(135, 30));
}

void InputWidget::initLongValueShowWidget()
{
    m_tipLabel = new QLabel;
    m_tipLabel->setFixedWidth(70);

    m_valueShowLabel = new QLabel;
//    m_valueShowLabel->setFixedWidth(60);
    m_valueShowLabel->setStyleSheet("color:rgb(70, 80, 90);");

    QHBoxLayout* hLayout = new QHBoxLayout(this);
    hLayout->addWidget(m_tipLabel);
    hLayout->addWidget(m_valueShowLabel);
    hLayout->setSpacing(5);
//    hLayout->setMargin(0);
    hLayout->setContentsMargins(36, 0, 0, 0);
    this->setFixedHeight(30);
//    this->setFixedSize(QSize(135, 30));
}

void InputWidget::initWheelValueShowWidget()
{
	m_tipLabel = new QLabel;
	m_tipLabel->setStyleSheet("color:black;font-weight:bold;");

	m_valueShowLabel = new QLabel;
	m_valueShowLabel->setStyleSheet("color:black;font-weight:bold;");

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_valueShowLabel);
    hLayout->addStretch();
	hLayout->setSpacing(5);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(185, 30));
}

void InputWidget::initWheelValueShowWidget_Green()
{
	initWheelValueShowWidget();
	m_tipLabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");
	m_valueShowLabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");
}

void InputWidget::initWheelComboBox_Green()
{
	initWheelComboBox();
	m_tipLabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");

	QPushButton* pButton = new QPushButton;
	pButton->setFixedSize(QSize(25, 25));
	pButton->setIcon(QIcon(":/Resources/Common/image/transform.png"));
	pButton->setIconSize(QSize(20, 20));
	pButton->setStyleSheet("QPushButton{border:none;}\
							QPushButton:pressed{padding-left:2px;padding-top:2px;}");
	QLayout* layout = this->layout();
	layout->addWidget(pButton);

	this->setFixedWidth(280);
}

void InputWidget::initWheelSignalWidget()
{
	m_tipLabel = new QLabel;
	m_tipLabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");

	m_signalIconLabel = new QLabel;
	m_signalIconLabel->setFixedSize(QSize(35, 35));
    m_signalIconLabel->setPixmap(QPixmap(":/Resources/Common/image/No_Signal.png").scaled(m_signalIconLabel->size()));

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_signalIconLabel);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);

	this->setFixedSize(130, 40);
}

void InputWidget::initWheelComboBox()
{
	m_tipLabel = new QLabel;
	m_tipLabel->setStyleSheet("color:black;");

	m_comboBox = new QComboBox;
	m_comboBox->setFixedSize(QSize(120, 22));
	m_comboBox->setStyleSheet("QComboBox{color:black;background:white;padding-left:5px;border:1px solid rgb(170, 230, 200);}\
								QComboBox:hover{border:1px solid rgb(170, 230, 200);}\
								QComboBox QAbstractItemView::item{height:60px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/Common/image/DownArrow.png);}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/Common/image/UpArrow.png);}\
								QComboBox::drop-down{background:rgb(217, 251, 234);}");

	connect(m_comboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(signalComboBoxIndexChanged(int)));

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(m_tipLabel);
	hLayout->addWidget(m_comboBox);
	hLayout->setSpacing(5);
	hLayout->setMargin(0);

	this->setFixedSize(QSize(180, 30));
}

void InputWidget::initWheelLineEdit()
{
	initLineEdit();
	m_tipLabel->setFixedWidth(85);
	m_tipLabel->setStyleSheet("color:black");
	m_lineEdit->setFixedHeight(22);
	m_lineEdit->setStyleSheet("QLineEdit{;padding-left:5px;border:1px solid rgb(170, 230, 200);}");
	this->setFixedWidth(210);
}

void InputWidget::initWheelLineEdit_Green()
{
	initLineEdit();
	m_tipLabel->setStyleSheet("color:rgb(51, 163, 195);font-size:15px;font-weight:bold;");
	m_lineEdit->setStyleSheet("QLineEdit{;padding-left:5px;border:1px solid rgb(170, 230, 200);}");
	this->setFixedWidth(260);
}

void InputWidget::setTipText(QString text)
{
	if (m_tipLabel != NULL)
	{
		switch (m_inputWidgetType)
		{
		case ValueShowWidget:
		case WheelValueShowWidget:
		case WheelValueShowWidget_Green:
		case WheelLineEdit:
		case WheelLineEdit_Green:
		case WheelSignalWidget:
		case WheelComboBox:
			text.append(" :");
			break;
		default:
			break;
		}
		m_tipLabel->setText(text);
		m_tipLabel->setScaledContents(true);
	}
};

void InputWidget::setTipLabelWidth(int width)
{
	if (m_tipLabel != NULL)
	{
		m_tipLabel->setFixedWidth(width);
	}
}

void InputWidget::setTipTextSize(int pointSize)
{
	if (m_tipLabel != NULL)
	{
		QFont font = m_tipLabel->font();
		font.setPointSize(pointSize);
		m_tipLabel->setFont(font);
	}
};

void InputWidget::setTipTextBold(bool isBold)
{
	if (m_tipLabel != NULL)
	{
		QFont font = m_tipLabel->font();
		font.setBold(isBold);
		m_tipLabel->setFont(font);
	}
}

void InputWidget::setLineEditText(QString text)
{
	if (m_lineEdit != NULL)
	{
		m_lineEdit->setText(text);
	}
}

void InputWidget::setLineEditWidth(int width)
{
	if (m_lineEdit != NULL)
	{
		m_lineEdit->setFixedWidth(width);
	}
}

void InputWidget::setLineEditReadOnly(bool isReadOnly)
{
	if (m_lineEdit != NULL)
	{
		m_lineEdit->setReadOnly(isReadOnly);
	}
}

void InputWidget::setButtonText(QString text)
{
	if (m_pButton != NULL)
	{
		m_pButton->setText(text);
	}
}

void InputWidget::setComboBoxContent(QStringList strList)
{
	if (m_comboBox != NULL)
	{
		m_comboBox->clear();
		m_comboBox->addItems(strList);
	}
}

void InputWidget::setComboBoxCurrentIndex(int currentIndex)
{
	if (m_comboBox != NULL)
	{
		m_comboBox->setCurrentIndex(currentIndex);
	}
}

void InputWidget::setComboBoxCurrentContent(QString strCurrentContent)
{
	if (m_comboBox != NULL)
	{
		m_comboBox->setCurrentText(strCurrentContent);
	}
}

void InputWidget::addComboBoxItem(QString itemText, QString itemData)
{
	if (m_comboBox != NULL)
	{
		m_comboBox->addItem(itemText, itemData);
	}
}

QComboBox* InputWidget::getComboBoxWidget()
{
    return m_comboBox;
}

void InputWidget::setComboBoxWidth(int width)
{
    if (m_comboBox != NULL)
    {
        m_comboBox->setFixedWidth(width);
    }
}

QString InputWidget::getLineEditText()
{
	if (m_lineEdit != NULL)
	{
		return m_lineEdit->text();
	}

	return QString();
}

QString InputWidget::getSpinBoxText()
{
	if (m_spinBox != NULL)
	{
		return m_spinBox->text();
	}
	return QString();
}

void InputWidget::setSpinBoxValue(int value)
{
	if (m_spinBox != NULL)
	{
		m_spinBox->setValue(value);
	}
}

int InputWidget::getComboBoxCurrentIndex()
{
	if (m_comboBox != NULL)
	{
		return m_comboBox->currentIndex();
	}
	return 0;
}

QString InputWidget::getComboBoxCurrentContent()
{
	if (m_comboBox != NULL)
	{
		return m_comboBox->currentText();
	}
	return QString();
}

QString InputWidget::getComboBoxContentByIndex(int index)
{
	if (m_comboBox != NULL)
	{
		return m_comboBox->itemText(index);
	}
	return QString();
}

QString InputWidget::getTimeEditContent()
{
	if (m_timeEdit != NULL)
	{
		QDateTime dateTime = m_timeEdit->dateTime();
		return dateTime.toString("hh:mm:ss");
	}
	return QString();
}

QString InputWidget::getDateTimeEditCotnent()
{
	if (m_dateTimeEdit != NULL)
	{
		QDateTime dateTime = m_dateTimeEdit->dateTime();
		return dateTime.toString("yyyy-MM-dd hh:mm:ss");
	}
	return QString();
}

void InputWidget::setSpinBoxRange(int minimum, int maximum)
{
	if (m_spinBox != NULL)
	{
		m_spinBox->setRange(minimum, maximum);
	}
}

void InputWidget::setSpinBoxWidth(int width)
{
	if (m_spinBox != NULL)
	{
		m_spinBox->setFixedWidth(width);
	}
}

void InputWidget::setConnectStatus(bool isConnect)
{
	if (m_statusIconLabel != NULL)
	{
		if (isConnect)
		{
			m_statusIconLabel->setPixmap(QPixmap(":/Resources/SystemConfig/status_connect.png").scaled(m_statusIconLabel->size()));
		}
		else
		{
			m_statusIconLabel->setPixmap(QPixmap(":/Resources/SystemConfig/status_disconnect.png").scaled(m_statusIconLabel->size()));
		}
	}
}

void InputWidget::setLinkingStatus(int iLink)
{
	
	if (iLink == 0)
	{
		m_statusIconLabel->setPixmap(QPixmap(":/Resources/SystemConfig/status_connect.png").scaled(m_statusIconLabel->size()));
	}
	else
	{
		m_statusIconLabel->setPixmap(QPixmap(":/Resources/SystemConfig/status_disconnect.png").scaled(m_statusIconLabel->size()));
	}
	
}

void InputWidget::setSliderInitValue(int initValue)
{
	if (m_sliderWidget != NULL)
	{
		m_sliderWidget->setValue(initValue);
	}
}

void InputWidget::setSliderValueRange(int minValue, int maxValue)
{
	if (m_sliderWidget != NULL)
	{
		m_sliderWidget->setRange(minValue, maxValue);
	}
}

int InputWidget::getSliderValue()
{
	if (m_sliderWidget != NULL)
	{
		return m_sliderWidget->value();
	}

	return 0;
}

void InputWidget::clearContent()
{
	switch (m_inputWidgetType)
	{
	case LineEdit:
		m_lineEdit->clear();
		break;
	case LineEditWithButton:
		m_lineEdit->clear();
		break;
	case SpinBox:
		m_spinBox->clear();
		break;
	case ComboBox:
		// 设置为第一项;
		m_comboBox->setCurrentIndex(0);
		break;
	default:
		break;
	}
}

void InputWidget::setFocusOnLineEdit()
{
	if (m_lineEdit != NULL)
	{
		m_lineEdit->setFocus();
	}
}

void InputWidget::setShowValue(QString strValue)
{
	if (m_valueShowLabel != NULL)
	{
		m_valueShowLabel->setText(strValue);
        m_valueShowLabel->setScaledContents(true);
	}
}

QString InputWidget::getShowValue()
{
	if (m_valueShowLabel != NULL)
	{
		return m_valueShowLabel->text();
	}

	return QString("");
}

void InputWidget::setSignalStatus(bool isConnect)
{
	if (m_signalIconLabel != NULL)
	{
        if (m_isConnected != isConnect)
        {
            m_isConnected = isConnect;
            if (isConnect)
            {
                m_signalIconLabel->setPixmap(QPixmap(":/Resources/Common/image/Signal.png").scaled(m_signalIconLabel->size()));
            }
            else
            {
                m_signalIconLabel->setPixmap(QPixmap(":/Resources/Common/image/No_Signal.png").scaled(m_signalIconLabel->size()));
            }
        }
	}
}

void InputWidget::setLineEditValidator(const QValidator *v)
{
	if (m_lineEdit != NULL)
	{
		m_lineEdit->setValidator(v);
	}
}

void InputWidget::setButtonStyleSheet(WheelButtonStyleSheet type)
{
    switch (type)
    {
    case BUTTON_STYLE_BLUE:
        m_pButton->setStyleSheet("QPushButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
        break;
    case BUTTON_STYLE_GRAY:
        m_pButton->setStyleSheet("QPushButton{font-family:Microsoft Yahei;color:white;background-color:rgb(105 , 105 , 105);border-radius:3px;}\
								QPushButton:hover{background-color:rgb(105 , 105 , 105);}\
								QPushButton:pressed{background-color:rgb(105 , 105 , 105);padding-left:2px;padding-top:2px;}");
        break;
    default:
        break;
    }
    
}
