#include "LoginWindow.h"
#include <QFile>
#include <QBoxLayout>
#include <QRegExpValidator>
#include <QPainter>
#include "BaseWidget.h"
#include <QGraphicsDropShadowEffect>
#include <QThread>
#include <QDebug>
#include <QScrollBar>
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include <QGuiApplication>
#include "InputWidget.h"
#include <QScreen>

#define LOGIN_INFO_WIDTH 330			// 登录信息宽度;
#define SHADOW_WIDTH 15					// 窗口阴影宽度;
#pragma execution_character_set("utf-8")

LoginWindow::LoginWindow(QWidget *parent)
	: QWidget(parent)
{
	this->setFixedSize(QSize(720, 350));
	initWidget();
	this->loadStyleSheet(":/Resources/LoginWindow/LoginWindow.css");

	this->setWindowFlag(Qt::FramelessWindowHint);
	setAttribute(Qt::WA_TranslucentBackground);
	setAttribute(Qt::WA_DeleteOnClose);

	// 设置阴影边框;
	auto shadowEffect = new QGraphicsDropShadowEffect(this);
	shadowEffect->setOffset(0, 0);
	shadowEffect->setColor(Qt::black);
	shadowEffect->setBlurRadius(SHADOW_WIDTH);
	this->setGraphicsEffect(shadowEffect);
	m_lineEditUserName->setFocus();
}

LoginWindow::~LoginWindow()
{
}

void LoginWindow::loadStyleSheet(const QString &sheetName)
{
	QFile file(sheetName);
	file.open(QFile::ReadOnly);
	if (file.isOpen())
	{
		QString styleSheet = this->styleSheet();
		styleSheet += QLatin1String(file.readAll());
		this->setStyleSheet(styleSheet);
	}
}

void LoginWindow::initWidget()
{
	initLoginWidget();
	initNetWorkSetWidget();

	QWidget* centerWidget = new QWidget;
	centerWidget->setFixedWidth(LOGIN_INFO_WIDTH * 2);

	QHBoxLayout* hCenterLayout = new QHBoxLayout(centerWidget);
	hCenterLayout->addWidget(m_loginWidget);
//	hCenterLayout->addWidget(m_networkSetWidget);
	hCenterLayout->addStretch();
	hCenterLayout->setSpacing(0);
	hCenterLayout->setMargin(0);

	m_leftWidget = new QScrollArea;
	m_leftWidget->setWidget(centerWidget);
	m_leftWidget->setFixedSize(QSize(LOGIN_INFO_WIDTH, 350 - 2 * SHADOW_WIDTH));
	m_leftWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	m_leftWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	// 切换滑动效果;
	QScrollBar* m_hScrollBar = m_leftWidget->horizontalScrollBar();
	m_animationForScroll = new QPropertyAnimation(m_hScrollBar, "value");
	m_animationForScroll->setDuration(600);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_leftWidget);
	hMainLayout->addStretch();
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(SHADOW_WIDTH);

//	readNetWorkSetInfo();
}

void LoginWindow::initLoginWidget()
{
    initScreenShowChoose();

	m_loginWidget = new QWidget;
	m_loginWidget->setObjectName("LeftWidget");
	m_loginWidget->setFixedSize(QSize(LOGIN_INFO_WIDTH, 350 - 2 * SHADOW_WIDTH));

	// ip设置按钮;
	m_pButtonNetWorkSet = new QPushButton;
	m_pButtonNetWorkSet->setObjectName("NetWorkSetButton");
	m_pButtonNetWorkSet->setFixedSize(QSize(16, 16));
	m_pButtonNetWorkSet->setIcon(QIcon(":/Resources/Common/image/SetButton.png"));
	m_pButtonNetWorkSet->setIconSize(QSize(15, 15));
	connect(m_pButtonNetWorkSet, &QPushButton::clicked, this, &LoginWindow::onNetWorkSet);
    m_pButtonNetWorkSet->setVisible(false);

	QHBoxLayout* hTopLayout = new QHBoxLayout;
	hTopLayout->addStretch();
	hTopLayout->addWidget(m_pButtonNetWorkSet);
	hTopLayout->setContentsMargins(0, 0, 10, 0);

	m_labelIcon = new QPushButton;
	m_labelIcon->setObjectName("TitleIcon");
	m_labelIcon->setFixedSize(QSize(45, 45));
	m_labelIcon->setIcon(QIcon(":/Resources/LoginWindow/loginWindowIcon.png"));
	m_labelIcon->setIconSize(QSize(45, 45));

	m_labelTitleText = new QLabel;
	m_labelTitleText->setObjectName("TitleText");

	QHBoxLayout* hTitleLayout = new QHBoxLayout;
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_labelIcon);
	hTitleLayout->addWidget(m_labelTitleText);
	hTitleLayout->addStretch();
	hTitleLayout->setSpacing(10);
	hTitleLayout->setMargin(0);

	m_labelUserName = new QLabel;
	m_labelUserName->setObjectName("InputLabel");
	m_labelUserName->setText(("用户名"));

	m_labelPassword = new QLabel;
	m_labelPassword->setObjectName("InputLabel");
	m_labelPassword->setText(("密码"));

	m_lineEditUserName = new QLineEdit;
	m_lineEditUserName->setFixedSize(QSize(200, 30));

	QRegExp regx("[0-9a-zA-Z\_]+$");
	QValidator *userNameValidator = new QRegExpValidator(regx, m_lineEditUserName);
	m_lineEditUserName->setValidator(userNameValidator);
	m_lineEditUserName->setText("root");

	m_lineEditPassword = new QLineEdit;
	m_lineEditPassword->setFixedSize(QSize(200, 30));
	m_lineEditPassword->setEchoMode(QLineEdit::Password);
	m_lineEditPassword->setText("q1w2e3r4");


	QValidator *passwordValidator = new QRegExpValidator(regx, m_lineEditPassword);
	m_lineEditPassword->setValidator(passwordValidator);
	// 输入密码之后按下Enter键自动登录;
    connect(m_lineEditPassword, &QLineEdit::returnPressed, this, [=] {
        m_pButtonLogin->setText("登录中...");
        emit signalLogin(m_lineEditUserName->text().toLatin1().toBase64(), m_lineEditPassword->text().toLatin1().toBase64(),
            m_lineEditIP->text(), m_lineEditPort->text());
    });

	QVBoxLayout* vUserNameLayout = new QVBoxLayout;
	vUserNameLayout->addWidget(m_labelUserName);
	vUserNameLayout->addWidget(m_lineEditUserName);
	vUserNameLayout->setSpacing(0);
	vUserNameLayout->setMargin(0);

	QVBoxLayout* vPasswordLayout = new QVBoxLayout;
	vPasswordLayout->addWidget(m_labelPassword);
	vPasswordLayout->addWidget(m_lineEditPassword);
	vPasswordLayout->setSpacing(0);
	vPasswordLayout->setMargin(0);

	m_pButtonLogin = new QPushButton;
	m_pButtonLogin->setObjectName("LoginButton");
	m_pButtonLogin->setFixedSize(QSize(120, 30));
	m_pButtonLogin->setText(("登录"));

	connect(m_pButtonLogin, &QPushButton::clicked, this, [=] {
		m_pButtonLogin->setText("登录中...");
		emit signalLogin(m_lineEditUserName->text().toLatin1().toBase64(), m_lineEditPassword->text().toLatin1().toBase64(),
							m_lineEditIP->text(), m_lineEditPort->text());
	});

	m_pButtonQuit = new QPushButton;
	m_pButtonQuit->setObjectName("LoginButton");
	m_pButtonQuit->setFixedSize(QSize(60, 30));
	m_pButtonQuit->setText(("退出"));
	
	connect(m_pButtonQuit, &QPushButton::clicked, this, [=] {
		emit signalQuit();
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(m_pButtonLogin);
	hButtonLayout->addWidget(m_pButtonQuit);
	hButtonLayout->setSpacing(20);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout;
	vCenterLayout->addLayout(vUserNameLayout);
	vCenterLayout->addLayout(vPasswordLayout);
	vCenterLayout->addStretch();
	vCenterLayout->addLayout(hButtonLayout);
    vCenterLayout->addWidget(m_screenChooseComboBox);
	vCenterLayout->addStretch();
	vCenterLayout->setSpacing(15);
	vCenterLayout->setMargin(0);

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addStretch();
	hCenterLayout->addLayout(vCenterLayout);
	hCenterLayout->addStretch();
	hCenterLayout->setSpacing(0);
	hCenterLayout->setMargin(0);

	QVBoxLayout* vLeftWidgetLayout = new QVBoxLayout(m_loginWidget);
	vLeftWidgetLayout->addLayout(hTopLayout);
	vLeftWidgetLayout->addLayout(hTitleLayout);
	vLeftWidgetLayout->addSpacing(35);
	vLeftWidgetLayout->addLayout(hCenterLayout);
	vLeftWidgetLayout->setSpacing(15);
	vLeftWidgetLayout->setContentsMargins(0, 25, 0, 5);

    // 如果只有一个屏幕则显示在当前屏幕上;
    if (qApp->screens().count() <= 1)
    {
        m_screenChooseComboBox->setVisible(false);
    }
}

void LoginWindow::initNetWorkSetWidget()
{
	m_networkSetWidget = new QWidget;
	m_networkSetWidget->setObjectName("LeftWidget");
	m_networkSetWidget->setFixedSize(QSize(LOGIN_INFO_WIDTH, 350 - 2 * SHADOW_WIDTH));

	m_labelNetworkIcon = new QPushButton;
	m_labelNetworkIcon->setObjectName("TitleIcon");
	m_labelNetworkIcon->setFixedSize(QSize(45, 45));
	m_labelNetworkIcon->setIcon(QIcon(":/Resources/LoginWindow/NetWorkSetIcon.png"));
	m_labelNetworkIcon->setIconSize(QSize(45, 45));

	m_labelNetworkTitleText = new QLabel;
	m_labelNetworkTitleText->setObjectName("TitleText");
	m_labelNetworkTitleText->setText("网络设置");

	QHBoxLayout* hTitleLayout = new QHBoxLayout;
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_labelNetworkIcon);
	hTitleLayout->addWidget(m_labelNetworkTitleText);
	hTitleLayout->addStretch();
	hTitleLayout->setSpacing(10);
	hTitleLayout->setMargin(0);

	m_labelIP = new QLabel;
	m_labelIP->setObjectName("InputLabel");
	m_labelIP->setText(("IP"));

	m_labelPort = new QLabel;
	m_labelPort->setObjectName("InputLabel");
	m_labelPort->setText(("Port"));

	m_lineEditIP = new QLineEdit;
	m_lineEditIP->setFixedSize(QSize(200, 30));

	QRegExp regxIP("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
	QValidator *ipValidator = new QRegExpValidator(regxIP, m_lineEditIP);
	m_lineEditIP->setValidator(ipValidator);

	m_lineEditPort = new QLineEdit;
	m_lineEditPort->setFixedSize(QSize(200, 30));

	QRegExp regxPort("^([0-9]|[1-9]\\d|[1-9]\\d{2}|[1-9]\\d{3}|[1-5]\\d{4}|6[0-4]\\d{3}|65[0-4]\\d{2}|655[0-2]\\d|6553[0-5])$");
	QValidator *portValidator = new QRegExpValidator(regxPort, m_lineEditPassword);
	m_lineEditPort->setValidator(portValidator);

	QVBoxLayout* vIPLayout = new QVBoxLayout;
	vIPLayout->addWidget(m_labelIP);
	vIPLayout->addWidget(m_lineEditIP);
	vIPLayout->setSpacing(0);
	vIPLayout->setMargin(0);

	QVBoxLayout* vPortLayout = new QVBoxLayout;
	vPortLayout->addWidget(m_labelPort);
	vPortLayout->addWidget(m_lineEditPort);
	vPortLayout->setSpacing(0);
	vPortLayout->setMargin(0);

	m_pButtonOK = new QPushButton(m_networkSetWidget);
	m_pButtonOK->setObjectName("LoginButton");
	m_pButtonOK->setFixedSize(QSize(60, 30));
	m_pButtonOK->setText(("保存"));

	connect(m_pButtonOK, &QPushButton::clicked, this, [=] {
		DLHangRailCommonTools::saveNetWorkInfo(m_lineEditIP->text(), m_lineEditPort->text());
		if (!m_loginWidget->isVisible())
		{
			m_loginWidget->setVisible(true);
		}

		m_animationForScroll->setEasingCurve(QEasingCurve::InBounce);
		m_animationForScroll->setDirection(QAbstractAnimation::Backward);
		m_animationForScroll->start();
	});

	m_pButtonBack = new QPushButton(m_networkSetWidget);
	m_pButtonBack->setObjectName("LoginButton");
	m_pButtonBack->setFixedSize(QSize(60, 30));
	m_pButtonBack->setText(("返回"));

	connect(m_pButtonBack, &QPushButton::clicked, this, [=] {
		if (!m_loginWidget->isVisible())
		{
			m_loginWidget->setVisible(true);
		}

		m_animationForScroll->setEasingCurve(QEasingCurve::InBounce);
		m_animationForScroll->setDirection(QAbstractAnimation::Backward);
		m_animationForScroll->start();
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(m_pButtonOK);
	hButtonLayout->addWidget(m_pButtonBack);
	hButtonLayout->setSpacing(20);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout;
	vCenterLayout->addLayout(vIPLayout);
	vCenterLayout->addLayout(vPortLayout);
	vCenterLayout->addStretch();
	vCenterLayout->addLayout(hButtonLayout);
	vCenterLayout->addStretch();
	vCenterLayout->setSpacing(10);
	vCenterLayout->setMargin(0);

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addStretch();
	hCenterLayout->addLayout(vCenterLayout);
	hCenterLayout->addStretch();
	hCenterLayout->setSpacing(0);
	hCenterLayout->setMargin(0);

	QVBoxLayout* vLeftWidgetLayout = new QVBoxLayout(m_networkSetWidget);
	vLeftWidgetLayout->addLayout(hTitleLayout);
	vLeftWidgetLayout->addSpacing(35);
	vLeftWidgetLayout->addLayout(hCenterLayout);
	vLeftWidgetLayout->setSpacing(15);
	vLeftWidgetLayout->setContentsMargins(0, 40, 0, 25);
}

void LoginWindow::readNetWorkSetInfo()
{
	// 从配置文件中读取ip、port;
	QString strIp, strPort;
	DLHangRailCommonTools::getNetWorkInfo(strIp, strPort);

	QRegExp regxIP("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
	if (strIp.contains(regxIP))
	{
		m_lineEditIP->setText(strIp);
	}	
	else
	{
		m_loginWidget->setVisible(false);
	}

	QRegExp regxPort("^([0-9]|[1-9]\\d|[1-9]\\d{2}|[1-9]\\d{3}|[1-5]\\d{4}|6[0-4]\\d{3}|65[0-4]\\d{2}|655[0-2]\\d|6553[0-5])$");
	if (strPort.contains(regxPort))
	{
		m_lineEditPort->setText(strPort);
	}
	else
	{
		m_loginWidget->setVisible(false);
	}	
}

void LoginWindow::initScreenShowChoose()
{
    m_screenChooseComboBox = new InputWidget(InputWidgetType::ComboBox);
    m_screenChooseComboBox->setTipText("屏幕选择");

    QList<QScreen* > screenList = qApp->screens();
    QStringList strNameList;
    for (int i = 0; i < screenList.count(); i++)
    {
        strNameList.append(screenList[i]->name());
    }
    m_screenChooseComboBox->setComboBoxContent(strNameList);
}

void LoginWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.drawPixmap(QRect(LOGIN_INFO_WIDTH + SHADOW_WIDTH, SHADOW_WIDTH, this->width() - LOGIN_INFO_WIDTH - SHADOW_WIDTH * 2,\
						this->height() - SHADOW_WIDTH * 2), QPixmap(":/Resources/LoginWindow/loginBackImage.png"));
}

void LoginWindow::setTitleText(QString titleText)
{
	m_labelTitleText->setText(titleText);
}

void LoginWindow::loginFailed()
{
	m_pButtonLogin->setText("登录");

	DLMessageBox* messageBox = new DLMessageBox;
	messageBox->setFixedWidth(250);
	messageBox->setMessageContent("登录失败");
	messageBox->setWindowModality(Qt::ApplicationModal);
	messageBox->show();
}

int LoginWindow::getCurrentScreenChoosedIndex()
{
    int currentScreenIndex = m_screenChooseComboBox->getComboBoxCurrentIndex();
    if (currentScreenIndex < 0)
    {
        currentScreenIndex = 0;
    }

    return currentScreenIndex;
}

void LoginWindow::onNetWorkSet()
{
	int maxValue = m_leftWidget->horizontalScrollBar()->maximum();
	m_animationForScroll->setStartValue(0);
	m_animationForScroll->setEndValue(maxValue);
	m_animationForScroll->setDirection(QAbstractAnimation::Forward);
	m_animationForScroll->setEasingCurve(QEasingCurve::OutBounce);
	m_animationForScroll->start();
}