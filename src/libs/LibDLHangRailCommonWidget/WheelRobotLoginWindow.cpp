#include "WheelRobotLoginWindow.h"
#include <QRegExpValidator>
#include <QPainter>
#include <QGraphicsDropShadowEffect>
#include <QFile>
#include "BaseWidget.h"
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include "InputWidget.h"
#include <QScreen>
#include <QGuiApplication>

#define LOGIN_INFO_WIDTH 330			// 登录信息宽度;
#define SHADOW_WIDTH 15					// 窗口阴影宽度;
#pragma execution_character_set("utf-8")

WheelRobotLoginWindow::WheelRobotLoginWindow(QWidget *parent)
	: QWidget(parent)
{
	this->setFixedSize(QSize(900, 650));
	initWidget();
	this->setWindowFlag(Qt::FramelessWindowHint);
	setAttribute(Qt::WA_TranslucentBackground);
	setAttribute(Qt::WA_DeleteOnClose);

	DLHangRailCommonTools::loadStyleSheet(this, ":/Resources/WheelRobotLoginWindow/WheelRobotLoginWindow.css");

	// 设置阴影边框;
	auto shadowEffect = new QGraphicsDropShadowEffect(this);
	shadowEffect->setOffset(0, 0);
	shadowEffect->setColor(Qt::black);
	shadowEffect->setBlurRadius(SHADOW_WIDTH);
	this->setGraphicsEffect(shadowEffect);

	// 设置输入框在用户名LineEdit中;
	m_lineEditUserName->setFocus();

    setTabOrder(m_lineEditUserName, m_lineEditPassword);
}

WheelRobotLoginWindow::~WheelRobotLoginWindow()
{
}

void WheelRobotLoginWindow::initWidget()
{
 	initLoginWidget();

	QPushButton* pButtonIcon = new QPushButton;
	pButtonIcon->setStyleSheet("border:none");
	pButtonIcon->setIcon(QIcon(":/Resources/WheelRobotLoginWindow/Image/titleIcon.png"));
	pButtonIcon->setFixedSize(QSize(150, 60));
	pButtonIcon->setIconSize(pButtonIcon->size());

	QLabel* labelTopSplit = new QLabel;
	labelTopSplit->setObjectName("TopTitleSplit");
	labelTopSplit->setFixedSize(QSize(260, 1));

	QLabel* labelTopTitle = new QLabel;
	labelTopTitle->setObjectName("LabelTopTitle");
	labelTopTitle->setText("中国海洋石油集团有限公司");
	labelTopTitle->setFixedSize(QSize(240, 25));

	QVBoxLayout* vTopTitleLayout = new QVBoxLayout;
	vTopTitleLayout->addWidget(pButtonIcon);
	vTopTitleLayout->addWidget(labelTopSplit);
	vTopTitleLayout->addWidget(labelTopTitle);
	vTopTitleLayout->setSpacing(3);
	vTopTitleLayout->setMargin(0);

	QHBoxLayout* hTopTitleLayout = new QHBoxLayout;
	hTopTitleLayout->addLayout(vTopTitleLayout);
	hTopTitleLayout->addStretch();
	hTopTitleLayout->setContentsMargins(100, 0, 0, 0);

	QLabel* titleLabelChinese = new QLabel;
	titleLabelChinese->setAlignment(Qt::AlignCenter);
	titleLabelChinese->setObjectName("TitleLabelChinese");
	titleLabelChinese->setText("集气站机器人监控系统");
	titleLabelChinese->setFixedSize(QSize(400, 40));

	QLabel* titleLabelEnglish = new QLabel;
	titleLabelEnglish->setAlignment(Qt::AlignCenter);
	titleLabelEnglish->setObjectName("TitleLabelEnglish");
	titleLabelEnglish->setText("Gas-gathering Station Monitoring System Of Robot");
	titleLabelEnglish->setFixedSize(QSize(400, 20));

	QLabel* labelCopyRight = new QLabel;
	labelCopyRight->setText("版权所有 : 中国海洋石油集团有限公司");
	labelCopyRight->setAlignment(Qt::AlignCenter);
	labelCopyRight->setObjectName("LabelCopyRight");

	QVBoxLayout* vCenterLayout = new QVBoxLayout;
	vCenterLayout->addWidget(titleLabelChinese);
	vCenterLayout->addWidget(titleLabelEnglish);
	vCenterLayout->addSpacing(30);
	vCenterLayout->addWidget(m_loginBackWidget);
	vCenterLayout->addSpacing(30);
	vCenterLayout->addWidget(labelCopyRight);
	vCenterLayout->setSpacing(10);
	vCenterLayout->setMargin(0);

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addStretch();
	hCenterLayout->addLayout(vCenterLayout);
	hCenterLayout->addStretch();
	hCenterLayout->setSpacing(0);
	hCenterLayout->setMargin(10);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addLayout(hTopTitleLayout);
	vMainLayout->addLayout(hCenterLayout);
	vMainLayout->addStretch();
	vMainLayout->setSpacing(50);
	vMainLayout->setMargin(35);
	
}

void WheelRobotLoginWindow::initLoginWidget()
{
    initScreenShowChoose();

	m_loginBackWidget = new QWidget(this);
	m_loginBackWidget->setObjectName("LoginBackWidget");
	m_loginBackWidget->setFixedSize(QSize(400, 200));

	QLabel* loginLabel = new QLabel;
	loginLabel->setText("登录 Sign In");
	loginLabel->setObjectName("LoginLabel");

	QLabel* labelUserName = new QLabel;
	labelUserName->setObjectName("LineEditLabel");
	labelUserName->setText(("用户名"));

	QLabel* labelPassword = new QLabel;
	labelPassword->setObjectName("LineEditLabel");
	labelPassword->setText(("密码"));

	m_lineEditUserName = new QLineEdit("root");
	m_lineEditUserName->setFixedSize(QSize(160, 30));

	QPushButton* pButtonUserName = new QPushButton;
	pButtonUserName->setStyleSheet("border:none;");
	pButtonUserName->setFixedSize(QSize(18, 18));
	pButtonUserName->setIcon(QIcon(":/Resources/WheelRobotLoginWindow/Image/UserNameLineEdit.png"));
	pButtonUserName->setIconSize(QSize(pButtonUserName->size()));

	QHBoxLayout* hUserNameLineEditLayout = new QHBoxLayout(m_lineEditUserName);
	hUserNameLineEditLayout->addWidget(pButtonUserName);
	hUserNameLineEditLayout->addStretch();
	hUserNameLineEditLayout->setContentsMargins(8, 0, 0, 0);

	QRegExp regx("[0-9]+$");
	QValidator *userNameValidator = new QRegExpValidator(regx, m_lineEditUserName);
//	m_lineEditUserName->setValidator(userNameValidator);

	m_lineEditPassword = new QLineEdit("q1w2e3r4");
	m_lineEditPassword->setFixedSize(QSize(160, 30));
	m_lineEditPassword->setEchoMode(QLineEdit::Password);
    // 输入密码之后按下Enter键自动登录;
    connect(m_lineEditPassword, &QLineEdit::returnPressed, this, [=] {
        emit signalLogin(m_lineEditUserName->text(), m_lineEditPassword->text().toLatin1().toBase64());
    });

	QPushButton* pButtonPassword = new QPushButton;
	pButtonPassword->setStyleSheet("border:none;");
	pButtonPassword->setFixedSize(QSize(22, 22));
	pButtonPassword->setIcon(QIcon(":/Resources/WheelRobotLoginWindow/Image/passwordLineEdit.png"));
	pButtonPassword->setIconSize(QSize(pButtonPassword->size()));

	QHBoxLayout* hPasswordLineEditLayout = new QHBoxLayout(m_lineEditPassword);
	hPasswordLineEditLayout->addWidget(pButtonPassword);
	hPasswordLineEditLayout->addStretch();
	hPasswordLineEditLayout->setContentsMargins(6, 0, 0, 0);

	QValidator *passwordValidator = new QRegExpValidator(regx, m_lineEditPassword);
//	m_lineEditPassword->setValidator(passwordValidator);

	m_pButtonLogin = new IconButton;
	m_pButtonLogin->setIcon(":/Resources/WheelRobotLoginWindow/Image/loginRightArrow.png");
	m_pButtonLogin->setFixedSize(QSize(75, 35));
	m_pButtonLogin->setText("登录");
	
	connect(m_pButtonLogin, &IconButton::clicked, this, [=] {
		emit signalLogin(m_lineEditUserName->text(), m_lineEditPassword->text().toLatin1().toBase64());
	});

	m_pButtonReset = new IconButton;
	m_pButtonReset->setIcon(":/Resources/WheelRobotLoginWindow/Image/ResetButton.png");
	m_pButtonReset->setFixedSize(QSize(75, 35));
	m_pButtonReset->setText("重置");
    connect(m_pButtonReset, &IconButton::clicked, this, [=] {
        m_lineEditUserName->clear();
        m_lineEditPassword->clear();
    });

	QVBoxLayout* vUserNameLayout = new QVBoxLayout;
	vUserNameLayout->addWidget(labelUserName);
	vUserNameLayout->addWidget(m_lineEditUserName);
	vUserNameLayout->setSpacing(8);
	vUserNameLayout->setMargin(0);

	QVBoxLayout* vPasswordLayout = new QVBoxLayout;
	vPasswordLayout->addWidget(labelPassword);
	vPasswordLayout->addWidget(m_lineEditPassword);
	vPasswordLayout->setSpacing(8);
	vPasswordLayout->setMargin(0);

	QHBoxLayout* hInputLayout = new QHBoxLayout;
	hInputLayout->addLayout(vUserNameLayout);
	hInputLayout->addLayout(vPasswordLayout);
	hInputLayout->setSpacing(30);
	hInputLayout->setMargin(0);

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(m_pButtonLogin);
	hButtonLayout->addWidget(m_pButtonReset);
    hButtonLayout->addWidget(m_screenChooseComboBox);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(10);
	hButtonLayout->setMargin(0);

	QVBoxLayout* hLoginBacklayout = new QVBoxLayout(m_loginBackWidget);;
	hLoginBacklayout->addWidget(loginLabel);
	hLoginBacklayout->addLayout(hInputLayout);
	hLoginBacklayout->addLayout(hButtonLayout);
	hLoginBacklayout->setSpacing(20);
	hLoginBacklayout->setContentsMargins(30, 20, 30, 30);

    // 如果只有一个屏幕则显示在当前屏幕上;
    if (qApp->screens().count() <= 1)
    {
        m_screenChooseComboBox->setVisible(false);
    }
}

void WheelRobotLoginWindow::initNetWorkSetWidget()
{
	
}

void WheelRobotLoginWindow::initScreenShowChoose()
{
    m_screenChooseComboBox = new InputWidget(InputWidgetType::WheelComboBox);
    m_screenChooseComboBox->setTipText("屏幕选择");
    m_screenChooseComboBox->setStyleSheet("*{color:black;}");

    QList<QScreen* > screenList = qApp->screens();
    QStringList strNameList;
    for (int i = 0; i < screenList.count(); i++)
    {
        strNameList.append(screenList[i]->name());
    }
    m_screenChooseComboBox->setComboBoxContent(strNameList);
}

void WheelRobotLoginWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.drawPixmap(QRect(SHADOW_WIDTH, SHADOW_WIDTH, this->width() - 2 * SHADOW_WIDTH, this->height() - 2 * SHADOW_WIDTH), QPixmap(":/Resources/WheelRobotLoginWindow/Image/BackImage.jpg"));
}

void WheelRobotLoginWindow::loginFailed()
{
	m_pButtonLogin->setText("登录");

	DLMessageBox* messageBox = new DLMessageBox;
	messageBox->setFixedWidth(250);
	messageBox->setMessageContent("登录失败");
	messageBox->setWindowModality(Qt::ApplicationModal);
	messageBox->show();
}

int WheelRobotLoginWindow::getCurrentScreenChoosedIndex()
{
    int currentScreenIndex = m_screenChooseComboBox->getComboBoxCurrentIndex();
    if (currentScreenIndex < 0)
    {
        currentScreenIndex = 0;
    }

    return currentScreenIndex;
}
