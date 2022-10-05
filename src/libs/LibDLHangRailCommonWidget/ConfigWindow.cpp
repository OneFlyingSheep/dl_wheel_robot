#include "ConfigWindow.h"
#include "InputWidget.h"
#include <QApplication>
#include <QFile>
#include <QDebug>
#include <QSettings>
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"

TipWindow::TipWindow(QWidget* parent)
	: BaseWidget(parent, PopupWindow)
{
	initWidget();
	this->setFixedSize(QSize(320, 150));
}

void TipWindow::initWidget()
{
	this->setTitleContent("��ʾ");
	m_tipIconLabel = new QLabel;
	m_tipIconLabel->setFixedSize(QSize(48, 48));
	m_tipIconLabel->setPixmap(QPixmap(":/Resources/TipIcon.png"));

	m_tipTextLabel = new QLabel("�������������������\n�������رճ������");
	m_tipTextLabel->setFont(QFont("Microsoft YaHei", 13));
	m_tipTextLabel->setWordWrap(true);

	m_pButtonOk = new QPushButton("ȷ��");
	m_pButtonOk->setObjectName("PatrolButton");
	m_pButtonOk->setFixedSize(QSize(80, 35));

	connect(m_pButtonOk, &QPushButton::clicked, this, [=] {
		this->accept();
	});
	QHBoxLayout* hContentLayout = new QHBoxLayout;
	hContentLayout->addWidget(m_tipIconLabel);
	hContentLayout->addStretch();
	hContentLayout->addWidget(m_tipTextLabel);
	hContentLayout->addStretch();
	hContentLayout->setSpacing(20);
	hContentLayout->setMargin(0);

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(m_pButtonOk);
	hContentLayout->setSpacing(0);
	hContentLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
	vMainLayout->addLayout(hContentLayout);
	vMainLayout->addLayout(hButtonLayout);
	vMainLayout->setSpacing(20);
	vMainLayout->setContentsMargins(20, 25, 20, 10);
}

UserTypeChooseWindow::UserTypeChooseWindow(QWidget* parent /* = NULL */)
	: BaseWidget(parent, BaseWidgetType::PopupWindow)
{
	initWidget();
	this->setFixedSize(QSize(280, 180));
}

void UserTypeChooseWindow::initWidget()
{
	this->setTitleContent("�û�����ѡ��");
	m_userTypeWidget = new InputWidget(InputWidgetType::ComboBox);
	m_userTypeWidget->setTipText("ѡ���û�");

	QStringList userTypeList;
	userTypeList << "��ͨ�û�" << "����Ա�û�";
	m_userTypeWidget->setComboBoxContent(userTypeList);

	m_pButtonOk = new QPushButton("ȷ��");
	m_pButtonOk->setObjectName("PatrolButton");
	m_pButtonOk->setFixedSize(QSize(60, 30));
	connect(m_pButtonOk, &QPushButton::clicked, this, [=] {
		this->accept();
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(m_pButtonOk);
	
	QVBoxLayout* vMainLayout = new QVBoxLayout(m_centerWidget);
	vMainLayout->addWidget(m_userTypeWidget);
	vMainLayout->addLayout(hButtonLayout);
	vMainLayout->setSpacing(10);
	vMainLayout->setContentsMargins(20, 25, 20, 10);
}

QString UserTypeChooseWindow::getUserType()
{
	return m_userTypeWidget->getComboBoxCurrentContent();
}

bool ConfigWindow::checkConfigFile(QByteArray configDBName)
{
	return DLHangRailCommonTools::checkConfigFile(configDBName);
}

ConfigWindow::ConfigWindow(QWidget* parent /* = NULL */)
	: BaseWidget(parent, PopupWindow)
{
	initWidget();

	QString strFilePath = QApplication::applicationDirPath() + DB_CONFIG_DATA_PATH;
	// ������������ļ���˵�������ļ�����Ϣ��ȫ����Ҫ��ȫ;
	if (QFile::exists(strFilePath))
	{
		initConfigData();
	}
	
	this->setFixedSize(QSize(300, 450));
}

ConfigWindow::~ConfigWindow()
{

}

void ConfigWindow::initWidget()
{
	this->setTitleContent("���ݿ�����");

	m_hostNameWidget = new InputWidget;
	m_hostNameWidget->setTipText("������");

	m_dbNameWidget = new InputWidget;
	m_dbNameWidget->setTipText("���ݿ���");

	m_userNameWidget = new InputWidget;
	m_userNameWidget->setTipText("�û���");

	m_userPwdWidget = new InputWidget;
	m_userPwdWidget->setTipText("�û�����");

	m_pButtonOk = new QPushButton("ȷ��", this);
	m_pButtonOk->setObjectName("PatrolButton");
	m_pButtonOk->setFixedSize(QSize(95, 30));
	
// 	m_pButtonCancel = new QPushButton("ȡ��", this);
// 	m_pButtonCancel->setFixedSize(QSize(95, 30));

	connect(m_pButtonOk, SIGNAL(clicked()), this, SLOT(onButtonOkClicked()));

	QHBoxLayout* hButtonLayout = new QHBoxLayout();
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(m_pButtonOk);
//	hButtonLayout->addWidget(m_pButtonCancel);
	hButtonLayout->setSpacing(20);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vWidgetLayout = new QVBoxLayout();
	vWidgetLayout->addWidget(m_hostNameWidget);
	vWidgetLayout->addWidget(m_dbNameWidget);
	vWidgetLayout->addWidget(m_userNameWidget);
	vWidgetLayout->addWidget(m_userPwdWidget);
	vWidgetLayout->addLayout(hButtonLayout);
	vWidgetLayout->setMargin(0);
	vWidgetLayout->setSpacing(20);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this->getCenterWidget());
	hMainLayout->addStretch();
	hMainLayout->addLayout(vWidgetLayout);
	hMainLayout->addStretch();
}

void ConfigWindow::initConfigData()
{
	DBInfoData_T dbInfoData = DLHangRailCommonTools::getDBInfoData();

	m_hostNameWidget->setLineEditText(dbInfoData.hostName);
	m_dbNameWidget->setLineEditText(dbInfoData.dbName);
	m_userNameWidget->setLineEditText(dbInfoData.userName);
	m_userPwdWidget->setLineEditText(dbInfoData.password);
}

void ConfigWindow::onButtonOkClicked()
{
	DBInfoData_T dbInfoData;
	dbInfoData.hostName = m_hostNameWidget->getLineEditText();
	dbInfoData.dbName = m_dbNameWidget->getLineEditText();
	dbInfoData.userName = m_userNameWidget->getLineEditText();
	dbInfoData.password = m_userPwdWidget->getLineEditText();

	DLHangRailCommonTools::setDBInfoData(dbInfoData);

	close();
}