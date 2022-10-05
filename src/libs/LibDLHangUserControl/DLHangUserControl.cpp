#include "DLHangUserControl.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include <QHBoxLayout>

#pragma execution_character_set("utf-8")

#define USER_INFO_WIDTH			300

DLHangUserControl::DLHangUserControl(QWidget* parent /* = NULL */)
	: QWidget(parent)
{
	initWidget();
	this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
}


DLHangUserControl::~DLHangUserControl()
{
}

void DLHangUserControl::initWidget()
{
	initUserInfoWidget();
	initUserListWidget();

	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addWidget(m_userInfoWidget);
	vLayout->addWidget(m_userListWidget);
	vLayout->setSpacing(10);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addLayout(vLayout);
	hMainLayout->addStretch();
	hMainLayout->setMargin(10);
}

void DLHangUserControl::initUserInfoWidget()
{
	m_userInfoWidget = new BaseWidget;
	m_userInfoWidget->setTitleContent("用户信息");
	m_userInfoWidget->setFixedSize(QSize(USER_INFO_WIDTH, 160));

	m_userNameInputWidget = new InputWidget;
	m_userNameInputWidget->setTipText("用户名");

	m_userPasswordInputWidget = new InputWidget;
	m_userPasswordInputWidget->setTipText("密码");

	m_userTypeInputWidget = new InputWidget(InputWidgetType::ComboBox);
	m_userTypeInputWidget->setTipText("用户权限");

	QVBoxLayout* vUSerInfoLayout = new QVBoxLayout(m_userInfoWidget->getCenterWidget());
	vUSerInfoLayout->addWidget(m_userNameInputWidget);
	vUSerInfoLayout->addWidget(m_userPasswordInputWidget);
	vUSerInfoLayout->addWidget(m_userTypeInputWidget);
	vUSerInfoLayout->setSpacing(10);
	vUSerInfoLayout->setContentsMargins(50, 10, 50, 10);
}

void DLHangUserControl::initUserListWidget()
{
	m_userListWidget = new BaseWidget;
	m_userListWidget->setTitleContent("用户列表");
	m_userListWidget->setFixedWidth(USER_INFO_WIDTH);

	m_userTreeWidget = new QTreeWidget;

	m_pButtonAddNewUser = new QPushButton;
	m_pButtonAddNewUser->setText("添加用户");
	m_pButtonAddNewUser->setFixedSize(QSize(75, 25));

	m_pButtonSaveUserInfo = new QPushButton;
	m_pButtonSaveUserInfo->setText("保存用户");
	m_pButtonSaveUserInfo->setFixedSize(QSize(75, 25));

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(m_pButtonAddNewUser);
	hButtonLayout->addWidget(m_pButtonSaveUserInfo);
	hButtonLayout->setSpacing(20);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vUserListLayout = new QVBoxLayout(m_userListWidget->getCenterWidget());
	vUserListLayout->addWidget(m_userTreeWidget);
	vUserListLayout->addLayout(hButtonLayout);
	vUserListLayout->setSpacing(15);
	vUserListLayout->setMargin(20);
}