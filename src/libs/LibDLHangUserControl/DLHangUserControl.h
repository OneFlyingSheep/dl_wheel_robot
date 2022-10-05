#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>

class BaseWidget;
class InputWidget;

/********用户管理页面********/

class DLHangUserControl : public QWidget
{
	Q_OBJECT

public:
	DLHangUserControl(QWidget* parent = NULL);
	~DLHangUserControl();

private:
    // 初始化控件;
	void initWidget();
	void initUserInfoWidget();
	void initUserListWidget();

private:
    // 背景Widget;
	BaseWidget * m_userInfoWidget;
	BaseWidget * m_userListWidget;

	// 用户列表;
	QTreeWidget* m_userTreeWidget;
	QPushButton* m_pButtonAddNewUser;
	QPushButton* m_pButtonSaveUserInfo;
	InputWidget* m_userNameInputWidget;
	InputWidget* m_userPasswordInputWidget;
	InputWidget* m_userTypeInputWidget;
};