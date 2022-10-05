#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>

class BaseWidget;
class InputWidget;

/********�û�����ҳ��********/

class DLHangUserControl : public QWidget
{
	Q_OBJECT

public:
	DLHangUserControl(QWidget* parent = NULL);
	~DLHangUserControl();

private:
    // ��ʼ���ؼ�;
	void initWidget();
	void initUserInfoWidget();
	void initUserListWidget();

private:
    // ����Widget;
	BaseWidget * m_userInfoWidget;
	BaseWidget * m_userListWidget;

	// �û��б�;
	QTreeWidget* m_userTreeWidget;
	QPushButton* m_pButtonAddNewUser;
	QPushButton* m_pButtonSaveUserInfo;
	InputWidget* m_userNameInputWidget;
	InputWidget* m_userPasswordInputWidget;
	InputWidget* m_userTypeInputWidget;
};