#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QScrollArea>
#include <QPropertyAnimation>

class InputWidget;
class LoginWindow : public QWidget
{
	Q_OBJECT

public:
	LoginWindow(QWidget *parent = NULL);
	~LoginWindow();

	// 设置标题文字;
	void setTitleText(QString titleText);

	// 登录失败;
	void loginFailed();

    // 获取当前登录时选择在哪个屏幕上显示;
    int getCurrentScreenChoosedIndex();

private:
	// 初始化窗口部件;
	void initWidget();
	// 初始化登录窗口;
	void initLoginWidget();
	// 初始化网络设置窗口;
	void initNetWorkSetWidget();
	// 读取网络设置配置文件;
	void readNetWorkSetInfo();
    // 初始化窗口在哪个屏幕上显示;
    void initScreenShowChoose();

	// 加载样式表;
	void loadStyleSheet(const QString &sheetName);
	// 绘制背景;
	void paintEvent(QPaintEvent *event);

signals:
	// 登录/退出;
	void signalLogin(QByteArray userName, QByteArray password, QString strIp, QString strPort);
	void signalQuit();

private slots:
	// 网络设置;
	void onNetWorkSet();

private:
	QPushButton * m_pButtonNetWorkSet;

	// 登录控件;
	QPushButton* m_labelIcon;
	QLabel* m_labelTitleText;

	QLabel* m_labelUserName;
	QLabel* m_labelPassword;

	QLineEdit* m_lineEditUserName;
	QLineEdit* m_lineEditPassword;

	QPushButton* m_pButtonLogin;
	QPushButton* m_pButtonQuit;

	// 网络设置控件;
	QPushButton* m_labelNetworkIcon;
	QLabel* m_labelNetworkTitleText;

	QLabel* m_labelIP;
	QLabel* m_labelPort;

	QLineEdit* m_lineEditIP;
	QLineEdit* m_lineEditPort;

	QPushButton* m_pButtonOK;
	QPushButton* m_pButtonBack;

	QScrollArea* m_leftWidget;

	QWidget* m_loginWidget;
	QWidget* m_networkSetWidget;

	QPropertyAnimation* m_animationForScroll;

    // 当前电脑屏幕选择ComboBox;
    InputWidget* m_screenChooseComboBox;
};
