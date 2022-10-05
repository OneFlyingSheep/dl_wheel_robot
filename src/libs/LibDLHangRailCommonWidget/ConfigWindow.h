#ifndef CONFIG_WINDOW_H
#define CONFIG_WINDOW_H

#include "BaseWidget.h"
#include <QWidget>
#include <QPainter>
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>

#pragma execution_character_set("utf-8") 

class InputWidget;

class WelcomeWindow : public QWidget
{
	Q_OBJECT
public:
	WelcomeWindow(QWidget* parent = NULL)
		:QWidget(parent)
	{
		QLabel* m_textLabel = new QLabel(this);
		m_textLabel->setAlignment(Qt::AlignCenter);
		m_textLabel->setStyleSheet("color:rgb(255, 215, 0);font-size:40px;font:bold;");
		m_textLabel->setText("程序启动中...");

		QHBoxLayout* mainLayout = new QHBoxLayout(this);
		mainLayout->addWidget(m_textLabel);

		this->setFixedSize(QSize(960, 720));
		this->setWindowFlag(Qt::FramelessWindowHint);
		this->setAttribute(Qt::WA_TranslucentBackground);
		this->setAttribute(Qt::WA_DeleteOnClose);
	};
	~WelcomeWindow() {};

private:
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.setRenderHint(QPainter::Antialiasing);
		painter.drawPixmap(this->rect(), QIcon(":/Resources/welcomeWindowBack.jpg").pixmap(this->size()));
	}
};

class TipWindow : public BaseWidget
{
	Q_OBJECT

public:
	TipWindow(QWidget* parent = NULL);

private:
	void initWidget();

private:
	QLabel* m_tipTextLabel;
	QLabel* m_tipIconLabel;
};

class UserTypeChooseWindow : public BaseWidget
{
public:
	UserTypeChooseWindow(QWidget* parent = NULL);

	QString getUserType();
private:
	void initWidget();

private:
	InputWidget* m_userTypeWidget;
};

class ConfigWindow : public BaseWidget
{
	Q_OBJECT

public:
	ConfigWindow(QWidget* parent = NULL);
	~ConfigWindow();

	static bool checkConfigFile(QByteArray configDBName);

private:
	void initWidget();
	// 从配置文件中读取字段;
	void initConfigData();

private slots:
	void onButtonOkClicked();

private:
	InputWidget* m_hostNameWidget;
	InputWidget* m_dbNameWidget;
	InputWidget* m_userNameWidget;
	InputWidget* m_userPwdWidget;

	QPushButton* m_pButtonOk;
	QPushButton* m_pButtonCancel;

};


#endif
