#ifndef CUSTOM_BUTTON_LIST_H
#define CUSTOM_BUTTON_LIST_H

#include <QWidget>
#include <QToolButton>
#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>

enum BUTTON_DISPLAY_TYPE 
{
	RIGHT_START_DISPLAY,
	LEFT_START_DISPLAY,
	BUTTON_DISPLAY_TYPE_NUM
};

class CustomButtonListWidget : public QWidget
{
	Q_OBJECT

public:
	CustomButtonListWidget(QWidget* parent = NULL, int iButtonDispalyType = LEFT_START_DISPLAY);
	
	// 添加控件;
	void addWidget(QWidget* childWidget);

	// 添加按钮;
	void addToolButton(int buttonId, QString buttonText, QString iconPath, QSize buttonSize = QSize(60, 20), QSize IconSize = QSize(16, 16));	
	// 添加按钮完成， 开始布局;
	void addWidgetFinished();

signals:
	// 通知按钮按下;
	void signalButtonClicked(int buttonId);

private:
	void paintEvent(QPaintEvent *event);

private:
	QButtonGroup * m_pToolButtonGroup;
	QList<QWidget*> m_widgetList;
	int m_iButtonDisplayType;
};

#endif // CUSTOM_BUTTON_LIST_H
