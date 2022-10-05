#pragma once

#include <QtWidgets/QWidget>
#include <QLabel>
#include <QToolButton>
#include <QStyle>
#include <QHBoxLayout>
#include <QPainter>
#include <QScrollArea>
#include "common/DLWheelRobotGlobalDef.hpp"

#pragma execution_character_set("utf-8")

/*************主窗口底部item控件**************/

class BottomWidgetItem : public QWidget
{
	Q_OBJECT

public:
	BottomWidgetItem(bool isMainItem = false);

	// 设置Item类型;
	void setItemType(SystemGuideMenuItemType itemType);
	// 根据当前item类型获取对应文字;
	QString getItemText(SystemGuideMenuItemType itemType);
	// 获取当前item类型;
	SystemGuideMenuItemType getItemType()
	{
		return m_itemType;
	}

	// 设置是否checked;
	void setChecked(bool isChecked)
	{
		m_isChecked = isChecked;
		update();
	}

private:
	// 事件;
	void paintEvent(QPaintEvent *event);
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

signals:
	// 关闭当前item;
	void signalItemClose(SystemGuideMenuItemType itemType);
	// 点击了当前item;
	void signalItemClicked(SystemGuideMenuItemType itemType);

private:
	QLabel* m_labelIcon;
	QLabel* m_labelText;
	QToolButton* m_pButtonClose;
	bool m_isChecked;
	bool m_isEnter;
	bool m_isMainItem;
	SystemGuideMenuItemType m_itemType;
};

/********主窗口底部控件*********/

class BottomWidget : public QWidget
{
	Q_OBJECT

public:
	BottomWidget(QWidget *parent = Q_NULLPTR);

    // 设置窗口宽度(主窗口显示的时候，必须要设置此宽度);
    void setScreenWidth(int width);

	// 添加bottomItem;
	void addBottomItem(SystemGuideMenuItemType itemType);

private:
	// 初始化控件;
	void initWidget();
	// 绘制事件;
	void paintEvent(QPaintEvent *event);
	// 是否已经存在该item;
	bool isExistItem(SystemGuideMenuItemType itemType);
signals:
	// 点击了当前item;
	void signalItemClicked(SystemGuideMenuItemType itemType);
	void CloseItemSignal(SystemGuideMenuItemType itemType);

public slots:
    // 更新item状态;
	void onUpdateItemState(SystemGuideMenuItemType itemType);

private:
	QList<BottomWidgetItem*> m_lstBottomItems;
	int m_screenWidth;

	QScrollArea* m_bottomItemArea;
	QWidget* m_itemBackWidget;
	QToolButton* m_pButtonMoveLeft;
	QToolButton* m_pButtonMoveRight;
};
