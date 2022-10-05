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

/*************�����ڵײ�item�ؼ�**************/

class BottomWidgetItem : public QWidget
{
	Q_OBJECT

public:
	BottomWidgetItem(bool isMainItem = false);

	// ����Item����;
	void setItemType(SystemGuideMenuItemType itemType);
	// ���ݵ�ǰitem���ͻ�ȡ��Ӧ����;
	QString getItemText(SystemGuideMenuItemType itemType);
	// ��ȡ��ǰitem����;
	SystemGuideMenuItemType getItemType()
	{
		return m_itemType;
	}

	// �����Ƿ�checked;
	void setChecked(bool isChecked)
	{
		m_isChecked = isChecked;
		update();
	}

private:
	// �¼�;
	void paintEvent(QPaintEvent *event);
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

signals:
	// �رյ�ǰitem;
	void signalItemClose(SystemGuideMenuItemType itemType);
	// ����˵�ǰitem;
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

/********�����ڵײ��ؼ�*********/

class BottomWidget : public QWidget
{
	Q_OBJECT

public:
	BottomWidget(QWidget *parent = Q_NULLPTR);

    // ���ô��ڿ��(��������ʾ��ʱ�򣬱���Ҫ���ô˿��);
    void setScreenWidth(int width);

	// ���bottomItem;
	void addBottomItem(SystemGuideMenuItemType itemType);

private:
	// ��ʼ���ؼ�;
	void initWidget();
	// �����¼�;
	void paintEvent(QPaintEvent *event);
	// �Ƿ��Ѿ����ڸ�item;
	bool isExistItem(SystemGuideMenuItemType itemType);
signals:
	// ����˵�ǰitem;
	void signalItemClicked(SystemGuideMenuItemType itemType);
	void CloseItemSignal(SystemGuideMenuItemType itemType);

public slots:
    // ����item״̬;
	void onUpdateItemState(SystemGuideMenuItemType itemType);

private:
	QList<BottomWidgetItem*> m_lstBottomItems;
	int m_screenWidth;

	QScrollArea* m_bottomItemArea;
	QWidget* m_itemBackWidget;
	QToolButton* m_pButtonMoveLeft;
	QToolButton* m_pButtonMoveRight;
};
