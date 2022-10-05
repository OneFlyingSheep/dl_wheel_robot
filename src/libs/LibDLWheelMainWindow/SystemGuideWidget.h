#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QPainter>
#include "common/DLWheelRobotGlobalDef.hpp"

class SystemGuideMenu;

/*********系统菜单item**********/

class SystemGuideItem : public QPushButton
{
	Q_OBJECT

public:
	SystemGuideItem(QWidget* parent = NULL)
		: QPushButton(parent)
	{
		m_pButtonIcon = new QLabel;
		m_pButtonIcon->setFixedSize(QSize(28, 28));

		m_pButtonText = new QLabel;
		m_pButtonText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

		m_pButtonRightArrow = new QLabel;
		m_pButtonRightArrow->setFixedSize(QSize(12, 12));
		m_pButtonRightArrow->setPixmap(QIcon(":/Resources/DLWheelMainWindow/Image/rightArrow.png").pixmap(m_pButtonRightArrow->size()));

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addWidget(m_pButtonIcon);
		hLayout->addWidget(m_pButtonText);
		hLayout->addWidget(m_pButtonRightArrow);
		hLayout->setSpacing(8);
		hLayout->setContentsMargins(15, 0, 10, 0);

		this->setCheckable(true);
		this->setFixedSize(QSize(190, 40));
		this->setStyleSheet("border:none;color:rgb(63,81,181);");
	}

    // 设置图标;
	void setIcon(QIcon icon)
	{
		m_pButtonIcon->setPixmap(icon.pixmap(m_pButtonIcon->size()));
	}

    // 设置文字;
	void setText(const QString& text)
	{
		m_pButtonText->setText(text);
	}

    // 设置菜单类型;
	void setMenuType(SystemGuideMenuType menuType)
	{
		m_menuType = menuType;
	}

    // 获取菜单类型;
	SystemGuideMenuType getMenuType()
	{
		return m_menuType;
	}

private:
    // 绘制边框;
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.fillRect(this->rect(), Qt::white);
		QPen pen;
		pen.setBrush(QColor(175, 191, 255));
		pen.setWidth(2);
		painter.setPen(pen);
		painter.drawRect(1, 1, this->width() - 2, this->height() - 2);
	}

    // 鼠标进入/进出事件，用于隐藏/显示子菜单;
	void enterEvent(QEvent *event)
	{
		emit signalEnterItem(m_menuType);
	}

	void leaveEvent(QEvent *event)
	{
		emit signalLeaveItem();
	}

signals:
    // 鼠标进入item信号;
	void signalEnterItem(SystemGuideMenuType);
    // 鼠标离开item信号;
	void signalLeaveItem();

private:
	QLabel* m_pButtonIcon;
	QLabel* m_pButtonText;
	QLabel* m_pButtonRightArrow;

	SystemGuideMenuType m_menuType;
};

class SystemGuideMenu;

/**********主菜单控件************/

class SystemGuideWidget : public QWidget
{
	Q_OBJECT

public:
	SystemGuideWidget(QWidget *parent = NULL);
	~SystemGuideWidget();

	// 隐藏菜单;
	void hideMenuWidget();

    // 设置当前登录用户权限，不同用户权限操作菜单需更新;
    void updateSystemMenu(WheelUserType currentLoginRole);

private:
    // 初始化菜单控件;
	void initWidget();
	void initListWidget();

    // 绘制边框，背景;
	void paintEvent(QPaintEvent *event);

    // 鼠标点击，显示主菜单;
	void mouseReleaseEvent(QMouseEvent *event);

    // 窗口不活跃时，隐藏窗口;
	bool eventFilter(QObject *watched, QEvent *event);

signals:
	// 菜单点击;
    void signalMenuItemClicked(SystemGuideMenuItemType itemType);

    // 窗口最小化;
    void signalWindowMinsize();

    // 异物检测;
    void signalCompareDetection();

private:
	QLabel* m_labelText;
	QLabel* m_labelArrow;

	QPushButton* m_pButtonUpArrow;
	QPushButton* m_pButtonDownArrow;

	QList<SystemGuideItem*> m_customItemList;
	QWidget* m_customListWidget;

	// 系统菜单;
	SystemGuideMenu* m_systemGuidMenu;
};
