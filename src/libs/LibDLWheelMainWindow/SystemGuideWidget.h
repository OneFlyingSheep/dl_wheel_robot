#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QPainter>
#include "common/DLWheelRobotGlobalDef.hpp"

class SystemGuideMenu;

/*********ϵͳ�˵�item**********/

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

    // ����ͼ��;
	void setIcon(QIcon icon)
	{
		m_pButtonIcon->setPixmap(icon.pixmap(m_pButtonIcon->size()));
	}

    // ��������;
	void setText(const QString& text)
	{
		m_pButtonText->setText(text);
	}

    // ���ò˵�����;
	void setMenuType(SystemGuideMenuType menuType)
	{
		m_menuType = menuType;
	}

    // ��ȡ�˵�����;
	SystemGuideMenuType getMenuType()
	{
		return m_menuType;
	}

private:
    // ���Ʊ߿�;
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

    // ������/�����¼�����������/��ʾ�Ӳ˵�;
	void enterEvent(QEvent *event)
	{
		emit signalEnterItem(m_menuType);
	}

	void leaveEvent(QEvent *event)
	{
		emit signalLeaveItem();
	}

signals:
    // ������item�ź�;
	void signalEnterItem(SystemGuideMenuType);
    // ����뿪item�ź�;
	void signalLeaveItem();

private:
	QLabel* m_pButtonIcon;
	QLabel* m_pButtonText;
	QLabel* m_pButtonRightArrow;

	SystemGuideMenuType m_menuType;
};

class SystemGuideMenu;

/**********���˵��ؼ�************/

class SystemGuideWidget : public QWidget
{
	Q_OBJECT

public:
	SystemGuideWidget(QWidget *parent = NULL);
	~SystemGuideWidget();

	// ���ز˵�;
	void hideMenuWidget();

    // ���õ�ǰ��¼�û�Ȩ�ޣ���ͬ�û�Ȩ�޲����˵������;
    void updateSystemMenu(WheelUserType currentLoginRole);

private:
    // ��ʼ���˵��ؼ�;
	void initWidget();
	void initListWidget();

    // ���Ʊ߿򣬱���;
	void paintEvent(QPaintEvent *event);

    // ���������ʾ���˵�;
	void mouseReleaseEvent(QMouseEvent *event);

    // ���ڲ���Ծʱ�����ش���;
	bool eventFilter(QObject *watched, QEvent *event);

signals:
	// �˵����;
    void signalMenuItemClicked(SystemGuideMenuItemType itemType);

    // ������С��;
    void signalWindowMinsize();

    // ������;
    void signalCompareDetection();

private:
	QLabel* m_labelText;
	QLabel* m_labelArrow;

	QPushButton* m_pButtonUpArrow;
	QPushButton* m_pButtonDownArrow;

	QList<SystemGuideItem*> m_customItemList;
	QWidget* m_customListWidget;

	// ϵͳ�˵�;
	SystemGuideMenu* m_systemGuidMenu;
};
