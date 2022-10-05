#ifndef CUSTOM_TABLE_WIDGET
#define CUSTOM_TABLE_WIDGET

#include <QWidget>
#include <QTableWidget>
#include "TurnPageWidget.h"
#include <QHeaderView>
#include <QStyleOptionButton>
#include <QStyle>
#include <QCheckBox>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

#define CHECK_BOX_COLUMN 0

class CheckBoxHeaderView : public QHeaderView
{
	Q_OBJECT
public:
	CheckBoxHeaderView(int checkColumnIndex,
		Qt::Orientation orientation,
		QWidget * parent = 0) :
		QHeaderView(orientation, parent),
		m_bPressed(false),
		m_bChecked(false),
		m_bTristate(false),
		m_bNoChange(false)
	{
		m_checkColIdx = checkColumnIndex;
	//	setHighlightSections(false);
		// ��Ӧ���;
		setSectionsClickable(true);
		this->setFixedHeight(30);
    //    this->setStyleSheet("QHeaderView::section{background:gray;}");
	}

	// ���ñ���checkBox״̬;
    void setIsChecked(bool isChecked)
    {
        m_bChecked = isChecked;
        updateSection(CHECK_BOX_COLUMN);
    }

    bool get_all_check_status()
    {
        return m_bChecked;
    }

protected:
	void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
	{
		painter->save();
		QHeaderView::paintSection(painter, rect, logicalIndex);
		painter->restore();

		if (logicalIndex == CHECK_BOX_COLUMN)
		{
			QStyleOptionButton option;
			option.initFrom(this);

			if (m_bTristate && m_bNoChange)
				option.state |= QStyle::State_NoChange;
			else
				option.state |= m_bChecked ? QStyle::State_On : QStyle::State_Off;

			QCheckBox checkBox;
            checkBox.setCheckable(true);
            checkBox.setChecked(true);
			option.iconSize = QSize(20, 20);
			option.rect = rect;
			style()->drawPrimitive(QStyle::PE_IndicatorCheckBox, &option, painter, &checkBox);
            painter->setPen(QPen(QColor(216, 216, 216)));
            painter->drawLine(QPoint(0, 0), QPoint(0, 30));
		}
	}

	// ��갴�±�ͷ;
	void mousePressEvent(QMouseEvent *event)
	{
		int nColumn = logicalIndexAt(event->pos());
		if ((event->buttons() & Qt::LeftButton) && (nColumn == CHECK_BOX_COLUMN))
		{
			m_bPressed = true;
		}
		else
		{
			QHeaderView::mousePressEvent(event);
		}
	}

	// ���ӱ�ͷ�ͷţ������źţ�����model����;
	void mouseReleaseEvent(QMouseEvent *event)
	{
		if (m_bPressed)
		{
			if (m_bTristate && m_bNoChange)
			{
				m_bChecked = true;
				m_bNoChange = false;
			}
			else
			{
				m_bChecked = !m_bChecked;
			}

			updateSection(CHECK_BOX_COLUMN);
			emit signalCheckStausChange(m_bChecked);
		}
		else
		{
			QHeaderView::mouseReleaseEvent(event);
		}
		m_bPressed = false;
	}

signals:
	void signalCheckStausChange(bool);

private:
	int m_checkColIdx;
	bool m_bPressed;
	bool m_bChecked;
	bool m_bTristate;
	bool m_bNoChange;
};

class CustomTableWidget : public QWidget
{
	Q_OBJECT

public:
	CustomTableWidget(int column, bool isWithCheckBox = true);
	// ��ȡtableWidget;
	QTableWidget* getTableWidget();
	// ���ñ�ͷ����;
	void setHorizontalHeaderLabels(QStringList headerLabels);

	// ���÷�ҳWidget��Ϣ;
	void setTurnPageInfo(int currentPageIndex,int totalPage, int totalCount, int perPageCount, int unCheckedCount = 0);

    // �����Ƿ���ʾ��ҳwidget;
    void setIsShowTurnPageWidget(bool isShow);

    // ���ñ���checkBox״̬;
    void setHeaderCheckBoxState(bool isCheck);

    // �����Ƿ�table�е�item����ɫ����滻;
    void setItemBackWhite();

    // ����δ�������Label�Ƿ�ɼ�;
    void setUnCheckedLabelVisibel(bool isVisible);

    bool get_all_check_status();
private:
	void initWidget();

signals:
    // ֪ͨҳ���仯;
    void signalPageChanged(int);

    // ֪ͨˢ�µ�ǰҳ;
    void signalRefreshCurrentPage();

    // ֪ͨ��ǰtableWidget˫���¼�;
    void signalTableDoubleClicked(int row);

private slots:
	void onHeaderCheckBoxStateChanged(bool isChecked);

private:
	TurnPageWidget* m_turnPageWidget;
	QTableWidget* m_tableWidget;

	int m_columnCount;

    bool m_isWithCheckBox;
    
    // ��checkBox�ı�ͷ;
    CheckBoxHeaderView *m_checkBoxHeader;
};

#endif
