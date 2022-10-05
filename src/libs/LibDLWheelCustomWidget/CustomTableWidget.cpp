#include "CustomTableWidget.h"
#include <QHeaderView>
#include <QVBoxLayout>

CustomTableWidget::CustomTableWidget(int column, bool isWithCheckBox)
    : m_columnCount(column)
    , m_isWithCheckBox(isWithCheckBox)
    , m_checkBoxHeader(NULL)
{
    if (isWithCheckBox)
    {
        m_columnCount++;
    }
    initWidget();
	this->setStyleSheet("QTableWidget{border:none; alternate-background-color:rgb(240, 250, 247);}\
		QTableWidget::item{padding:5px;background:transparent;}\
        QTableWidget::item:selected{ background:rgb(106, 194, 172);}");
}

void CustomTableWidget::initWidget()
{
	m_tableWidget = new QTableWidget;
	m_tableWidget->setAlternatingRowColors(true);
	m_tableWidget->setColumnCount(m_columnCount);
    m_tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	m_tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_tableWidget->verticalHeader()->setVisible(false);
    if (m_isWithCheckBox)
    {
        m_checkBoxHeader = new CheckBoxHeaderView(0, Qt::Horizontal, m_tableWidget);
        connect(m_checkBoxHeader, &CheckBoxHeaderView::signalCheckStausChange, this, &CustomTableWidget::onHeaderCheckBoxStateChanged);
        m_tableWidget->setHorizontalHeader(m_checkBoxHeader);
        m_tableWidget->setColumnWidth(0, 30);
        m_tableWidget->setColumnWidth(1, 40);
        m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        m_tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    }
    else
    {
        m_tableWidget->setColumnWidth(0, 40);
        m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    }

    connect(m_tableWidget, &QTableWidget::cellDoubleClicked, this, [=](int row) {
        emit signalTableDoubleClicked(row);
    });

	m_turnPageWidget = new TurnPageWidget;
    connect(m_turnPageWidget, &TurnPageWidget::signalPageChanged, this, [=](int pageIndex) {
        setHeaderCheckBoxState(false);
        emit signalPageChanged(pageIndex);
    });
    connect(m_turnPageWidget, &TurnPageWidget::signalRefreshCurrentPage, this, [=] {
        setHeaderCheckBoxState(false);
        emit signalRefreshCurrentPage();
    });

	QVBoxLayout* vTableLayout = new QVBoxLayout(this);
	vTableLayout->addWidget(m_tableWidget);
	vTableLayout->addWidget(m_turnPageWidget);
	vTableLayout->setSpacing(0);
	vTableLayout->setMargin(0);
}

void CustomTableWidget::onHeaderCheckBoxStateChanged(bool isChecked)
{
	if (isChecked)
	{
		for (int i = 0; i < m_tableWidget->rowCount(); i++)
		{
			QTableWidgetItem* item = m_tableWidget->item(i, 0);
            if (item != NULL)
            {
                item->setCheckState(Qt::Checked);
            }
		}
	}
	else
	{
		for (int i = 0; i < m_tableWidget->rowCount(); i++)
		{
			QTableWidgetItem* item = m_tableWidget->item(i, 0);
            if (item != NULL)
            {
                item->setCheckState(Qt::Unchecked);
            }
		}
	}
}

QTableWidget* CustomTableWidget::getTableWidget()
{
	return m_tableWidget;
}

void CustomTableWidget::setHorizontalHeaderLabels(QStringList headerLabels)
{
	headerLabels.prepend("");
	m_tableWidget->setHorizontalHeaderLabels(headerLabels);
}

void CustomTableWidget::setTurnPageInfo(int currentPageIndex,int totalPage, int totalCount, int perPageCount, int unCheckedCount /* = 0 */)
{
	m_turnPageWidget->setTurnPageInfo(currentPageIndex, totalPage, totalCount, perPageCount, unCheckedCount);
}

void CustomTableWidget::setIsShowTurnPageWidget(bool isShow)
{
    m_turnPageWidget->setVisible(isShow);
}

void CustomTableWidget::setHeaderCheckBoxState(bool isCheck)
{
    if (m_checkBoxHeader != NULL)
    {
        m_checkBoxHeader->setIsChecked(isCheck);
    }
}

void CustomTableWidget::setItemBackWhite()
{
    this->setStyleSheet("QTableWidget{border:none; alternate-background-color:rgb(240, 250, 247);}\
		QTableWidget::item:normal{padding:5px;}\
        QTableWidget::item:selected{ background:rgb(106, 194, 172);}");
    m_tableWidget->setAlternatingRowColors(false);
}

void CustomTableWidget::setUnCheckedLabelVisibel(bool isVisible)
{
    m_turnPageWidget->setUnCheckedLabelVisibel(isVisible);
}

bool CustomTableWidget::get_all_check_status()
{
    return m_checkBoxHeader->get_all_check_status();
}
