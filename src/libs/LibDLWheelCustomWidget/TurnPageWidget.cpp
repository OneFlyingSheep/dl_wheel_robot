#include "TurnPageWidget.h"
#include <QHBoxLayout>
#include <QPainter>
#include <QButtonGroup>
#include <QIntValidator>

#pragma execution_character_set("utf-8")

TurnPageWidget::TurnPageWidget(QWidget* parent)
	:QWidget(parent)
    , m_currentPageIndex(1)
    , m_currentTotalPage(1)
{
	initWidget();
	this->setFixedHeight(30);
}

void TurnPageWidget::setTurnPageInfo(int currentPageIndex, int totalPage, int totalCount, int perPageCount, int unCheckedCount)
{
    // 如果总条数小于每页显示条数，则每页显示条数等于总条数;
    if (totalCount < perPageCount)
    {
        perPageCount = totalCount;
    }

    // 限制LineEdit页数范围;
    m_currentPageIndex = currentPageIndex;
    m_currentTotalPage = totalPage;

    m_lineEditCurrentPage->setValidator(new QIntValidator(1, totalPage, this));
    m_lineEditCurrentPage->setText(QString::number(m_currentPageIndex));

	m_labelPageCount->setText(QString("页,共 %1页").arg(totalPage));
	m_labelPageInfo->setText(QString("显示 1-%1条， 共 %2条").arg(perPageCount).arg(totalCount));

    m_unCheckedCountLabel->setText(QString("，未审核 %1 条").arg(unCheckedCount));
}

void TurnPageWidget::setUnCheckedLabelVisibel(bool isVisible)
{
    m_unCheckedCountLabel->setVisible(isVisible);
}

void TurnPageWidget::initWidget()
{
    QButtonGroup* buttonGroup = new QButtonGroup(this);

	m_pButtonHeadPage = new TurnPageButton;
	m_pButtonHeadPage->setButtonIcon(":/Resources/Common/image/HeadPage_Normal.png");
    buttonGroup->addButton(m_pButtonHeadPage, HeadPage);

	m_pButtonLastPage = new TurnPageButton;
	m_pButtonLastPage->setButtonIcon(":/Resources/Common/image/LastPage_Normal.png");
    buttonGroup->addButton(m_pButtonLastPage, LastPage);

	m_pButtonNextPage = new TurnPageButton;
	m_pButtonNextPage->setButtonIcon(":/Resources/Common/image/NextPage_Normal.png");
    buttonGroup->addButton(m_pButtonNextPage, NextPage);

	m_pButtonTailPage = new TurnPageButton;
	m_pButtonTailPage->setButtonIcon(":/Resources/Common/image/TailPage_Normal.png");
    buttonGroup->addButton(m_pButtonTailPage, TailPage);

	m_pButtonRefreshPage = new TurnPageButton;
	m_pButtonRefreshPage->setButtonIcon(":/Resources/Common/image/RefreshPage.png");
    buttonGroup->addButton(m_pButtonRefreshPage, RefreshPage);

    connect(buttonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &TurnPageWidget::onButtonClicked);

	m_labelPageCount = new QLabel("页,共 0页");
	m_labelPageInfo = new QLabel("显示 1-0条， 共 0条");

    m_unCheckedCountLabel = new QLabel("，未审核 0 条");
    m_unCheckedCountLabel->setVisible(false);

	m_lineEditCurrentPage = new QLineEdit;
	m_lineEditCurrentPage->setFixedSize(QSize(30, 20));
	m_lineEditCurrentPage->setAlignment(Qt::AlignRight);
    connect(m_lineEditCurrentPage, &QLineEdit::editingFinished, this, [=] {
        m_currentPageIndex = m_lineEditCurrentPage->text().toInt();
        emit signalPageChanged(m_currentPageIndex);
    });

	QHBoxLayout* hButtonLayout = new QHBoxLayout(this);
	hButtonLayout->addWidget(m_pButtonHeadPage);
	hButtonLayout->addWidget(m_pButtonLastPage);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(new QLabel("第"));
	hButtonLayout->addWidget(m_lineEditCurrentPage);
	hButtonLayout->addWidget(m_labelPageCount);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(m_pButtonNextPage);
	hButtonLayout->addWidget(m_pButtonTailPage);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(m_pButtonRefreshPage);
	hButtonLayout->addWidget(new SpliterLabel);
	hButtonLayout->addWidget(m_labelPageInfo);
    hButtonLayout->addWidget(m_unCheckedCountLabel);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(5);
	hButtonLayout->setContentsMargins(5, 0, 0, 0);
}

void TurnPageWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(175, 191, 255));
}

void TurnPageWidget::onButtonClicked(int buttonId)
{
    TurnPageButtonType buttonType = TurnPageButtonType(buttonId);
    switch (buttonType)
    {
    case HeadPage:
    {
        m_currentPageIndex = 1;
        emit signalPageChanged(m_currentPageIndex);
    }
    break;
    case LastPage:
    {
        m_currentPageIndex--;
        if (m_currentPageIndex < 1)
        {
            m_currentPageIndex = 1;
        }
        emit signalPageChanged(m_currentPageIndex);
    }
    break;
    case NextPage:
    {
        m_currentPageIndex++;
        if (m_currentPageIndex > m_currentTotalPage)
        {
            m_currentPageIndex = m_currentTotalPage;
        }
        emit signalPageChanged(m_currentPageIndex);
    }
    break;
    case TailPage:
    {
        m_currentPageIndex = m_currentTotalPage;
        emit signalPageChanged(m_currentPageIndex);
    }
    break;
    case RefreshPage:
    {
        emit signalRefreshCurrentPage();
    }
    break;
    default:
        break;
    }
}
