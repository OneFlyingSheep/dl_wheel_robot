#ifndef _TURN_PAGE_WIDGET_H
#define _TURN_PAGE_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

enum TurnPageButtonType
{
    HeadPage = 0,           // 首页;
    LastPage,               // 上一页;
    NextPage,               // 下一页;
    TailPage,               // 尾页;
    RefreshPage             // 刷新;
};

class SpliterLabel : public QLabel
{
public:
	SpliterLabel()
	{
		this->setFixedSize(QSize(3, 16));
		this->setStyleSheet("border:none;border-left:1px solid #69A09B;border-right:2px solid white;");
	}
};

class TurnPageButton : public QPushButton
{
public:
	TurnPageButton(QWidget* parent = NULL)
	{
		this->setFixedSize(QSize(16, 16));
		this->setStyleSheet("QPushButton{border:none;}\
								QPushButton:pressed{padding-left:2px;padding-top:2px;}");
	}

	// 设置正常显示button的icon路径;
	void setButtonIcon(QString iconPath)
	{
		m_iconPath = iconPath;
		this->setIconSize(QSize(14, 14));
		this->setIcon(QIcon(iconPath));
	}

	// 设置按钮是否disable;
	void setButtonDisable(bool isDisable)
	{
		if (isDisable)
		{
			m_iconPath.replace("Normal", "Disable");
		}
		else
		{
			m_iconPath.replace("Disable", "Normal");
		}
		this->setIcon(QIcon(m_iconPath));
		this->setDisabled(isDisable);
	}

private:
	QString m_iconPath;
};

class TurnPageWidget : public QWidget
{
	Q_OBJECT

public:
	TurnPageWidget(QWidget* parent = NULL);
	
	// 设置翻页Widget信息;
	void setTurnPageInfo(int currentPageIndex, int totalPage, int totalCount, int perPageCount, int unCheckedCount = 0);

    // 设置未审核条数Label是否可见;
    void setUnCheckedLabelVisibel(bool isVisible);

private:
	void initWidget();

	void paintEvent(QPaintEvent *event);

signals:
    // 通知页数变化;
    void signalPageChanged(int);

    // 通知刷新当前页;
    void signalRefreshCurrentPage();

private slots:
    void onButtonClicked(int buttonId);

private:
    // 翻页按钮;
	TurnPageButton* m_pButtonHeadPage;
	TurnPageButton* m_pButtonLastPage;
	TurnPageButton* m_pButtonNextPage;
	TurnPageButton* m_pButtonTailPage;
	TurnPageButton* m_pButtonRefreshPage;
    // 页数信息;
	QLabel* m_labelPageCount;
	QLabel* m_labelPageInfo;
    
    // 首页中需要增加未审核条数;
    QLabel* m_unCheckedCountLabel;

    // 当前页数输入、显示框;
	QLineEdit* m_lineEditCurrentPage;

    // 当前table的页数;
    int m_currentPageIndex;
    int m_currentTotalPage;
};

#endif // !_TURN_PAGE_WIDGET_H
