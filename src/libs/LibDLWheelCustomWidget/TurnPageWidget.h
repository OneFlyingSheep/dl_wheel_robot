#ifndef _TURN_PAGE_WIDGET_H
#define _TURN_PAGE_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

enum TurnPageButtonType
{
    HeadPage = 0,           // ��ҳ;
    LastPage,               // ��һҳ;
    NextPage,               // ��һҳ;
    TailPage,               // βҳ;
    RefreshPage             // ˢ��;
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

	// ����������ʾbutton��icon·��;
	void setButtonIcon(QString iconPath)
	{
		m_iconPath = iconPath;
		this->setIconSize(QSize(14, 14));
		this->setIcon(QIcon(iconPath));
	}

	// ���ð�ť�Ƿ�disable;
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
	
	// ���÷�ҳWidget��Ϣ;
	void setTurnPageInfo(int currentPageIndex, int totalPage, int totalCount, int perPageCount, int unCheckedCount = 0);

    // ����δ�������Label�Ƿ�ɼ�;
    void setUnCheckedLabelVisibel(bool isVisible);

private:
	void initWidget();

	void paintEvent(QPaintEvent *event);

signals:
    // ֪ͨҳ���仯;
    void signalPageChanged(int);

    // ֪ͨˢ�µ�ǰҳ;
    void signalRefreshCurrentPage();

private slots:
    void onButtonClicked(int buttonId);

private:
    // ��ҳ��ť;
	TurnPageButton* m_pButtonHeadPage;
	TurnPageButton* m_pButtonLastPage;
	TurnPageButton* m_pButtonNextPage;
	TurnPageButton* m_pButtonTailPage;
	TurnPageButton* m_pButtonRefreshPage;
    // ҳ����Ϣ;
	QLabel* m_labelPageCount;
	QLabel* m_labelPageInfo;
    
    // ��ҳ����Ҫ����δ�������;
    QLabel* m_unCheckedCountLabel;

    // ��ǰҳ�����롢��ʾ��;
	QLineEdit* m_lineEditCurrentPage;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;
    int m_currentTotalPage;
};

#endif // !_TURN_PAGE_WIDGET_H
