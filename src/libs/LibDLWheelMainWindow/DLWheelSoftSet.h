#ifndef DL_WHEEL_SOFT_SET_H
#define DL_WHEEL_SOFT_SET_H

#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QLineEdit>

#pragma execution_character_set("utf-8")

/*********网页文档********/

class Document : public QObject
{
    Q_OBJECT
        Q_PROPERTY(QString text MEMBER m_text NOTIFY textChanged FINAL)
public:
    explicit Document(QObject *parent = nullptr) : QObject(parent) {}

    void setText(const QString &text)
    {
        if (text == m_text)
            return;
        m_text = text;
        emit textChanged(m_text);
    }

signals:
    void textChanged(const QString &text);

private:
    QString m_text;
};

/*******软件设置页面********/

class DLWheelSoftSet : public QWidget
{
	Q_OBJECT

public:
	DLWheelSoftSet(QWidget* parent = 0);

	// 窗口控件初始化;
	void initWidget();

private:
    // 窗口各个控件初始化;
	void initTopWidget();
	void initBottomBackWidget();
    void initCenterWidget();
    // 背景绘制;
	void paintEvent(QPaintEvent *event);

private:
	QWidget* m_topBackWidget;
	QToolButton* m_pButtonPrint;
	QToolButton* m_pButtonBackUp;
	QToolButton* m_pButtonGoForward;
    QToolButton* m_pButtonReload;

	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonSearch;

	QWidget* m_centerBackWidget;

	QWidget* m_bottomBackWidget;
	QPushButton* m_pButtonUserManual;
	QPushButton* m_pButtonCommonProblem;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    Document m_content;
};

#endif
