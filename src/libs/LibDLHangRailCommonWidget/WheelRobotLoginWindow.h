#pragma once

#include <QWidget>
#include <QLabel>
#include <QToolButton>
#include <QPushButton>
#include <QLineEdit>
#include <QScrollArea>
#include <QPropertyAnimation>
#include <QBoxLayout>
#include <QPainter>

class InputWidget;
class IconButton : public QWidget
{
	Q_OBJECT

public:
	IconButton(QWidget* parent = NULL)
		: QWidget(parent)
	{
		m_labelText = new QLabel;
		m_labelText->setStyleSheet("font-size:16px;");
		m_labelIcon = new QLabel;
		m_labelIcon->setStyleSheet("margin-top:2px;");
		m_labelIcon->setFixedSize(QSize(16, 16));

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addWidget(m_labelText);
		hLayout->addWidget(m_labelIcon);
		hLayout->setSpacing(0);
		hLayout->setContentsMargins(10, 0, 10, 0);
	}

	void setIcon(QString iconPath)
	{
		m_labelIcon->setPixmap(QPixmap(iconPath).scaled(m_labelIcon->size()));
	}

	void setText(QString text)
	{
		m_labelText->setText(text);
		m_labelText->setScaledContents(true);
	}

signals:
	void clicked();

private:
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.fillRect(this->rect(), QColor(230, 160, 70));
	}

	void mousePressEvent(QMouseEvent *e)
	{
		QLayout* layout = this->layout();
		layout->setContentsMargins(12, 2, 8, 0);
	}

	void mouseReleaseEvent(QMouseEvent *e)
	{
		QLayout* layout = this->layout();
		layout->setContentsMargins(10, 0, 10, 0);
		emit clicked();
	}

private:
	QLabel* m_labelText;
	QLabel* m_labelIcon;
};

class WheelRobotLoginWindow : public QWidget
{
	Q_OBJECT

public:
	WheelRobotLoginWindow(QWidget *parent = NULL);
	~WheelRobotLoginWindow();

	// ��¼ʧ��;
	void loginFailed();

    // ��ȡ��ǰ��¼ʱѡ�����ĸ���Ļ����ʾ;
    int getCurrentScreenChoosedIndex();

private:
	// ��ʼ�����ڲ���;
	void initWidget();
	// ��ʼ����¼����;
	void initLoginWidget();
	// ��ʼ���������ô���;
	void initNetWorkSetWidget();
    // ��ʼ���������ĸ���Ļ����ʾ;
    void initScreenShowChoose();

	// ���Ʊ���;
	void paintEvent(QPaintEvent *event);

signals:
	// ��¼/�˳�;
	void signalLogin(QString userName, QString password);
	void signalQuit();

private slots:	

private:
	QWidget * m_loginBackWidget;
	QLineEdit* m_lineEditUserName;
	QLineEdit* m_lineEditPassword;

	IconButton* m_pButtonLogin;
	IconButton* m_pButtonReset;

    // ��ǰ������Ļѡ��ComboBox;
    InputWidget* m_screenChooseComboBox;
};
