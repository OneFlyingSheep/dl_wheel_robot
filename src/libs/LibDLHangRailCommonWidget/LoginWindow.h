#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QScrollArea>
#include <QPropertyAnimation>

class InputWidget;
class LoginWindow : public QWidget
{
	Q_OBJECT

public:
	LoginWindow(QWidget *parent = NULL);
	~LoginWindow();

	// ���ñ�������;
	void setTitleText(QString titleText);

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
	// ��ȡ�������������ļ�;
	void readNetWorkSetInfo();
    // ��ʼ���������ĸ���Ļ����ʾ;
    void initScreenShowChoose();

	// ������ʽ��;
	void loadStyleSheet(const QString &sheetName);
	// ���Ʊ���;
	void paintEvent(QPaintEvent *event);

signals:
	// ��¼/�˳�;
	void signalLogin(QByteArray userName, QByteArray password, QString strIp, QString strPort);
	void signalQuit();

private slots:
	// ��������;
	void onNetWorkSet();

private:
	QPushButton * m_pButtonNetWorkSet;

	// ��¼�ؼ�;
	QPushButton* m_labelIcon;
	QLabel* m_labelTitleText;

	QLabel* m_labelUserName;
	QLabel* m_labelPassword;

	QLineEdit* m_lineEditUserName;
	QLineEdit* m_lineEditPassword;

	QPushButton* m_pButtonLogin;
	QPushButton* m_pButtonQuit;

	// �������ÿؼ�;
	QPushButton* m_labelNetworkIcon;
	QLabel* m_labelNetworkTitleText;

	QLabel* m_labelIP;
	QLabel* m_labelPort;

	QLineEdit* m_lineEditIP;
	QLineEdit* m_lineEditPort;

	QPushButton* m_pButtonOK;
	QPushButton* m_pButtonBack;

	QScrollArea* m_leftWidget;

	QWidget* m_loginWidget;
	QWidget* m_networkSetWidget;

	QPropertyAnimation* m_animationForScroll;

    // ��ǰ������Ļѡ��ComboBox;
    InputWidget* m_screenChooseComboBox;
};
