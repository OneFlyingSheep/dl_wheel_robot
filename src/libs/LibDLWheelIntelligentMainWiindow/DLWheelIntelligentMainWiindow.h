#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QPushButton>
#include <LibDLIntelligentSocket/LibDLIntelligentSocket.h>
#include <QGroupBox>
#include <QTimer>

class DLWheelIntelligentMainWiindow : public QWidget
{
    Q_OBJECT
public:
    DLWheelIntelligentMainWiindow(QWidget* parent = NULL);

private:
    void initSystemMsgButton();
    void initMonitorMsgButton();
    void initWidget();

    bool onCheckConnect();
private:
    QGroupBox* m_systemButtonBackWidget;
    QWidget* m_monitorButtonBackWidget;
    boost::shared_ptr<LibDLIntelligentSocket> m_socketClent;
    QTimer *testTimer;
};

#endif // MAINWINDOW_H
