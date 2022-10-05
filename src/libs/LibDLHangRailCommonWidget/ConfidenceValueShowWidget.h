#ifndef _CONFIDENCE_VALUE_SHOW_WIDGET
#define _CONFIDENCE_VALUE_SHOW_WIDGET

#include <QWidget>
#include <QTimer>

class ConfidenceValueShowWidget : public QWidget
{
	Q_OBJECT
public:
    ConfidenceValueShowWidget(QWidget* parent = NULL);
       
    // 设置当前置信度;
    void setCurrentValue(float value);

private slots:
	void TimeOutSlot();
private:
    void paintEvent(QPaintEvent *event);

private:
    int m_value;
    QTimer *m_pFlashTimer;
    bool m_isFlash;
};
#endif