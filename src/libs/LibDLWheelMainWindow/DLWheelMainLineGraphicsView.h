#ifndef DL_WHEEL_MAIN_LINE_GRAPHICS_VIEW_H
#define DL_WHEEL_MAIN_LINE_GRAPHICS_VIEW_H

#include <QGraphicsView>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>

#define TEXT_MAX_WIDTH 110      //���������ʾ����;

class DLWheelMainLineGraphicsView : public QGraphicsView
{
	Q_OBJECT

public:
    DLWheelMainLineGraphicsView(QWidget* parent = NULL);

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // �������¼�;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseDoubleClickEvent(QMouseEvent *event);
	
private:
	QWidget* m_topBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonSearch;

	QWidget* m_centerWidget;

	QWidget* m_bottomBackWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ͼ��view�����ű���(view�Ŀ�/��ͼ�Ŀ�);
    double m_scaleFactor;
};

#endif
