#ifndef DL_WHEEL_MAIN_LINE_GRAPHICS_VIEW_H
#define DL_WHEEL_MAIN_LINE_GRAPHICS_VIEW_H

#include <QGraphicsView>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>

#define TEXT_MAX_WIDTH 110      //文字最大显示长度;

class DLWheelMainLineGraphicsView : public QGraphicsView
{
	Q_OBJECT

public:
    DLWheelMainLineGraphicsView(QWidget* parent = NULL);

	// 窗口控件初始化;
	void initWidget();

private:
    // 鼠标操作事件;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseDoubleClickEvent(QMouseEvent *event);
	
private:
	QWidget* m_topBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonSearch;

	QWidget* m_centerWidget;

	QWidget* m_bottomBackWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 地图与view的缩放比例(view的宽/地图的宽);
    double m_scaleFactor;
};

#endif
