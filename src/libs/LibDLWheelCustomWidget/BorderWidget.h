#ifndef BORDER_WIDGET_H
#define BORDER_WIDGET_H

#include <QWidget>
#include <QLabel>

class BorderWidget : public QWidget
{
public:
	BorderWidget(QWidget* parent = NULL);

	void setTitleText(const QString& text);

	QWidget* getCenterWidget();
private:
	void initWidget();

	void paintEvent(QPaintEvent *event);

private:
	QLabel* m_titleLabel;
	QWidget* m_centerWidget;
};

#endif // !BORDER_WIDGET_H
