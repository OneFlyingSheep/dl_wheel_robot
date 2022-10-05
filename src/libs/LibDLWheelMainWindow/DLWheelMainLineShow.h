#ifndef DL_WHEEL_MAIN_LINE_SHOW_H
#define DL_WHEEL_MAIN_LINE_SHOW_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>

#define TEXT_MAX_WIDTH 110      //文字最大显示长度;

class DLWheelMainLineGraphicsView;

/*******页面底部矩形颜色提示控件*********/

class ColorLabel : public QWidget
{
public:
	ColorLabel()
	{
		m_colorLabel = new QLabel;
		m_colorLabel->setFixedSize(QSize(16, 16));
		m_textLabel = new QLabel;

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addWidget(m_colorLabel);
		hLayout->addWidget(m_textLabel);
		hLayout->setSpacing(2);
		hLayout->setMargin(0);
	}

	// 设置颜色;
	void setColor(QColor color)
	{
		m_colorLabel->setStyleSheet(QString("border:1px solid black;background:rgb(%1, %2, %3);").arg(color.red()).arg(color.green()).arg(color.blue()));
	}

	// 设置文字;
	void setText(QString text)
	{
        m_textLabel->setToolTip(text);
        QFontMetrics fontWidth(m_colorLabel->font());
        int width = fontWidth.width(text);
        //当字符串宽度大于最大宽度时进行转换;
        if (width >= TEXT_MAX_WIDTH)
        {
            //右部显示省略号;
            text = fontWidth.elidedText(text, Qt::ElideRight, TEXT_MAX_WIDTH);  
        }
		m_textLabel->setText(text);
	}

private:
	QLabel* m_colorLabel;
	QLabel* m_textLabel;
};

/**********主接线展示页面**********/

class DLWheelMainLineShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelMainLineShow();

	// 窗口控件初始化;
	void initWidget();

private:
    // 初始化顶部输入框控件;
	void initTopWidget();
    // 初始化中心控件;
	void initCenterWidget();
    // 初始化底部颜色提示框控件;
	void initBottomWidget();
	
private:
    // 基本控件;
	QWidget* m_topBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonSearch;

	QWidget* m_centerWidget;

	QWidget* m_bottomBackWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 主接线控件;
    DLWheelMainLineGraphicsView* m_dLWheelMainLineGraphicsView;
};

#endif
