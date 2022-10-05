#include <QPainter>
#include <QMouseEvent>
#include "nswitchcontrol.h"

NSwitchControl::NSwitchControl(QWidget *parent)
	: QWidget(parent),
	m_nHeight(24),
	m_bChecked(false),
	m_radius(8.0),
	m_nMargin(3),
	m_checkedColor(0, 150, 136),
	m_thumbColor(Qt::gray),
	m_disabledColor(190, 190, 190),
	m_background(Qt::lightGray),
	m_slideTime(10),
    m_nX(1)
{
	// 鼠标滑过光标形状 - 手型
	setCursor(Qt::PointingHandCursor);

	// 连接信号槽
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

// 绘制开关
void NSwitchControl::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);

	QPainter painter(this);
	painter.setPen(Qt::NoPen);
	painter.setRenderHint(QPainter::Antialiasing);

	QPainterPath path;
	QColor background;
	QColor thumbColor;
	qreal dOpacity;
	if (isEnabled()) 
    {  
        // 可用状态;
		if (m_bChecked) 
        { 
            // 关闭状态:
			background = m_checkedColor;
			thumbColor = m_checkedColor;
            dOpacity = 1;// 0.600;
		}
		else 
        {
            //打开状态;
			background = m_background;
			thumbColor = m_thumbColor;
            dOpacity = 1;// 0.800;
		}
	}
	else
    {  
        // 不可用状态;
		background = m_background;
		dOpacity = 0.260;
		thumbColor = m_disabledColor;
	}
	// 绘制大椭圆;
	painter.setBrush(background);
	painter.setOpacity(dOpacity);
    m_nMargin = 0;
	path.addRoundedRect(QRectF(m_nMargin, m_nMargin, width() - 2 * m_nMargin, height() - 2 * m_nMargin), height() / 2, height() / 2);
	painter.drawPath(path.simplified());

	// 绘制小椭圆;
	painter.setBrush(Qt::white);
	painter.setOpacity(1.0);
	painter.drawEllipse(QRectF(m_nX, 1, height() - 2, height() - 2));
}

// 鼠标按下事件
void NSwitchControl::mousePressEvent(QMouseEvent *event)
{
	if (isEnabled()) {
		if (event->buttons() & Qt::LeftButton) {
			event->accept();
		}
		else {
			event->ignore();
		}
	}
}

// 鼠标释放事件 - 切换开关状态、发射toggled()信号
void NSwitchControl::mouseReleaseEvent(QMouseEvent *event)
{
	if (isEnabled()) {
		if ((event->type() == QMouseEvent::MouseButtonRelease) && (event->button() == Qt::LeftButton)) {
			event->accept();
			m_bChecked = !m_bChecked;
			emit toggled(m_bChecked);
			m_timer.start(m_slideTime);
		}
		else {
			event->ignore();
		}
	}
}

// 大小改变事件
void NSwitchControl::resizeEvent(QResizeEvent *event)
{
    m_nHeight = this->height();
	QWidget::resizeEvent(event);
}

// 默认大小
QSize NSwitchControl::sizeHint() const
{
	return minimumSizeHint();
}

// 最小大小
QSize NSwitchControl::minimumSizeHint() const
{
	return QSize(2 * (m_nHeight + m_nMargin), m_nHeight + 2 * m_nMargin);
}

// 切换状态 - 滑动
void NSwitchControl::onTimeout()
{
    int widthxxx = this->width();
	if (m_bChecked) {
		m_nX += 1;
		if (m_nX >= width() - m_nHeight)
			m_timer.stop();
	}
	else {
		m_nX -= 1;
		if (m_nX <= 1)
			m_timer.stop();
	}
	update();
}

// 返回开关状态 - 打开：true 关闭：false
bool NSwitchControl::isToggled() const
{
	return m_bChecked;
}

// 设置开关状态
void NSwitchControl::setToggle(bool checked)
{
	m_bChecked = checked;
	m_timer.start(m_slideTime);
}

// 设置背景颜色
void NSwitchControl::setBackgroundColor(QColor color)
{
	m_background = color;
}

// 设置选中颜色
void NSwitchControl::setCheckedColor(QColor color)
{
	m_checkedColor = color;
}

// 设置不可用颜色
void NSwitchControl::setDisbaledColor(QColor color)
{
	m_disabledColor = color;
}

void NSwitchControl::setSlideTime(int slideTime)
{
	m_slideTime = slideTime;
}