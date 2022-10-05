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
	// ��껬�������״ - ����
	setCursor(Qt::PointingHandCursor);

	// �����źŲ�
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

// ���ƿ���
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
        // ����״̬;
		if (m_bChecked) 
        { 
            // �ر�״̬:
			background = m_checkedColor;
			thumbColor = m_checkedColor;
            dOpacity = 1;// 0.600;
		}
		else 
        {
            //��״̬;
			background = m_background;
			thumbColor = m_thumbColor;
            dOpacity = 1;// 0.800;
		}
	}
	else
    {  
        // ������״̬;
		background = m_background;
		dOpacity = 0.260;
		thumbColor = m_disabledColor;
	}
	// ���ƴ���Բ;
	painter.setBrush(background);
	painter.setOpacity(dOpacity);
    m_nMargin = 0;
	path.addRoundedRect(QRectF(m_nMargin, m_nMargin, width() - 2 * m_nMargin, height() - 2 * m_nMargin), height() / 2, height() / 2);
	painter.drawPath(path.simplified());

	// ����С��Բ;
	painter.setBrush(Qt::white);
	painter.setOpacity(1.0);
	painter.drawEllipse(QRectF(m_nX, 1, height() - 2, height() - 2));
}

// ��갴���¼�
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

// ����ͷ��¼� - �л�����״̬������toggled()�ź�
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

// ��С�ı��¼�
void NSwitchControl::resizeEvent(QResizeEvent *event)
{
    m_nHeight = this->height();
	QWidget::resizeEvent(event);
}

// Ĭ�ϴ�С
QSize NSwitchControl::sizeHint() const
{
	return minimumSizeHint();
}

// ��С��С
QSize NSwitchControl::minimumSizeHint() const
{
	return QSize(2 * (m_nHeight + m_nMargin), m_nHeight + 2 * m_nMargin);
}

// �л�״̬ - ����
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

// ���ؿ���״̬ - �򿪣�true �رգ�false
bool NSwitchControl::isToggled() const
{
	return m_bChecked;
}

// ���ÿ���״̬
void NSwitchControl::setToggle(bool checked)
{
	m_bChecked = checked;
	m_timer.start(m_slideTime);
}

// ���ñ�����ɫ
void NSwitchControl::setBackgroundColor(QColor color)
{
	m_background = color;
}

// ����ѡ����ɫ
void NSwitchControl::setCheckedColor(QColor color)
{
	m_checkedColor = color;
}

// ���ò�������ɫ
void NSwitchControl::setDisbaledColor(QColor color)
{
	m_disabledColor = color;
}

void NSwitchControl::setSlideTime(int slideTime)
{
	m_slideTime = slideTime;
}