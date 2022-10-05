/*
    EquipTreeDelegate.cpp

    A delegate that allows the user to change integer values from the model
    using a spin box widget.
*/
#include <QPainter>
#include <QDebug>
#include <QMouseEvent>
#include "EquipTreeDelegate.h"

#include "DefData.h"


//! [0]
EquipTreeDelegate::EquipTreeDelegate(QObject *parent)
    : QStyledItemDelegate(parent)
{
	//获取不同等级的颜色
	m_alarmColors[Alarm_Normal] = QColor(0, 128, 0);
	m_alarmColors[Alarm_Waring] = QColor(0, 0, 255);
	m_alarmColors[Alarm_Common] = QColor(255, 255, 0);
	m_alarmColors[Alarm_Serious] = QColor(255, 128, 10);
	m_alarmColors[Alarm_Dangerous] = QColor(255, 0, 0);
	m_alarmColors[Alarm_NoIdentifyAbnormal] = QColor(Qt::gray);
}

void EquipTreeDelegate::SetIsDrawCheckBox(bool bIsExistCheckBox)
{
	m_bIsExistCheckBox = bIsExistCheckBox;
}

void EquipTreeDelegate::SetView(QTreeView *pTreeView)
{
    m_pTreeView = pTreeView;
}
//! [0]

void EquipTreeDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{//绘制
    Q_UNUSED(option)
    QRect visualRect = m_pTreeView->visualRect(index);
    painter->save();
    painter->translate(visualRect.left(), visualRect.top()+1);
    painter->setPen(Qt::gray);
    if (m_bIsExistCheckBox)
    {
        painter->setPen(Qt::gray);
        painter->drawRect(QRect(3, 6, 13, 13));
        if (index.model()->data(index, ITEM_STATE).toInt() == Checked_Status)
        {
            painter->setPen(Qt::NoPen);
            painter->setBrush(QColor(15, 150, 250));
			painter->drawPixmap(QPointF(3, 6.5), QPixmap(":/Resources/Common/image/checked.png").scaled(QSize(13, 13)));
        }

		else if (index.model()->data(index, ITEM_STATE).toInt() == PartiallyChecked_Status)
		{
			painter->setPen(Qt::NoPen);
			painter->setBrush(QColor(15, 150, 250));
			painter->drawPixmap(QPointF(3, 6.5), QPixmap(":/Resources/Common/image/partiallychecked.png").scaled(QSize(13, 13)));
		}

        painter->translate(18, 0);
    }

    painter->setPen(Qt::gray);
    painter->setBrush(QBrush(m_alarmColors[(index.model()->data(index, ITEM_LEVEL).toInt())% Alarm_NUM]));
    painter->drawRect(QRect(3, 5, 12, 15));

    painter->translate(18, 17);
    painter->setPen(Qt::black);
    QString strText = index.model()->itemData(index).value(ITEM_TEXT).toString();
    painter->drawText(QPoint(0, 0), strText);
    painter->restore();
	QStyledItemDelegate::paint(painter, option, index);
}

bool EquipTreeDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{//处理鼠标事件
    QMouseEvent *pMouseEvent = dynamic_cast<QMouseEvent *>(event);
    if (pMouseEvent->button() == Qt::LeftButton && event->type() == QEvent::MouseButtonRelease)
    {
		QPoint mousePressPoint = pMouseEvent->pos();
		QRect visualRect = m_pTreeView->visualRect(index);
		int iItemType = m_pTreeView->model()->data(index, ITEM_TYPE).toInt();
		QString strUUid = m_pTreeView->model()->data(index, ITEM_UUID).toString();
        if (m_bIsExistCheckBox)
        {
            QRect checkBox(visualRect.x() + 3, visualRect.y() + 5, 15, 15);
            if (checkBox.contains(mousePressPoint))
            {
                qDebug() << "pressed name:" << index.model()->data(index, ITEM_TEXT).toString();
                int iCheckState = index.model()->data(index, ITEM_STATE).toInt();
				if (UnChecked_Status == iCheckState || PartiallyChecked_Status == iCheckState)
				{//非选中状态或半选状态
					iCheckState = Checked_Status;
				}
				else
				{//选中状态
					iCheckState = UnChecked_Status;
				}
				m_pTreeView->model()->setData(index, iCheckState, ITEM_STATE);
				emit ItemStateChangeSignal(index);
            }
			else
			{
				if (RootNode_Device == iItemType)
				{//点击设备
					emit signalDeviceNodeClicked(strUUid);
				}
			}
        }
		else
		{
			if (RootNode_Device == iItemType)
			{//点击设备
				emit signalDeviceNodeClicked(strUUid);
			}
		}
    }
	else if(pMouseEvent->button() == Qt::LeftButton && event->type() == QEvent::MouseButtonDblClick)
	{
		QPoint mousePressPoint = pMouseEvent->pos();
		QRect visualRect = m_pTreeView->visualRect(index);
		QRect checkBox(visualRect.x() + 3, visualRect.y() + 5, 15, 15);
		if (checkBox.contains(mousePressPoint))
		{
			return true;
		}
	}

    return QStyledItemDelegate::editorEvent(event, model, option, index);
}

QSize EquipTreeDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{//设置item的尺寸
    QSize size = QStyledItemDelegate::sizeHint(option, index);
	size.setHeight(size.height() + 4);
	return size;
}


