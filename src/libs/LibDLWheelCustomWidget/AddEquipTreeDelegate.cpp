/*
    AddEquipTreeDelegate.cpp

    A delegate that allows the user to change integer values from the model
    using a spin box widget.
*/
#include <QPainter>
#include <QDebug>
#include <QMouseEvent>
#include "AddEquipTreeDelegate.h"
#include "DefData.h"

//! [0]
AddEquipTreeDelegate::AddEquipTreeDelegate(QObject *parent)
    : QStyledItemDelegate(parent)
{

}


void AddEquipTreeDelegate::SetView(QTreeView *pTreeView)
{
    m_pTreeView = pTreeView;
}
//! [0]

void AddEquipTreeDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{//绘制
    Q_UNUSED(option)
    QRect visualRect = m_pTreeView->visualRect(index);

	//qDebug() << "item type:" << m_pTreeView->model()->data(index, ITEM_TYPE).toInt() << ";state:" << m_pTreeView->model()->data(index, ITEM_STATE).toString();
	if (m_pTreeView->model()->data(index, ITEM_TYPE).toInt() == RootNode_Device && m_pTreeView->model()->data(index, ITEM_STATE).toString() == "0")
	{
		painter->save();
		painter->setPen(Qt::NoPen);
		painter->setBrush(Qt::magenta);
		painter->drawRect(visualRect);
		painter->restore();
	}

    painter->save();
    painter->translate(visualRect.left(), visualRect.top()+ 17);

    painter->setPen(Qt::black);
    QString strText = index.model()->itemData(index).value(ITEM_TEXT).toString();
    painter->drawText(QPoint(0, 0), strText);
    painter->restore();
	QStyledItemDelegate::paint(painter, option, index);

}

bool AddEquipTreeDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{//处理鼠标事件
    QMouseEvent *pMouseEvent = dynamic_cast<QMouseEvent *>(event);
   if(pMouseEvent->button() == Qt::LeftButton && event->type() == QEvent::MouseButtonDblClick)
	{
	   if (m_pTreeView->model()->data(index, ITEM_TYPE).toInt() == RootNode_Device)
	   {
		   emit EquipmentItemDoubleClickedSignal(index);
	   }
	}

    return QStyledItemDelegate::editorEvent(event, model, option, index);
}

QSize AddEquipTreeDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{//设置item的尺寸
    QSize size = QStyledItemDelegate::sizeHint(option, index);
	size.setHeight(size.height() + 4);
	return size;
}


