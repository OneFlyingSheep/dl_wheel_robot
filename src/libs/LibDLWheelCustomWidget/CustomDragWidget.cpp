#include "CustomDragWidget.h"
#include <QMouseEvent>
#include <QApplication>
#include <QDrag>
#include <QMimeData>
#include <QTimer>

CustomDragWidget::CustomDragWidget(QWidget *parent)
    : QWidget(parent)
    , m_vMainLayout(NULL)
    , m_dragWidget(NULL)
    , m_isMouseMoved(false)
    , m_dragOnWidget(NULL)
{
    this->setAcceptDrops(true);
}

void CustomDragWidget::addDragWidget(THRESHOLD_ELEMENT data, bool isNeedUpdateLayout /* = true */)
{
    DragWidget* dragWidget = new DragWidget;
    connect(dragWidget, &DragWidget::signalDeleteDragWidget, this, [=] {
        // 右键删除某个Item;
        DragWidget* dragWidget = static_cast<DragWidget*>(sender());
        if (dragWidget != NULL)
        {
            if (dragWidget == m_dragWidget)
            {
                m_dragWidget = NULL;
            }
            m_dragWidgetList.removeOne(dragWidget);
            updateLayout();
            dragWidget->deleteLater();
        }
    });
    dragWidget->setData(data);

    if (m_dragWidget != NULL)
    {
        int index = m_dragWidgetList.indexOf(m_dragWidget);
        // 找到当前光标所在位置;
        if (index >= 0)
        {
            m_dragWidgetList.insert(index + 1, dragWidget);
            m_dragWidget->setIsDraged(false);
            m_dragWidget = dragWidget;
            m_dragWidget->setIsDraged(true);
        }
        // 未选中任何item，则在末尾插入新数据;
        else
        {
            m_dragWidgetList.append(dragWidget);
        }
    }
    else
    {
        m_dragWidgetList.append(dragWidget);
    }    

    if (isNeedUpdateLayout)
    {
        updateLayout();
    }
}

QList<THRESHOLD_ELEMENT> CustomDragWidget::getDataList()
{
    QList<THRESHOLD_ELEMENT> dataList;
    for (int i = 0; i < m_dragWidgetList.count(); i++)
    {
        dataList.append(m_dragWidgetList[i]->getData());
    }

    return dataList;
}

void CustomDragWidget::setDataList(QList<THRESHOLD_ELEMENT> dataList)
{
    // 将当前的拖拽操作对象置为空;
    m_dragWidget = NULL;
    m_dragOnWidget = NULL;
    // 删除原有的dragWidget;
    for (int i = 0; i < m_dragWidgetList.count(); i++)
    {
        delete m_dragWidgetList[i];
    }
    m_dragWidgetList.clear();

    // 根据所给数据重新添加dragWidget到布局中;
    for (int i = 0; i < dataList.count(); i++)
    {
        addDragWidget(dataList[i], false);
    }
   
    updateLayout();
}

void CustomDragWidget::updateLayout()
{
    if (m_vMainLayout != NULL)
    {
        for (int i = 0; i < m_hLayoutList.count(); i++)
        {
            m_vMainLayout->removeItem(m_hLayoutList[i]);
            delete m_hLayoutList[i];
        }
        m_hLayoutList.clear();
        delete m_vMainLayout;
    }
    m_vMainLayout = new QVBoxLayout(this);
    

    int dragWidgetWidth = 0;
    QHBoxLayout* hLayout = new QHBoxLayout;
    hLayout->setMargin(0);
    hLayout->setSpacing(0);

    m_hLayoutList.append(hLayout);
    m_vMainLayout->addLayout(hLayout);
    for (int i = 0; i < m_dragWidgetList.count(); i++)
    {
        int width = this->width();
        dragWidgetWidth += m_dragWidgetList[i]->width();
        if (dragWidgetWidth > this->width() - 20)
        {
            hLayout->addStretch();
            dragWidgetWidth = m_dragWidgetList[i]->width();
            hLayout = new QHBoxLayout;
            hLayout->setMargin(0);
            hLayout->setSpacing(0);

            m_hLayoutList.append(hLayout);
            m_vMainLayout->addLayout(hLayout);
            hLayout->addWidget(m_dragWidgetList[i]);
        }
        else
        {
            hLayout->addWidget(m_dragWidgetList[i]);
        }
    }
    hLayout->addStretch();
    m_vMainLayout->addStretch();
    m_vMainLayout->setSpacing(10);
    m_vMainLayout->setMargin(10);
    if (!m_dragWidgetList.isEmpty())
    {
        int height = m_hLayoutList.count() * m_dragWidgetList.first()->height() + (m_hLayoutList.count() - 1) * 10 + 20;
        this->setFixedHeight(height);
    }
}

void CustomDragWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        DragWidget* dragWidget = static_cast<DragWidget*>(childAt(event->pos()));
        if (!dragWidget)
            return;

        if (m_dragWidget != NULL)
        {
            m_dragWidget->setIsDraged(false);
        }

        m_dragWidget = dragWidget;
        m_dragWidget->setIsDraged(true);

        m_dragStartPosition = event->pos();

        m_isMouseMoved = false;
    }
}

void CustomDragWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!(event->buttons() & Qt::LeftButton) || m_dragWidget == NULL)
        return;
    if ((event->pos() - m_dragStartPosition).manhattanLength()< QApplication::startDragDistance())
        return;

    m_isMouseMoved = true;

    QDrag *drag = new QDrag(this);
    QMimeData *mimeData = new QMimeData;
    mimeData->setData("widget/pos", "");
    drag->setMimeData(mimeData);
    drag->setPixmap(m_dragWidget->grab(m_dragWidget->rect()));

    Qt::DropAction dropAction = drag->exec(Qt::MoveAction | Qt::MoveAction);
}

void CustomDragWidget::mouseReleaseEvent(QMouseEvent* event)
{

}

void CustomDragWidget::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasFormat("widget/pos")) {
        if (event->source() == this) {
            event->setDropAction(Qt::MoveAction);
            event->accept();
        }
        else {
            event->acceptProposedAction();
        }
    }
    else {
        event->ignore();
    }
}

void CustomDragWidget::dragMoveEvent(QDragMoveEvent *event)
{
    if (m_dragOnWidget != NULL)
    {
        m_dragOnWidget->setIsDragDown(false);
    }

    QPoint mousePoint = event->pos();
    DragWidget* dragWidget = static_cast<DragWidget*>(childAt(mousePoint));
    if (dragWidget == NULL)
    {
        return;
    }
    
    m_dragOnWidget = dragWidget;
    mousePoint = m_dragOnWidget->mapFromParent(mousePoint);
    m_dragOnWidget->setIsDragDown(true, mousePoint);

    qDebug() << dragWidget->getText();
}

void CustomDragWidget::dropEvent(QDropEvent *event)
{
    if (m_dragOnWidget != NULL)
    {
        m_dragOnWidget->setIsDragDown(false);
        m_dragOnWidget = NULL;
    }

    DragWidget* dragWidget = static_cast<DragWidget*>(childAt(event->pos()));
    if (dragWidget == NULL)
    {
        return;
    }
    else if (dragWidget == m_dragWidget)
    {
        m_dragWidget->setIsDraged(false);
        m_dragWidget = NULL;
        return;
    }
    
    QRect dragWidgetRect = dragWidget->rect();
    QPoint mousePoint = dragWidget->mapFromParent(event->pos());
    m_dragWidgetList.removeOne(m_dragWidget);
    int index = m_dragWidgetList.indexOf(dragWidget);
    if (mousePoint.x() > dragWidgetRect.center().x())
    {
        m_dragWidgetList.insert(index + 1, m_dragWidget);
    }
    else
    {
        m_dragWidgetList.insert(index, m_dragWidget);
    }
    
    updateLayout();
}
