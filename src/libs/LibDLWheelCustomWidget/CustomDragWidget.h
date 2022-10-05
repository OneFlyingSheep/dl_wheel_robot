#pragma once

#include <QtWidgets/QWidget>
#include <QPainter>
#include <QVBoxLayout>
#include <QDebug>
#include <QMenu>
#include <QMouseEvent>
#include <QScrollArea>
#include "common/DLWheelRobotGlobalDef.hpp"

class DragWidget : public QWidget
{
    Q_OBJECT
public:
    DragWidget(QWidget* parent = NULL)
        : QWidget(parent)
        , m_isDraged(false)
        , m_isDragDown(false)
    {
        this->setFont(QFont("Microsoft YaHei", 15));
    }

    void setData(THRESHOLD_ELEMENT data)
    {
        m_itemData = data;
        
        if (m_itemData.val.isEmpty())
        {
            return;
        }
        else
        {
            
            switch (data.type)
            {
            case THRESHOLD_TYPE_INCLUDE:
            {
                m_strText = "包含于 {";
                for (int i = 0; i < data.val.count(); i++)
                {
                    m_strText.append(data.val.at(i));
                    m_strText.append(",");
                }
                m_strText.remove(m_strText.count() - 1, 1);
                m_strText.append("}");
            }
                break;
            case THRESHOLD_TYPE_EXCLUDE:
            {
                m_strText = "不包含于 {";
                for (int i = 0; i < data.val.count(); i++)
                {
                    m_strText.append(data.val.at(i));
                    m_strText.append(",");
                }
                m_strText.remove(m_strText.count() - 1, 1);
                m_strText.append("}");
            }
                break;
            case THRESHOLD_TYPE_IN_RANGE:
            {
                m_strText = "在区间";
                m_strText.append(m_itemData.val.first());
            }
            break;
            case THRESHOLD_TYPE_NOT_IN_RANGE:
            {
                m_strText = "不在区间";
                m_strText.append(m_itemData.val.first());
            }
            break;
            case THRESHOLD_TYPE_VARIABLE:
            {
                m_strText = "{变化量}";
            }
            break;
            case THRESHOLD_TYPE_CURRENT_VARIABLE:
            {
                m_strText = "{当前值}";
            }
            break;
            case THRESHOLD_TYPE_INIT_VAL:
            {
                m_strText = "{初值}";
            }
            break;
            case THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE:
            {
                m_strText = "{初值差绝对值}";
            }
            break;
            case THRESHOLD_TYPE_INITIAL_DIFFERENCE_VALUE:
            {
                m_strText = "{初值差}";
            }
            break;
            case THRESHOLD_TYPE_DIFFERENCE_TEMPERATURE_BETWEEN_PHASES:
            {
                m_strText = "{相间温差}";
            }
            break;
            case THRESHOLD_TYPE_CHANGE_RATE:
            {
                m_strText = "{变化率}";
            }
            break;
            default:
            {
                m_strText = m_itemData.val.first();
            }
                break;
            }
        }

        QFontMetrics fontMetrics = this->fontMetrics();
        int textLength = fontMetrics.width(m_strText) + 10;
        int fontHeight = fontMetrics.height();
        this->setFixedSize(QSize(textLength + 10, fontMetrics.height()));
    }

    THRESHOLD_ELEMENT getData()
    {
        return m_itemData;
    }

    QString getText()
    {
        return m_strText;
    }

    void setIsDraged(bool isDraged)
    {
        m_isDraged = isDraged;
        update();
    }

    void setIsDragDown(bool isDragDown, QPoint mousePoint = QPoint(0, 0))
    {
        m_isDragDown = isDragDown;
        m_mousePoint = mousePoint;
        update();
    }

private:
    void paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.save();
        painter.setPen(Qt::NoPen);
        if (m_isDraged)
        {
            painter.setBrush(Qt::green);
            painter.drawRoundedRect(this->rect(), 5, 5);
        }
        else if (m_isDragDown)
        {
            painter.setBrush(QColor(235, 235, 235));
            painter.drawRoundedRect(this->rect(), 5, 5);
            painter.setBrush(Qt::lightGray);
            
            if (m_mousePoint.x() > this->rect().center().x())
            {
                painter.drawRoundedRect(this->rect().adjusted(this->width() / 2, 0, 0, 0), 5, 5);
            }
            else
            {
                painter.drawRoundedRect(this->rect().adjusted(0, 0, -this->width() / 2, 0), 5, 5);
            }
        }
        painter.restore();
        painter.translate(QPoint(8, 0));
        painter.setFont(QFont("Microsoft YaHei", 15));
        painter.drawText(this->rect().adjusted(0, 0, -1, -1), m_strText);
    }

    void mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::RightButton)
        {
            QMenu* deleteMenu = new QMenu;
            QAction* deleteAction = deleteMenu->addAction("Delete");
            connect(deleteAction, &QAction::triggered, this, [=] {
                emit signalDeleteDragWidget();
            });
            deleteMenu->exec(QCursor::pos());
            deleteMenu->deleteLater();
        }

        return __super::mouseReleaseEvent(event);
    }

    void mouseDoubleClickEvent(QMouseEvent *event)
    {
        qDebug() << "mouseDoubleClickEvent : " << m_strText;

        return __super::mouseDoubleClickEvent(event);
    }

signals:
    void signalDeleteDragWidget();

private:
    // 当前显示的文字;
    QString m_strText;

    // 当前是否被拖拽或者是否被选中;
    bool m_isDraged;

    // 当前是否处于被拖拽的item下方;
    bool m_isDragDown;

    // 鼠标位于item的位置;
    QPoint m_mousePoint;

    THRESHOLD_ELEMENT m_itemData;
};

class CustomDragWidget : public QWidget
{
    Q_OBJECT

public:
    CustomDragWidget(QWidget *parent = NULL);

    void addDragWidget(THRESHOLD_ELEMENT data, bool isNeedUpdateLayout = true);

    QList<THRESHOLD_ELEMENT> getDataList();

    void setDataList(QList<THRESHOLD_ELEMENT> dataList);

private:

    void updateLayout();

    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent* event);

    void dragEnterEvent(QDragEnterEvent *event);

    void dragMoveEvent(QDragMoveEvent *event);

    void dropEvent(QDropEvent *event);

private:
    QPoint m_dragStartPosition;

    DragWidget* m_dragWidget;

    DragWidget* m_dragOnWidget;

    QList<DragWidget*> m_dragWidgetList;

    QVBoxLayout* m_vMainLayout;

    QList<QHBoxLayout*> m_hLayoutList;

    bool m_isMouseMoved;
};

class CustomDragArea : public QScrollArea
{
    Q_OBJECT
public:
    CustomDragArea(QWidget* parent = NULL)
    {
        this->setFixedHeight(200);

        m_textWidget = new CustomDragWidget;
        this->setWidget(m_textWidget);
        this->setContentsMargins(10, 10, 10, 10);
        this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

        this->setStyleSheet("background:white");
    }

    void addItemData(THRESHOLD_ELEMENT data)
    {
        m_textWidget->setFixedWidth(this->width());
        m_textWidget->addDragWidget(data);
    }

    QList<THRESHOLD_ELEMENT> getDataList()
    {
        return m_textWidget->getDataList();
    }

    void setDataList(QList<THRESHOLD_ELEMENT> dataList)
    {
        m_textWidget->setDataList(dataList);
    }

private:
    CustomDragWidget* m_textWidget;
};
