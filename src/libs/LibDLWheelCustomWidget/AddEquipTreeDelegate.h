#ifndef ADD_EQUIPMENT_TREE_DELEGATE_H
#define ADD_EQUIPMENT_TREE_DELEGATE_H

#include <QStyledItemDelegate>
#include <QTreeView>
#include "common/DLWheelRobotGlobalDef.hpp"

//! [0]
class AddEquipTreeDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    AddEquipTreeDelegate(QObject *parent = 0);
    void SetView(QTreeView *pTreeView);														//设置view
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;			//重写绘制事件
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index);		//重新实现事件
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;		//设置节点的尺寸

signals:
	//void ItemStateChangeSignal(const QModelIndex &index);									//节点状态发生改变的槽函数
	//void signalDeviceNodeClicked(QString strDeviceId);										// 设备节点点击;
	void EquipmentItemDoubleClickedSignal(const QModelIndex &index);
private:

private:
    QTreeView *m_pTreeView;																	//记录treeview
};
//! [0]ADD_EQUIPMENT_TREE_DELEGATE_H

#endif
