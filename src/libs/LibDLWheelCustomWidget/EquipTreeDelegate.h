#ifndef DELEGATE_H
#define DELEGATE_H

#include <QStyledItemDelegate>
#include <QTreeView>
#include "common/DLWheelRobotGlobalDef.hpp"

//! [0]
class EquipTreeDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    EquipTreeDelegate(QObject *parent = 0);
	void SetIsDrawCheckBox(bool bIsExistCheckBox);											//设置是否绘制checkbox
    void SetView(QTreeView *pTreeView);														//设置view
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;			//重写绘制事件
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index);		//重新实现事件
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;		//设置节点的尺寸

signals:
	void ItemStateChangeSignal(const QModelIndex &index);									//节点状态发生改变的槽函数
	void signalDeviceNodeClicked(QString strDeviceId);										// 设备节点点击;
private:
    QTreeView *m_pTreeView;																	//记录treeview
	bool m_bIsExistCheckBox;																//记录是否绘制checkbox
	QColor m_alarmColors[DeviceAlarmLevel::Alarm_NUM];
};
//! [0]

#endif
