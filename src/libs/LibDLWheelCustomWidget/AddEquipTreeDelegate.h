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
    void SetView(QTreeView *pTreeView);														//����view
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;			//��д�����¼�
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index);		//����ʵ���¼�
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;		//���ýڵ�ĳߴ�

signals:
	//void ItemStateChangeSignal(const QModelIndex &index);									//�ڵ�״̬�����ı�Ĳۺ���
	//void signalDeviceNodeClicked(QString strDeviceId);										// �豸�ڵ���;
	void EquipmentItemDoubleClickedSignal(const QModelIndex &index);
private:

private:
    QTreeView *m_pTreeView;																	//��¼treeview
};
//! [0]ADD_EQUIPMENT_TREE_DELEGATE_H

#endif
