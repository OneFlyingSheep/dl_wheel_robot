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
	void SetIsDrawCheckBox(bool bIsExistCheckBox);											//�����Ƿ����checkbox
    void SetView(QTreeView *pTreeView);														//����view
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;			//��д�����¼�
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index);		//����ʵ���¼�
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;		//���ýڵ�ĳߴ�

signals:
	void ItemStateChangeSignal(const QModelIndex &index);									//�ڵ�״̬�����ı�Ĳۺ���
	void signalDeviceNodeClicked(QString strDeviceId);										// �豸�ڵ���;
private:
    QTreeView *m_pTreeView;																	//��¼treeview
	bool m_bIsExistCheckBox;																//��¼�Ƿ����checkbox
	QColor m_alarmColors[DeviceAlarmLevel::Alarm_NUM];
};
//! [0]

#endif
