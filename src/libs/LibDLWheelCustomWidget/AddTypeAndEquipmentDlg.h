#ifndef ADDTYPEANDEQUIPMENTDLG_H
#define ADDTYPEANDEQUIPMENTDLG_H

#include <QWidget>

namespace Ui {
class AddTypeAndEquipmentDlg;
}

class QListWidgetItem;

enum _ADD_TYPE 
{
	ADD_TYPE,						//添加类型
	ADD_TYPE_AND_EQUIPMENT,			//添加设备类型
	ADD_EQUIPMENT,					//添加设备
	TYPE_NUM
};

class AddTypeAndEquipmentDlg : public QWidget
{
    Q_OBJECT

public:
    explicit AddTypeAndEquipmentDlg(QWidget *parent = 0);
    ~AddTypeAndEquipmentDlg();

	void ClearContent();
	void SetWgtType(int iType);					//设置弹窗的类型
	int GetWgtType();							//获取弹窗的类型
	void SetDatas(QList<QStringList> lstAllDatas, QList<QStringList> lstSelectDatas);		//设置data
	QList<QStringList> GetDatas();

signals:
	void AddEquipmentTypeOkBtnSignal();
	void AddEquipmentTypeCancelBtnSignal();

private slots:
	void AddAllBtnSlot();						//添加所有data
	void AddOneBtnSlot();						//添加一个data
	void DelOneBtnSlot();						//删除一个data
	void DelAllBtnSlot();						//删除一个data
	void AllLstWgtDouClickedSlot(QListWidgetItem *pCurItem);
	void SelectLstWgtDouClickedSlot(QListWidgetItem *pCurItem);

	void SearchAllEditSlot(const QString &strText);
	void SearchSelectEditSlot(const QString &strText);
private:
    Ui::AddTypeAndEquipmentDlg *ui;
	int m_iWgtType;							//弹窗类型
	QList<QStringList> m_lstAllDatas;		//所有的数据
};

#endif // ADDTYPEANDEQUIPMENTDLG_H
