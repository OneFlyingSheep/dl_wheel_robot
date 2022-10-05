#ifndef ADDTYPEANDEQUIPMENTDLG_H
#define ADDTYPEANDEQUIPMENTDLG_H

#include <QWidget>

namespace Ui {
class AddTypeAndEquipmentDlg;
}

class QListWidgetItem;

enum _ADD_TYPE 
{
	ADD_TYPE,						//�������
	ADD_TYPE_AND_EQUIPMENT,			//����豸����
	ADD_EQUIPMENT,					//����豸
	TYPE_NUM
};

class AddTypeAndEquipmentDlg : public QWidget
{
    Q_OBJECT

public:
    explicit AddTypeAndEquipmentDlg(QWidget *parent = 0);
    ~AddTypeAndEquipmentDlg();

	void ClearContent();
	void SetWgtType(int iType);					//���õ���������
	int GetWgtType();							//��ȡ����������
	void SetDatas(QList<QStringList> lstAllDatas, QList<QStringList> lstSelectDatas);		//����data
	QList<QStringList> GetDatas();

signals:
	void AddEquipmentTypeOkBtnSignal();
	void AddEquipmentTypeCancelBtnSignal();

private slots:
	void AddAllBtnSlot();						//�������data
	void AddOneBtnSlot();						//���һ��data
	void DelOneBtnSlot();						//ɾ��һ��data
	void DelAllBtnSlot();						//ɾ��һ��data
	void AllLstWgtDouClickedSlot(QListWidgetItem *pCurItem);
	void SelectLstWgtDouClickedSlot(QListWidgetItem *pCurItem);

	void SearchAllEditSlot(const QString &strText);
	void SearchSelectEditSlot(const QString &strText);
private:
    Ui::AddTypeAndEquipmentDlg *ui;
	int m_iWgtType;							//��������
	QList<QStringList> m_lstAllDatas;		//���е�����
};

#endif // ADDTYPEANDEQUIPMENTDLG_H
