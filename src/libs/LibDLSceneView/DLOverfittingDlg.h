#ifndef __DL_OVERFITING_DLG_H__
#define __DL_OVERFITING_DLG_H__

#pragma execution_character_set("utf-8")

#include <QDialog>


namespace Ui {
	class DLOverfittingDlg;
}

class DLCustomScene;

class  DLOverfittingDlg : public  QDialog
{
    Q_OBJECT

public:
    DLOverfittingDlg(QWidget *parent = NULL);
    ~DLOverfittingDlg();

	void SetScene(DLCustomScene *pCustomScene);

	void SetSelectLineID(const QString &strLineName);			//��������ߵ�����	
	void SetSelectPointIDs(const QString &strPointName);		//������ϵ������

	void ClearContent();

private slots:
	void SelectLineBtnSlot();//ѡ����ϱ�׼�ߵĲۺ���
	void SelectPointsBtnSlot();//ѡ����ϵ�Ĳۺ���
	void StartOverfittingBtnSlot();			//��ʼ���
	void CancelBtnSlot();		//ȡ�����

private:
	Ui::DLOverfittingDlg *ui;
	DLCustomScene *m_pCustomScene;
};

#endif //__DL_OVERFITING_DLG_H__