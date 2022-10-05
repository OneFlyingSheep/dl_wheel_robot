#ifndef DLViewGrabDlg_H
#define DLViewGrabDlg_H

#include <QDialog>

namespace Ui {
class DLViewGrabDlg;
}
class QPixmap;

class DLViewGrabDlg : public QDialog
{
    Q_OBJECT

public:
    explicit DLViewGrabDlg(QWidget *parent = 0);
    ~DLViewGrabDlg();
	void SetPixmap(const QPixmap &pixmap);

private slots:
	void SaveBtnSlot();
	void CancelBtnSlot();
private:
    Ui::DLViewGrabDlg *ui;
	QPixmap m_pixmap;
};

#endif // DLViewGrabDlg_H
