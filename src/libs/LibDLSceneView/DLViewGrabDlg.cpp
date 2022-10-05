#include <QPixmap>
#include <QFileDialog>
#include "DLViewGrabDlg.h"
#include "ui_DLViewGrabDlg.h"

DLViewGrabDlg::DLViewGrabDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DLViewGrabDlg)
{
    ui->setupUi(this);
	connect(ui->pushBtnSave, SIGNAL(clicked()), this, SLOT(SaveBtnSlot()));
	connect(ui->pushBtnCancel, SIGNAL(clicked()), this, SLOT(CancelBtnSlot()));
	//resize(500, 500);
	this->setFixedSize(518, 349);
}

DLViewGrabDlg::~DLViewGrabDlg()
{
    delete ui;
}

void DLViewGrabDlg::SetPixmap(const QPixmap &pixmap)
{
	m_pixmap = pixmap;
	ui->lblPixmap->setPixmap(pixmap);
	ui->lblPixmap->setScaledContents(true);
}

void DLViewGrabDlg::SaveBtnSlot()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "D:/untitled.png", tr("Images (*.png *.xpm *.jpg)"));
	if (!fileName.isEmpty())
	{
		m_pixmap.save(fileName);
		accept();
		close();
	}
}

void DLViewGrabDlg::CancelBtnSlot()
{
	close();
}
