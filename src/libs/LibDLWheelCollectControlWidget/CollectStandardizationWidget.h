#pragma once

#include <QWidget>

namespace Ui {
	class CollectStandardizationWidget;
}


class CollectStandardizationWidget : public QWidget
{
	Q_OBJECT

public:
	explicit CollectStandardizationWidget(QWidget *parent = 0);
	~CollectStandardizationWidget();

	//������ֵ
	void CalculateZoomValue();
	//�л�zoomֵ
	void CutZoomTxt();
	//�Ƿ�ʹ��Ĭ��ֵ
	void DefaultTxt();
signals:
	void startCollectPhotoSignals();

private slots:

private:
	Ui::CollectStandardizationWidget *ui;
};
