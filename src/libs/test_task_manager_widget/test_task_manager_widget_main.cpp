#include <stdio.h>
#include <QApplication>
#include "WheelTaskManageWidget/WheelTaskManageWidget.h"

int main(int argc,char **argv)
{
	QApplication a(argc,argv);

	DLWheelTaskManage widget(Menu_CustomTask);
	widget.show();
	return a.exec();
}