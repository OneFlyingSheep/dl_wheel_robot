#include "libRobotControlWidget.h"

#include <iostream>
#include <QApplication>
using namespace std;

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	LibRobotControlWidget* nWidget = new LibRobotControlWidget;
	nWidget->show();

	a.exec();
	return 0;
}