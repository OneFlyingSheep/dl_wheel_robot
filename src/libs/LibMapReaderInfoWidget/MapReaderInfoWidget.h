#ifndef MAPREADERINFORWIDGET_ALEXWEI_H
#define MAPREADERINFORWIDGET_ALEXWEI_H


#include <QWidget>

struct MapHandleInfo
{
	int error_code_;
	QString result_desc_;
	QString _strTitle;

	MapHandleInfo(int error_code, const QString& result_desc, const QString &strTitle = "") {
		error_code_ = error_code;
		result_desc_ = result_desc;
		_strTitle = strTitle;
	}
};

class QLabel;
class QPushButton;
class QTextEdit;


class MapReaderInfoWidget : public QWidget
{
	Q_OBJECT

public:
	MapReaderInfoWidget(QWidget *parent = 0);
	~MapReaderInfoWidget();
	void set_info(const MapHandleInfo &info);
	bool getState();

private slots:
	void slot_on_close();

private:
	QLabel *info_label_;
	QPushButton *close_pushButton_;
	QTextEdit *info_textEdit_;
	bool exe_result_;

};


#endif//