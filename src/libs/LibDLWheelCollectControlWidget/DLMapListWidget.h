#ifndef __DLMAP_LIST_WIDGET_H__
#define __DLMAP_LIST_WIDGET_H__

#include <QWidget>
#include <QListWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class  DLMapListWidget : public  QWidget
{
    Q_OBJECT

public:
    DLMapListWidget(QWidget *parent = NULL);
    ~DLMapListWidget();

public:
    void bind_query_2d_map_func();
	void bind_query_smap_func();

public:
    void load_2d_map_file(QStringList);
	void load_smap_file(QStringList);
	void set_type(int file_type);

signals:
    void  sig_select_file(QString download_path, QString  file_name, int file_type);


 private  slots:
    void  slot_on_choose();
    void  slot_on_click_select(QListWidgetItem*  item);
    void  slot_on_doubleClick_select(QListWidgetItem*  item);
	void  slot_on_choose_download_path();


private:
	int file_type_;
    QListWidget * file_listWidget_;
    QLabel  *  file_name_label_;
	QLineEdit  *  file_name_lineEdit_;
	QLabel  *  download_file_path_label_;
	QLineEdit  *  download_file_path_lineEdit_;
	QPushButton  *  select_button_;
    QPushButton  *  sure_button_;

};

#endif //__DLMAP_LIST_WIDGET_H__