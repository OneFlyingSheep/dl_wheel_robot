#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "DLMapListWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDataTransfer/DataTransfer.h"

DLMapListWidget::DLMapListWidget(QWidget *parent)
	:QWidget(parent)
{

	setWindowFlags(windowFlags() | Qt::Window);
	file_type_ = -1;
    file_listWidget_ = new  QListWidget;
    file_name_label_ = new  QLabel("文件：");
    file_name_lineEdit_ = new  QLineEdit;
    file_name_lineEdit_->setFocusPolicy(Qt::NoFocus);

	download_file_path_label_ = new QLabel("下载路径");
	download_file_path_lineEdit_ = new QLineEdit;
	download_file_path_lineEdit_->setFocusPolicy(Qt::NoFocus);
	select_button_ = new QPushButton("选择");

	sure_button_ = new  QPushButton("确定");

    QHBoxLayout  *select_layout = new  QHBoxLayout;
    select_layout->addWidget(file_name_label_);
    select_layout->addWidget(file_name_lineEdit_);
    select_layout->addWidget(sure_button_);

	QHBoxLayout  *set_path_layout = new  QHBoxLayout;
	set_path_layout->addWidget(download_file_path_label_);
	set_path_layout->addWidget(download_file_path_lineEdit_);
	set_path_layout->addWidget(select_button_);

    this->setWindowTitle(tr("文件选择"));
    QVBoxLayout  *main_layout = new  QVBoxLayout;
    main_layout->addWidget(file_listWidget_);
    main_layout->addLayout(set_path_layout);
	main_layout->addLayout(select_layout);
    this->setLayout(main_layout);

	bind_query_2d_map_func();
	bind_query_smap_func();

    connect(file_listWidget_, SIGNAL(itemClicked(QListWidgetItem  *)), this, SLOT(slot_on_click_select(QListWidgetItem*)));
    connect(file_listWidget_, SIGNAL(itemDoubleClicked(QListWidgetItem  *)), this, SLOT(slot_on_doubleClick_select(QListWidgetItem*)));
    connect(select_button_, SIGNAL(clicked()), this, SLOT(slot_on_choose_download_path()));
	connect(sure_button_, SIGNAL(clicked()), this, SLOT(slot_on_choose()));
}


DLMapListWidget::~DLMapListWidget()
{

}


void DLMapListWidget::set_type(int file_type)
{
	download_file_path_lineEdit_->clear();
	file_name_lineEdit_->clear();
	file_listWidget_->clear();
	file_type_ = file_type;
	if (file_type == DataTransfer::FILE_2D) {
		this->setWindowTitle("请选择2D文件");
	}
	else if(file_type == DataTransfer::FILE_SMAP) {
		this->setWindowTitle("请选择smap文件");
	}
}


void DLMapListWidget::bind_query_2d_map_func()
{

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotQueryMapList.connect(boost::bind(&DLMapListWidget::load_2d_map_file, this, _1));
    
}


void DLMapListWidget::bind_query_smap_func()
{

	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotQuerySMapList.connect(boost::bind(&DLMapListWidget::load_smap_file, this, _1));

}


void DLMapListWidget::load_2d_map_file(QStringList list)
{
	for (int i = 0; i < list.size(); ++i)
	{
		file_listWidget_->addItem(list[i]);
	}
	update();
}


void DLMapListWidget::load_smap_file(QStringList list)
{
	for (int i = 0; i < list.size(); ++i)
	{
		file_listWidget_->addItem(list[i]);
	}
	update();
}


void  DLMapListWidget::slot_on_choose_download_path()
{
	QString dir;
	if (file_type_ == 0) {
		dir = QFileDialog::getExistingDirectory(this, tr("设置下载路径"), "D:/RCF_Server/2d", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	}
	else {
		dir = QFileDialog::getExistingDirectory(this, tr("设置下载路径"), "D:/RCF_Server/smap", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	}

	if (dir.isNull() || dir.isEmpty()) {
		return;
	}
	download_file_path_lineEdit_->setText(dir);
}




void  DLMapListWidget::slot_on_choose()
{
	QString download_path = download_file_path_lineEdit_->text();
	QString  file_name = file_name_lineEdit_->text();
	if (download_path.isNull() || download_path.isEmpty() || file_name.isNull() || file_name.isEmpty()) {
		this->close();
		return;
	}
	emit  sig_select_file(download_path, file_name, file_type_);
    this->close();
}


void  DLMapListWidget::slot_on_click_select(QListWidgetItem*  item)
{
    QString  file_name = item->text();
    file_name_lineEdit_->setText(file_name);
}


void  DLMapListWidget::slot_on_doubleClick_select(QListWidgetItem*  item)
{
	QString download_path = download_file_path_lineEdit_->text();
    QString  file_name = item->text();
    file_name_lineEdit_->setText(file_name);
	if (download_path.isNull() || download_path.isEmpty() || file_name.isNull() || file_name.isEmpty()) {
		this->close();
		return;
	}
	emit  sig_select_file(download_path, file_name, file_type_);
    this->close();
}