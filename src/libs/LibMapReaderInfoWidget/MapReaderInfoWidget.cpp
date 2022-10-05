#include <QtWidgets>
#include "MapReaderInfoWidget.h"


MapReaderInfoWidget::MapReaderInfoWidget(QWidget *parent)
	: QWidget(parent)
{
	exe_result_ = true;
	info_label_ = new QLabel("");
	info_label_->setAlignment(Qt::AlignCenter);
	QFont ft;
	ft.setPointSize(12);
	info_label_->setFont(ft);

	QHBoxLayout *layout = new QHBoxLayout;
	close_pushButton_ = new QPushButton(QStringLiteral("关闭"));
	close_pushButton_->setFixedWidth(60);
	layout->addStretch();
	layout->addWidget(close_pushButton_);

	info_textEdit_ = new QTextEdit;

	QVBoxLayout *main_layout = new QVBoxLayout;
	main_layout->addStretch();
	main_layout->addWidget(info_label_);
	main_layout->addWidget(info_textEdit_);
	main_layout->addLayout(layout);
	main_layout->addStretch();
	this->setFixedSize(400, 300);
	this->setLayout(main_layout);
	this->setWindowFlags(Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
	this->setStyleSheet("background-color:rgb(224, 255, 219);");

	connect(close_pushButton_, SIGNAL(clicked()), this, SLOT(slot_on_close()));
	
}


MapReaderInfoWidget::~MapReaderInfoWidget()
{

}

void MapReaderInfoWidget::slot_on_close()
{
	this->hide();
}


bool MapReaderInfoWidget::getState()
{
	return exe_result_;
}

void MapReaderInfoWidget::set_info(const MapHandleInfo &info)
{
	if (info.error_code_ == 0) {
		close_pushButton_->hide();
		info_textEdit_->hide();
		exe_result_ = true;
		info_label_->setText(info.result_desc_);
	}
	else {
		//info_label_->setText(QStringLiteral("地图保存信息如下"));
		info_label_->setText(info._strTitle);
		close_pushButton_->show();
		info_textEdit_->show();
		exe_result_ = false;
		info_textEdit_->setText(info.result_desc_);
	}

}






