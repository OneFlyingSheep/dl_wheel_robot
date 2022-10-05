#ifndef __DL_WHEEL_COMPARE_DETECTION_H__
#define __DL_WHEEL_COMPARE_DETECTION_H__

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QMouseEvent>
#include <QFileDialog>

#pragma execution_character_set("utf-8")

/********图片显示控件*********/

class PictureLabel : public QLabel
{
public:
    PictureLabel(QWidget* parent = NULL)
    {
        this->setAlignment(Qt::AlignCenter);
        this->setPixmap(QPixmap(":/Resources/Common/image/ImportImage.png"));
        this->setStyleSheet("background:lightGray;");
        this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    // 获取文件路径;
    QString getFilePath()
    {
        return m_filePath;
    }

private:
    // 鼠标点击选择显示图片;
    void mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            QString strFilePath = QFileDialog::getOpenFileName(this, "选择文件", "", "*.jpg");
            if (!strFilePath.isEmpty())
            {
                m_filePath = strFilePath;
                this->setPixmap(QPixmap(strFilePath).scaled(this->size()));
            }
        }
    }

private:
    QString m_filePath;
};

class BaseWidget;

/********异物检测页面*********/

class DLWheelCompareDetection : public QWidget
{
	Q_OBJECT

public:
    DLWheelCompareDetection(QWidget *parent = NULL);
	~DLWheelCompareDetection();

private:
    // 放置一些控制按钮区域;
    void initControlBackWidget();
    void initWidget();

private:
    BaseWidget* m_centerWidget;
    PictureLabel* m_leftImageLabel;
    PictureLabel* m_rightImageLabel;
    QLabel* m_detectionResultLabel;

    QWidget* m_controlBackWidget;
};

#endif ///