#ifndef __DL_WHEEL_COMPARE_DETECTION_H__
#define __DL_WHEEL_COMPARE_DETECTION_H__

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QMouseEvent>
#include <QFileDialog>

#pragma execution_character_set("utf-8")

/********ͼƬ��ʾ�ؼ�*********/

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

    // ��ȡ�ļ�·��;
    QString getFilePath()
    {
        return m_filePath;
    }

private:
    // �����ѡ����ʾͼƬ;
    void mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            QString strFilePath = QFileDialog::getOpenFileName(this, "ѡ���ļ�", "", "*.jpg");
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

/********������ҳ��*********/

class DLWheelCompareDetection : public QWidget
{
	Q_OBJECT

public:
    DLWheelCompareDetection(QWidget *parent = NULL);
	~DLWheelCompareDetection();

private:
    // ����һЩ���ư�ť����;
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