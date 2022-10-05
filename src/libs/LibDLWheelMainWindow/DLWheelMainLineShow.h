#ifndef DL_WHEEL_MAIN_LINE_SHOW_H
#define DL_WHEEL_MAIN_LINE_SHOW_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>

#define TEXT_MAX_WIDTH 110      //���������ʾ����;

class DLWheelMainLineGraphicsView;

/*******ҳ��ײ�������ɫ��ʾ�ؼ�*********/

class ColorLabel : public QWidget
{
public:
	ColorLabel()
	{
		m_colorLabel = new QLabel;
		m_colorLabel->setFixedSize(QSize(16, 16));
		m_textLabel = new QLabel;

		QHBoxLayout* hLayout = new QHBoxLayout(this);
		hLayout->addWidget(m_colorLabel);
		hLayout->addWidget(m_textLabel);
		hLayout->setSpacing(2);
		hLayout->setMargin(0);
	}

	// ������ɫ;
	void setColor(QColor color)
	{
		m_colorLabel->setStyleSheet(QString("border:1px solid black;background:rgb(%1, %2, %3);").arg(color.red()).arg(color.green()).arg(color.blue()));
	}

	// ��������;
	void setText(QString text)
	{
        m_textLabel->setToolTip(text);
        QFontMetrics fontWidth(m_colorLabel->font());
        int width = fontWidth.width(text);
        //���ַ�����ȴ��������ʱ����ת��;
        if (width >= TEXT_MAX_WIDTH)
        {
            //�Ҳ���ʾʡ�Ժ�;
            text = fontWidth.elidedText(text, Qt::ElideRight, TEXT_MAX_WIDTH);  
        }
		m_textLabel->setText(text);
	}

private:
	QLabel* m_colorLabel;
	QLabel* m_textLabel;
};

/**********������չʾҳ��**********/

class DLWheelMainLineShow : public QWidget
{
	Q_OBJECT

public:
	DLWheelMainLineShow();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
    // ��ʼ�����������ؼ�;
	void initTopWidget();
    // ��ʼ�����Ŀؼ�;
	void initCenterWidget();
    // ��ʼ���ײ���ɫ��ʾ��ؼ�;
	void initBottomWidget();
	
private:
    // �����ؼ�;
	QWidget* m_topBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonSearch;

	QWidget* m_centerWidget;

	QWidget* m_bottomBackWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // �����߿ؼ�;
    DLWheelMainLineGraphicsView* m_dLWheelMainLineGraphicsView;
};

#endif
