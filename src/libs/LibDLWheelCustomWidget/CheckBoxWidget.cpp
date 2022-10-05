#include "CheckBoxWidget.h"
#include <QGridLayout>
#include <QDesktopWidget>
#include <QApplication>

#define TIP_LABEL_WIDTH 80						// ÿ�б�������Label���;
#define CHECKBOX_WIDTH 130						// checkBox���;
#define CHECKBOX_COUNT_EACH_LINE 8				// ÿ��checkBox��Ŀ;

#pragma execution_character_set("utf-8")

CheckBoxWidget::CheckBoxWidget(QWidget *parent)
	: QWidget(parent)
	, m_checkBoxType(WheelCheckBoxTypeEnum::WHEEL_DEVICE_AREA_NAME)
{
	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();

	m_checkBoxInterval = (screenRect.width() - TIP_LABEL_WIDTH - CHECKBOX_WIDTH * CHECKBOX_COUNT_EACH_LINE - 20) / (CHECKBOX_COUNT_EACH_LINE - 1);
}

CheckBoxWidget::~CheckBoxWidget()
{
}

void CheckBoxWidget::setCheckBoxType(WheelCheckBoxTypeEnum type)
{
	m_checkBoxType = type;
}

WheelCheckBoxTypeEnum CheckBoxWidget::getCheckBoxType()
{
	return m_checkBoxType;
}

void CheckBoxWidget::addCheckBoxWidget(QString widgetText, QList<CheckBoxInfo> checkBoxInfoList)
{
	m_checkBoxInfoList = checkBoxInfoList;
	// ����;
	QLabel* tipLabel = new QLabel;
	tipLabel->setText(widgetText + ":");
	tipLabel->setAlignment(Qt::AlignTop);
	tipLabel->setFixedWidth(TIP_LABEL_WIDTH);

	bool isNeedShowMore = false;
	int checkBoxLineCount = 1;
	// �����ǰ�����checkBox��Ŀ������6����Ҫ�����ʾ����һ��;
	if (checkBoxInfoList.count() > 6)
	{
		isNeedShowMore = true;
		// ȷ����ǰ������;
		qreal count = 1.0 * (checkBoxInfoList.count() + 2) / CHECKBOX_COUNT_EACH_LINE;
		if (count > int(count))
		{
			checkBoxLineCount = count + 1;
		}
		else
		{
			checkBoxLineCount = count;
		}
	}

	QGridLayout* gCheckBoxLayout = new QGridLayout;
	int currentIndex = 0;
	for (int i = 0; i < checkBoxLineCount; i++)
	{
		for (int j = 0; j < CHECKBOX_COUNT_EACH_LINE; j++)
		{
			if (currentIndex >= checkBoxInfoList.count())
			{
				gCheckBoxLayout->addItem(new QSpacerItem(CHECKBOX_WIDTH, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), i, j);
				break;
			}
			// �����ǰ�ǵ�һ������Ҫ���ȫ��һ��;
			if (i == 0 && j == 0)
			{
				QCheckBox* checkBox = new QCheckBox;
				checkBox->setFixedWidth(CHECKBOX_WIDTH);
				checkBox->setText("ȫ��");
				gCheckBoxLayout->addWidget(checkBox, i, j);
				connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onChooseAllCheckBoxClicked);
				continue;
			}
			if (i == 0 && j == 7)
			{
				QCheckBox* checkBox = new QCheckBox;
				checkBox->setFixedWidth(CHECKBOX_WIDTH);
				checkBox->setText("��ʾ����");
				gCheckBoxLayout->addWidget(checkBox, i, j);
				connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onShowMoreCheckBoxClicked);
				continue;
			}

			QCheckBox* checkBox = new QCheckBox;
			checkBox->setFixedWidth(CHECKBOX_WIDTH);
			checkBox->setText(checkBoxInfoList[currentIndex].checkBoxText);
			m_checkBoxList.append(checkBox);
			gCheckBoxLayout->addWidget(checkBox, i, j);
			connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onCheckBoxItemClicked);
			
			// ��ǰcheckBox�Ƿ�ɵ��;
			if (!checkBoxInfoList[currentIndex].isEnable)
			{
				checkBox->setDisabled(true);
			}

			currentIndex++;
			// ��һ��֮���checkBox������;
			if (i > 0)
			{
				checkBox->setVisible(false);
			}
		}
	}

	gCheckBoxLayout->setVerticalSpacing(10);
	gCheckBoxLayout->setHorizontalSpacing(m_checkBoxInterval);
	gCheckBoxLayout->setMargin(0);

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(tipLabel);
	hLayout->addLayout(gCheckBoxLayout);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);
}

void CheckBoxWidget::addCheckBoxWidget(QString widgetText, QList<WheelPackageCheckBoxStruct> checkBoxDataList)
{
	m_checkBoxDataList = checkBoxDataList;
	// ����;
	QLabel* tipLabel = new QLabel;
	tipLabel->setText(widgetText + ":");
	tipLabel->setAlignment(Qt::AlignTop);
	tipLabel->setFixedWidth(TIP_LABEL_WIDTH);

	bool isNeedShowMore = false;
	int checkBoxLineCount = 1;
	// �����ǰ�����checkBox��Ŀ������6����Ҫ�����ʾ����һ��;
	if (m_checkBoxDataList.count() > 6)
	{
		isNeedShowMore = true;
		// ȷ����ǰ������;
		qreal count = 1.0 * (m_checkBoxDataList.count() + 2) / CHECKBOX_COUNT_EACH_LINE;
		if (count > int(count))
		{
			checkBoxLineCount = count + 1;
		}
		else
		{
			checkBoxLineCount = count;
		}
	}

	QGridLayout* gCheckBoxLayout = new QGridLayout;
	int currentIndex = 0;
	for (int i = 0; i < checkBoxLineCount; i++)
	{
		for (int j = 0; j < CHECKBOX_COUNT_EACH_LINE; j++)
		{
			if (currentIndex >= m_checkBoxDataList.count())
			{
				gCheckBoxLayout->addItem(new QSpacerItem(CHECKBOX_WIDTH, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), i, j);
				break;
			}
			// �����ǰ�ǵ�һ������Ҫ���ȫ��һ��;
			if (i == 0 && j == 0)
			{
				QCheckBox* checkBox = new QCheckBox;
				checkBox->setFixedWidth(CHECKBOX_WIDTH);
				checkBox->setText("ȫ��");
				gCheckBoxLayout->addWidget(checkBox, i, j);
				connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onChooseAllCheckBoxClicked);
                m_checkBoxChooseAllList.append(checkBox);
				continue;
			}
			if (i == 0 && j == 7)
			{
				QCheckBox* checkBox = new QCheckBox;
				checkBox->setFixedWidth(CHECKBOX_WIDTH);
				checkBox->setText("��ʾ����");
				gCheckBoxLayout->addWidget(checkBox, i, j);
				connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onShowMoreCheckBoxClicked);
				continue;
			}

			QCheckBox* checkBox = new QCheckBox;
			checkBox->setFixedWidth(CHECKBOX_WIDTH);
			checkBox->setObjectName(m_checkBoxDataList[currentIndex].checkbox_uuid);
			checkBox->setText(m_checkBoxDataList[currentIndex].checkbox_name);
			m_checkBoxList.append(checkBox);
			gCheckBoxLayout->addWidget(checkBox, i, j);
			connect(checkBox, &QCheckBox::stateChanged, this, &CheckBoxWidget::onCheckBoxItemClicked);

			// ��ǰcheckBox�Ƿ�ɵ��;
			if (!m_checkBoxDataList[currentIndex].is_enabled)
			{
				checkBox->setDisabled(true);
			}

			if (m_checkBoxDataList[currentIndex].is_check)
			{
				checkBox->setChecked(true);
			}

			currentIndex++;
			// ��һ��֮���checkBox������;
			if (i > 0)
			{
				checkBox->setVisible(false);
			}
		}
	}

	gCheckBoxLayout->setVerticalSpacing(10);
	gCheckBoxLayout->setHorizontalSpacing(m_checkBoxInterval);
	gCheckBoxLayout->setMargin(0);

	QHBoxLayout* hLayout = new QHBoxLayout(this);
	hLayout->addWidget(tipLabel);
	hLayout->addLayout(gCheckBoxLayout);
	hLayout->setSpacing(10);
	hLayout->setMargin(0);
}

QStringList CheckBoxWidget::getCheckedIdList()
{
	QStringList strIdList;
	for each (QCheckBox* checkBox in m_checkBoxList)
	{
		if (checkBox->isChecked() && !checkBox->objectName().isEmpty())
		{
			strIdList.append(checkBox->objectName());
		}
	}

	return strIdList;
}

void CheckBoxWidget::resetCheckBox()
{
    for (int i = 0; i< m_checkBoxList.count(); i++)
    {
        m_checkBoxList[i]->setChecked(false);
    }

    for (int i = 0; i < m_checkBoxChooseAllList.count(); i++)
    {
        m_checkBoxChooseAllList[i]->setChecked(false);
    }
}

void CheckBoxWidget::onChooseAllCheckBoxClicked(int checkBoxState)
{
	bool isChecked = false;
	if (checkBoxState == Qt::Checked)
	{
		isChecked = true;
	}
	for (int i = 0; i < m_checkBoxList.count(); i++)
	{
		if (m_checkBoxList[i]->isEnabled())
		{
			m_checkBoxList[i]->setChecked(isChecked);
		}
	}
}

void CheckBoxWidget::onShowMoreCheckBoxClicked(int checkBoxState)
{
	bool isChecked = false;
	if (checkBoxState == Qt::Checked)
	{
		isChecked = true;
	}
	for (int i = CHECKBOX_COUNT_EACH_LINE - 2; i < m_checkBoxList.count(); i++)
	{
		m_checkBoxList[i]->setVisible(isChecked);
	}
}

void CheckBoxWidget::onCheckBoxItemClicked(int checkBoxState)
{
	QObject* checkBoxWidget = sender();
	QString strCheckBoxId = checkBoxWidget->objectName();
	bool isChecked = false;
	if (checkBoxState == Qt::Checked)
	{
		isChecked = true;
	}

	emit signalCheckBoxClicked(strCheckBoxId, isChecked);
}