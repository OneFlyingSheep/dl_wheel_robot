#include "AudioAnalyseGraphWidget.h"
#include <QPainter>  
#include <QFile> 

#define MIN(x,y) (x)<(y)?(x):(y)  
#define MAX(x,y) (x)>(y)?(x):(y)  

AudioAnalyseGraphWidget::AudioAnalyseGraphWidget(QWidget *parent)
	: QWidget(parent)
	, m_SamplesPerPixel(0.0)
	, m_OffsetInSamples(0.0)
{
}

AudioAnalyseGraphWidget::~AudioAnalyseGraphWidget()
{

}

void AudioAnalyseGraphWidget::setAudioFilePath(QString strFilePath)
{
	if (!QFile::exists(strFilePath))
	{
		QPixmap image(this->size());
		QPainter painter(&image);
		painter.fillRect(image.rect(), Qt::black);
		m_audioGraphImage = image;
	}
	else
	{
		int W = this->width();			// 900;  
		int H = this->height();			// 350;  

		QPixmap pix(W, H);				// 以此为参数创建一个位图变量;  
		pix.fill(Qt::black);			// 填充位图背景色;  
		QPainter p(&pix);				// 以位图为参数创建一个QPainter 对象;  
										// p.translate(-ur.x(),-ur.y()); // 平移坐标系;  
										// p.drawRect(100,100,200,100);  

		QPen pen;
		pen.setColor(QColor(00, 35, 00));   // 背景网格画笔;  
		p.setPen(pen);						// 设置画笔;  

											// 画背景网格图;
		int i;
		double x, y;
		int col_num = 30;   // 背景网格列数;  
		int row_num = 13;   // 背景网格行数;  

		for (i = 0; i <= col_num; i++)  // 画竖线;  
		{
			x = i * W / col_num;
			p.drawLine((int)x, 0, (int)x, H);
		}

		for (i = 0; i <= row_num; i++)   // 画横线;  
		{
			y = i * H / row_num;
			p.drawLine(0, (int)y, W, (int)y);
		}

		pen.setColor(QColor(0x4B, 0xF3, 0xA7));   // 波形图画笔;  
		p.setPen(pen);      // 设置画笔;  


		m_Wavefile.WavRead(strFilePath);
		m_DrawWave = true;

		if (m_DrawWave)
		{
			// RectangleF visBounds = grfx.VisibleClipBounds;  

			if (m_SamplesPerPixel == 0.0)   // 计算每个像素要显示多少采样点;  
			{
				m_SamplesPerPixel = (m_Wavefile.datanum / W);
			}

			p.drawLine(0, H / 2, W, H / 2);

			// p.translate(0,H/2);  
			// grfx.ScaleTransform(1, -1);  

			if (m_Wavefile.bitpersample == 16)
				Draw16Bit(p, W, H);
			else if (m_Wavefile.bitpersample == 8)
				Draw8Bit(p, W, H);
			// QMessageBox::information(this,"Information",tr(s.c_str()));  
		}
		m_audioGraphImage = pix;
	}
	update();
}

void AudioAnalyseGraphWidget::paintEvent(QPaintEvent *e)
{
	QPainter painter(this);
	painter.drawPixmap(this->rect(), m_audioGraphImage.scaled(this->size()));
}

void AudioAnalyseGraphWidget::Draw16Bit(QPainter &p, int W, int H)
{
	int prevX = 0;
	int prevY = 0;

	int i = 0;

	// index is how far to offset into the data array ; 
	int index = m_OffsetInSamples;
	int maxSampleToShow = (int)((m_SamplesPerPixel * W) + m_OffsetInSamples);

	maxSampleToShow = MIN(maxSampleToShow, m_Wavefile.datanum);

	while (index < maxSampleToShow)
	{
		short maxVal = -32767;
		short minVal = 32767;

		// finds the max & min peaks for this pixel; 
		for (int x = 0; x < m_SamplesPerPixel; x++)
		{
			maxVal = MAX(maxVal, m_Wavefile.Data[x + index]);
			minVal = MIN(minVal, m_Wavefile.Data[x + index]);
		}

		// scales based on height of window  
		int scaledMinVal = (int)(((minVal + 32768) * H) / 65536);
		int scaledMaxVal = (int)(((maxVal + 32768) * H) / 65536);

		scaledMinVal = H - scaledMinVal;
		scaledMaxVal = H - scaledMaxVal;

		//  if samples per pixel is small or less than zero, we are out of zoom range, so don't display anything  
		if (m_SamplesPerPixel > 0.0000000001)
		{
			// if the max/min are the same, then draw a line from the previous position,  
			// otherwise we will not see anything  
			if (scaledMinVal == scaledMaxVal)
			{
				if (prevY != 0)
					p.drawLine(prevX, prevY, i, scaledMaxVal);
			}
			else
			{
				p.drawLine(i, scaledMinVal, i, scaledMaxVal);
			}
		}
		else
			return;

		prevX = i;
		prevY = scaledMaxVal;

		i++;
		index = (int)(i * m_SamplesPerPixel) + m_OffsetInSamples;
	}
}

void AudioAnalyseGraphWidget::Draw8Bit(QPainter &p, int W, int H)
{
	int prevX = 0;
	int prevY = 0;

	int i = 0;

	// index is how far to offset into the data array  
	int index = m_OffsetInSamples;
	int maxSampleToShow = (int)((m_SamplesPerPixel * W) + m_OffsetInSamples);

	maxSampleToShow = MIN(maxSampleToShow, m_Wavefile.datanum);

	while (index < maxSampleToShow)
	{
		short maxVal = 0;
		short minVal = 255;

		// finds the max & min peaks for this pixel  
		for (int x = 0; x < m_SamplesPerPixel; x++)
		{
			short low, high;
			low = (short)(m_Wavefile.Data[x + index] & 0x00ff);
			high = (short)(m_Wavefile.Data[x + index] >> 8 & 0x00ff);
			maxVal = MAX(maxVal, low);
			minVal = MIN(minVal, low);
			maxVal = MAX(maxVal, high);
			minVal = MIN(minVal, high);
		}

		// scales based on height of window  
		int scaledMinVal = (int)(((minVal)* H) / 256);
		int scaledMaxVal = (int)(((maxVal)* H) / 256);

		scaledMinVal = H - scaledMinVal;
		scaledMaxVal = H - scaledMaxVal;

		// if samples per pixel is small or less than zero, we are out of zoom range, so don't display anything  
		if (m_SamplesPerPixel > 0.0000000001)
		{
			// if the max/min are the same, then draw a line from the previous position,  
			// otherwise we will not see anything  
			if (scaledMinVal == scaledMaxVal)
			{
				if (prevY != 0)
					p.drawLine(prevX, prevY, i, scaledMaxVal);
			}
			else
			{
				p.drawLine(i, scaledMinVal, i, scaledMaxVal);
			}
		}
		else
			return;

		prevX = i;
		prevY = scaledMaxVal;

		i++;
		index = (int)(i * m_SamplesPerPixel) + m_OffsetInSamples;
	}
}