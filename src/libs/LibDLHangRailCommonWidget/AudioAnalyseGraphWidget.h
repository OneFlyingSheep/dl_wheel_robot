#ifndef WIDGET_H  
#define WIDGET_H  

#include <QWidget>  
#include "WaveFile.h"  

class AudioAnalyseGraphWidget : public QWidget
{
	Q_OBJECT

public:
	explicit AudioAnalyseGraphWidget(QWidget *parent = 0);
	~AudioAnalyseGraphWidget();

	// 设置音频文件路径;
	void setAudioFilePath(QString strFilePath);

private:
	void paintEvent(QPaintEvent *);
	void Draw16Bit(QPainter &p, int W, int H);
	void Draw8Bit(QPainter &p, int W, int H);

private:
	//波形图所需变量;
	WaveFile m_Wavefile;
	bool m_DrawWave;
	double m_SamplesPerPixel;
	double m_ZoomFactor;
	int m_OffsetInSamples;
	QPixmap m_audioGraphImage;
};

#endif // WIDGET_H  