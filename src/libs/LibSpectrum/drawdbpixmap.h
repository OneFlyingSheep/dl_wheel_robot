#ifndef DRAW_DB_PIXMAP_H
#define DRAW_DB_PIXMAP_H

#include <QAudioFormat>
#include <QPixmap>
#include <QScopedPointer>
#include <QWidget>

#include "frequencyspectrum.h"

class DrawDBPixmap : public QWidget
{
    Q_OBJECT

public:
    explicit DrawDBPixmap(QWidget *parent = 0);
    ~DrawDBPixmap();
	void reset();
	void SetData(qreal rTimes, QVector<FrequencySpectrum> vSpectrums);

public slots:
	void bufferLengthChanged(qint64 bufferSize);    //更新buffer长度
	void playPositionChanged(qint64 playPosition);	//更新当前播放坐标
	void windowChanged(qint64 position, qint64 length);		//

protected:
	void paintEvent(QPaintEvent *event) override;
	void resizeEvent(QResizeEvent *event) override;
	void wheelEvent(QWheelEvent *event);

private:
	void CalcXCell();
	int CalcAllPoint();
	void CreatePath();
	void ResizePixmap();

private:
	qreal m_rTimes = 0.0;
	QVector<FrequencySpectrum> m_vSpectrums;
	qreal m_rXCell = 2.0;
	qreal m_rMinVal = 0.0;
	qreal m_rMaxVal = 0.0;

	QPainterPath m_painterPath;

	qint64 m_bufferLength = 0;
	qint64 m_playPosition = 0;
	qint64 m_windowPosition = 0;
	qint64 m_windowLength = 0;
};

#endif // DRAW_DB_PIXMAP_H
