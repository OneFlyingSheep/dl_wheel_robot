#ifndef _VIDEO_PLAYER_H
#define _VIDEO_PLAYER_H


#include <QtWidgets/QWidget>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QVideoProbe>
#include <QFile>
#include <QPainter>
#include <QAudioBuffer>
#include <QTimer>

class DeviceImageWidget : public QWidget
{
public:
	DeviceImageWidget(QWidget* parent = NULL)
	{

	}

	void setImageFilePath(QString strFilePath)
	{
		if (QFile::exists(strFilePath))
		{
			m_deviceImage = QPixmap(strFilePath).scaled(this->size());
		}
		else
		{
			QPixmap image(this->size());
			QPainter painter(&image);
			painter.fillRect(image.rect(), Qt::black);
			m_deviceImage = image;
		}
		update();
	}

private:
	void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		painter.drawPixmap(this->rect(), m_deviceImage);
	}

private:
	QPixmap m_deviceImage;
};

class InfraredTemperature;
class InfraredImageWidget : public QWidget
{
	Q_OBJECT

public:
	InfraredImageWidget(QWidget* parent = NULL);
	~InfraredImageWidget();
	// ���ú���ͼƬ·��;
	void setInfraredImagePath(QString strBmpPath, QString strDdtPath);

private:
	// ����ƶ��¼�;
	void mouseMoveEvent(QMouseEvent *event);
	// �����¼�;
	void paintEvent(QPaintEvent *event);
	// ����뿪widget�¼�;
	void leaveEvent(QEvent *event);

private:

	InfraredTemperature * m_infraredTemperatureObject;
	// ���ƺ���ͼƬ;
	QPixmap m_infraredImage;
	// ���صĺ�������;
	QString m_infraredValue;
	// ����ƶ�������;
	QPoint m_mouseMovePoint;
	// �Ƿ���ƺ�������;
	bool m_isDrawInfraredValue;

	QString m_strBmpPath;
};

class WaveformWidget : public QWidget
{
    Q_OBJECT
public:
    WaveformWidget(QWidget* parent = NULL)
    {
        this->setFixedSize(QSize(600, 300));
    }

    void addWaveformData(QVector<qreal> levels)
    {
        m_waveformDataVector.append(levels);

        int pos = m_waveformDataVector.size() - 100;
        if (pos < 0)
        {
            pos = 0;
        }
        m_waveformDataVector = m_waveformDataVector.mid(pos, 100);
        update();
    }

    void resetWaveform()
    {
        m_waveformDataVector.clear();
        update();
    }

private:
    void paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        painter.fillRect(this->rect(), Qt::black);
        painter.translate(QPoint(0, 200));

        if (m_waveformDataVector.size() < 2)
        {
            return;
        }

        qreal startPoint, endPoint;
        startPoint = -m_waveformDataVector.last() * 100;
        for (int i = m_waveformDataVector.size() - 2; i >= 0; i--)
        {
            endPoint = -m_waveformDataVector.at(i) * 100;
            painter.setPen(Qt::red);
            painter.drawLine(QPoint(0, startPoint), QPoint(6, endPoint));
            startPoint = endPoint;
            painter.translate(QPoint(6, 0));
        }

        //         if (m_waveformDataVector.size() < 2)
        //         {
        //             return;
        //         }
        //         for (int i = m_waveformDataVector.size() - 1; i >= 0; i--)
        //         {
        //             int value = m_waveformDataVector[i] * 100;
        //             painter.fillRect(QRect(0, -value, 10, value), Qt::red);
        //             painter.translate(QPoint(12, 0));
        //         }
    }

private:
    QVector<qreal> m_waveformDataVector;
};

enum PlayType
{
    AudioPlay,
    VideoPlay
};

class VideoPlayer : public QWidget
{
	Q_OBJECT

public:
	VideoPlayer(QWidget *parent = Q_NULLPTR, PlayType playType = VideoPlay);

	// ������Ƶ����·��;
	void setVideoPlayPath(QString filePath);

	// ֹͣ��Ƶ����;
	void stopVideoPlay();

	// ��ʾ;
	void showVideoWidget();

 private:
	// ��ʼ����Ƶ����;
	void initMediaPlayer();
	// ��ʼ����Ƶ���ſؼ�;
	void initVideoPlayerControl();
    // ��ʼ����Ƶ���ſؼ�;
    void initAudioPlayerControl();
	
	// ���ݷ��ص���Ƶ���Ž��ȸ��½���ʱ����ʾ;
	void updateVideoTimeInfo(qint64 currentTimeInfo);


protected:
	// �¼�;
	bool eventFilter(QObject *watched, QEvent *event);

	void paintEvent(QPaintEvent *event);

private slots:
	// ���ص�ǰ��Ƶ������ʱ��;
	void onVideoTotalTimeChanged(qint64 videoTotalTime);
	// ���ص�ǰ��Ƶ���Ž���;
	void onVideoProcessChanged(qint64 currentProcessTime);

	// ��Ƶ����/��ͣ;
	void onVideoPlayButtonClicked();

	// ��Ƶ��������/�ر�;
	void onVideoVoiceSwitchButtonClicked();

	// ��Ƶ���/��ԭ;
	void onFullScreenButtonClicked();
	// �����ı�;
	void onVolumeChanged();
	// ��Ƶ���ȸı�;
	void onChangeVideoProcess(int seconds);

	// ��Ƶ����״̬�ı�;
	void onVideoStatusChanged(QMediaPlayer::State state);

    void processBuffer(QAudioBuffer buffer);

private:

	// ��Ƶ���ſؼ�;
	QMediaPlayer* m_mediaPlayer;
	QVideoWidget* m_videoWidget;


	// ����/��ͣ��ť;
	QPushButton* m_pButtonVideoPlay;
	// ��Ƶ��������;
	QPushButton* m_pButtonVolumeSwitch;
	// ��Ƶ��󻯰�ť;
	QPushButton* m_pButtonVideoFullScreen;

	// ��Ƶ���Ž�����ʾLabel;
	QLabel* m_videoProcessTimeLabel;

	// ��Ƶ���Ž��Ȼ���;
	QSlider* m_videoPlayProcessSlider;
	// ��Ƶ�������ڻ���;
	QSlider* m_videoVolumeSlider;
	
	// ��Ƶ���ſؼ�backWidget;
	QWidget* m_videoControlWidget;

	// ��Ƶ������ʱ��(��λ:��);
	qint64 m_videoTotalTime;

	QVideoProbe *m_videoProbe = nullptr;

	// ��ǰ������Ƶ·��;
	QString m_strVideoFilePath;

    // ��ǰ���ŵ�����;
    PlayType m_playType;

    // ��Ƶ���ƴ���;
    WaveformWidget* m_waveformWidget;

    QTimer m_audioRefreshTimer;
};

#endif // _VIDEO_PLAYER_H