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
	// 设置红外图片路径;
	void setInfraredImagePath(QString strBmpPath, QString strDdtPath);

private:
	// 鼠标移动事件;
	void mouseMoveEvent(QMouseEvent *event);
	// 绘制事件;
	void paintEvent(QPaintEvent *event);
	// 鼠标离开widget事件;
	void leaveEvent(QEvent *event);

private:

	InfraredTemperature * m_infraredTemperatureObject;
	// 绘制红外图片;
	QPixmap m_infraredImage;
	// 返回的红外数据;
	QString m_infraredValue;
	// 鼠标移动的坐标;
	QPoint m_mouseMovePoint;
	// 是否绘制红外数据;
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

	// 设置视频播放路径;
	void setVideoPlayPath(QString filePath);

	// 停止视频播放;
	void stopVideoPlay();

	// 显示;
	void showVideoWidget();

 private:
	// 初始化视频播放;
	void initMediaPlayer();
	// 初始化视频播放控件;
	void initVideoPlayerControl();
    // 初始化音频播放控件;
    void initAudioPlayerControl();
	
	// 根据返回的视频播放进度更新界面时间显示;
	void updateVideoTimeInfo(qint64 currentTimeInfo);


protected:
	// 事件;
	bool eventFilter(QObject *watched, QEvent *event);

	void paintEvent(QPaintEvent *event);

private slots:
	// 返回当前视频播放总时长;
	void onVideoTotalTimeChanged(qint64 videoTotalTime);
	// 返回当前视频播放进度;
	void onVideoProcessChanged(qint64 currentProcessTime);

	// 视频播放/暂停;
	void onVideoPlayButtonClicked();

	// 视频声音开启/关闭;
	void onVideoVoiceSwitchButtonClicked();

	// 视频最大化/还原;
	void onFullScreenButtonClicked();
	// 音量改变;
	void onVolumeChanged();
	// 视频进度改变;
	void onChangeVideoProcess(int seconds);

	// 视频播放状态改变;
	void onVideoStatusChanged(QMediaPlayer::State state);

    void processBuffer(QAudioBuffer buffer);

private:

	// 视频播放控件;
	QMediaPlayer* m_mediaPlayer;
	QVideoWidget* m_videoWidget;


	// 播放/暂停按钮;
	QPushButton* m_pButtonVideoPlay;
	// 视频音量开关;
	QPushButton* m_pButtonVolumeSwitch;
	// 视频最大化按钮;
	QPushButton* m_pButtonVideoFullScreen;

	// 视频播放进度显示Label;
	QLabel* m_videoProcessTimeLabel;

	// 视频播放进度滑块;
	QSlider* m_videoPlayProcessSlider;
	// 视频声音调节滑块;
	QSlider* m_videoVolumeSlider;
	
	// 视频播放控件backWidget;
	QWidget* m_videoControlWidget;

	// 视频播放总时长(单位:秒);
	qint64 m_videoTotalTime;

	QVideoProbe *m_videoProbe = nullptr;

	// 当前播放视频路径;
	QString m_strVideoFilePath;

    // 当前播放的类型;
    PlayType m_playType;

    // 音频绘制窗口;
    WaveformWidget* m_waveformWidget;

    QTimer m_audioRefreshTimer;
};

#endif // _VIDEO_PLAYER_H