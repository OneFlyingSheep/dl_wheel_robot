#include <common/DLRobotCommonDef.h>
#include <MMSYSTEM.H>
#pragma comment(lib, "WINMM.LIB")

class LibAudioClient
{
public:
	LibAudioClient();
	~LibAudioClient();

    bool init_remote_audio();

    void stop_remote_audio();

    bool move_on_audio();

    virtual bool init() = 0;

    virtual bool sent_data(uint8_t *data, uint32_t len) = 0;

private:
    static DWORD CALLBACK MicCallBack(HWAVEIN hWaveIn, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);

protected:
    HWAVEIN hWaveIn;
    WAVEHDR WaveInHdr;
    WAVEHDR WaveInHdr2;
    BYTE *pBuffer1;
    BYTE *pBuffer2;

    bool bMoveOn;
};