#include "LibAudioClient.h"

LibAudioClient::LibAudioClient()
{
    pBuffer1 = NULL;
    pBuffer2 = NULL;
}

LibAudioClient::~LibAudioClient()
{
    if (pBuffer1 != NULL)
    {
        delete[]pBuffer1;
        pBuffer1 = NULL;
    }

    if (pBuffer2 != NULL)
    {
        delete[]pBuffer2;
        pBuffer2 = NULL;
    }
}

bool LibAudioClient::init_remote_audio()
{
    MMRESULT result;

    // Specify recording parameters
    WAVEFORMATEX pFormat;
    pFormat.wFormatTag = WAVE_FORMAT_PCM;
    pFormat.nSamplesPerSec = 8000;
    pFormat.wBitsPerSample = 16;
    pFormat.nChannels = 1;
    pFormat.nBlockAlign = (pFormat.wBitsPerSample * pFormat.nChannels) >> 3;
    pFormat.nAvgBytesPerSec = pFormat.nBlockAlign * pFormat.nSamplesPerSec;    
    pFormat.cbSize = 0;

    result = waveInOpen(&hWaveIn, WAVE_MAPPER, &pFormat, DWORD_PTR(MicCallBack), DWORD_PTR(this), CALLBACK_FUNCTION | WAVE_FORMAT_DIRECT);
    if (result)
    {
        ROS_ERROR("Failed to open waveform input device.");
        return false;
    }

    DWORD bufsize = 1024;
    if (pBuffer1 != NULL)
    {
        delete[]pBuffer1;
        pBuffer1 = NULL;
    }    

    if (pBuffer2 != NULL)
    {
        delete[]pBuffer2;
        pBuffer2 = NULL;
    }

    pBuffer1 = new BYTE[bufsize];
    pBuffer2 = new BYTE[bufsize];

    WaveInHdr.lpData = (LPSTR)pBuffer1;
    WaveInHdr.dwBufferLength = bufsize;
    WaveInHdr.dwBytesRecorded = 0;
    WaveInHdr.dwUser = 0;
    WaveInHdr.dwFlags = 0;
    WaveInHdr.dwLoops = 1;
    WaveInHdr.lpNext = NULL;
    WaveInHdr.reserved = 0;
    //将建立好的wHdr1做为备用
    waveInPrepareHeader(hWaveIn, &WaveInHdr, sizeof(WAVEHDR));

    WaveInHdr2.lpData = (LPSTR)pBuffer2;
    WaveInHdr2.dwBufferLength = bufsize;
    WaveInHdr2.dwBytesRecorded = 0;
    WaveInHdr2.dwUser = 0;
    WaveInHdr2.dwFlags = 0;
    WaveInHdr2.dwLoops = 1;
    WaveInHdr2.lpNext = NULL;
    WaveInHdr2.reserved = 0;
    //将建立好的wHdr2做为备用
    waveInPrepareHeader(hWaveIn, &WaveInHdr2, sizeof(WAVEHDR));
    //将两个wHdr添加到waveIn中去
    waveInAddBuffer(hWaveIn, &WaveInHdr, sizeof(WAVEHDR));
    waveInAddBuffer(hWaveIn, &WaveInHdr2, sizeof(WAVEHDR));

    bMoveOn = true;

    result = waveInStart(hWaveIn);

    return true;
}

void LibAudioClient::stop_remote_audio()
{
//    ROS_INFO("LibAudioClient: stop_remote_audio 1 !");
//    waveInStop(hWaveIn);
    
    
    bMoveOn = false;
//     waveInUnprepareHeader(hWaveIn, &WaveInHdr, sizeof(WAVEHDR));
//     waveInUnprepareHeader(hWaveIn, &WaveInHdr2, sizeof(WAVEHDR));

    ROS_INFO("LibAudioClient: stop_remote_audio 1 !");
    waveInReset(hWaveIn);
    ROS_INFO("LibAudioClient: stop_remote_audio 2 !");
	waveInClose(hWaveIn);
    ROS_INFO("LibAudioClient: stop_remote_audio 3 !");
    
}

bool LibAudioClient::move_on_audio()
{
    return bMoveOn;
}

DWORD CALLBACK LibAudioClient::MicCallBack(HWAVEIN hWaveIn, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2)
{
    LibAudioClient *pAudioClient = (LibAudioClient *)dwInstance;

    PWAVEHDR whd = (PWAVEHDR)dwParam1;

    switch (uMsg)
    {
    case WIM_OPEN:
        break;
    case WIM_DATA:
    {
        pAudioClient->sent_data((uint8_t *)whd->lpData, whd->dwBytesRecorded);
        if (pAudioClient->move_on_audio())
        {
            waveInAddBuffer(hWaveIn, whd, sizeof(WAVEHDR));
        }
        else
        {
        //    waveInClose(hWaveIn);
        }
        break;
    }
    case WIM_CLOSE:
//         waveInStop(hWaveIn);
//         waveInReset(hWaveIn);
    //    waveInClose(hWaveIn);
        break;
    default:
        break;
    }
    return 0;
}
