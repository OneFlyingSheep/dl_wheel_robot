#ifndef WAVEFILE_H  
#define WAVEFILE_H  

#include <string>  
#include <QFile>
using namespace std;

class WaveFile
{
public:
	struct wavehead
	{
		char sign[4];						// "RIFF"��־ 4;
		unsigned long int flength;			// �ļ����� 8;
		char wavesign[4];					// "WAVE"��־ 12;
		char fmtsign[4];					// "fmt"��־ 16;
		unsigned long int unused;		    //  �����ֽڣ�������20;
		unsigned short formattype;			// ��ʽ���10HΪPCM��ʽ����������) 22;
		unsigned short  channelnum;			// ͨ������������Ϊ1��˫����Ϊ2 24;
		unsigned long int  samplerate;		// �����ʣ�ÿ��������������ʾÿ��ͨ���Ĳ����ٶ� 28;
		unsigned long int transferrate;		// �������ʣ�ÿ���ֽ���;
		unsigned short int adjustnum;		// ���ݵ�������һ�����ݵ�λ��ռ���ֽ�;
		unsigned short int databitnum;		// ÿ����������λ����������*8 36;
	}head;
	unsigned long int datalength;			// ������������; 
	unsigned long int totalsample;			// ��������;
	unsigned long int bitpersample;			// ����λ��;
	unsigned long int datanum;				// ���ݿ��С��������λ��Ϊ16���������������Ĵ�С����Ϊ8��ÿ��short�͸ߵ�λ�ɴ洢�������ݣ�����1/2��С����;

	short *Data; // ���ݿ�ָ��;

	WaveFile() {}
	~WaveFile() {}

	// wav�ļ�ͷ��Ϣ�ṹ;
	struct WAVFILEHEADER
	{
		// RIFF ͷ;
		char RiffName[4];
		unsigned long nRiffLength;

		// �������ͱ�ʶ��;
		char WavName[4];

		// ��ʽ���еĿ�ͷ;
		char FmtName[4];
		unsigned long nFmtLength;

		// ��ʽ���еĿ�����;
		unsigned short nAudioFormat;
		unsigned short nChannleNumber;
		unsigned long nSampleRate;
		unsigned long nBytesPerSecond;
		unsigned short nBytesPerSample;
		unsigned short nBitsPerSample;

		// ������Ϣ(��ѡ),���� nFmtLength ���ж�;
		// ��չ���С;
		unsigned short nAppendMessage;
		// ��չ����Ϣ;
		char* AppendMessageData;

		//Fact��,��ѡ�ֶΣ�һ�㵱wav�ļ���ĳЩ���ת�����ɣ��������Chunk;
		char FactName[4];
		unsigned long nFactLength;
		char FactData[4];

		// ���ݿ��еĿ�ͷ;
		char    DATANAME[4];
		unsigned long   nDataLength;

		// �����Ǹ��ӵ�һЩ������Ϣ;
		int fileDataSize;               // �ļ���Ƶ���ݴ�С;
		int fileHeaderSize;             // �ļ�ͷ��С;
		int fileTotalSize;              // �ļ��ܴ�С;


										// ������Ӧ�ý��������ݳ�ʼ��������ֻ��ʼ����ѡ������;
		WAVFILEHEADER()
		{
			nAppendMessage = 0;
			AppendMessageData = NULL;
			strcpy(FactName, "");
			nFactLength = 0;
			strcpy(FactData, "");
		}

	};

	// ����wav�ļ���ͷ��Ϣ;
	bool anlysisWavFileHeader(QString fileName, WAVFILEHEADER& WavFileHeader)
	{
		QFile fileInfo(fileName);
		if (!fileInfo.open(QIODevice::ReadOnly))
		{
			return false;
		}

		// ��ȡ ��Դ�����ļ���־ "RIFF";
		fileInfo.read(WavFileHeader.RiffName, sizeof(WavFileHeader.RiffName));


		// ��ȡ RIFF ͷ���ֽ���;
		fileInfo.read((char*)&WavFileHeader.nRiffLength, sizeof(WavFileHeader.nRiffLength));
		// ��ȡ �����ļ���ʶ�� "WAVE";
		fileInfo.read(WavFileHeader.WavName, sizeof(WavFileHeader.WavName));

		// ��ȡ ���θ�ʽ��־ "fmt ";
		fileInfo.read(WavFileHeader.FmtName, sizeof(WavFileHeader.FmtName));

		// ��ȡ ��ʽ���п����ݴ�С;
		fileInfo.read((char*)&WavFileHeader.nFmtLength, sizeof(WavFileHeader.nFmtLength));

		// ��ȡ ��ʽ����;
		fileInfo.read((char*)&WavFileHeader.nAudioFormat, sizeof(WavFileHeader.nAudioFormat));

		// ��ȡ ��Ƶͨ����Ŀ;
		fileInfo.read((char*)&WavFileHeader.nChannleNumber, sizeof(WavFileHeader.nChannleNumber));

		// ��ȡ ����Ƶ��;
		fileInfo.read((char*)&WavFileHeader.nSampleRate, sizeof(WavFileHeader.nSampleRate));

		// ��ȡ �������ݴ�������;
		fileInfo.read((char*)&WavFileHeader.nBytesPerSecond, sizeof(WavFileHeader.nBytesPerSecond));

		// ��ȡ ���ݿ���뵥λ;
		fileInfo.read((char*)&WavFileHeader.nBytesPerSample, sizeof(WavFileHeader.nBytesPerSample));

		// ��ȡ ÿ�β����õ�����������λ��ֵ;
		fileInfo.read((char*)&WavFileHeader.nBitsPerSample, sizeof(WavFileHeader.nBitsPerSample));

		// ���ݸ�ʽ���п����ݴ�С���ж��Ƿ��и�����Ϣ;
		QString strAppendMessageData;           // ������չ���е���չ��Ϣ;
		if (WavFileHeader.nFmtLength >= 18)
		{
			// ��ȡ������Ϣռ�����ֽ�;
			fileInfo.read((char*)&WavFileHeader.nAppendMessage, sizeof(WavFileHeader.nAppendMessage));
			// ���� �ر�ע�� nFmtLength һ��������� 16 ����18 ��������һ��wav�ļ� nFmtLength Ϊ50;
			// ˵�����Ƕ�ȡ��fmt��ʽ������и�����Ϣ������һ�д����ȡ�������ֽ�����
			// �������ֽڼ�Ϊ��չ��Ĵ�С����ʣ��� 50 - 18 = 32�ֽڼ�Ϊ��չ���е���չ��Ϣ;
			// ������չ���б�����ʲô��ʽ��������ʱ�޷���֪������char�����鱣��;
			// ���� ��չ���С ����ͨ�� WavFileHeader.nAppendMessage �����ļ��ж�ȡ����չ���С�� Ҳ����ͨ�� nFmtLength����ʽ�鳤�ȣ� - 18 �õ�;
			int appendMessageLength = WavFileHeader.nFmtLength - 18;
			WavFileHeader.AppendMessageData = new char[appendMessageLength];
			fileInfo.read(WavFileHeader.AppendMessageData, appendMessageLength);
			// ����Ҳ������ĩβ���ַ��������鿴����,�������ڲ�ȷ����չ��Ϣ�ľ����ʽ;
			//WavFileHeader.AppendMessageData[appendMessageLength] = '\0';
			// ת��QString �鿴��չ��Ϣ����;
			strAppendMessageData = QString(WavFileHeader.AppendMessageData);
		}

		// ����Fact��Ϊ��ѡ�����ܴ��ڣ�������Ҫ�ж�;
		char chunkName[5];
		fileInfo.read(chunkName, sizeof(chunkName) - 1);
		// ��Ҫ�����ַ������� '\0'������ת��QString�����ͨ��strlen������chunkName���ַ�����Ҳ�����
		chunkName[4] = '\0';
		QString strChunkName(chunkName);
		if (strChunkName.compare("fact") == 0)
		{
			// ����fact��,��ȡ����;
			strcpy(WavFileHeader.FactName, chunkName);
			// ��ȡfact�鳤��;
			fileInfo.read((char*)&WavFileHeader.nFactLength, sizeof(WavFileHeader.nFactLength));
			// ��ȡfact������;
			fileInfo.read(WavFileHeader.FactData, sizeof(WavFileHeader.FactData));

			// ����Fact�� , ��ȡ ���ݿ��ʶ��;
			fileInfo.read(WavFileHeader.DATANAME, sizeof(WavFileHeader.DATANAME));
		}
		else
		{
			// ������Fact�飬ֱ�Ӹ�ֵ;
			strcpy(WavFileHeader.DATANAME, chunkName);
		}


		// ��ȡ ���ݿ��С;
		fileInfo.read((char*)&WavFileHeader.nDataLength, sizeof(WavFileHeader.nDataLength));

		// ��ȡ ��Ƶ���ݴ�С;
		WavFileHeader.fileDataSize = fileInfo.readAll().size();

		// �ļ��ܴ�С;
		WavFileHeader.fileTotalSize = WavFileHeader.nRiffLength + 8;

		//�ļ�ͷ��С;
		WavFileHeader.fileHeaderSize = WavFileHeader.fileTotalSize - WavFileHeader.fileDataSize;

		fileInfo.close();
		return true;
	}

	void WavRead(QString fileName)
	{
		WAVFILEHEADER WavFileHeader;
		bool isOpen = anlysisWavFileHeader(fileName, WavFileHeader);
		std::string filename = fileName.toLocal8Bit().data();
		FILE *fp;
		if ((fp = fopen(filename.c_str(), "rb")) == NULL)
		{
			exit(0);
		}
		fseek(fp, WavFileHeader.fileHeaderSize, SEEK_CUR);
		totalsample = WavFileHeader.nDataLength / WavFileHeader.nBytesPerSample;
		bitpersample = WavFileHeader.nBitsPerSample / WavFileHeader.nChannleNumber;
		datanum = totalsample * bitpersample / 16;

		Data = new short[datanum + 10];   // �������ݿ飬������λ��Ϊ16���������������Ĵ�С����Ϊ8��ÿ��short�͸ߵ�λ�ɴ洢�������ݣ�����1/2��С����;

		if (bitpersample == 16)
		{
			for (int i = 0; !feof(fp) && i < datanum; i++)  // ��������;
			{
				fread(&Data[i], 2, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // ����˫�����������ڶ�������;
					fseek(fp, 2, SEEK_CUR);
			}
		}
		else
		{
			for (int i = 0; !feof(fp) && i < datanum; i++)  // ��������;
			{
				short low, high;
				fread(&low, 1, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // ����˫�����������ڶ�������;
					fseek(fp, 1, SEEK_CUR);
				fread(&high, 1, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // ����˫�����������ڶ�������;
					fseek(fp, 1, SEEK_CUR);
				Data[i] = (low & 0x00ff) | (high << 8 & 0xff00);
			}
		}

		fclose(fp);
	}
};

#endif // WAVEFILE_H  