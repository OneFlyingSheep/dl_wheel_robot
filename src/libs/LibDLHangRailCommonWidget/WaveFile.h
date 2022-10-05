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
		char sign[4];						// "RIFF"标志 4;
		unsigned long int flength;			// 文件长度 8;
		char wavesign[4];					// "WAVE"标志 12;
		char fmtsign[4];					// "fmt"标志 16;
		unsigned long int unused;		    //  过渡字节（不定）20;
		unsigned short formattype;			// 格式类别（10H为PCM形式的声音数据) 22;
		unsigned short  channelnum;			// 通道数，单声道为1，双声道为2 24;
		unsigned long int  samplerate;		// 采样率（每秒样本数），表示每个通道的播放速度 28;
		unsigned long int transferrate;		// 传输速率，每秒字节数;
		unsigned short int adjustnum;		// 数据调整数，一个数据单位所占的字节;
		unsigned short int databitnum;		// 每样本的数据位数，调整数*8 36;
	}head;
	unsigned long int datalength;			// 采样数据总数; 
	unsigned long int totalsample;			// 采样点数;
	unsigned long int bitpersample;			// 采样位数;
	unsigned long int datanum;				// 数据块大小，若采样位数为16，开辟数据总数的大小，若为8，每个short型高地位可存储两个数据，开辟1/2大小即可;

	short *Data; // 数据块指针;

	WaveFile() {}
	~WaveFile() {}

	// wav文件头信息结构;
	struct WAVFILEHEADER
	{
		// RIFF 头;
		char RiffName[4];
		unsigned long nRiffLength;

		// 数据类型标识符;
		char WavName[4];

		// 格式块中的块头;
		char FmtName[4];
		unsigned long nFmtLength;

		// 格式块中的块数据;
		unsigned short nAudioFormat;
		unsigned short nChannleNumber;
		unsigned long nSampleRate;
		unsigned long nBytesPerSecond;
		unsigned short nBytesPerSample;
		unsigned short nBitsPerSample;

		// 附加信息(可选),根据 nFmtLength 来判断;
		// 扩展域大小;
		unsigned short nAppendMessage;
		// 扩展域信息;
		char* AppendMessageData;

		//Fact块,可选字段，一般当wav文件由某些软件转化而成，则包含该Chunk;
		char FactName[4];
		unsigned long nFactLength;
		char FactData[4];

		// 数据块中的块头;
		char    DATANAME[4];
		unsigned long   nDataLength;

		// 以下是附加的一些计算信息;
		int fileDataSize;               // 文件音频数据大小;
		int fileHeaderSize;             // 文件头大小;
		int fileTotalSize;              // 文件总大小;


										// 理论上应该将所有数据初始化，这里只初始化可选的数据;
		WAVFILEHEADER()
		{
			nAppendMessage = 0;
			AppendMessageData = NULL;
			strcpy(FactName, "");
			nFactLength = 0;
			strcpy(FactData, "");
		}

	};

	// 解析wav文件的头信息;
	bool anlysisWavFileHeader(QString fileName, WAVFILEHEADER& WavFileHeader)
	{
		QFile fileInfo(fileName);
		if (!fileInfo.open(QIODevice::ReadOnly))
		{
			return false;
		}

		// 读取 资源交换文件标志 "RIFF";
		fileInfo.read(WavFileHeader.RiffName, sizeof(WavFileHeader.RiffName));


		// 读取 RIFF 头后字节数;
		fileInfo.read((char*)&WavFileHeader.nRiffLength, sizeof(WavFileHeader.nRiffLength));
		// 读取 波形文件标识符 "WAVE";
		fileInfo.read(WavFileHeader.WavName, sizeof(WavFileHeader.WavName));

		// 读取 波形格式标志 "fmt ";
		fileInfo.read(WavFileHeader.FmtName, sizeof(WavFileHeader.FmtName));

		// 读取 格式块中块数据大小;
		fileInfo.read((char*)&WavFileHeader.nFmtLength, sizeof(WavFileHeader.nFmtLength));

		// 读取 格式种类;
		fileInfo.read((char*)&WavFileHeader.nAudioFormat, sizeof(WavFileHeader.nAudioFormat));

		// 读取 音频通道数目;
		fileInfo.read((char*)&WavFileHeader.nChannleNumber, sizeof(WavFileHeader.nChannleNumber));

		// 读取 采样频率;
		fileInfo.read((char*)&WavFileHeader.nSampleRate, sizeof(WavFileHeader.nSampleRate));

		// 读取 波形数据传输速率;
		fileInfo.read((char*)&WavFileHeader.nBytesPerSecond, sizeof(WavFileHeader.nBytesPerSecond));

		// 读取 数据块对齐单位;
		fileInfo.read((char*)&WavFileHeader.nBytesPerSample, sizeof(WavFileHeader.nBytesPerSample));

		// 读取 每次采样得到的样本数据位数值;
		fileInfo.read((char*)&WavFileHeader.nBitsPerSample, sizeof(WavFileHeader.nBitsPerSample));

		// 根据格式块中块数据大小，判断是否有附加信息;
		QString strAppendMessageData;           // 保存扩展域中的扩展信息;
		if (WavFileHeader.nFmtLength >= 18)
		{
			// 读取附加信息占两个字节;
			fileInfo.read((char*)&WavFileHeader.nAppendMessage, sizeof(WavFileHeader.nAppendMessage));
			// 这里 特别注意 nFmtLength 一般情况下是 16 或者18 ，但是有一个wav文件 nFmtLength 为50;
			// 说明我们读取完fmt格式块后面有附加信息，上面一行代码读取了两个字节数据
			// 这两个字节即为扩展域的大小，而剩余的 50 - 18 = 32字节即为扩展域中的扩展信息;
			// 对于扩展域中保存了什么格式的数据暂时无法得知，先用char型数组保存;
			// 这里 扩展域大小 可以通过 WavFileHeader.nAppendMessage （从文件中读取的扩展域大小） 也可以通过 nFmtLength（格式块长度） - 18 得到;
			int appendMessageLength = WavFileHeader.nFmtLength - 18;
			WavFileHeader.AppendMessageData = new char[appendMessageLength];
			fileInfo.read(WavFileHeader.AppendMessageData, appendMessageLength);
			// 这里也可以在末尾加字符结束符查看数据,但是现在不确定扩展信息的具体格式;
			//WavFileHeader.AppendMessageData[appendMessageLength] = '\0';
			// 转成QString 查看扩展信息数据;
			strAppendMessageData = QString(WavFileHeader.AppendMessageData);
		}

		// 由于Fact块为可选，可能存在，所以需要判断;
		char chunkName[5];
		fileInfo.read(chunkName, sizeof(chunkName) - 1);
		// 需要加上字符结束符 '\0'，否则转成QString会出错，通过strlen来计算chunkName的字符长度也会出错。
		chunkName[4] = '\0';
		QString strChunkName(chunkName);
		if (strChunkName.compare("fact") == 0)
		{
			// 存在fact块,读取数据;
			strcpy(WavFileHeader.FactName, chunkName);
			// 读取fact块长度;
			fileInfo.read((char*)&WavFileHeader.nFactLength, sizeof(WavFileHeader.nFactLength));
			// 读取fact块数据;
			fileInfo.read(WavFileHeader.FactData, sizeof(WavFileHeader.FactData));

			// 存在Fact块 , 读取 数据块标识符;
			fileInfo.read(WavFileHeader.DATANAME, sizeof(WavFileHeader.DATANAME));
		}
		else
		{
			// 不存在Fact块，直接赋值;
			strcpy(WavFileHeader.DATANAME, chunkName);
		}


		// 读取 数据块大小;
		fileInfo.read((char*)&WavFileHeader.nDataLength, sizeof(WavFileHeader.nDataLength));

		// 读取 音频数据大小;
		WavFileHeader.fileDataSize = fileInfo.readAll().size();

		// 文件总大小;
		WavFileHeader.fileTotalSize = WavFileHeader.nRiffLength + 8;

		//文件头大小;
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

		Data = new short[datanum + 10];   // 开辟数据块，若采样位数为16，开辟数据总数的大小，若为8，每个short型高地位可存储两个数据，开辟1/2大小即可;

		if (bitpersample == 16)
		{
			for (int i = 0; !feof(fp) && i < datanum; i++)  // 读入数据;
			{
				fread(&Data[i], 2, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // 若是双声道，跳过第二个声道;
					fseek(fp, 2, SEEK_CUR);
			}
		}
		else
		{
			for (int i = 0; !feof(fp) && i < datanum; i++)  // 读入数据;
			{
				short low, high;
				fread(&low, 1, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // 若是双声道，跳过第二个声道;
					fseek(fp, 1, SEEK_CUR);
				fread(&high, 1, 1, fp);
				if (WavFileHeader.nChannleNumber == 2)  // 若是双声道，跳过第二个声道;
					fseek(fp, 1, SEEK_CUR);
				Data[i] = (low & 0x00ff) | (high << 8 & 0xff00);
			}
		}

		fclose(fp);
	}
};

#endif // WAVEFILE_H  