#include "ftp_manager.h"
 
/*****************util api******************/
int FTPImpl::get_file_size(FILE *file)
{
	int size = 0;
	fseek(file, 0L, SEEK_END);
	size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return size;
}
 
/******************curl api****************/
CURL *FTPImpl::curl_init()
{
	curl_global_init(CURL_GLOBAL_DEFAULT); 
	CURL *curl = curl_easy_init();
	if(NULL == curl)
	{
		fprintf(stderr, "Init curl failed.\n");
		exit(1);
	}
	return curl;
}
 
size_t ftp_read(void *ptr, size_t size, size_t nmemb, void *stream)
{
	curl_off_t nread;
	size_t retcode = fread(ptr, size, nmemb, (FILE *)stream);
	nread = (curl_off_t)retcode;
	return retcode;
}

void FTPImpl::curl_set_upload_opt(CURL *curl, const char *url, const char *user_key, FILE *file, curl_off_t size)
{
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_USERPWD, user_key);
	curl_easy_setopt(curl, CURLOPT_READDATA, file);	
	curl_easy_setopt(curl, CURLOPT_UPLOAD, 1);
	curl_easy_setopt(curl, CURLOPT_INFILESIZE, size);
	curl_easy_setopt(curl, CURLOPT_FTP_CREATE_MISSING_DIRS, 1);
	curl_easy_setopt(curl, CURLOPT_READFUNCTION, ftp_read);
//	curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
}
 
void FTPImpl::curl_set_download_opt(CURL *curl, const char *url, const char *user_key, FILE *file)
{
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_USERPWD, user_key);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);
//	curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
}
 
void FTPImpl::curl_exit(CURL *curl)
{
	curl_easy_cleanup(curl);
	curl_global_cleanup(); 
}
 
CURLcode FTPImpl::curl_perform(CURL *curl)
{
	CURLcode ret = curl_easy_perform(curl);
	if(ret != 0)
	{
		fprintf(stderr, "Perform curl failed.\n");
		curl_exit(curl);
		/*exit(1);*/
		return ret;
	}
	return ret;
}

FTPImpl::FTPImpl()
{
	
}

FTPImpl::~FTPImpl()
{
	
}
 
/****************ftp upload & download api******************/
FTP_STATE FTPImpl::upload(const FTP_OPT ftp_option)
{
	FTP_STATE state;
	CURL *curl;
	struct stat file_info;
	curl_off_t fsize;

	if (stat(ftp_option.file.c_str(), &file_info))
	{
		printf("the uploaded file does not exist!");
		return FTP_UPLOAD_FAILED;
	}
	fsize = (curl_off_t)file_info.st_size;

	FILE *fp = fopen(ftp_option.file.c_str(), "rb");
	if(NULL == fp)
	{
		fprintf(stderr, "Open file failed at %s:%d\n", __FILE__, __LINE__);
		return FTP_UPLOAD_FAILED;
	}
 
	curl = curl_init();
	curl_set_upload_opt(curl, ftp_option.url.c_str(), ftp_option.user_key.c_str(), fp, fsize);
	if (CURLE_OK == curl_perform(curl))
	{
		state = FTP_UPLOAD_SUCCESS;
		curl_exit(curl);
	}
	else
		state = FTP_UPLOAD_FAILED;
	fclose(fp);
	return state;
}
 
FTP_STATE FTPImpl::download(const FTP_OPT ftp_option)
{
	FTP_STATE state;
	CURL *curl;
	FILE *fp = fopen(ftp_option.file.c_str(), "w");
	if(NULL == fp)
	{
		fprintf(stderr, "Open file failed at %s:%d\n", __FILE__, __LINE__);
		return FTP_UPLOAD_FAILED;
	}
 
	curl = curl_init();
	curl_set_download_opt(curl, ftp_option.url.c_str(), ftp_option.user_key.c_str(), fp);
	if (CURLE_OK == curl_perform(curl))
	{
		curl_exit(curl);
		state = FTP_DOWNLOAD_SUCCESS;
	}
	else
		state = FTP_DOWNLOAD_FAILED;
 
	fclose(fp);
	return state;
}