#ifndef __FTP_MANAGER_H_INCLUDE__
#define __FTP_MANAGER_H_INCLUDE__

#include <stdio.h>
#include <stdlib.h>
#include <curl/curl.h>
#include <string>

/*FTP OPERATION CODE*/
typedef enum FTP_STATE
{
	FTP_UPLOAD_SUCCESS,
	FTP_UPLOAD_FAILED,
	FTP_DOWNLOAD_SUCCESS,
	FTP_DOWNLOAD_FAILED 
}FTP_STATE;
 
/*FTP OPERATIONS OPTIONS*/
typedef struct FTP_OPT
{
	std::string url;			/*url of ftp*/
	std::string user_key;		/*username:password*/
	std::string file;			/*local_filepath*/
}FTP_OPT;
 
#ifdef __cplusplus
	extern "C" {
#endif

class FTPImpl
{
public:
    FTPImpl();
    ~FTPImpl();

	/*upload file to ftp server*/
	FTP_STATE upload(const FTP_OPT ftp_option);
	 
	/*download file from ftp server*/
	FTP_STATE download(const FTP_OPT ftp_option);

private:
	int get_file_size(FILE *file);
	CURL *curl_init();
	void curl_set_upload_opt(CURL *curl, const char *url, const char *user_key, FILE *file, curl_off_t size);
	void curl_set_download_opt(CURL *curl, const char *url, const char *user_key, FILE *file);
	void curl_exit(CURL *curl);
	CURLcode curl_perform(CURL *curl);
};

#ifdef __cplusplus
	}
#endif

#endif  //__FTP_MANAGER_H_INCLUDE__
