#ifndef __HTTP_CLINET_H__
#define __HTTP_CLINET_H__


#ifdef _DEBUG
#pragma comment(lib, "libcurl-d_imp.lib")
 #else
#pragma comment(lib, "libcurl_imp.lib")
#endif

#include <string>
#include <functional>
#include "mongoose.h"
#include <curl/curl.h>

#define USE_PUBLIC_NET 0

struct HttpMessage
{
	std::string url;
	std::string token;
	std::string seesion_id;
	std::string post_data;
	std::string response_data;
};

// 此处必须用function类，typedef再后面函数指针赋值无效
using ReqCallback = std::function<void (std::string)>;

class HttpClient
{
public:
    HttpClient() {}
    ~HttpClient() {}

    static int SendPostReq(HttpMessage &msg);
	static int SendGetReq(HttpMessage &msg);

    static void OnHttpEvent(mg_connection *connection, int event_type, void *event_data);
    static int s_exit_flag;
    static ReqCallback s_req_callback;

};




#endif // __HTTP_CLINET_H__
