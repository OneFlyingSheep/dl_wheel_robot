#include "http_client.h"
 
// 初始化client静态变量
int HttpClient::s_exit_flag = 0;
ReqCallback HttpClient::s_req_callback = 0;
static std::string response_;

// 客户端的网络请求响应
void HttpClient::OnHttpEvent(mg_connection *connection, int event_type, void *event_data)
{
    http_message *hm = (struct http_message *)event_data;
    int connect_status;

    switch (event_type)
    {
    case MG_EV_CONNECT:
        connect_status = *(int *)event_data;
        if (connect_status != 0)
        {
            printf("Error connecting to server, error code: %d\n", connect_status);
            s_exit_flag = 1;
        }
        break;
    case MG_EV_HTTP_REPLY:
    {
        printf("Got reply:\n%.*s\n", (int)hm->body.len, hm->body.p);
        response_ = hm->body.p;
		switch (hm->resp_code)
		{
		case 200:
			connection->flags |= MG_F_SEND_AND_CLOSE;
			s_exit_flag = 1; // 每次收到请求后关闭本次连接，重置标记
		default:
			break;
		} 
 

        // 回调处理
        //s_req_callback(response);
    }
        break;
    case MG_EV_CLOSE:
        if (s_exit_flag == 0)
        {
            printf("Server closed connection\n");
            s_exit_flag = 1;
        };
        break;
    default:
        break;
    }
}


size_t WritePostBodyResp(void *buffer, size_t size, size_t nmemb, void *userp)
{
	((std::string*)userp)->append((char*)buffer, 0, size*nmemb);
	return size * nmemb;
}

size_t WritePostHeaderResp(void *buffer, size_t size, size_t nmemb, void *userp)
{
	((std::string*)userp)->append((char*)buffer, 0, size*nmemb);
	return size * nmemb;
}

int HttpPost(char* url, char* body, std::string &response)
{
	std::string respHeadData;
	CURL* curl;
	CURLcode res;

	//设置头
	struct curl_slist *headers = NULL;
	headers = curl_slist_append(headers, "Content-Type:application/json;charset=UTF-8");

	curl = curl_easy_init();
	if (curl == NULL)
	{
		return 1;
	}

	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body);
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

	curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, WritePostHeaderResp);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WritePostBodyResp);
	curl_easy_setopt(curl, CURLOPT_WRITEHEADER, &respHeadData);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 5000); //libcurl存在毫秒超时bug,如果设备小于1000ms立即返回失败
	curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5000); //设置超时时间

	bool bCA = FALSE;
	if (!bCA)
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, FALSE);//设定为不验证证书和HOST
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, FALSE);
	}
	else
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, TRUE);
		curl_easy_setopt(curl, CURLOPT_CAINFO, "");
	}

	res = curl_easy_perform(curl);
	
	int len = strlen(response.c_str());
	len = response.capacity();
	len = response.length();


	curl_slist_free_all(headers);
	curl_easy_cleanup(curl);

	return res;
}

// 发送一次请求，并回调处理，然后关闭本次连接
int HttpClient::SendPostReq(HttpMessage &msg)
{
	std::string response;

	std::string respHeadData;
	CURL* curl;
	CURLcode res;

	//设置头
	msg.token = "X-Access-Token:" + msg.token;

	struct curl_slist *headers = NULL;
	headers = curl_slist_append(headers, "Content-Type:application/json;charset=UTF-8");
	headers = curl_slist_append(headers, msg.token.c_str());

	curl = curl_easy_init();
	if (curl == NULL)
	{
		return -1;
	}

	curl_easy_setopt(curl, CURLOPT_URL, msg.url.c_str());
	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, msg.post_data.c_str());
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

	curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, WritePostHeaderResp);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WritePostBodyResp);
	curl_easy_setopt(curl, CURLOPT_WRITEHEADER, &respHeadData);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 30000); //libcurl存在毫秒超时bug,如果设备小于1000ms立即返回失败
	curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 30000); //设置超时时间

	bool bCA = FALSE;
	if (!bCA)
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, FALSE);//设定为不验证证书和HOST
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, FALSE);
	}
	else
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, TRUE);
		curl_easy_setopt(curl, CURLOPT_CAINFO, "");
	}

	res = curl_easy_perform(curl);

	msg.response_data.resize(response.size());
	msg.response_data.assign(response.c_str());

	curl_slist_free_all(headers);
	curl_easy_cleanup(curl);

	return res;
}


size_t WriteGetResp(void *buffer, size_t size, size_t nmemb, void *userp)
{
	((std::string*)userp)->append((char*)buffer, 0, size*nmemb);
	return size * nmemb;
}


int HttpClient::SendGetReq(HttpMessage &msg)
{
	std::string response;
	CURL* curl;
	CURLcode res;

	curl = curl_easy_init();
	if (curl == NULL)
	{
		return 1;
	}

	//设置头
	msg.token = "X-Access-Token:" + msg.token;
	struct curl_slist *headers = NULL;
	headers = curl_slist_append(headers, msg.token.c_str());

	curl_easy_setopt(curl, CURLOPT_URL, msg.url.c_str());
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
	//curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "GET");

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteGetResp);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 5000); //libcurl存在毫秒超时bug,如果设备小于1000ms立即返回失败
	curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5000); //设置超时时间

	bool bCA = FALSE;
	if (!bCA)
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, FALSE);//设定为不验证证书和HOST
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, FALSE);
	}
	else
	{
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, TRUE);
		curl_easy_setopt(curl, CURLOPT_CAINFO, "");
	}

	res = curl_easy_perform(curl);

	msg.response_data.resize(response.size());
	msg.response_data.assign(response.c_str());

	if (res != CURLE_OK)
	{
		return res;
	}

	curl_easy_cleanup(curl);

	return 0;
}
