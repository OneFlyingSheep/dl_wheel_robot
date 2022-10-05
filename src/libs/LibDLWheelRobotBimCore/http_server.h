#pragma once


#include <string>
#include <string.h>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include "mongoose.h"
#include "json/json.h"
#include <thread>


// 定义http返回callback
typedef void OnRspCallback(mg_connection *c, std::string);

// 定义http请求handler
typedef std::function<bool(std::string, std::string, mg_connection *c, OnRspCallback)> ReqHandler;

class HttpServer
{
public:
	HttpServer();
	~HttpServer();

public:
	// 初始化设置
	void Init(const std::string &port);

	// 启动httpserver
	bool Start();

	// 关闭
	bool Close();

	// 注册事件处理函数
	void AddHandler(const std::string &url, ReqHandler req_handler);

	// 移除时间处理函数
	void RemoveHandler(const std::string &url);

	// 网页根目录
	static std::string s_web_dir;

	// web服务器选项
	static mg_serve_http_opts s_server_option;

	// 回调函数映射表
	static std::unordered_map<std::string, ReqHandler> s_handler_map;

private:
	// 静态事件响应函数
	static void OnHttpWebsocketEvent(mg_connection *connection, int event_type, void *event_data);

	// 静态事件响应函数
	static void HandleHttpEvent(mg_connection *connection, http_message *http_req);

	// 静态事件响应函数
	static void SendHttpRsp(mg_connection *connection, std::string rsp);

	// 判断是否是websoket类型连接
	static int isWebsocket(const mg_connection *connection);

	// 处理websocket通信消息事件
	static void HandleWebsocketMessage(mg_connection *connection, int event_type, websocket_message *ws_msg);

	// 发送消息到指定连接
	static void SendWebsocketMsg(mg_connection *connection, std::string msg);

	// 给所有连接广播消息
	static void BroadcastWebsocketMsg(std::string msg);

	// 缓存websocket连接
	static std::unordered_set<mg_connection*> s_websocket_session_set;

	std::thread *loop_thread_;
	void mgr_loop();

private:
	std::string port_;    // 端口
	mg_mgr      mgr_;     // 连接管理器
};

