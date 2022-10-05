#ifndef BACK_QUERY_MSG_H
#define BACK_QUERY_MSG_H
#include "LibSocket/baseMsgOperation.h"
#include <boost/thread/thread.hpp>
#include <boost/signals2.hpp>
#include <QStringList>
class backGroundQueryMsg :
    public baseMsgOperation
{
public:
    backGroundQueryMsg();
    virtual ~backGroundQueryMsg();

public:
    void registerHandles();
    void recon(boost::asio::ip::tcp::endpoint &end);

    void testOnNode(int id);
    void testOnOffset(float offset);
    void testOnlift(float length);
    void testCamPtz(float pan, float tilt);
    void testRotate(float Rotate);
    void testPD(float pd);

    void getRobotQueryStatus();

    boost::signals2::signal<void(QString, QString)> signal_motorDebugInfo;
    boost::signals2::signal<void(unsigned char)> signal_sickDebugInfo;

private:
    uint16_t getMsgId();

    //req: 8000(0x1F40)        query_robot_status_on_node
    void query_robot_status_on_node(boost::shared_ptr<roboKitMsg> msg);

    //req: 8001(0x1F41)        query_robot_status_offset
    void query_robot_status_offset(boost::shared_ptr<roboKitMsg> msg);

    //req: 8002(0x1F42)        query_robot_status_lift
    void query_robot_status_lift(boost::shared_ptr<roboKitMsg> msg);

    //req: 8003(0x1F43)        query_robot_status_cam_ptz
    void query_robot_status_cam_ptz(boost::shared_ptr<roboKitMsg> msg);

    //req: 8004(0x1F44)        query_robot_status_body_rotate
    void query_robot_status_body_rotate(boost::shared_ptr<roboKitMsg> msg);

    //req: 8005(0x1F45)        query_robot_status_pd
    void query_robot_status_pd(boost::shared_ptr<roboKitMsg> msg);

    //req: 8006 (0x1F46)        query_robot_status_snoise_status
    void query_robot_status_snoise_status(boost::shared_ptr<roboKitMsg> msg);

    //req: 8007 (0x1F47)        query_robot_status_speed
    void query_robot_status_speed(boost::shared_ptr<roboKitMsg> msg);

    //req: 8010 (0x1F4A)        query_robot_status_stop_btn
    void query_robot_status_stop_btn(boost::shared_ptr<roboKitMsg> msg);

    //req: 8011 (0x1F4B)        query_robot_status_pd_arm_status
    void query_robot_status_pd_arm_status(boost::shared_ptr<roboKitMsg> msg);

    //req: 8012 (0x1F4C)        query_robot_hardware_status
    void query_robot_hardware_status(boost::shared_ptr<roboKitMsg> msg);

    //req: 8013 (0x1F4D)        query_robot_version
    void query_robot_version(boost::shared_ptr<roboKitMsg> msg);

    //req: 8014 (0x1F4E)        query_robot_version
    void query_robot_motor_status_debug(boost::shared_ptr<roboKitMsg> msg);

    //req: 8015 (0x1F4F)        query_robot_version
    void query_robot_sick_status_debug(boost::shared_ptr<roboKitMsg> msg);
    
private:
    boost::mutex getMsgIdMutex;
    bool bStatusLoopRunning;
    boost::thread *m_statusLoopThread;

    QStringList motorStatusString;
};
#endif
