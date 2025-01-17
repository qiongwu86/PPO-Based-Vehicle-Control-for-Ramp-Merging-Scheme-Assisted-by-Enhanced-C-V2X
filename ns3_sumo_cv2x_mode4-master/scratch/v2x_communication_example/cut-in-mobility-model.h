#ifndef CUT_IN_MOBILITY_MODEL_H
#define CUT_IN_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/output-stream-wrapper.h"
#include <list>
#include <map>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>


#define CLIP_SPEED(speed) ((speed > MAX_SPEED) ? MAX_SPEED : speed)

const double MAX_SPEED = 25.0;
const double RAID_15 = 15*M_PI/180;
const double SIN_15 = std::sin(15*M_PI/180);
const double COS_15 = std::cos(15*M_PI/180);
const double TAN_15 = std::tan(15*M_PI/180);

const double TWO_LINES_DIST = 200 / COS_15;

const double SENSING_DISTANCE = 100.0;
const double HEADWAY_TIME = 1.5;

const double Z_OF_VEHICLE = 10.0;


namespace ns3 {


class CutInMobilityModel : public MobilityModel 
{
public:
    static const uint32_t BUFFER_LENGTH = 20;

    struct HEADER
    {
        uint32_t m_packet_type; // 0 = first, 1 = end, 2 = action request
        uint32_t m_ID;
        uint32_t m_length;
        uint32_t m_mode; // 0=cacc, 1=RL
    };

    struct SELF_STATE
    {
        double m_x;
        double m_y;
        double m_speed;
        double m_body_angle;
    };

    struct CACC_STATE
    {
        double m_self_project;
        double m_self_speed;
        double m_acc;
        bool m_exist;
        double m_prev_project;
        double m_prev_speed;
    };

    struct RL_STATE
    {
        double m_self_project;
        double m_y_rear;
        double m_y_front;
        double m_self_speed;
        double m_body_angle;
        double m_relative_distance_prev;
        double m_relative_speed_prev;
        double m_relative_distance_foll;
        double m_relative_speed_foll;
    };
    
    struct SEND_BUFFER
    {
        HEADER m_header;
        SELF_STATE m_self_state;
        union 
        {
            CACC_STATE m_cacc_state;
            RL_STATE m_rl_state;
        };
    };

    struct Action
    {
        double m_acc;
        double m_steer;
    };

    struct V2V_PACKET
    {
        double m_x;
        double m_y;
        double m_projection;
        double m_longitude_speed;
        uint32_t m_id;
        int64_t m_time_stamp;
    };

    typedef V2V_PACKET BUFFER_OF_VEHICLE;

    enum BelongTo
    {
        MAIN_ROAD,
        MERGE_ROAD
    };

    static TypeId GetTypeId (void);
    static std::list<Vector> GetInitPosition(BelongTo road);
    static void PrintPositionErrorRate(ns3::Ptr<ns3::OutputStreamWrapper> log_stream);
    static void PrintAoIOverRate(ns3::Ptr<ns3::OutputStreamWrapper> log_stream);
    static void InitServer();

    static std::list<CutInMobilityModel*> CutInMobilityModels;
    static SEND_BUFFER send_buffer;
    static uint32_t total_vehicles;
    static int ClientSock;
    static char recv_buffer[1024];
    static int PositionError[4][5];
    static int PositionErrorCount[4];
    static int AoIOverNum[4][5];
    static int AoITotalNum[4];

    static int AoIRation[7];
    static int PosErrRatio[11];

public:
    // constructor & destructor
    CutInMobilityModel ();
    virtual ~CutInMobilityModel ();

    // 车辆间通信
    void DoPassV2VPacket(const V2V_PACKET&);
    V2V_PACKET DoGenerateV2VPacket();

    // 设置ID，初始位置、开始运动时间、首次获取动作时间
    void InitAndScheduleToStart(uint32_t ID, uint64_t start_time, Vector init_position, BelongTo road);


private:

    void StartToMove(double start_velocity);

    void ChangeAction();
    // 1.更新自身状态
    void UpdateSelfState();
    // 2.刷新部分车辆buffer，x在不同位置有不同操作
    void RefreshBuffer();
    // 3.生成state
    void GenerateState();
    // 4.获取动作
    void GetAction();

    // 类内刷新本车的所有车辆buffer
    void RefreshEntireBuffer();



    // 计算投影和

    void CalculatePositionError();
    void CalculateAoIOver();

    virtual Vector DoGetPosition (void) const;
    virtual void DoSetPosition (const Vector &position);
    virtual Vector DoGetVelocity (void) const;




    // 上次输入动作时间，每次改变action时刷新该值
    int64_t m_basetime_ms;

    // 本车ID
    uint32_t m_ID;

    // self state
    double m_x;
    double m_y;
    double m_speed;
    double m_body_angle;

    // actions
    double m_acc;
    double m_steer;

    // main or merge
    BelongTo m_belong_to;

    // buffer
    BUFFER_OF_VEHICLE m_vehstates[BUFFER_LENGTH];
};

} // namespace ns3

#endif 