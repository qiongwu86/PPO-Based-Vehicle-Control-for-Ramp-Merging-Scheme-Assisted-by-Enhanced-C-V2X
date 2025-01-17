#include "cut-in-mobility-model.h"
#include "ns3/simulator.h"
#include <list>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (CutInMobilityModel);


CutInMobilityModel::SEND_BUFFER CutInMobilityModel::send_buffer = {0};
char CutInMobilityModel::recv_buffer[1024] = {0};
int CutInMobilityModel::PositionError[4][5] = {0};
int CutInMobilityModel::PositionErrorCount[4] = {0};
int CutInMobilityModel::AoIOverNum[4][5] = {0};
int CutInMobilityModel::AoITotalNum[4] = {0};

int CutInMobilityModel::AoIRation[7] = {0};
int CutInMobilityModel::PosErrRatio[11] = {0};

uint32_t CutInMobilityModel::total_vehicles = 0;
int CutInMobilityModel::ClientSock = 0;
std::list<CutInMobilityModel*> CutInMobilityModel::CutInMobilityModels;


TypeId CutInMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CutInMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<CutInMobilityModel> ();
  return tid;
}

std::list<Vector> 
CutInMobilityModel::GetInitPosition (CutInMobilityModel::BelongTo belong_to)
{
    std::list<Vector> return_list;
    Ptr<ns3::UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    // VehPosition pos;
    int vehnum = rand() % (7-6+1) + 6;
    double start_point = RV->GetValue(0.0, 5.0);
    if (belong_to == CutInMobilityModel::BelongTo::MAIN_ROAD)
        start_point += 200*(1/COS_15 - 1);
    // double x = -(375.0 + RV->GetValue(0.0, 5.0));
    double interval = 200.0 / double(vehnum - 1);

    for (int i = 0; i < vehnum; i++)
    {
        double y = (belong_to == BelongTo::MAIN_ROAD) ? 0.0 : (-3.75-TAN_15*200);
        double x = -(375.0 + start_point);

        Vector pos = {x, y, Z_OF_VEHICLE};
        return_list.push_back(pos);
        start_point += (interval + RV->GetValue(-3.0, 3.0));
    }

    return return_list;
}


void 
CutInMobilityModel::InitServer()
{
    struct sockaddr_in ServerAddr;
    memset(&ServerAddr, '0', sizeof (sockaddr_in));
    ServerAddr.sin_family = AF_INET;
    ServerAddr.sin_port = htons(10889);
    ServerAddr.sin_addr.s_addr = inet_addr("---.---.---.---");

    ClientSock = socket(AF_INET, SOCK_STREAM, 0);

    connect(ClientSock, (const struct sockaddr*) &ServerAddr, sizeof (sockaddr));
    std::cout << "Connect to server"<< std::endl;

    send_buffer.m_header.m_packet_type = 0;
    send_buffer.m_header.m_length = sizeof (SEND_BUFFER);
    send_buffer.m_header.m_ID = total_vehicles;

    send(ClientSock, &send_buffer, sizeof(SEND_BUFFER), 0);

    std::cout << "Send init packet" << std::endl;

    int recv_len = recv(ClientSock, &recv_buffer, 1024, 0);

    NS_ASSERT(recv_len == sizeof(Action));
    NS_ASSERT((((Action*) recv_buffer)->m_acc == 0.0) & (((Action*)recv_buffer)->m_steer == 0.0));
}



CutInMobilityModel::CutInMobilityModel()
{
    CutInMobilityModels.push_back(this);
    total_vehicles++;
}

CutInMobilityModel::~CutInMobilityModel()
{
    total_vehicles--;
    if (total_vehicles == 0)
    {
        send_buffer.m_header.m_packet_type = 1;
        send(ClientSock, &send_buffer, sizeof(SEND_BUFFER), 0);
        close(ClientSock);
    }
}


// 车辆间通信
void CutInMobilityModel::DoPassV2VPacket(const V2V_PACKET& s)
{
    m_vehstates[s.m_id] = s;
}


CutInMobilityModel::V2V_PACKET CutInMobilityModel::DoGenerateV2VPacket()
{
    double delta_t = 1e-3 * (Simulator::Now().GetMilliSeconds() - m_basetime_ms);
    double x = m_x + delta_t * m_speed * std::cos(m_body_angle);
    double y = m_y + delta_t * m_speed * std::sin(m_body_angle);
    double body_angle = m_body_angle + delta_t * m_speed * std::tan(m_steer) / 4.5;
    double speed = CLIP_SPEED(m_speed + delta_t * m_acc);

    if (m_belong_to == BelongTo::MERGE_ROAD)
    {
        if ((m_x < -375) & (-375 < x))
        {
            body_angle = RAID_15;
            double temp = COS_15 * (x + 375);
            x = -375 + temp;
            y += TAN_15 * temp;
        }    
        if ((m_x < -175 ) & (-175< x))
        {
            body_angle = 0.0;
            double temp = sqrt(pow(x + 175, 2) + pow(y + 3.75, 2));
            x = temp - 175;
            y = -3.75;
        }
        if ((m_x < 0.0) & (0.0 < x))
            body_angle = 0.0;
    }

    double projection = 0.0;
    double longitude_speed = 0.0;

    if (m_belong_to == BelongTo::MAIN_ROAD)
    {
        projection = x;
        longitude_speed = speed;
    }
    else
    {
        if (x > -175.0)
            projection = x;
        else if (x < -375.0)
            projection = -(175.0 + TWO_LINES_DIST + (-375.0 - x));
        else
            projection = -(175.0 + (-175.0 - x) / COS_15);

        if ((x < -175) | (x > 0))
            longitude_speed = speed;
        else
            longitude_speed = speed * std::cos(body_angle);
    }

    return {
        x,
        y,
        projection,
        longitude_speed,
        m_ID,
        Simulator::Now().GetMilliSeconds()
    };
}

void 
CutInMobilityModel::InitAndScheduleToStart
(uint32_t ID, uint64_t start_time, Vector init_position, CutInMobilityModel::BelongTo road)
{
    m_belong_to = road;
    m_ID = ID;

    m_x = init_position.x;
    m_y = init_position.y;
    m_speed = 0.0;
    m_body_angle = 0.0;

    m_acc = 0.0;
    m_steer = 0.0;

    Ptr<ns3::UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    double start_velocity = 15.0 + RV->GetValue(-3.0, 3.0);

    Simulator::Schedule(MilliSeconds(start_time), &CutInMobilityModel::StartToMove, this, start_velocity);
    Simulator::Schedule(MilliSeconds(start_time + 1), &CutInMobilityModel::RefreshEntireBuffer, this);
    uint32_t offset = rand()%(101 - 2 + 1) + 2;
    // uint32_t offset = 2;
    Simulator::Schedule(MilliSeconds(start_time+offset), &CutInMobilityModel::ChangeAction, this);
}

void CutInMobilityModel::StartToMove(double start_velocity)
{
    m_basetime_ms = Simulator::Now().GetMilliSeconds();
    m_speed = start_velocity;
}

void CutInMobilityModel::RefreshBuffer()
{
    if (m_x < -175)
    {
        // std::list<CutInMobilityModel*>::iterator prev = CutInMobilityModels.end(), foll = CutInMobilityModels.end();
        CutInMobilityModel* prev = this, *foll = this;
        double prev_x = (m_x+200.0), foll_x = (m_x-200.0);
        for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
        {
            if (((*ptr)->m_belong_to == m_belong_to))
             {   
                if (((*ptr)->m_x < prev_x) & ((*ptr)->m_x > m_x))
                    prev = (*ptr);
                else if (((*ptr)->m_x > foll_x) & ((*ptr)->m_x < m_x))
                    foll = (*ptr);
            }
        }
        if (prev != this)
            m_vehstates[prev->m_ID] = prev->DoGenerateV2VPacket();
        if (foll != this)
            m_vehstates[foll->m_ID] = foll->DoGenerateV2VPacket();
    }
    else 
        RefreshEntireBuffer();

}

void CutInMobilityModel::GenerateState()
{
    send_buffer.m_header.m_ID = m_ID;
    send_buffer.m_header.m_length = sizeof(SEND_BUFFER);
    send_buffer.m_header.m_packet_type = 2;

    send_buffer.m_self_state = {
        m_x,
        m_y,
        m_speed,
        m_body_angle
    };

    V2V_PACKET self_state = DoGenerateV2VPacket();

    if ((m_belong_to == BelongTo::MERGE_ROAD) & (m_x > -175.0) & (m_x < 0.0))
    {
        send_buffer.m_header.m_mode = 1;
        // CACC-TP
	double self_proj = self_state.m_projection,
               prev_proj = self_proj + SENSING_DISTANCE;
        double self_longitude_speed = self_state.m_longitude_speed;

        send_buffer.m_rl_state.m_self_speed = self_longitude_speed;
        send_buffer.m_rl_state.m_x = m_x;
        send_buffer.m_rl_state.m_y = m_y;
	send_buffer.m_rl_state.m_body_angle = m_body_angle;
        send_buffer.m_rl_state.m_self_project = self_proj;
        send_buffer.m_rl_state.m_acc = m_acc;

        uint32_t prev_id = m_ID;
        int64_t current_time = Simulator::Now().GetMilliSeconds();
        NS_ASSERT(current_time == m_basetime_ms);
        for (uint32_t temp_id = 0; temp_id < total_vehicles; temp_id++)
        {
            NS_ASSERT(temp_id == m_vehstates[temp_id].m_id);
            double temp_proj = m_vehstates[temp_id].m_projection +
                               m_vehstates[temp_id].m_longitude_speed * 1e-3 * (current_time - m_vehstates[temp_id].m_time_stamp);
            if (temp_id != m_ID)
            {
                if ((temp_proj > self_proj) & (temp_proj < prev_proj))
                    prev_id = temp_id, prev_proj = temp_proj;
            }
	}

        if (prev_id != m_ID)
            send_buffer.m_rl_state.m_prev_project = prev_proj, 
            send_buffer.m_rl_state.m_prev_speed = m_vehstates[prev_id].m_longitude_speed,
            send_buffer.m_rl_state.m_exist = true;

        else
            send_buffer.m_rl_state.m_exist = false;

    }
        //RL, 无需预测位置
     //   double self_proj = self_state.m_projection, 
       //        prev_proj = self_proj + SENSING_DISTANCE, 
         //      foll_proj = self_proj - SENSING_DISTANCE;
      //  double self_longitude_speed = self_state.m_longitude_speed;
//
  //      send_buffer.m_rl_state.m_self_project = self_proj;
    //    send_buffer.m_rl_state.m_y_rear = m_y;
      //  send_buffer.m_rl_state.m_y_front = m_y + std::sin(m_body_angle) * 4.5;
        //send_buffer.m_rl_state.m_self_speed = m_speed;
        //send_buffer.m_rl_state.m_body_angle = m_body_angle;

       // uint32_t prev_id = m_ID, foll_id = m_ID;
        //int64_t current_time = Simulator::Now().GetMilliSeconds();
        //NS_ASSERT(current_time == m_basetime_ms);
        //for (uint32_t temp_id = 0; temp_id < total_vehicles; temp_id++)
        //{
          //  NS_ASSERT(temp_id == m_vehstates[temp_id].m_id);
            //if (temp_id != m_ID)
            //{
              //  if ((m_vehstates[temp_id].m_projection < self_proj) & (m_vehstates[temp_id].m_projection > foll_proj))
                //    foll_id = temp_id, foll_proj = m_vehstates[temp_id].m_projection;
               // else if ((m_vehstates[temp_id].m_projection > self_proj) & (m_vehstates[temp_id].m_projection < prev_proj))
                 //   prev_id = temp_id, prev_proj = m_vehstates[temp_id].m_projection;
                
            //}
       // }

        //if (prev_id != m_ID)
          //  send_buffer.m_rl_state.m_relative_distance_prev = m_vehstates[prev_id].m_projection - self_proj,
            //send_buffer.m_rl_state.m_relative_speed_prev = m_vehstates[prev_id].m_longitude_speed - self_longitude_speed;
        //else
          //  send_buffer.m_rl_state.m_relative_distance_prev = self_longitude_speed * HEADWAY_TIME,
            //send_buffer.m_rl_state.m_relative_speed_prev = 0.0;

        //if (foll_id != m_ID)
          //  send_buffer.m_rl_state.m_relative_distance_foll = self_proj - m_vehstates[foll_id].m_projection,
            //send_buffer.m_rl_state.m_relative_speed_prev = self_longitude_speed - m_vehstates[foll_id].m_longitude_speed;
       // else
         //   send_buffer.m_rl_state.m_relative_distance_foll = self_longitude_speed* HEADWAY_TIME,
           // send_buffer.m_rl_state.m_relative_speed_foll = 0.0;
    //}
    else
    {
        send_buffer.m_header.m_mode = 0;
        // CACC
        double self_proj = self_state.m_projection, 
               prev_proj = self_proj + SENSING_DISTANCE;
        double self_longitude_speed = self_state.m_longitude_speed;

        send_buffer.m_cacc_state.m_self_speed = self_longitude_speed;
        send_buffer.m_cacc_state.m_self_project = self_proj;
        send_buffer.m_cacc_state.m_acc = m_acc;

        uint32_t prev_id = m_ID;
        int64_t current_time = Simulator::Now().GetMilliSeconds();
        NS_ASSERT(current_time == m_basetime_ms);
        for (uint32_t temp_id = 0; temp_id < total_vehicles; temp_id++)
        {
            NS_ASSERT(temp_id == m_vehstates[temp_id].m_id);
            double temp_proj = m_vehstates[temp_id].m_projection + 
                               m_vehstates[temp_id].m_longitude_speed * 1e-3 * (current_time - m_vehstates[temp_id].m_time_stamp);
            if (temp_id != m_ID)
            {
                if ((temp_proj > self_proj) & (temp_proj < prev_proj))
                    prev_id = temp_id, prev_proj = temp_proj;
            }
        }

        if (prev_id != m_ID)
            send_buffer.m_cacc_state.m_prev_project = prev_proj,
            send_buffer.m_cacc_state.m_prev_speed = m_vehstates[prev_id].m_longitude_speed,
            send_buffer.m_cacc_state.m_exist = true;

        else
            send_buffer.m_cacc_state.m_exist = false;

    }
}

void CutInMobilityModel::RefreshEntireBuffer()
{
    for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
            m_vehstates[(*ptr)->m_ID] = (*ptr)->DoGenerateV2VPacket();
}

void CutInMobilityModel::UpdateSelfState()
{
    double last_x = m_x;

    double delta_t = 1e-3 * (Simulator::Now().GetMilliSeconds() - m_basetime_ms);
    m_x += delta_t * m_speed * std::cos(m_body_angle);
    m_y += delta_t * m_speed * std::sin(m_body_angle);
    m_body_angle += delta_t * m_speed * std::tan(m_steer) / 4.5;
    m_speed = CLIP_SPEED(m_speed + delta_t * m_acc);

    if (m_belong_to == BelongTo::MERGE_ROAD)
    {
        if ((last_x < -375) & (-375 < m_x))
        {
            m_body_angle = RAID_15;
            double temp = COS_15 * (m_x + 375);
            m_x = -375 + temp;
            m_y += TAN_15 * temp;
        }    
        if ((last_x < -175 ) & (-175< m_x))
        {
            m_body_angle = 0.0;
            double temp = sqrt(pow(m_x + 175, 2) + pow(m_y + 3.75, 2));
            m_x = temp - 175;
            m_y = -3.75;
        }
        if ((last_x < 0.0) & (0.0 < m_x))
            m_body_angle = 0.0;
    }
    m_basetime_ms = Simulator::Now().GetMilliSeconds();
}

inline Vector
CutInMobilityModel::DoGetVelocity (void) const
{
  double t = (Simulator::Now ().GetMilliSeconds() - m_basetime_ms) * 1e-3;
  double speed_new = m_speed + m_acc * t;
  return Vector (speed_new * std::cos(m_body_angle),
                 speed_new * std::sin(m_body_angle),
                 0.0);
}

inline Vector
CutInMobilityModel::DoGetPosition (void) const
{
  double t = (Simulator::Now ().GetMilliSeconds() - m_basetime_ms) * 1e-3;
  double half_t_square = t*t*0.5;
  return Vector (m_x + m_speed * std::cos(m_body_angle) * t + m_acc * std::cos(m_body_angle) * half_t_square,
                 m_y + m_speed * std::sin(m_body_angle) * t + m_acc * std::sin(m_body_angle) * half_t_square,
                 Z_OF_VEHICLE);
}

void 
CutInMobilityModel::DoSetPosition (const Vector &position)
{
    return;
}

void
CutInMobilityModel::GetAction()
{
    // CalculatePositionError();
    send(ClientSock, &send_buffer, sizeof(SEND_BUFFER), 0);

    // std::cout << "Send request action packet" << std::endl;

    int recv_len = recv(ClientSock, &recv_buffer, 1024, 0);
    NS_ASSERT(recv_len == sizeof(Action));
}

void 
CutInMobilityModel::ChangeAction()
{
    UpdateSelfState();
    RefreshBuffer();
    if (m_x < -175)
    {
        CalculateAoIOver();
        CalculatePositionError();
    }    

    
    GenerateState();
    GetAction();

    m_acc = ((Action*)recv_buffer)->m_acc, m_steer = ((Action*)recv_buffer)->m_steer;
    Simulator::Schedule(MilliSeconds(100), &CutInMobilityModel::ChangeAction, this);
}

void 
CutInMobilityModel::CalculatePositionError()
{
    //  position, ego_position;
    double distance, error;
    for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
    {
        if ((*ptr)->m_ID == m_ID)
            continue;
        
        Vector position = (*ptr)->DoGetPosition();
        // ego_position = DoGetPosition();
        distance = sqrt(pow(m_x-(*ptr)->m_x, 2) + 
                        pow(m_y-(*ptr)->m_y, 2));
        error = sqrt(pow(position.x - m_vehstates[(*ptr)->m_ID].m_x, 2) + 
                     pow(position.y - m_vehstates[(*ptr)->m_ID].m_y, 2));

        // std::cout << m_ID << "  " << error << " " << m_vehstates[(*ptr)->m_ID].m_genTime << std::endl;

        int ind1, ind2;
        ind1 = int(distance/50.0);
        ind1 = (ind1 <= 4) ? ind1 : 4;
        // switch (int(distance/50.0))
        // {
        // case 0:
        //     ind1 = 0;
        //     break;
        // case 1:
        //     ind1 = 1;
        //     break;
        // case 2:
        //     ind1 = 2;
        //     break;
        // case 3:
        //     ind1 = 3;
        //     break;
        // default:
        //     ind1 = 4;
        //     break;
        // }
        ind2 = int(error);
        ind2 = (error <= 5) ? ind2 : 5;
        // switch ( int(error / 0.5) )
        // {
        //     case 0:
        //     case 1:
        //         ind2 = 0;
        //         break;
        //     case 2:
        //     case 3:
        //         ind2 = 1;
        //         break;
        //     case 4:
        //     case 5:
        //         ind2 = 2;
        //         break;
        //     case 6:
        //     case 7:
        //         ind2 = 3;
        //         break;
        //     case 8:
        //     case 9:
        //         ind2 = 4;
        //         break;
        //     default:
        //         ind2 = 5;
        //         break;
        // }

        for (int i = ind1; i < 4; i++)
        {
            PositionErrorCount[i]++;
            for (int j = 0; j < ind2; j++)
                PositionError[i][j]++;
        }

        int ind3 = int(error/0.5);
        ind3 = (ind3 <= 10) ? ind3 : 10; 
        if (distance <= 150.0)
            PosErrRatio[ind3]++;
    }
}

void 
CutInMobilityModel::CalculateAoIOver()
{
    double distance;
    int64_t aoi;
    int64_t current_time = Simulator::Now().GetMilliSeconds();
    for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
    {
        if ((*ptr)->m_ID == m_ID)
            continue;
        
        // position = (*ptr)->DoGetPosition();
        // ego_position = DoGetPosition();
        distance = sqrt(pow(m_x-(*ptr)->m_x, 2) + 
                        pow(m_y-(*ptr)->m_y, 2));
        aoi = current_time - m_vehstates[(*ptr)->m_ID].m_time_stamp;

        // std::cout << m_ID << "  " << error << " " << m_vehstates[(*ptr)->m_ID].m_genTime << std::endl;

        int ind1, ind2;
        ind1 = int(distance/50.0);
        ind1 = (ind1 <= 4) ? ind1 : 4;
        // switch (int(distance/50.0))
        // {
        // case 0:
        //     ind1 = 0;
        //     break;
        // case 1:
        //     ind1 = 1;
        //     break;
        // case 2:
        //     ind1 = 2;
        //     break;
        // case 3:
        //     ind1 = 3;
        //     break;
        // default:
        //     ind1 = 4;
        //     break;
        // }
        switch ( aoi / 10 )
        {
            case 0:
            case 1:
            case 2:
            case 3:
                ind2 = 0;
                break;
            case 4:
            case 5:
                ind2 = 1;
                break;
            case 6:
            case 7:
                ind2 = 2;
                break;
            case 8:
            case 9:
                ind2 = 3;
                break;
            case 10:
            case 11:
                ind2 = 4;
                break;
            default:
                ind2 = 5;
                break;
        }

        for (int i = ind1; i < 4; i++)
        {
            AoITotalNum[i]++;
            for (int j = 0; j < ind2; j++)
                AoIOverNum[i][j]++;
        }
    
        int ind3 = aoi / 20;
        ind3 = (ind3 <= 6) ? ind3 : 6;
        if (distance < 150.0)
            AoIRation[ind3] ++;
    }
}

void 
CutInMobilityModel::PrintAoIOverRate(ns3::Ptr<ns3::OutputStreamWrapper> log_stream)
{
    *log_stream -> GetStream() << "AoIOverRate: " << std::endl;
    for(int i = 0; i < 4; i++)
    {
        *log_stream -> GetStream() << 50*(i+1) << "m: \n";
        for (int j = 0; j < 5; j++)
            *log_stream -> GetStream() << AoIOverNum[i][j] << "  " << AoITotalNum[i] << std::endl;
    }
    for (int i = 0; i < 7; i++)
        *log_stream -> GetStream() << AoIRation[i] << "  ";
    *log_stream -> GetStream() << std::endl;
}

void CutInMobilityModel::PrintPositionErrorRate(ns3::Ptr<ns3::OutputStreamWrapper> log_stream)
{
    *log_stream -> GetStream() << "PositionErrorRate: " << std::endl;
    for(int i = 0; i < 4; i++)
    {
        *log_stream -> GetStream() << 50*(i+1) << "m: \n";
        for (int j = 0; j < 5; j++)
            *log_stream -> GetStream() << PositionError[i][j] << "  " << PositionErrorCount[i] << std::endl;
    }
    for (int i = 0; i < 11; i++)
        *log_stream -> GetStream() << PosErrRatio[i] << "  ";
    *log_stream -> GetStream() << std::endl;
}

}
