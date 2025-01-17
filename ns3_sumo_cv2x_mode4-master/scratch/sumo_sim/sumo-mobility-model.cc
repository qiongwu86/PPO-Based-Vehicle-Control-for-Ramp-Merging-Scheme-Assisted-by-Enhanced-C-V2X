#include "sumo-mobility-model.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/traced-value.h"
#include "ns3/type-id.h"
#include "ns3/vector.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <libsumo/libtraci.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#define STEP_LENGTH "--step-length 0.001"
#define END_TIME "--end 110"

namespace ns3{
NS_OBJECT_ENSURE_REGISTERED (SUMOMobilityModel);
std::map<uint32_t, Vector> SUMOMobilityModel::AllVehiclePosition;
std::vector<std::string> SUMOMobilityModel::AllVehicleIDInSUMO;
std::map<uint32_t, std::string> SUMOMobilityModel::AllVehcileMap;
std::vector<std::vector<int64_t>> SUMOMobilityModel::AoIRecord {DISLEN, std::vector<int64_t>(AOIRECORDLEN, 0)};
std::vector<std::vector<int64_t>> SUMOMobilityModel::PosRecord {DISLEN, std::vector<int64_t>(POSRECORDLEN, 0)};
std::vector<SUMOMobilityModel*> SUMOMobilityModel::AllSUMOMobilityModelObj;

TypeId SUMOMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SUMOMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<SUMOMobilityModel> ();
  return tid;
}

uint32_t SUMOMobilityModel::InitSUMOMobilituModel(const std::string& sim_filename)
{
  libtraci::Simulation::start(
      {"sumo", "-c",sim_filename, STEP_LENGTH, END_TIME});
  std::cout << "SUMO start success" << std::endl;
  auto ID_num{libtraci::Simulation::getLoadedNumber()};
  std::cout << "Total " << ID_num << " vehicles\n";
  AllVehicleIDInSUMO = LIBSUMO_NAMESPACE::Vehicle::getLoadedIDList();
  return ID_num;
}

void SUMOMobilityModel::SUMOMobilityModelSchedule(int64_t start_time)
{
  libtraci::Simulation::step(1);
  CallSUMOStep();
  Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable>();
  for (auto mob_ptr : AllSUMOMobilityModelObj)
  {
    int64_t rd = static_cast<int64_t>(RV->GetValue(0.0, 100.0));
    Simulator::Schedule(MilliSeconds(start_time+rd), &SUMOMobilityModel::FakeControl, mob_ptr);
    for (auto other_ptr : AllSUMOMobilityModelObj)
    {
      mob_ptr->m_SelfBuffer.insert(std::make_pair(other_ptr->m_id.first, V2X_PACKET()));
      mob_ptr->SavePacket(other_ptr->GeneratePacket());
    }
  }
}


void SUMOMobilityModel::SaveResults(ns3::Ptr<ns3::OutputStreamWrapper> log_stream)
{
  *log_stream -> GetStream() << "------------------------------ \n";
  *log_stream -> GetStream() << "AoIOverRate: " << std::endl;
  *log_stream -> GetStream() << "------------------------------ \n";
  for (auto dis_ptr = AoIRecord.begin(); dis_ptr != AoIRecord.end(); dis_ptr++)
  {
    std::for_each((*dis_ptr).begin(), (*dis_ptr).end(), [&](int64_t i){
      *log_stream -> GetStream() << i << " ";
    });
    *log_stream -> GetStream() << "\n";
  }
  for (auto dis_ptr = AoIRecord.begin(); dis_ptr != AoIRecord.end(); dis_ptr++)
  {
    std::for_each((*dis_ptr).begin()+1, (*dis_ptr).end(), [&](int64_t i){
      *log_stream -> GetStream() << static_cast<double>(i) / (*(*dis_ptr).begin()) << " ";
    });
    *log_stream -> GetStream() << "\n";
  }
  *log_stream -> GetStream() << "------------------------------ \n";
  *log_stream -> GetStream() << "PositionError: " << std::endl;
  *log_stream -> GetStream() << "------------------------------ \n";
  for (auto dis_ptr = PosRecord.begin(); dis_ptr != PosRecord.end(); dis_ptr++)
  {
    std::for_each((*dis_ptr).begin(), (*dis_ptr).end(), [&](int64_t i){
      *log_stream -> GetStream() << i << " ";
    });
    *log_stream -> GetStream() << "\n";
  }
  for (auto dis_ptr = PosRecord.begin(); dis_ptr != PosRecord.end(); dis_ptr++)
  {
    std::for_each((*dis_ptr).begin()+1, (*dis_ptr).end(), [&](int64_t i){
      *log_stream -> GetStream() << static_cast<double>(i) / (*(*dis_ptr).begin()) << " ";
    });
    *log_stream -> GetStream() << "\n";
  }
}

SUMOMobilityModel::~SUMOMobilityModel()
{
  auto it = std::find(AllSUMOMobilityModelObj.begin(), AllSUMOMobilityModelObj.end(), this);
  AllSUMOMobilityModelObj.erase(it);
}


void SUMOMobilityModel::BindToSUMO(uint32_t id)
{
  static std::vector<std::string>::iterator _itr = AllVehicleIDInSUMO.begin();
  m_id.first = id;
  m_id.second = *_itr;
  AllVehcileMap.insert(m_id);
  AllVehiclePosition.insert(std::make_pair(id, Vector(0.0, 0.0, 0.0)));
  AllSUMOMobilityModelObj.insert(AllSUMOMobilityModelObj.begin(), this);
  _itr++;
}

SUMOMobilityModel::V2X_PACKET SUMOMobilityModel::GeneratePacket() const
{
  auto pos = AllVehiclePosition.find(m_id.first)->second;
  auto time = Simulator::Now().GetMilliSeconds();
  return {m_id.first, time, pos.x, pos.y, pos.z};
}

void SUMOMobilityModel::SavePacket(const V2X_PACKET pkt)
{
  uint32_t id_sent = pkt.m_ID;
  m_SelfBuffer.find(id_sent)->second = pkt;
}

Vector SUMOMobilityModel::DoGetPosition (void) const
{
  auto pos = AllVehiclePosition.find(m_id.first)->second;
      return {pos.x, pos.y, pos.z};
}

Vector SUMOMobilityModel::DoGetVelocity (void) const
{
    return {0.0, 0.0, 0.0};
}

void 
SUMOMobilityModel::CalculateError()
{
    int64_t aoi, current_time {Simulator::Now().GetMilliSeconds()};
    Vector3D other_position, self_position {AllVehiclePosition.find(m_id.first)->second};
    double poserr, distance;
    for (auto itr : AllVehiclePosition)
    {
      if (m_id.first == itr.first)
        continue;
      
      other_position = itr.second;
      distance = sqrt(pow(self_position.x-other_position.x, 2) + pow(self_position.y-other_position.y, 2));
      if (distance >= DISSTEP * DISLEN)
        continue;

      const V2X_PACKET& v2x_pkt = m_SelfBuffer.find(itr.first)->second;

      aoi = current_time - v2x_pkt.m_time;
      int AoIIndex = static_cast<int>(aoi/AOISTEP);
      AoIIndex = (AoIIndex <= AOIRECORDLEN) ? AoIIndex : AOIRECORDLEN;

      poserr = sqrt(
        pow(other_position.x-v2x_pkt.m_x, 2) + 
        pow(other_position.y-v2x_pkt.m_y, 2)
      );
      int PosIndex = static_cast<int>(poserr/POSERRSTEP);
      PosIndex = (PosIndex <= POSRECORDLEN) ? PosIndex : POSRECORDLEN;

      int DisIndex = static_cast<int>(distance/DISSTEP);
      DisIndex = (DisIndex <= DISLEN) ? DisIndex : DISLEN;

      for(int i = DisIndex; i < DISLEN; i++)
      {
        PosRecord[i][0]++, AoIRecord[i][0]++;
        for (int j = 1; j <= (PosIndex-1); j++)
          PosRecord[i][j]++;
        for (int j = 1; j <= (AoIIndex-1); j++)
          AoIRecord[i][j]++;
      }
    }
}

void SUMOMobilityModel::FakeControl()
{
  CalculateError();
  Simulator::Schedule(MilliSeconds(100), &SUMOMobilityModel::FakeControl, this);
}

void SUMOMobilityModel::CallSUMOStep()
{
  libtraci::Simulation::step();
  std::for_each(AllVehcileMap.begin(), AllVehcileMap.end(), [](const std::pair<uint32_t, std::string>& _pair){
    auto pos = LIBSUMO_NAMESPACE::Vehicle::getPosition(_pair.second);
    AllVehiclePosition.find(_pair.first)->second = {pos.x, pos.y, 0.0};
  });
  Simulator::Schedule(MilliSeconds(1), &SUMOMobilityModel::CallSUMOStep);
}


}