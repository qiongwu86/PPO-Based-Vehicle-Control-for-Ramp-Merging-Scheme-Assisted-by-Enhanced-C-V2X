#ifndef SUMO_MOBILITY_MODEL_H
#define SUMO_MOBILITY_MODEL_H
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"
#include "ns3/output-stream-wrapper.h"
#include <libsumo/libtraci.h>
#include <map>
#include <cstdint>
#include <memory>
#include <vector>

#define DISSTEP (50.0)
#define DISLEN (4)
#define AOISTEP (50)
#define AOIRECORDLEN (6)
#define POSERRSTEP (1)
#define POSRECORDLEN (6)

namespace ns3 {
class SUMOMobilityModel : public MobilityModel
{
public:
struct V2X_PACKET{
    uint64_t m_ID;
    int64_t m_time;
    double m_x;
    double m_y;
    double m_z;
};
public:
  static TypeId GetTypeId (void);
  static uint32_t InitSUMOMobilituModel(const std::string& sim_filename);
  static void SUMOMobilityModelSchedule(int64_t start_time);
  static const std::map<uint32_t, Vector>& GetAllVehiclePosition() {return AllVehiclePosition;}
  static void SaveResults(ns3::Ptr<ns3::OutputStreamWrapper> log_stream);
  static void CloseSUMO(){libtraci::Simulation::close();}

  SUMOMobilityModel (){}
  virtual ~SUMOMobilityModel ();
  void BindToSUMO(uint32_t id);
  V2X_PACKET GeneratePacket() const;
  void SavePacket(const V2X_PACKET pkt);

private:
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position) {}
  virtual Vector DoGetVelocity (void) const;
  void CalculateError();
  void FakeControl();
  std::pair<uint32_t, std::string> m_id;
  std::map<uint32_t, V2X_PACKET> m_SelfBuffer;

  static std::map<uint32_t, Vector> AllVehiclePosition;
  static std::vector<std::string> AllVehicleIDInSUMO;
  static std::map<uint32_t, std::string> AllVehcileMap;
  static void CallSUMOStep();
  static std::vector<std::vector<int64_t>> AoIRecord;
  static std::vector<std::vector<int64_t>> PosRecord;
  static std::vector<SUMOMobilityModel*> AllSUMOMobilityModelObj;

  //std::map<
};
}
#endif