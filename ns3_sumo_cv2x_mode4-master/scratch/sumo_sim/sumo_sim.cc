#include "ns3/mobility-model.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/internet-module.h"
#include "ns3/random-direction-2d-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/command-line.h"
#include "sumo-mobility-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/position-allocator.h"
#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <sstream>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SUMO_SIM");

// Global variables
std::vector<std::pair<uint64_t, uint64_t>> TX_RX {DISLEN, std::pair<uint64_t, uint64_t> {0, 0}};


int64_t start_time {5000};
int lenCam;

void 
PrintStatus (uint32_t s_period)
{
    for (auto _pair: TX_RX)
    {
        std::cout << "t=" <<  Simulator::Now().GetSeconds();
        std::cout << "\t Rx/Tx="<< _pair.first << "/" << _pair.second;
        std::cout << "\t PRR=" << static_cast<double>(_pair.first)/_pair.second << std::endl;
    }
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period);
}


void 
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr<Node> node = socket->GetNode();
    Ptr<SUMOMobilityModel> posMobility = node->GetObject<SUMOMobilityModel>();
    SUMOMobilityModel::V2X_PACKET v2x_pkt = posMobility->GeneratePacket();
    Ptr<Packet> pkt = Create<Packet>((uint8_t*)(&v2x_pkt), lenCam);
    socket->Send(pkt);

    int64_t current_time = Simulator::Now().GetMilliSeconds(); 
    if (current_time > start_time)
    {
        const std::map<uint32_t, Vector>& allvehpos {SUMOMobilityModel::GetAllVehiclePosition()};
        double distance;
        for (auto vehpos : allvehpos)
        {
            if (node->GetId() == vehpos.first)
                continue;
            
            distance = sqrt(pow(v2x_pkt.m_x-vehpos.second.x, 2) + 
            pow(v2x_pkt.m_y-vehpos.second.y, 2));
            int DisIndex = static_cast<int>(distance/DISSTEP);
            DisIndex = (DisIndex <= DISLEN) ? DisIndex : DISLEN;

            std::for_each(TX_RX.begin()+DisIndex, TX_RX.end(), [](std::pair<uint64_t, uint64_t>& tx_rx){tx_rx.second++;});
        }
            
    }
}


static void
ReceivePacket(Ptr<Socket> socket)
{   
    Ptr<Node> node = socket->GetNode();
    Ptr<SUMOMobilityModel> posMobility = node->GetObject<SUMOMobilityModel>();
    Vector posRx = posMobility->GetPosition();


    // SendMsg msg;
    SUMOMobilityModel::V2X_PACKET v2v_pkt;
    Ptr<Packet> packet = socket->Recv (); 
    packet->CopyData((uint8_t*)&v2v_pkt, sizeof(SUMOMobilityModel::V2X_PACKET));
    posMobility->SavePacket(v2v_pkt);

    int64_t current_time = Simulator::Now().GetMilliSeconds(); 
    if (current_time > start_time)
    {
        double distance = sqrt(pow((v2v_pkt.m_x - posRx.x),2.0)+pow((v2v_pkt.m_y - posRx.y), 2.0));
        int DisIndex = static_cast<int>(distance/DISSTEP);
        DisIndex = (DisIndex <= DISLEN) ? DisIndex : DISLEN;
        std::for_each(TX_RX.begin()+DisIndex, TX_RX.end(), [](std::pair<uint64_t, uint64_t>& tx_rx){
            tx_rx.first++;
            tx_rx.first = (tx_rx.first > tx_rx.second) ? tx_rx.second : tx_rx.first;
            });
    }
}

int 
main (int argc, char *argv[])
{

    std::clock_t start, end;
    start = std::clock();

    LogComponentEnable ("SUMO_SIM", LOG_INFO);





    // Initialize some values
    // NOTE: commandline parser is currently (05.04.2019) not working for uint8_t (Bug 2916)
    int run = 1;
    uint16_t simTime = 30;                 // Simulation time in seconds
    lenCam = 190;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled 
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint16_t numSubchannel = 3;             // Number of subchannels per subframe
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint16_t pRsvp = 20;				    // Resource reservation interval 
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 20;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    uint32_t VehicleNum;
    NodeContainer VehicleContainer;

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("run", "run value of simulation", run);
    cmd.AddValue ("VehicleNum", "Number of Vehicles", VehicleNum);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch); 
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel); 
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    cmd.AddValue ("lenCam", "Packetsize in Bytes", lenCam);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp); 
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep); 
    cmd.Parse (argc, argv);
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(run);


    // rx_data = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + rx_data;
    // log_rx_data = ascii.CreateFileStream(rx_data);


    NS_LOG_INFO ("Starting network configuration..."); 

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));
    slBandwidth = adjacencyPscchPssch ? (sizeSubchannel * numSubchannel) : ((sizeSubchannel+2) * numSubchannel);
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));


    ConfigStore inputConfig; 
    inputConfig.ConfigureDefaults(); 

    // Create Node and install SUMOMobilityModel
    NS_LOG_INFO ("Installing Mobility Model...");
    VehicleNum = SUMOMobilityModel::InitSUMOMobilituModel("traffic_files/sim.sumocfg");
    VehicleContainer.Create(VehicleNum);
    MobilityHelper SUMOMMH;
    SUMOMMH.SetMobilityModel("ns3::SUMOMobilityModel");
    SUMOMMH.Install(VehicleContainer);
    for (auto nodeptr = VehicleContainer.Begin(); nodeptr != VehicleContainer.End(); nodeptr++)
    {
        auto mmptr {(*nodeptr)->GetObject<SUMOMobilityModel>()};
        mmptr->BindToSUMO((*nodeptr)->GetId());
    }
    SUMOMobilityModel::SUMOMobilityModelSchedule(start_time);


    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");
    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
    // V2X Helper
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper); 

    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1); 
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>(); 
    pos_eNB->Add(Vector(5,-10,30));
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    lteHelper->InstallEnbDevice(eNodeB);

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (VehicleContainer);
    BuildingsHelper::MakeMobilityModelConsistent (); 

    // Install LTE devices to all UEs 
    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer allDevs = lteHelper->InstallUeDevice (VehicleContainer);

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack..."); 
    InternetStackHelper internet;
    internet.Install (VehicleContainer); 

    // Assign IP adress to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface; 
    ueIpIface = epcHelper->AssignUeIpv4Address (allDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(auto nodeptr = VehicleContainer.Begin(); nodeptr != VehicleContainer.End(); nodeptr++)
    {
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting((*nodeptr)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    NS_LOG_INFO("Attaching UE's to LTE network...");
    //Attach each UE to the best available eNB
    lteHelper->Attach(allDevs); 

    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;
    txGroups = lteV2xHelper->AssociateForV2xBroadcast(allDevs, VehicleNum); 
    lteV2xHelper->PrintGroups(txGroups); 

    NS_LOG_INFO ("Installing applications...");
    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses; 
    uint32_t groupL2Address = 0x00; 
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
    uint16_t application_port = 8000; // Application port to TX/RX

    for(std::vector<NetDeviceContainer>::iterator gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
    {
        NetDeviceContainer txUe ((*gIt).Get(0));
        NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));

        Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
        lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
        tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
        lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

        //Individual Socket Traffic Broadcast everyone
        Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        host->Bind();
        host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
        host->SetAllowBroadcast(true);
        host->ShutdownRecv();

        Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
        ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));


        Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
        sink->SetRecvCallback (MakeCallback (&ReceivePacket));

        //store and increment addresses
        groupL2Addresses.push_back (groupL2Address);
        groupL2Address++;
        clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
    }

        NS_LOG_INFO ("Creating Sidelink Configuration...");
        Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
        ueSidelinkConfiguration->SetSlEnabled(true);
        ueSidelinkConfiguration->SetV2xEnabled(true);

        LteRrcSap::SlV2xPreconfiguration preconfiguration;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;
        
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

        SlV2xPreconfigPoolFactory pFactory;
        pFactory.SetHaveUeSelectedResourceConfig (true);
        pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
        pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
        pFactory.SetSizeSubchannel (sizeSubchannel);
        pFactory.SetNumSubchannel (numSubchannel);
        pFactory.SetStartRbSubchannel (startRbSubchannel);
        pFactory.SetStartRbPscchPool (0);
        pFactory.SetDataTxP0 (-4);
        pFactory.SetDataTxAlpha (0.9);

        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
        ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration); 

        // Print Configuration
        // *log_rx_data->GetStream() << "RxPackets;RxTime;RxId;TxId;TxTime;xPos;yPos" << std::endl;

        NS_LOG_INFO ("Installing Sidelink Configuration...");
        lteHelper->InstallSidelinkV2xConfiguration (allDevs, ueSidelinkConfiguration);

        NS_LOG_INFO ("Enabling LTE traces...");
        lteHelper->EnableTraces();

        // *log_simtime->GetStream() << "Simtime;TotalRx;TotalTx;PRR" << std::endl; 
        Simulator::Schedule(Seconds(1), &PrintStatus, 1);
        // Simulator::Schedule(Seconds(1.0), &SaveDistance);

        NS_LOG_INFO ("Starting Simulation...");
        Simulator::Stop(MilliSeconds(simTime*1000+40));
        Simulator::Run();
        Simulator::Destroy();

        NS_LOG_INFO("Simulation done.");

        end = std::clock();
        std::cout << (double)(end - start) / CLOCKS_PER_SEC << std::endl;

        std::string average_pos_err = "./simulation_results/standard_protocol_" + std::to_string(VehicleNum) + "_results";
        AsciiTraceHelper ascii;
        Ptr<ns3::OutputStreamWrapper> position_error = ascii.CreateFileStream(average_pos_err);
        SUMOMobilityModel::SaveResults(position_error);

        *position_error -> GetStream() << "------------------------------ \n";
        *position_error -> GetStream() << "PDR: " << std::endl;
        *position_error -> GetStream() << "------------------------------ \n";
        std::for_each(TX_RX.begin(), TX_RX.end(), [&](std::pair<int64_t, int64_t> TRpair){
            *position_error->GetStream() << TRpair.first / static_cast<double>(TRpair.second) << std::endl;
        });

        SUMOMobilityModel::CloseSUMO();
        return 0;  
}   