#include "ns3/simulator.h"
#include "ns3/internet-module.h"
#include "ns3/random-direction-2d-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/command-line.h"
#include "cut-in-mobility-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/position-allocator.h"
#include <cfloat>
#include <sstream>
#include <ctime>
#include <iostream>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("v2x_communication_mode_4");

// Output 
std::string rx_data = "_rxdata.csv";
Ptr<OutputStreamWrapper> log_rx_data;

// Global variables
uint32_t ctr_totRx = 0; 	// Counter for total received packets
uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
uint16_t lenCam;  
double baseline= 150.0;     // Baseline distance in meter (150m for urban, 320m for freeway)

uint32_t ueVehNum;
uint32_t infVehNum;
uint32_t allVehNum;

// Responders users 
NodeContainer ueVeh;
NodeContainer infVeh;
NodeContainer allVeh;


// average AOI
int64_t last_pkt_time[14][14]  {{0}};
int64_t last_comm_time[14][14]  {{0}};
int64_t all_aoi[14][14] = {{0}};
int64_t all_time[14][14] = {{0}};

const uint32_t ERR_ID = 999;


void 
PrintStatus (uint32_t s_period)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx; 
    }
	// *log_simtime->GetStream() << Simulator::Now ().GetSeconds () << ";" << ctr_totRx << ";" << ctr_totTx << ";" << (double) ctr_totRx / ctr_totTx << std::endl; 
    std::cout << "t=" <<  Simulator::Now().GetSeconds() << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx << "\t PRR=" << (double) ctr_totRx / ctr_totTx << std::endl;
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period);
}


void 
SidelinkV2xAnnouncementMacTrace_(Ptr<Socket> socket)
{
    Ptr<Node> node = socket->GetNode();
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posTx = posMobility->GetPosition();

    CutInMobilityModel::V2V_PACKET pkt = {posTx.x, posTx.y, 0.0, 0.0, ERR_ID, 0};
    Ptr<Packet> packet = Create<Packet>((uint8_t*)(&pkt), lenCam);
    socket->Send(packet);

    // check for each UE distance to transmitter
    int64_t rxTime = Simulator::Now().GetMilliSeconds(); 
    if (rxTime > 5000)
    {
        for (NodeContainer::Iterator nodeIt = allVeh.Begin(); nodeIt != allVeh.End(); nodeIt++)
        {
            Ptr<MobilityModel> mob = (*nodeIt)->GetObject<MobilityModel>(); 
            Vector posRx = mob->GetPosition();
            
            double distance = sqrt(pow((posTx.x - posRx.x),2.0)
                                +pow((posTx.y - posRx.y),2.0));
            if  ((distance > 0) && (distance <= baseline))
                ctr_totTx++;
        }
    }
}



void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode(); 
    Ptr<CutInMobilityModel> posMobility = node->GetObject<CutInMobilityModel>();
    Vector posTx = posMobility->GetPosition();

    CutInMobilityModel::V2V_PACKET v2v_pkt = posMobility->DoGenerateV2VPacket();
    Ptr<Packet> packet = Create<Packet>((uint8_t*)(&v2v_pkt), lenCam);
    socket->Send(packet);

    // check for each UE distance to transmitter
    int64_t rxTime = Simulator::Now().GetMilliSeconds(); 
    if (rxTime > 5000)
    {
        for (NodeContainer::Iterator nodeIt = allVeh.Begin(); nodeIt != allVeh.End(); nodeIt++)
        {
            Ptr<MobilityModel> mob = (*nodeIt)->GetObject<MobilityModel>(); 
            Vector posRx = mob->GetPosition();
            
            double distance = sqrt(pow((posTx.x - posRx.x),2.0)
                                +pow((posTx.y - posRx.y),2.0));
            if  ((distance > 0) && (distance <= baseline))
            {
                ctr_totTx++;
            }
        }
    }
}


static void 
ReceivePacket_(Ptr<Socket> socket)
{
    // socket->Recv ();
    Ptr<Node> node = socket->GetNode();
    int64_t rxTime = Simulator::Now().GetMilliSeconds();    

    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();

    // SendMsg msg;
    CutInMobilityModel::V2V_PACKET v2v_pkt;
    Ptr<Packet> packet = socket->Recv (); 
    packet->CopyData((uint8_t*)&v2v_pkt, sizeof(CutInMobilityModel::V2V_PACKET));

    double distance = sqrt(pow((v2v_pkt.m_x - posRx.x),2.0)+pow((v2v_pkt.m_y - posRx.y), 2.0));
    if ((distance <= baseline) && (rxTime > 5000))
        ctr_totRx++; 
}

static void
ReceivePacket(Ptr<Socket> socket)
{   
    Ptr<Node> node = socket->GetNode();
    uint32_t rxID = node->GetId();
    int64_t rxTime = Simulator::Now().GetMilliSeconds();
    Ptr<CutInMobilityModel> posMobility = node->GetObject<CutInMobilityModel>();
    Vector posRx = posMobility->GetPosition();


    // SendMsg msg;
    CutInMobilityModel::V2V_PACKET v2v_pkt;
    Ptr<Packet> packet = socket->Recv (); 
    packet->CopyData((uint8_t*)&v2v_pkt, sizeof(CutInMobilityModel::V2V_PACKET));
    uint32_t txID = v2v_pkt.m_id;
    if (txID != ERR_ID)
        posMobility->DoPassV2VPacket(v2v_pkt);

    double distance = sqrt(pow((v2v_pkt.m_x - posRx.x),2.0)+pow((v2v_pkt.m_y - posRx.y), 2.0));
    if ((distance <= baseline) && (rxTime > 5000))     
        ctr_totRx++; 
    
    if ((rxTime >= 5000) && (txID != ERR_ID))
    {
        int64_t head = last_comm_time[txID][rxID] - last_pkt_time[txID][rxID];
        int64_t height = rxTime - last_comm_time[txID][rxID];
        int64_t bottom = head + height;

        all_aoi[txID][rxID] += (head + bottom) * height / 2;
        all_time[txID][rxID] += height;

        last_comm_time[txID][rxID] = rxTime;
        last_pkt_time[txID][rxID] = v2v_pkt.m_time_stamp;
    }
}

int 
main (int argc, char *argv[])
{

    std::clock_t start, end;
    start = std::clock();

    LogComponentEnable ("v2x_communication_mode_4", LOG_INFO);





    // Initialize some values
    // NOTE: commandline parser is currently (05.04.2019) not working for uint8_t (Bug 2916)
    int run = 1;
    uint16_t simTime = 40;                 // Simulation time in seconds
    ueVehNum = 0;                  // Number of vehicles
    infVehNum = 40;
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

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("run", "run value of simulation", run);

    cmd.AddValue ("ueVehNum", "Number of Vehicles", ueVehNum);
    cmd.AddValue ("infVehNum", "Number of Inf Vehicles", infVehNum);

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
    cmd.AddValue ("log_rx_data", "name of the rx data logfile", rx_data);
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);
    cmd.Parse (argc, argv);
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(run);


    AsciiTraceHelper ascii;
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

    NS_LOG_INFO ("Installing Mobility Model...");
    std::list<Vector> main_init_pos = CutInMobilityModel::GetInitPosition(CutInMobilityModel::BelongTo::MAIN_ROAD);
    uint32_t main_veh_num = main_init_pos.size();

    std::list<Vector> merge_init_pos = CutInMobilityModel::GetInitPosition(CutInMobilityModel::BelongTo::MERGE_ROAD);
    uint32_t merge_veh_num = merge_init_pos.size();

    ueVehNum = main_veh_num + merge_veh_num;
    allVehNum = ueVehNum + infVehNum;

    ueVeh.Create(ueVehNum);
    infVeh.Create (infVehNum);

    allVeh.Add(ueVeh);
    allVeh.Add(infVeh);

    for (uint32_t i = 0; i < ueVehNum; i++)
    {
        for (uint32_t j = 0; j < ueVehNum; j++)
        {
            last_comm_time[i][j] = 5000;
            last_pkt_time[i][j] = 5000;
        }
    }

    // Install constant random positions 
    // ue veh
    Ptr<ListPositionAllocator> LPA = CreateObject<ListPositionAllocator>();
    MobilityHelper CutInMMH;
    CutInMMH.SetMobilityModel("ns3::CutInMobilityModel"); 
    CutInMMH.SetPositionAllocator(LPA);

    for (uint32_t i = 0; i < ueVehNum; i++)
        LPA->Add(Vector(0.0, 0.0, 0.0));
    CutInMMH.Install(ueVeh);

    std::list<Vector>::iterator posItr=main_init_pos.begin();
    for (uint32_t i = 0; i < main_veh_num; i++, posItr++)
        ueVeh.Get(i)->GetObject<CutInMobilityModel>()->
        InitAndScheduleToStart(i, 5000, *posItr, CutInMobilityModel::BelongTo::MAIN_ROAD);

    posItr = merge_init_pos.begin();
    for (uint32_t i = main_veh_num; i < main_veh_num + merge_veh_num; i++, posItr++)
        ueVeh.Get(i)->GetObject<CutInMobilityModel>()->
        InitAndScheduleToStart(i, 5000, *posItr, CutInMobilityModel::BelongTo::MERGE_ROAD);

    CutInMobilityModel::InitServer();

    // inf veh
    Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    MobilityHelper RD2DMMH; 

    Ptr<ns3::RandomRectanglePositionAllocator> RRPLA = CreateObject<RandomRectanglePositionAllocator>();
    RRPLA->SetAttribute("X", StringValue ("ns3::UniformRandomVariable[Min=-575.0|Max=-175.0]"));
    RRPLA->SetAttribute("Y", StringValue ("ns3::UniformRandomVariable[Min=-55.0|Max=5.0]"));

    RD2DMMH.SetMobilityModel("ns3::RandomDirection2dMobilityModel", "Bounds", RectangleValue (Rectangle (-575.0, -175.0, -55.0, 5.0)));
    RD2DMMH.SetPositionAllocator(RRPLA);
    RD2DMMH.Install(infVeh);


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
    BuildingsHelper::Install (allVeh);
    BuildingsHelper::MakeMobilityModelConsistent (); 

    // Install LTE devices to all UEs 
    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer allDevs = lteHelper->InstallUeDevice (allVeh);

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack..."); 
    InternetStackHelper internet;
    internet.Install (allVeh); 

    // Assign IP adress to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface; 
    ueIpIface = epcHelper->AssignUeIpv4Address (allDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(uint32_t u = 0; u < allVeh.GetN(); ++u)
    {
        Ptr<Node> ueNode = allVeh.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    NS_LOG_INFO("Attaching UE's to LTE network...");
    //Attach each UE to the best available eNB
    lteHelper->Attach(allDevs); 

    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;
    txGroups = lteV2xHelper->AssociateForV2xBroadcast(allDevs, allVehNum); 
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

        uint32_t vehID = txUe.Get(0)->GetNode()->GetId();

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
        if (vehID < ueVehNum)
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));
        else
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace_, host));

        Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
        if (vehID < ueVehNum)
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));
        else
            sink->SetRecvCallback (MakeCallback (&ReceivePacket_));

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


        std::string average_pos_err = "./simulation_results/" + std::to_string(infVehNum) + "_" + std::to_string(run) + "_position_error";
        Ptr<ns3::OutputStreamWrapper> position_error = ascii.CreateFileStream(average_pos_err);

        std::string average_AoI_over = "./simulation_results/" + std::to_string(infVehNum) + "_" + std::to_string(run) + "_AoI_over";
        Ptr<ns3::OutputStreamWrapper> AoI_over = ascii.CreateFileStream(average_AoI_over);

        CutInMobilityModel::PrintAoIOverRate(AoI_over);
        CutInMobilityModel::PrintPositionErrorRate(position_error);

        double temp_ave_aoi = 0.0;
        double ave_aoi_num = 0.0;
        for (uint32_t i = 0; i < ueVehNum; i++)
        {
            for (uint32_t j = 0; j < ueVehNum; j++)
            {
                if (all_aoi[i][j] != 0)
                {
                    temp_ave_aoi += ((double)all_aoi[i][j]) / ((double)all_time[i][j]);
                    ave_aoi_num += 1;
                }
            }
        }

        *AoI_over->GetStream() << "Average AOI = " << temp_ave_aoi/ave_aoi_num << std::endl;
        std::cout << ave_aoi_num << std::endl;

        return 0;  
}   