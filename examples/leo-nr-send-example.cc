#include <fstream>
#include <iostream>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <map>
#include <sstream>

#include "ns3/leo-module.h"
#include "ns3/leo-ground-node-helper.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/nr-pdcp-header.h" // Include the PDCP header definition
#include "ns3/packet.h"

using namespace ns3;

static std::ofstream traceFileOutputStream;
static bool logging = true; // whether to enable logging from the simulation, another option is by
                         // exporting the NS_LOG environment variable

// Map to store transmission times for delay calculation
static std::map<uint32_t, Time> packetTxTimeMap;


// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

NS_LOG_COMPONENT_DEFINE ("LeoNrSendExample");

void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  auto mobility = DynamicCast<const GeocentricConstantPositionMobilityModel> (position);
  if (mobility)
    {
      auto pos = mobility->GetGeocentricPosition();
      auto geo = mobility->GetGeographicPosition();
      Ptr<const Node> node = position->GetObject<Node>();
      traceFileOutputStream << Simulator::Now () << "," << node->GetId () << "," << pos.x << "," << pos.y << "," << pos.z << "," << mobility->GetVelocity() << "," << geo.x << "," << geo.y << "," << geo.z << std::endl;
    }
}

void PacketSinkRxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t packetId = packet->GetUid();
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            std::string nodeId = context.substr(nodePos + 10, endPos - nodePos - 10);
            nodeInfo = " [Car Node" + nodeId + "]";
        }
    }
    
    // Calculate delay if we have the transmission time
    std::string delayStr = "N/A";
    if (packetTxTimeMap.find(packetId) != packetTxTimeMap.end()) {
        Time delay = currentTime - packetTxTimeMap[packetId];
        delayStr = std::to_string(delay.GetMilliSeconds()) + "ms";
        // Remove from map to save memory
        packetTxTimeMap.erase(packetId);
    }
    
    std::cout << "[" << currentTime.GetSeconds() << "s] UDP RX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "Delay=" << delayStr << ", "
              << "Context=" << context << std::endl;
}

void UdpClientTxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t packetId = packet->GetUid();
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            std::string nodeId = context.substr(nodePos + 10, endPos - nodePos - 10);
            nodeInfo = " [RemoteHost Node" + nodeId + "]";
        }
    }
    
    // Store transmission time for delay calculation
    packetTxTimeMap[packetId] = currentTime;
    
    std::cout << "[" << currentTime.GetSeconds() << "s] UDP TX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "PacketId=" << packetId << ", "
              << "Context=" << context << std::endl;
}



void
TxDataTrace (std::string context, Ptr<const Packet> p, const ns3::Address& addr)
{
    // Determine if this is gNB or UE based on context
    std::string deviceType = "5G";
    if (context.find("NrGnbNetDevice") != std::string::npos) {
        deviceType = "gNB (Satellite)";
    } else if (context.find("NrUeNetDevice") != std::string::npos) {
        deviceType = "UE (Car)";
    }
    
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] " << deviceType << " TX: " 
          << "Size=" << p->GetSize() << " bytes, "
          << "Destination=" << addr << ", "
          << "Context=" << context << std::endl;
}

void
RxDataTrace (std::string context, Ptr<const Packet> p)
{
    // Determine if this is gNB or UE based on context
    std::string deviceType = "5G";
    if (context.find("NrGnbNetDevice") != std::string::npos) {
        deviceType = "gNB (Satellite)";
    } else if (context.find("NrUeNetDevice") != std::string::npos) {
        deviceType = "UE (Car)";
    }
    
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] " << deviceType << " RX: " 
          << "Size=" << p->GetSize() << " bytes, "
          << "Context=" << context << std::endl;
}

void PointToPointTxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID from context for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            nodeInfo = " Node" + context.substr(nodePos + 10, endPos - nodePos - 10);
        }
    }
    
    std::cout << "[" << currentTime.GetSeconds() << "s] P2P TX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "Context=" << context << std::endl;
}

void PointToPointRxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID from context for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            nodeInfo = " Node" + context.substr(nodePos + 10, endPos - nodePos - 10);
        }
    }
    
    std::cout << "[" << currentTime.GetSeconds() << "s] P2P RX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "Context=" << context << std::endl;
}

// TCP trace functions
void TcpSinkRxTrace(std::string context, Ptr<const Packet> packet, const Address &from)
{
    uint32_t packetId = packet->GetUid();
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            std::string nodeId = context.substr(nodePos + 10, endPos - nodePos - 10);
            nodeInfo = " [Node" + nodeId + "]";
        }
    }
    
    // Calculate delay if we have the transmission time
    std::string delayStr = "N/A";
    if (packetTxTimeMap.find(packetId) != packetTxTimeMap.end()) {
        Time delay = currentTime - packetTxTimeMap[packetId];
        delayStr = std::to_string(delay.GetMilliSeconds()) + "ms";
        // Remove from map to save memory
        packetTxTimeMap.erase(packetId);
    }
    
    std::cout << "[" << currentTime.GetSeconds() << "s] TCP RX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "Delay=" << delayStr << ", "
              << "From=" << from << ", "
              << "Context=" << context << std::endl;
}

void BulkSendTxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t packetId = packet->GetUid();
    uint32_t packetSize = packet->GetSize();
    Time currentTime = Simulator::Now();
    
    // Extract node ID for better identification
    std::string nodeInfo = "";
    size_t nodePos = context.find("/NodeList/");
    if (nodePos != std::string::npos) {
        size_t endPos = context.find("/", nodePos + 10);
        if (endPos != std::string::npos) {
            std::string nodeId = context.substr(nodePos + 10, endPos - nodePos - 10);
            nodeInfo = " [Node" + nodeId + "]";
        }
    }
    
    // Store transmission time for delay calculation
    packetTxTimeMap[packetId] = currentTime;
    
    std::cout << "[" << currentTime.GetSeconds() << "s] TCP TX" << nodeInfo << ": " 
              << "Size=" << packetSize << " bytes, "
              << "PacketId=" << packetId << ", "
              << "Context=" << context << std::endl;
}


int main(int argc, char *argv[])
{

    std::string scenario = "NTN-Suburban"; // scenario
    double frequency = 28e9;      // central frequency
    double bandwidth = 400e6;     // bandwidth

    double carSpeed = 30.0;  // m/s
    double carLatitude = 19.5;
    double carLongitude = 1.5;
    double txPower = 40; // txPower
    // Satellite parameters
    double satAntennaGainDb = 58.5; // dB
    // UE Parameters
    double ueAntennaGainDb = 39.7; // dB
    int numCars = 1; // number of cars in the simulation
    std::string duration = "420ms";
    std::string traceFile = "";
    std::string antennaModel = "IsotropicAntennaModel"; // Antenna model
    std::string satMode = "single"; // Satellite mode, either single or multiple
    Time mobilityPrecision = MilliSeconds(1000); // Precision for mobility updates
    bool enableGnb = true; // Whether to enable gNB transmission
    std::string appType = "UDP"; // Application type: UDP or TCP
    bool bidirectional = false; // Whether to enable bidirectional traffic
    uint32_t packetSize = 1024; // Packet size in bytes
    uint32_t maxPackets = 10; // Maximum number of packets
    Time packetInterval = MilliSeconds(10); // Interval between packets
    auto rnd_seed = time(nullptr);
 
    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario",
                 "The scenario for the simulation. Valid options are: "
                 "NTN-DenseUrban, NTN-Urban, NTN-Suburban, and NTN-Rural",
                 scenario);
    cmd.AddValue("frequency", "The central carrier frequency in Hz.", frequency);
    cmd.AddValue("bandwidth", "The total bandwidth in Hz.", bandwidth);
    cmd.AddValue("carSpeed", "Speed of the car in m/s", carSpeed);
    cmd.AddValue("carLatitude", "Initial latitude of the car (only with 1 car)", carLatitude);
    cmd.AddValue("carLongitude", "Initial longitude of the car (only with 1 car)", carLongitude);
    cmd.AddValue("numCars", "Number of cars in the simulation", numCars);
    cmd.AddValue("antennaModel", "The antenna model to use for the simulation. "
                 "Valid options are: IsotropicAntennaModel, CircularApertureAntennaModel, ParabolicAntennaModel, "
                 "ThreeGppAntennaModel, and CosineAntennaModel",
                 antennaModel);
    cmd.AddValue("satMode", "The satellite mode to use for the simulation. 'single', 'multiple', 'single-dislocated", satMode);
    cmd.AddValue("satAntennaGainDb", "The satellite antenna gain in dB", satAntennaGainDb);
    cmd.AddValue("ueAntennaGainDb", "The UE antenna gain in dB", ueAntennaGainDb);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("logging", "If set to 0, log components will be disabled.", logging);
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("txPower", "Transmission power in dBm", txPower);
    cmd.AddValue("seed", "Random seed for the simulation (default: current time)", rnd_seed);
    cmd.AddValue("mobilityPrecision",
                "Precision for mobility updates (e.g., 50ms, 100ms, etc.)",
                mobilityPrecision);
    cmd.AddValue("enableGnb", "Enable gNB transmission (1) or disable (0)", enableGnb);
    cmd.AddValue("appType", "Application type: UDP or TCP", appType);
    cmd.AddValue("bidirectional", "Enable bidirectional traffic (1) or disable (0)", bidirectional);
    cmd.AddValue("packetSize", "Packet size in bytes", packetSize);
    cmd.AddValue("maxPackets", "Maximum number of packets to send", maxPackets);
    cmd.AddValue("packetInterval", "Interval between packets (e.g., 10ms, 100ms)", packetInterval);
    cmd.Parse(argc, argv);

  srand(rnd_seed);

  Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                        TimeValue(MilliSeconds(100))); // update the channel at every 10 ms
  Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                        TimeValue(MilliSeconds(0))); // do not update the channel condition

  LeoOrbitNodeHelper orbit;
  
  orbit.SetPrecision(mobilityPrecision); // Set precision for position updates
  // Create and configure satellites using LEO orbit helper
  NodeContainer satellites;
  if (satMode == "multiple") {
    satellites = orbit.Install(LeoOrbit(300, 20, 10, 10));
  } else if (satMode == "single-dislocated") {
    satellites = orbit.Install (300, 20, 90, 180);
  }else if (satMode == "single") {
    satellites = orbit.Install (300, 20, 0, 0);
  } else {
    NS_FATAL_ERROR("Invalid satellite mode: " << satMode);
  }
  // Create ground nodes (cars)
  NodeContainer cars;
  if (numCars > 1) {
    cars.Create(numCars); // Create multiple cars
    for (int i = 0; i < cars.GetN(); ++i){
        auto car = cars.Get(i);
        auto rndLat = (rand()%180000)/1000.0 - 90.0; // Random latitude
        auto rndLon = (rand()%360000)/1000.0 - 180.0; // Random longitude
        auto rndAzimuth = (rand()%360000)/1000.0; // Random azimuth

        // Install GndConstantVelocityMobilityModel on cars
        Ptr<GndConstantVelocityMobilityModel> mobilityModel = CreateObject<GndConstantVelocityMobilityModel>();
        mobilityModel->SetAttribute("InitialLatitude", DoubleValue(rndLat));
        mobilityModel->SetAttribute("InitialLongitude", DoubleValue(rndLon));
        mobilityModel->SetAttribute("Altitude", DoubleValue(2));
        mobilityModel->SetAttribute("Speed", DoubleValue(carSpeed));
        mobilityModel->SetAttribute("Azimuth", DoubleValue(rndAzimuth));
        mobilityModel->SetAttribute("Precision", TimeValue(mobilityPrecision));
        car->AggregateObject(mobilityModel);
    }
  } else {
    cars.Create(1); // Create a single car
    Ptr<GndConstantVelocityMobilityModel> mobilityModel = CreateObject<GndConstantVelocityMobilityModel>();
    mobilityModel->SetAttribute("InitialLatitude", DoubleValue(carLatitude));
    mobilityModel->SetAttribute("InitialLongitude", DoubleValue(carLongitude));
    mobilityModel->SetAttribute("Altitude", DoubleValue(3));
    mobilityModel->SetAttribute("Speed", DoubleValue(carSpeed));
    mobilityModel->SetAttribute("Azimuth", DoubleValue(0));
    mobilityModel->SetAttribute("Precision", TimeValue(mobilityPrecision));
    cars.Get(0)->AggregateObject(mobilityModel);
  }

  if (traceFile != "")
    {
      traceFileOutputStream.open (traceFile);
      if (!traceFileOutputStream.is_open ()){
        NS_FATAL_ERROR ("Could not open trace file " << traceFile);
      }
      Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                       MakeCallback (&CourseChange));
      traceFileOutputStream << "Time,Node,X,Y,Z,Speed,Latitude,Longitude,Altitude" << std::endl;
    }

    /*
     * Default values for the simulation. We are progressively removing all
     * the instances of SetDefault, but we need it for legacy code (LTE)
     */
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    
    // Configure NR MAC scheduler parameters
    Config::SetDefault("ns3::NrMacSchedulerNs3::DlCtrlSymbols", UintegerValue(1));
    Config::SetDefault("ns3::NrMacSchedulerNs3::UlCtrlSymbols", UintegerValue(1));
    
    /*
     * Create NR simulation helpers
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
 
    /*
     * Spectrum configuration. We create a single operational band and configure the scenario.
     */
 
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; // in this example we have a single band, and that band is
                                    // composed of a single component carrier
 
    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
     * a single BWP per CC and a single BWP in CC.
     *
     * Hence, the configured spectrum is:
     *
     * |---------------Band---------------|
     * |---------------CC-----------------|
     * |---------------BWP----------------|
     */
    CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    // Create the channel helper
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    // Set and configure the channel to the current band
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});
 
    // Configure ideal beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
 
    // Configure scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    if (antennaModel == "IsotropicAntennaModel")
    {
        // Antennas for the UEs
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                            "Gain", DoubleValue(ueAntennaGainDb)
                                        )));
    
        // Antennas for the gNbs
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                            "Gain", DoubleValue(satAntennaGainDb)
                                        )));
    }
    else if (antennaModel == "CircularApertureAntennaModel")
    {
        // Antennas for the UEs
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<CircularApertureAntennaModel>(
                                            "AntennaMaxGainDb", DoubleValue(ueAntennaGainDb)
                                        )));
    
        // Antennas for the gNbs
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<CircularApertureAntennaModel>(
                                            "AntennaMaxGainDb", DoubleValue(satAntennaGainDb)
                                        )));
    }
    else if (antennaModel == "ParabolicAntennaModel")
    {
        // Antennas for the UEs
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<ParabolicAntennaModel>()));
    
        // Antennas for the gNbs
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<ParabolicAntennaModel>()));
    }
    else if (antennaModel == "ThreeGppAntennaModel")
    {
        // Antennas for the UEs
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<ThreeGppAntennaModel>()));
    
        // Antennas for the gNbs
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<ThreeGppAntennaModel>()));
    }
    else if (antennaModel == "CosineAntennaModel")
    {
        // Antennas for the UEs
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<CosineAntennaModel>(
                                            "MaxGain", DoubleValue(ueAntennaGainDb)
                                        )));
    
        // Antennas for the gNbs
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObjectWithAttributes<CosineAntennaModel>(
                                            "MaxGain", DoubleValue(satAntennaGainDb)
                                        )));
    }
    else
    {
        NS_FATAL_ERROR("Invalid antenna model specified: " << antennaModel);
    }
 

 
    // install nr net devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(cars, allBwps);
 
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
 
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        // Set gNB transmission power based on enableGnb parameter
        if (enableGnb) {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(txPower);
            std::cout << "gNB enabled with transmission power: " << txPower << " dBm" << std::endl;
        } else {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(-1000); // Effectively disable transmission
            std::cout << "gNB disabled (transmission power set to -1000 dBm)" << std::endl;
        }
    }

 
    InternetStackHelper internet;
    internet.Install(cars);
    internet.Install(satellites);
 
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    
    // Install applications on remote host (satellite) connected to PGW
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    
    // Create the remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    internet.Install(remoteHostContainer);
 
    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
 
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
 
    // assign IP address to UEs, and install applications
    uint16_t dlPortBase = 1234;
    uint16_t ulPortBase = 2234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    ApplicationContainer ulClientApps; // For uplink traffic
    ApplicationContainer ulServerApps; // For uplink traffic
    
    if (appType == "UDP") {
        std::cout << "Installing UDP applications" << std::endl;
        
        for (uint32_t u = 0; u < cars.GetN(); ++u)
        {
            uint16_t dlPort = dlPortBase + u; // Different port for each car
            uint16_t ulPort = ulPortBase + u; // Different port for each car
            
            // Downlink: Remote host -> Car
            UdpServerHelper dlPacketSinkHelper(dlPort);
            serverApps.Add(dlPacketSinkHelper.Install(cars.Get(u)));
     
            UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
            dlClient.SetAttribute("Interval", TimeValue(packetInterval));
            dlClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
            dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));
            clientApps.Add(dlClient.Install(remoteHost)); // Remote host sends to car
            
            if (bidirectional) {
                // Uplink: Car -> Remote host
                UdpServerHelper ulPacketSinkHelper(ulPort);
                ulServerApps.Add(ulPacketSinkHelper.Install(remoteHost));
                
                UdpClientHelper ulClient(internetIpIfaces.GetAddress(1), ulPort); // Remote host IP
                ulClient.SetAttribute("Interval", TimeValue(packetInterval));
                ulClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
                ulClient.SetAttribute("PacketSize", UintegerValue(packetSize));
                ulClientApps.Add(ulClient.Install(cars.Get(u))); // Car sends to remote host
            }
            
            std::cout << "Car " << u << " - DL Port: " << dlPort << ", UL Port: " << ulPort << std::endl;
        }
    }
    else if (appType == "TCP") {
        std::cout << "Installing TCP applications" << std::endl;
        
        for (uint32_t u = 0; u < cars.GetN(); ++u)
        {
            uint16_t dlPort = dlPortBase + u; // Different port for each car
            uint16_t ulPort = ulPortBase + u; // Different port for each car
            
            // Downlink: Remote host -> Car
            PacketSinkHelper dlSinkHelper("ns3::TcpSocketFactory", 
                                          InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlSinkHelper.Install(cars.Get(u)));
            
            BulkSendHelper dlBulkSendHelper("ns3::TcpSocketFactory", 
                                            InetSocketAddress(ueIpIface.GetAddress(u), dlPort));
            dlBulkSendHelper.SetAttribute("MaxBytes", UintegerValue(packetSize * maxPackets));
            dlBulkSendHelper.SetAttribute("SendSize", UintegerValue(packetSize));
            clientApps.Add(dlBulkSendHelper.Install(remoteHost)); // Remote host sends to car
            
            if (bidirectional) {
                // Uplink: Car -> Remote host
                PacketSinkHelper ulSinkHelper("ns3::TcpSocketFactory", 
                                              InetSocketAddress(Ipv4Address::GetAny(), ulPort));
                ulServerApps.Add(ulSinkHelper.Install(remoteHost));
                
                BulkSendHelper ulBulkSendHelper("ns3::TcpSocketFactory", 
                                                InetSocketAddress(internetIpIfaces.GetAddress(1), ulPort));
                ulBulkSendHelper.SetAttribute("MaxBytes", UintegerValue(packetSize * maxPackets));
                ulBulkSendHelper.SetAttribute("SendSize", UintegerValue(packetSize));
                ulClientApps.Add(ulBulkSendHelper.Install(cars.Get(u))); // Car sends to remote host
            }
            
            std::cout << "Car " << u << " - DL Port: " << dlPort << ", UL Port: " << ulPort << std::endl;
        }
    }
    else {
        NS_FATAL_ERROR("Invalid application type: " << appType << ". Must be UDP or TCP.");
    }
 
    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    if (logging)
    {
      // Connect trace sources for packet tracking
      if (appType == "UDP") {
          // Trace UDP application layer
          Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpClient/Tx", MakeCallback(&UdpClientTxTrace));
          Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpServer/Rx", MakeCallback(&PacketSinkRxTrace));
      } else if (appType == "TCP") {
          // Trace TCP application layer
          Config::Connect("/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx", MakeCallback(&BulkSendTxTrace));
          Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback(&TcpSinkRxTrace));
      }
      
      // Trace Point-to-Point devices (backhaul)
      Config::Connect("/NodeList/*/DeviceList/*/$ns3::PointToPointNetDevice/MacTx", 
                      MakeCallback(&PointToPointTxTrace));
      Config::Connect("/NodeList/*/DeviceList/*/$ns3::PointToPointNetDevice/MacRx", 
                      MakeCallback(&PointToPointRxTrace));
      
      // Trace NR devices
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/Tx",
                        MakeCallback (&TxDataTrace));
      // Similarly for the gNB side or reception:
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/Tx",
                        MakeCallback (&TxDataTrace));
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/Rx",
                        MakeCallback (&RxDataTrace));
      // Similarly for the gNB side or reception:
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/Rx",
                        MakeCallback (&RxDataTrace));
    }
 
    // start server and client apps
    serverApps.Start(Seconds(0.1)); // Earlier start
    clientApps.Start(Seconds(0.2)); // Start transmission quickly after attachment
    serverApps.Stop(Time(duration) - Seconds(0.01));
    clientApps.Stop(Time(duration) - Seconds(0.1)); // Reduced stop margin
    
    if (bidirectional) {
        ulServerApps.Start(Seconds(0.1)); // Start uplink servers
        ulClientApps.Start(Seconds(0.3)); // Start uplink clients slightly later
        ulServerApps.Stop(Time(duration) - Seconds(0.01));
        ulClientApps.Stop(Time(duration) - Seconds(0.1));
        std::cout << "Bidirectional traffic enabled (uplink + downlink)" << std::endl;
    } else {
        std::cout << "Unidirectional traffic (downlink only)" << std::endl;
    }
 
    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    Simulator::Schedule(Seconds(0.1), [appType, bidirectional]() {
        std::cout << "============= Starting " << appType << " applications ";
        if (bidirectional) {
            std::cout << "(bidirectional) =============";
        } else {
            std::cout << "(downlink only) =============";
        }
        std::cout << std::endl;
    });


    Simulator::Stop (Time (duration));
    std::cout << "============= Starting simulation for " << duration << " =============" << std::endl;
    
    /*
    Time printInterval = Seconds(5);
    Time simDuration = Time(duration);
    for (Time t = printInterval; t < simDuration; t += printInterval) {
        Simulator::Schedule(t, [t]() {
            std::cout << "Simulation time: " << t.GetSeconds() << "s" << std::endl;
        });
    }
    */
    
    Simulator::Run();

    // Print statistics
    std::cout << "\n============= TRAFFIC STATISTICS =============" << std::endl;
    std::cout << "Application Type: " << appType << std::endl;
    std::cout << "Traffic Mode: " << (bidirectional ? "Bidirectional" : "Downlink only") << std::endl;
    std::cout << "Packet Size: " << packetSize << " bytes" << std::endl;
    std::cout << "Max Packets: " << maxPackets << std::endl;
    std::cout << "Packet Interval: " << packetInterval.GetMilliSeconds() << "ms" << std::endl;
    
    std::cout << "\n--- DOWNLINK STATISTICS (Remote Host -> Car) ---" << std::endl;
    
    if (appType == "UDP") {
        for (uint32_t u = 0; u < serverApps.GetN(); ++u)
        {
            Ptr<UdpServer> serverApp = serverApps.Get(u)->GetObject<UdpServer>();
            auto receivedPackets = serverApp->GetReceived();
            auto totalBytes = receivedPackets * packetSize;
            
            std::cout << "Car Node " << serverApp->GetNode()->GetId() 
                      << " - Packets: " << receivedPackets << "/" << maxPackets
                      << " (" << (100.0 * receivedPackets / maxPackets) << "%), "
                      << "Total Bytes: " << totalBytes 
                      << (receivedPackets == maxPackets ? " ✅" : " ❌") << std::endl;
        }
    } else if (appType == "TCP") {
        for (uint32_t u = 0; u < serverApps.GetN(); ++u)
        {
            Ptr<PacketSink> sinkApp = serverApps.Get(u)->GetObject<PacketSink>();
            auto totalBytes = sinkApp->GetTotalRx();
            
            std::cout << "Car Node " << sinkApp->GetNode()->GetId() 
                      << " - Total Bytes: " << totalBytes
                      << " (" << (100.0 * totalBytes / (packetSize * maxPackets)) << "%) "
                      << (totalBytes >= packetSize * maxPackets ? " ✅" : " ❌") << std::endl;
        }
    }
    
    if (bidirectional) {
        std::cout << "\n--- UPLINK STATISTICS (Car -> Remote Host) ---" << std::endl;
        
        if (appType == "UDP") {
            for (uint32_t u = 0; u < ulServerApps.GetN(); ++u)
            {
                Ptr<UdpServer> ulServerApp = ulServerApps.Get(u)->GetObject<UdpServer>();
                auto receivedPackets = ulServerApp->GetReceived();
                auto totalBytes = receivedPackets * packetSize;
                
                std::cout << "Remote Host - Packets: " << receivedPackets << "/" << maxPackets
                          << " (" << (100.0 * receivedPackets / maxPackets) << "%), "
                          << "Total Bytes: " << totalBytes
                          << (receivedPackets == maxPackets ? " ✅" : " ❌") << std::endl;
            }
        } else if (appType == "TCP") {
            for (uint32_t u = 0; u < ulServerApps.GetN(); ++u)
            {
                Ptr<PacketSink> ulSinkApp = ulServerApps.Get(u)->GetObject<PacketSink>();
                auto totalBytes = ulSinkApp->GetTotalRx();
                
                std::cout << "Remote Host - Total Bytes: " << totalBytes
                    << " (" << (100.0 * totalBytes / (packetSize * maxPackets)) << "%) "
                    << (totalBytes >= packetSize * maxPackets ? " ✅" : " ❌") << std::endl;
            }
        }
    }

    Simulator::Destroy();

    if (traceFileOutputStream.is_open()){
      traceFileOutputStream.close();
    }
}