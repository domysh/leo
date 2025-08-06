#include <fstream>
#include <iostream>
#include <cmath>
#include <csignal>
#include <cstdlib>

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

using namespace ns3;

static std::ofstream traceFileOutputStream;
static bool logging = false; // whether to enable logging from the simulation, another option is by
                         // exporting the NS_LOG environment variable


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
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] PacketSink RX: " 
              << packet->GetSize() << " bytes at " << context << std::endl;
}

void UdpClientTxTrace(std::string context, Ptr<const Packet> packet)
{
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] UdpClient TX: " 
              << packet->GetSize() << " bytes at " << context << std::endl;
}

void PointToPointTxTrace(std::string context, Ptr<const Packet> packet)
{
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] P2P TX: " 
              << packet->GetSize() << " bytes at " << context << std::endl;
}

void PointToPointRxTrace(std::string context, Ptr<const Packet> packet)
{
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] P2P RX: " 
              << packet->GetSize() << " bytes at " << context << std::endl;
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
    Time mobilityPrecision = MilliSeconds(1000); // Precision for mobility updates
    bool enableGnb = true; // Whether to enable gNB transmission
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
    cmd.Parse(argc, argv);

  srand(rnd_seed);

  Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                        TimeValue(MilliSeconds(10))); // update the channel at every 10 ms
  Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                        TimeValue(MilliSeconds(0))); // do not update the channel condition

  LeoOrbitNodeHelper orbit;
  MobilityHelper mobility;
  
  orbit.SetPrecision(mobilityPrecision); // Set precision for position updates
  // Create and configure satellites using LEO orbit helper
  // Use higher altitude and more reasonable inclination to improve elevation angles
  NodeContainer satellites = orbit.Install (300.0, 20, 0.0, 0.0);
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
        mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                                    "InitialLatitude", DoubleValue (rndLat),
                                    "InitialLongitude", DoubleValue (rndLon),
                                    "Altitude", DoubleValue (2),
                                    "Speed", DoubleValue (carSpeed),
                                    "Azimuth", DoubleValue (rndAzimuth),
                                    "Precision", TimeValue (mobilityPrecision)); // Update every second
        mobility.Install(car);
    }
  } else {
    cars.Create(1); // Create a single car
    mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                                "InitialLatitude", DoubleValue (carLatitude),
                                "InitialLongitude", DoubleValue (carLongitude),
                                "Altitude", DoubleValue (3),
                                "Speed", DoubleValue (carSpeed),
                                "Azimuth", DoubleValue (0),
                                "Precision", TimeValue (mobilityPrecision));
    mobility.Install(cars.Get(0));
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
  if (logging)
    {
        // Propagation and Channel Models
        //LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_LOGIC);
        //LogComponentEnable ("ThreeGppSpectrumPropagationLossModel", LOG_LEVEL_LOGIC);
        //LogComponentEnable ("ThreeGppChannelModel", LOG_LEVEL_LOGIC);
        //LogComponentEnable ("ChannelConditionModel", LOG_LEVEL_LOGIC);
        
        // Application Layer
        LogComponentEnable ("UdpClient", LOG_LEVEL_LOGIC);
        LogComponentEnable ("UdpServer", LOG_LEVEL_LOGIC);
        LogComponentEnable ("UdpSocketImpl", LOG_LEVEL_LOGIC);
        
        // Transport Layer
        LogComponentEnable ("UdpL4Protocol", LOG_LEVEL_LOGIC);
        
        // Network Layer (IP)
        LogComponentEnable ("Ipv4L3Protocol", LOG_LEVEL_LOGIC);
        LogComponentEnable ("Ipv4StaticRouting", LOG_LEVEL_LOGIC);
        LogComponentEnable ("Ipv4GlobalRouting", LOG_LEVEL_LOGIC);
        
        // NR Protocol Stack
        LogComponentEnable ("NrRlcUm", LOG_LEVEL_LOGIC);
        LogComponentEnable ("NrPdcp", LOG_LEVEL_LOGIC);
        LogComponentEnable ("NrGnbMac", LOG_LEVEL_LOGIC);
        LogComponentEnable ("NrUeMac", LOG_LEVEL_LOGIC);
        
        // Point-to-Point
        LogComponentEnable ("PointToPointNetDevice", LOG_LEVEL_LOGIC);
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
 
    // install nr net devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(cars, allBwps);
 
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
 
    // Set gNB transmission power based on enableGnb parameter
    if (enableGnb) {
        nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetTxPower(txPower);
        std::cout << "gNB enabled with transmission power: " << txPower << " dBm" << std::endl;
    } else {
        nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetTxPower(-1000); // Effectively disable transmission
        std::cout << "gNB disabled (transmission power set to -1000 dBm)" << std::endl;
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
 
    // assign IP address to UEs, and install UDP downlink applications
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < cars.GetN(); ++u)
    {
        Ptr<Node> ueNode = cars.Get(u);
        UdpServerHelper dlPacketSinkHelper(dlPort);
        serverApps.Add(dlPacketSinkHelper.Install(cars.Get(u)));
 
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(10));
        dlClient.SetAttribute("PacketSize", UintegerValue(1024));
        clientApps.Add(dlClient.Install(remoteHost)); // Remote host sends to car
    }
 
    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
 
    // Connect trace sources for packet tracking
    if (logging) {
        // Trace UDP application layer
        Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpClient/Tx", MakeCallback(&UdpClientTxTrace));
        Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpServer/Rx", MakeCallback(&PacketSinkRxTrace));
        
        // Trace Point-to-Point devices (backhaul)
        Config::Connect("/NodeList/*/DeviceList/*/$ns3::PointToPointNetDevice/MacTx", 
                       MakeCallback(&PointToPointTxTrace));
        Config::Connect("/NodeList/*/DeviceList/*/$ns3::PointToPointNetDevice/MacRx", 
                       MakeCallback(&PointToPointRxTrace));
    }
 
    // start server and client apps
    serverApps.Start(Seconds(0.1)); // Earlier start
    clientApps.Start(Seconds(0.2)); // Start transmission quickly after attachment
    serverApps.Stop(Time(duration) - Seconds(0.01));
    clientApps.Stop(Time(duration) - Seconds(0.1)); // Reduced stop margin
 
    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    Simulator::Schedule(Seconds(0.1), []() {
        std::cout << "============= Starting UDP client and server applications =============" << std::endl;
    });


    Simulator::Stop (Time (duration));
    std::cout << "============= Starting simulation for " << duration << " =============" << std::endl;
    

    Time printInterval = Seconds(5);
    Time simDuration = Time(duration);
    for (Time t = printInterval; t < simDuration; t += printInterval) {
        Simulator::Schedule(t, [t]() {
            std::cout << "Simulation time: " << t.GetSeconds() << "s" << std::endl;
        });
    }
    
    try {
        Simulator::Run();
    } catch (const std::exception& e) {
        std::cout << "Simulation caught exception (normal for mobility scenarios): " << e.what() << std::endl;
    }

 
    Ptr<UdpServer> serverApp = serverApps.Get(0)->GetObject<UdpServer>();
    uint64_t receivedPackets = serverApp->GetReceived();
 
    Simulator::Destroy();

    if (traceFileOutputStream.is_open()){
      traceFileOutputStream.close();
    }

    if (receivedPackets == 10)
    {
        std::cout << "Test passed! [Received packets: " << receivedPackets << " == 10]" << std::endl;
    }
    else
    {
        std::cout << "Test failed! [Received packets: " << receivedPackets << " != 10]" << std::endl;
    }
}