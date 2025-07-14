#include <fstream>

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

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

NS_LOG_COMPONENT_DEFINE ("LeoNrSendExample");

std::ofstream traceFileOutputStream;

void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  Vector pos = position->GetPosition ();
  Ptr<const Node> node = position->GetObject<Node> ();
  traceFileOutputStream << Simulator::Now () << "," << node->GetId () << "," << pos.x << "," << pos.y << "," << pos.z << "," << position->GetVelocity ().GetLength() << std::endl;
}

int main(int argc, char *argv[])
{

    std::string scenario = "UMa"; // scenario
    double frequency = 28e9;      // central frequency
    double bandwidth = 100e6;     // bandwidth
    bool logging = true; // whether to enable logging from the simulation, another option is by
                         // exporting the NS_LOG environment variable
    double carSpeed = 30.0;  // m/s
    double carLatitude = 0.0;
    double carLongitude = 0.0;
    double txPower = 40; // txPower
    std::string duration = "5s";
    std::string traceFile = "";
 
    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario",
                 "The scenario for the simulation. Choose among 'RMa', 'UMa', 'UMi', "
                 "'InH-OfficeMixed', 'InH-OfficeOpen'.",
                 scenario);
    cmd.AddValue("frequency", "The central carrier frequency in Hz.", frequency);
    cmd.AddValue("carSpeed", "Speed of the car in m/s", carSpeed);
    cmd.AddValue("carLatitude", "Initial latitude of the car", carLatitude);
    cmd.AddValue("carLongitude", "Initial longitude of the car", carLongitude);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("logging", "If set to 0, log components will be disabled.", logging);
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.Parse(argc, argv);

  LeoOrbitNodeHelper orbit;
  NodeContainer satellites = orbit.Install (LeoOrbit (400, 20, 10, 1));

  // Create ground nodes (cars)
  NodeContainer cars;
  cars.Create (3); // Create 1 car

  // Install GndConstantVelocityMobilityModel on cars
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                             "InitialLatitude", DoubleValue (carLatitude),
                             "InitialLongitude", DoubleValue (carLongitude),
                             "Altitude", DoubleValue (3),
                             "Speed", DoubleValue (carSpeed),
                             "Azimuth", DoubleValue (0),
                             "Precision", TimeValue (Seconds (1.0))); // Update every second
  mobility.Install (cars.Get(0));
  mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                            "InitialLatitude", DoubleValue (carLatitude),
                            "InitialLongitude", DoubleValue (carLongitude),
                            "Altitude", DoubleValue (3),
                            "Speed", DoubleValue (carSpeed),
                            "Azimuth", DoubleValue (120),
                            "Precision", TimeValue (Seconds (1.0))); // Update every second
  mobility.Install (cars.Get(1));
  mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                            "InitialLatitude", DoubleValue (carLatitude),
                            "InitialLongitude", DoubleValue (carLongitude),
                            "Altitude", DoubleValue (3),
                            "Speed", DoubleValue (carSpeed),
                            "Azimuth", DoubleValue (240),
                            "Precision", TimeValue (Seconds (1.0))); // Update every second
mobility.Install (cars.Get(2));

  if (traceFile != "")
    {
      traceFileOutputStream.open (traceFile);
      if (!traceFileOutputStream.is_open ()){
        NS_FATAL_ERROR ("Could not open trace file " << traceFile);
      }
      Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                       MakeCallback (&CourseChange));
      traceFileOutputStream << "Time,Node,X,Y,Z,Speed" << std::endl;
    }
  if (logging)
    {
        //LogComponentEnable ("ThreeGppSpectrumPropagationLossModel", LOG_LEVEL_ALL);
        LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_ALL);
        //LogComponentEnable ("ThreeGppChannelModel", LOG_LEVEL_ALL);
        //LogComponentEnable ("ChannelConditionModel", LOG_LEVEL_ALL);
        LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
        //LogComponentEnable ("NrRlcUm", LOG_LEVEL_LOGIC);
        //LogComponentEnable ("NrPdcp", LOG_LEVEL_INFO);
    }
    /*
     * Default values for the simulation. We are progressively removing all
     * the instances of SetDefault, but we need it for legacy code (LTE)
     */
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

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
    channelHelper->ConfigureFactories(
        scenario,
        "Default",
        "ThreeGpp"); // Configure the spectrum channel with the scenario
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});
 
    // Configure ideal beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
 
    // Configure scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());
 
    // Antennas for the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
 
    // Antennas for the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
 
    // install nr net devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(cars, allBwps);
 
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
 
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetTxPower(txPower);
    nrHelper->GetGnbPhy(gnbNetDev.Get(1), 0)->SetTxPower(txPower);
 
    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.010));
 
    InternetStackHelper internet;
    internet.Install(cars);
 
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
 
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
        dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(1)));
        // dlClient.SetAttribute ("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("MaxPackets", UintegerValue(10));
        dlClient.SetAttribute("PacketSize", UintegerValue(1500));
        clientApps.Add(dlClient.Install(remoteHost));
    }
 
    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
 
    // start server and client apps
    serverApps.Start(Seconds(0.4));
    clientApps.Start(Seconds(0.4));
    serverApps.Stop(Time(duration));
    clientApps.Stop(Time(duration) - Seconds(0.2));
 
    // enable the traces provided by the nr module
    nrHelper->EnableTraces();
 
    Simulator::Stop (Time (duration));
    Simulator::Run();

 
    Ptr<UdpServer> serverApp = serverApps.Get(0)->GetObject<UdpServer>();
    uint64_t receivedPackets = serverApp->GetReceived();
 
    Simulator::Destroy();

    if (traceFileOutputStream.is_open()){
      traceFileOutputStream.close();
    }
     cout << "Received packets: " << receivedPackets << std::endl;
    if (receivedPackets == 10)
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}