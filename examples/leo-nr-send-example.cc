#include <fstream>
#include <iostream>
#include <cmath>

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

static Ptr<ThreeGppPropagationLossModel>
    m_propagationLossModel; //!< the PropagationLossModel object
static Ptr<ThreeGppSpectrumPropagationLossModel>
    m_spectrumLossModel;          //!< the SpectrumPropagationLossModel object
static std::ofstream traceFileOutputStream;
static std::ofstream resultsFile; //!< The results file

/*
**
 * @brief Create the PSD for the TX
 *
 * @param fcHz the carrier frequency in Hz
 * @param pwrDbm the transmission power in dBm
 * @param bwHz the bandwidth in Hz
 * @param rbWidthHz the Resource Block (RB) width in Hz
 *
 * @return the pointer to the PSD
 *
Ptr<SpectrumValue>
CreateTxPowerSpectralDensity(double fcHz, double pwrDbm, double bwHz, double rbWidthHz)
{
    unsigned int numRbs = std::floor(bwHz / rbWidthHz);
    double f = fcHz - (numRbs * rbWidthHz / 2.0);
    double powerTx = pwrDbm; // dBm power
 
    Bands rbs; // A vector representing each resource block
    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += rbWidthHz / 2;
        rb.fc = f;
        f += rbWidthHz / 2;
        rb.fh = f;
 
        rbs.push_back(rb);
    }
    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(model);
 
    double powerTxW = std::pow(10., (powerTx - 30) / 10); // Get Tx power in Watts
    double txPowerDensity = (powerTxW / bwHz);
 
    for (auto psd = txPsd->ValuesBegin(); psd != txPsd->ValuesEnd(); ++psd)
    {
        *psd = txPowerDensity;
    }
 
    return txPsd; // [W/Hz]
}
 
**
 * @brief A structure that holds the parameters for the
 * ComputeSnr function. In this way the problem with the limited
 * number of parameters of method Schedule is avoided.
 *
struct ComputeSnrParams
{
    Ptr<MobilityModel> txMob;        //!< the tx mobility model
    Ptr<MobilityModel> rxMob;        //!< the rx mobility model
    double txPow;                    //!< the tx power in dBm
    double noiseFigure;              //!< the noise figure in dB
    Ptr<PhasedArrayModel> txAntenna; //!< the tx antenna array
    Ptr<PhasedArrayModel> rxAntenna; //!< the rx antenna array
    double frequency;                //!< the carrier frequency in Hz
    double bandwidth;                //!< the total bandwidth in Hz
    double resourceBlockBandwidth;   //!< the Resource Block bandwidth in Hz
 
    **
     * @brief Constructor
     * @param pTxMob the tx mobility model
     * @param pRxMob the rx mobility model
     * @param pTxPow the tx power in dBm
     * @param pNoiseFigure the noise figure in dB
     * @param pTxAntenna the tx antenna array
     * @param pRxAntenna the rx antenna array
     * @param pFrequency the carrier frequency in Hz
     * @param pBandwidth the total bandwidth in Hz
     * @param pResourceBlockBandwidth the Resource Block bandwidth in Hz
     *
    ComputeSnrParams(Ptr<MobilityModel> pTxMob,
                     Ptr<MobilityModel> pRxMob,
                     double pTxPow,
                     double pNoiseFigure,
                     Ptr<PhasedArrayModel> pTxAntenna,
                     Ptr<PhasedArrayModel> pRxAntenna,
                     double pFrequency,
                     double pBandwidth,
                     double pResourceBlockBandwidth)
    {
        txMob = pTxMob;
        rxMob = pRxMob;
        txPow = pTxPow;
        noiseFigure = pNoiseFigure;
        txAntenna = pTxAntenna;
        rxAntenna = pRxAntenna;
        frequency = pFrequency;
        bandwidth = pBandwidth;
        resourceBlockBandwidth = pResourceBlockBandwidth;
    }
};

**
 * @brief Create the noise PSD for the
 *
 * @param fcHz the carrier frequency in Hz
 * @param noiseFigureDb the noise figure in dB
 * @param bwHz the bandwidth in Hz
 * @param rbWidthHz the Resource Block (RB) width in Hz
 *
 * @return the pointer to the noise PSD
 *
Ptr<SpectrumValue>
CreateNoisePowerSpectralDensity(double fcHz, double noiseFigureDb, double bwHz, double rbWidthHz)
{
    unsigned int numRbs = std::floor(bwHz / rbWidthHz);
    double f = fcHz - (numRbs * rbWidthHz / 2.0);
 
    Bands rbs;              // A vector representing each resource block
    std::vector<int> rbsId; // A vector representing the resource block IDs
    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += rbWidthHz / 2;
        rb.fc = f;
        f += rbWidthHz / 2;
        rb.fh = f;
 
        rbs.push_back(rb);
        rbsId.push_back(numrb);
    }
    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(model);
 
    // see "LTE - From theory to practice"
    // Section 22.4.4.2 Thermal Noise and Receiver Noise Figure
    const double ktDbmHz = -174.0;                        // dBm/Hz
    double ktWHz = std::pow(10.0, (ktDbmHz - 30) / 10.0); // W/Hz
    double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);
 
    double noisePowerSpectralDensity = ktWHz * noiseFigureLinear;
 
    for (auto rbId : rbsId)
    {
        (*txPsd)[rbId] = noisePowerSpectralDensity;
    }
 
    return txPsd; // W/Hz
}

**
 * Compute the average SNR
 * @param params A structure that holds the parameters that are needed to perform calculations in
 * ComputeSnr
 *
static void
ComputeSnr(ComputeSnrParams& params)
{
    Ptr<SpectrumValue> txPsd = CreateTxPowerSpectralDensity(params.frequency,
                                                            params.txPow,
                                                            params.bandwidth,
                                                            params.resourceBlockBandwidth);
    Ptr<SpectrumValue> rxPsd = txPsd->Copy();
    NS_LOG_DEBUG("Average tx power " << 10 * log10(Sum(*txPsd) * params.resourceBlockBandwidth)
                                     << " dB");
 
    // create the noise PSD
    Ptr<SpectrumValue> noisePsd = CreateNoisePowerSpectralDensity(params.frequency,
                                                                  params.noiseFigure,
                                                                  params.bandwidth,
                                                                  params.resourceBlockBandwidth);
    NS_LOG_DEBUG("Average noise power "
                 << 10 * log10(Sum(*noisePsd) * params.resourceBlockBandwidth) << " dB");
 
    // apply the pathloss
    double propagationGainDb = m_propagationLossModel->CalcRxPower(0, params.txMob, params.rxMob);
    NS_LOG_DEBUG("Pathloss " << -propagationGainDb << " dB");
    double propagationGainLinear = std::pow(10.0, (propagationGainDb) / 10.0);
    *(rxPsd) *= propagationGainLinear;
 
    NS_ASSERT_MSG(params.txAntenna, "params.txAntenna is nullptr!");
    NS_ASSERT_MSG(params.rxAntenna, "params.rxAntenna is nullptr!");
 
    Ptr<SpectrumSignalParameters> rxSsp = Create<SpectrumSignalParameters>();
    rxSsp->psd = rxPsd;
    rxSsp->txAntenna =
        ConstCast<AntennaModel, const AntennaModel>(params.txAntenna->GetAntennaElement());
 
    // apply the fast fading and the beamforming gain
    rxSsp = m_spectrumLossModel->CalcRxPowerSpectralDensity(rxSsp,
                                                            params.txMob,
                                                            params.rxMob,
                                                            params.txAntenna,
                                                            params.rxAntenna);
    NS_LOG_DEBUG("Average rx power " << 10 * log10(Sum(*rxSsp->psd) * params.bandwidth) << " dB");
 
    // compute the SNR
    NS_LOG_DEBUG("Average SNR " << 10 * log10(Sum(*rxSsp->psd) / Sum(*noisePsd)) << " dB");
 
    // print the SNR and pathloss values in the output file
    resultsFile << Simulator::Now().GetSeconds() << " "
                << 10 * log10(Sum(*rxSsp->psd) / Sum(*noisePsd)) << " " << propagationGainDb
                << std::endl;
}


*/

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

NS_LOG_COMPONENT_DEFINE ("LeoNrSendExample");

void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  Vector pos = position->GetPosition ();
  Ptr<const Node> node = position->GetObject<Node> ();
  traceFileOutputStream << Simulator::Now () << "," << node->GetId () << "," << pos.x << "," << pos.y << "," << pos.z << "," << position->GetVelocity ().GetLength() << std::endl;
}

int main(int argc, char *argv[])
{

    std::string scenario = "NTN-Suburban"; // scenario
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
  MobilityHelper mobility;
  
  // Create and configure satellites using LEO orbit helper
  NodeContainer satellites = orbit.Install (LeoOrbit (300, 20, 1, 1)); // 300km altitude, 20° inclination, 1 satellite per plane, 1 plane
  
  // Create ground nodes (cars)
  NodeContainer cars;
  cars.Create (1); // Create 3 cars
  
  // Install GndConstantVelocityMobilityModel on cars
  mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                             "InitialLatitude", DoubleValue (carLatitude),
                             "InitialLongitude", DoubleValue (carLongitude),
                             "Altitude", DoubleValue (3),
                             "Speed", DoubleValue (carSpeed),
                             "Azimuth", DoubleValue (0),
                             "Precision", TimeValue (Seconds (1.0))); // Update every second
  mobility.Install (cars.Get(0));

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
        "ThreeGpp"); // Use TwoRay propagation model instead of ThreeGpp for simplicity
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
    std::cout << "Received packets: " << receivedPackets << std::endl;
    if (receivedPackets == 10)
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}