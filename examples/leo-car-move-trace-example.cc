#include <fstream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/leo-ground-node-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LeoCarMoveTraceExample");

void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  Vector pos = position->GetPosition ();
  Ptr<const Node> node = position->GetObject<Node> ();
  std::cout << Simulator::Now () << "," << node->GetId () << "," << pos.x << "," << pos.y << "," << pos.z << "," << position->GetVelocity ().GetLength() << std::endl;
}

int main(int argc, char *argv[])
{
  CommandLine cmd;
  std::string orbitFile;
  std::string traceFile;
  std::string duration = "60s";
  double carSpeed = 30.0;  // m/s
  double carLatitude = 0.0;  // Milan
  double carLongitude = 0.0;  // Milan
  double carAzimuth = 90.0;   // Northeast
  
  cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
  cmd.AddValue("precision", "ns3::LeoCircularOrbitMobilityModel::Precision");
  cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
  cmd.AddValue("carSpeed", "Speed of the car in m/s", carSpeed);
  cmd.AddValue("carLatitude", "Initial latitude of the car", carLatitude);
  cmd.AddValue("carLongitude", "Initial longitude of the car", carLongitude);
  cmd.AddValue("carAzimuth", "Initial azimuth of the car in degrees", carAzimuth);
  cmd.Parse (argc, argv);
  
  LeoOrbitNodeHelper orbit;
  NodeContainer satellites = orbit.Install (LeoOrbit (1200, 20, 1, 1));

  // Create ground nodes (cars)
  NodeContainer cars;
  cars.Create (1); // Create 1 car
  
  // Install GndConstantVelocityMobilityModel on cars
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::GndConstantVelocityMobilityModel",
                             "InitialLatitude", DoubleValue (carLatitude),
                             "InitialLongitude", DoubleValue (carLongitude),
                             "Altitude", DoubleValue (0.0),
                             "Speed", DoubleValue (carSpeed),
                             "Azimuth", DoubleValue (carAzimuth),
                             "Precision", TimeValue (Seconds (1.0))); // Update every second
  mobility.Install (cars);

  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  std::streambuf *coutbuf = std::cout.rdbuf();
  // redirect cout if traceFile is specified
  std::ofstream out;
  out.open (traceFile);
  if (out.is_open ())
    {
      std::cout.rdbuf(out.rdbuf());
    }

  std::cout << "Time,Node,X,Y,Z,Speed" << std::endl;

  Simulator::Stop (Time (duration));
  Simulator::Run ();
  Simulator::Destroy ();

  out.close ();
  std::cout.rdbuf(coutbuf);
}
