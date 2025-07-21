/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include "math.h"

#include "ns3/double.h"
#include "ns3/simulator.h"
#include "ns3/geographic-positions.h"
#include "leo-orbit.h"
#include "leo-ground-mobility-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GndConstantVelocityMobilityModel");

NS_OBJECT_ENSURE_REGISTERED (GndConstantVelocityMobilityModel);

TypeId
GndConstantVelocityMobilityModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::GndConstantVelocityMobilityModel")
    .SetParent<GeocentricConstantPositionMobilityModel> ()
    .SetGroupName ("Leo")
    .AddConstructor<GndConstantVelocityMobilityModel> ()
    .AddAttribute ("Precision",
                "The time precision with which to compute position updates. 0 means arbitrary precision",
                TimeValue (Seconds (1)),
                MakeTimeAccessor (&GndConstantVelocityMobilityModel::m_precision),
                MakeTimeChecker ())
    .AddAttribute ("Altitude",
                     "A height from the earth's surface in meters",
                        DoubleValue (0.0),
                        MakeDoubleAccessor (&GndConstantVelocityMobilityModel::m_altitude),
                        MakeDoubleChecker<double> ())
    .AddAttribute ("Speed",
                        "The velocity of the node in m/s",
                        DoubleValue (0),
                        MakeDoubleAccessor (&GndConstantVelocityMobilityModel::m_velocity),
                        MakeDoubleChecker<double> (0.0))
    .AddAttribute ("Azimuth",
                     "The azimuth of the velocity vector in degrees",
                        DoubleValue (0.0),
                        MakeDoubleAccessor (&GndConstantVelocityMobilityModel::m_azimuth),
                        MakeDoubleChecker<double> (0.0, 360.0))
    .AddAttribute ("InitialLatitude",
                     "Initial latitude position in degrees",
                        DoubleValue (0.0),
                        MakeDoubleAccessor (&GndConstantVelocityMobilityModel::m_initialLatitude),
                        MakeDoubleChecker<double> (-90.0, 90.0))
    .AddAttribute ("InitialLongitude",
                     "Initial longitude position in degrees",
                        DoubleValue (0.0),
                        MakeDoubleAccessor (&GndConstantVelocityMobilityModel::m_initialLongitude),
                        MakeDoubleChecker<double> (-180.0, 180.0))
    ;
  return tid;
}

GndConstantVelocityMobilityModel::GndConstantVelocityMobilityModel()
: GeocentricConstantPositionMobilityModel (),
  m_initialLatitude(0.0),
  m_initialLongitude(0.0),
  m_altitude(0.0),
  m_azimuth(0.0),
  m_velocity(0.0),
  m_precision(Seconds(1))
{
  NS_LOG_FUNCTION (this);
  Update(); // Initialize position
}

double
GndConstantVelocityMobilityModel::GetVelocity () const
{
  return m_velocity;
}

void
GndConstantVelocityMobilityModel::SetVelocity (double velocity)
{
  m_velocity = velocity;
  Update();
}

double
GndConstantVelocityMobilityModel::GetAzimuth () const
{
  return m_azimuth;
}

void
GndConstantVelocityMobilityModel::SetAzimuth (double azimuth)
{
  m_azimuth = azimuth;
  Update();
}

Vector
GndConstantVelocityMobilityModel::DoGetVelocity () const
{
  // Convert spherical velocity to Cartesian coordinates
  // For ground movement, we need to compute velocity in local frame
  double latRad = m_initialLatitude * M_PI / 180.0;
  double azimuthRad = m_azimuth * M_PI / 180.0;
  
  // Velocity components in local East-North-Up frame
  double vEast = m_velocity * sin(azimuthRad);
  double vNorth = m_velocity * cos(azimuthRad);
  double vUp = 0.0; // Ground movement
  
  // Convert to ECEF coordinates (simplified approximation)
  Vector velocity;
  velocity.x = -vEast * sin(latRad) + vNorth * cos(latRad);
  velocity.y = vEast * cos(latRad) + vNorth * sin(latRad);
  velocity.z = vUp;
  
  return velocity;
}

Vector
GndConstantVelocityMobilityModel::CalcPosition (Time t) const
{
  // Calculate current position based on initial position, velocity, azimuth and time
  double timeSeconds = t.GetSeconds();
  
  // Distance traveled
  double distance = m_velocity * timeSeconds;
  
  // Convert to angular displacement on Earth's surface
  double earthRadius = GeographicPositions::EARTH_SPHERE_RADIUS + m_altitude; // in meters
  double angularDistance = distance / earthRadius; // in radians
  
  // Convert initial position to radians
  double lat1 = m_initialLatitude * M_PI / 180.0;
  double lon1 = m_initialLongitude * M_PI / 180.0;
  double bearing = m_azimuth * M_PI / 180.0;
  
  // Calculate new position using great circle navigation
  double lat2 = asin(sin(lat1) * cos(angularDistance) + 
                     cos(lat1) * sin(angularDistance) * cos(bearing));
  double lon2 = lon1 + atan2(sin(bearing) * sin(angularDistance) * cos(lat1),
                             cos(angularDistance) - sin(lat1) * sin(lat2));
  
  // Convert to Cartesian ECEF coordinates
  double cosLat = cos(lat2);
  double sinLat = sin(lat2);
  double cosLon = cos(lon2);
  double sinLon = sin(lon2);
  
  Vector position;
  position.x = earthRadius * cosLat * cosLon;
  position.y = earthRadius * cosLat * sinLon;
  position.z = earthRadius * sinLat;
  
  return position;
}

Vector 
GndConstantVelocityMobilityModel::Update ()
{
  m_position = CalcPosition (Simulator::Now ());
  NotifyCourseChange ();

  if (m_precision > Seconds (0))
    {
      Simulator::Schedule (m_precision, &GndConstantVelocityMobilityModel::Update, this);
    }

  return m_position;
}

Vector
GndConstantVelocityMobilityModel::DoGetPosition (void) const
{
  if (m_precision == Time (0))
    {
      // Notice: NotifyCourseChange () will not be called
      return CalcPosition (Simulator::Now ());
    }
  return m_position;
}

void
GndConstantVelocityMobilityModel::DoSetPosition (const Vector &position)
{
  // Update our internal geographic position
  // Convert Cartesian position back to lat/lon (simplified)
  double earthRadius = GeographicPositions::EARTH_SPHERE_RADIUS + m_altitude; // in meters
  double x = position.x;
  double y = position.y;
  double z = position.z;
  
  double lat = asin(z / earthRadius);
  double lon = atan2(y, x);
  
  m_initialLatitude = lat * 180.0 / M_PI;
  m_initialLongitude = lon * 180.0 / M_PI;
  
  Update ();
}

Vector
GndConstantVelocityMobilityModel::DoGetGeographicPosition() const
{
  // Convert from topocentric to geographic coordinates
  return GeographicPositions::TopocentricToGeographicCoordinates(
      DoGetPosition(),
      GetCoordinateTranslationReferencePoint(),
      GeographicPositions::SPHERE
    );
}

void
GndConstantVelocityMobilityModel::DoSetGeographicPosition(const Vector& latLonAlt)
{
  NS_ASSERT_MSG((latLonAlt.x >= -90) && (latLonAlt.x <= 90),
                "Latitude must be between -90 deg and +90 deg");
  NS_ASSERT_MSG(latLonAlt.z >= 0, "Altitude must be higher or equal than 0 meters");
   // TODO FIX
   /*
  m_initialLatitude = latLonAlt.x;
  m_initialLongitude = latLonAlt.y;
  m_altitude = latLonAlt.z;
  */
  
  Update();
}

Vector
GndConstantVelocityMobilityModel::DoGetGeocentricPosition() const
{
  Vector geographicPos = DoGetGeographicPosition();
  return GeographicPositions::GeographicToCartesianCoordinates(
    geographicPos.x, geographicPos.y, geographicPos.z, 
    GeographicPositions::SPHERE);
}

void
GndConstantVelocityMobilityModel::DoSetGeocentricPosition(const Vector& position)
{
  Vector geographicCoordinates = GeographicPositions::CartesianToGeographicCoordinates(
    position, GeographicPositions::SPHERE);
  DoSetGeographicPosition(geographicCoordinates);
}

Vector
GndConstantVelocityMobilityModel::GetGeographicPosition() const
{
  return DoGetGeographicPosition();
}

void
GndConstantVelocityMobilityModel::SetGeographicPosition(const Vector& latLonAlt)
{
  DoSetGeographicPosition(latLonAlt);
}

Vector
GndConstantVelocityMobilityModel::GetGeocentricPosition() const
{
  return DoGetGeocentricPosition();
}

void
GndConstantVelocityMobilityModel::SetGeocentricPosition(const Vector& position)
{
  DoSetGeocentricPosition(position);
}

void
GndConstantVelocityMobilityModel::SetCoordinateTranslationReferencePoint(const Vector& refPoint)
{
  // Use base class implementation
  GeocentricConstantPositionMobilityModel::SetCoordinateTranslationReferencePoint(refPoint);
}

Vector
GndConstantVelocityMobilityModel::GetCoordinateTranslationReferencePoint() const
{
  // Use base class implementation
  return GeocentricConstantPositionMobilityModel::GetCoordinateTranslationReferencePoint();
}

Vector
GndConstantVelocityMobilityModel::GetPosition() const
{
  return DoGetPosition();
}

void
GndConstantVelocityMobilityModel::SetPosition(const Vector& position)
{
  DoSetPosition(position);
}

} // namespace ns3
