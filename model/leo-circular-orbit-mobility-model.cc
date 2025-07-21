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
#include "leo-circular-orbit-mobility-model.h"
#include "leo-orbit.h"
#include "ns3/angles.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LeoCircularOrbitMobilityModel");

NS_OBJECT_ENSURE_REGISTERED (LeoCircularOrbitMobilityModel);

TypeId
LeoCircularOrbitMobilityModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LeoCircularOrbitMobilityModel")
    .SetParent<GeocentricConstantPositionMobilityModel> ()
    .SetGroupName ("Leo")
    .AddConstructor<LeoCircularOrbitMobilityModel> ()
    .AddAttribute ("Altitude",
                   "A height from the earth's surface in kilometers",
                   DoubleValue (1000.0),
                   MakeDoubleAccessor (&LeoCircularOrbitMobilityModel::SetAltitude,
                   		       &LeoCircularOrbitMobilityModel::GetAltitude),
                   MakeDoubleChecker<double> ())
    // TODO check value limits
    .AddAttribute ("Inclination",
                   "The inclination of the orbital plane in degrees",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&LeoCircularOrbitMobilityModel::SetInclination,
                   		       &LeoCircularOrbitMobilityModel::GetInclination),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Precision",
                   "The time precision with which to compute position updates. 0 means arbitrary precision",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&LeoCircularOrbitMobilityModel::m_precision),
                   MakeTimeChecker ())
    ;
  return tid;
}

LeoCircularOrbitMobilityModel::LeoCircularOrbitMobilityModel() : GeocentricConstantPositionMobilityModel (), m_longitude (0.0), m_offset (0.0), m_position ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

LeoCircularOrbitMobilityModel::~LeoCircularOrbitMobilityModel()
{
}

double
LeoCircularOrbitMobilityModel::GetSpeed () const
{
  return sqrt (LEO_EARTH_GM_KM_E10 / m_orbitHeight) * 1e5;
}

Vector
LeoCircularOrbitMobilityModel::DoGetGeocentricVelocity () const
{
  Vector3D pos = DoGetPosition ();
  pos = Vector3D (pos.x / pos.GetLength (), pos.y / pos.GetLength (), pos.z / pos.GetLength ());
  Vector3D heading = CrossProduct (PlaneNorm (), pos);
  return Product (GetSpeed (), heading);
}

Vector
LeoCircularOrbitMobilityModel::DoGetVelocity () const
{
  return CartesianToTopocentric (DoGetGeocentricVelocity (), GetCoordinateTranslationReferencePoint(), GeographicPositions::SPHERE);
}

Vector3D
LeoCircularOrbitMobilityModel::PlaneNorm () const
{
  double lat = CalcLatitude ();
  return Vector3D (sin (-m_inclination) * cos (lat),
  		   sin (-m_inclination) * sin (lat),
  		   cos (m_inclination));
}

double
LeoCircularOrbitMobilityModel::GetProgress (Time t) const
{
  int sign = 1;
  // ensure correct gradient (not against earth rotation)
  if (m_inclination > M_PI/2)
    {
      sign = -1;
    }
  // 2pi * (distance travelled / circumference of earth) + offset
  return sign * (((GetSpeed () * t.GetSeconds ()) / GeographicPositions::EARTH_SPHERE_RADIUS)) + m_offset;
}

Vector3D
LeoCircularOrbitMobilityModel::RotatePlane (double a, const Vector3D &x) const
{
  Vector3D n = PlaneNorm ();

  return Product (DotProduct (n, x), n)
    + Product (cos (a), CrossProduct (CrossProduct (n, x), n))
    + Product (sin (a), CrossProduct (n, x));
}

double
LeoCircularOrbitMobilityModel::CalcLatitude() const
{
  return m_longitude + ((Simulator::Now().GetDouble () / Hours (24).GetDouble ()) * 2 * M_PI);
}

Vector
LeoCircularOrbitMobilityModel::CalcPosition (Time t) const
{
  double lat = CalcLatitude ();
  // account for orbit latitude and earth rotation offset
  Vector3D x = Product (m_orbitHeight*1000, Vector3D (cos (m_inclination) * cos (lat),
  			       cos (m_inclination) * sin (lat),
  			       sin (m_inclination)));

  return RotatePlane (GetProgress (t), x);
}

Vector LeoCircularOrbitMobilityModel::Update ()
{
  m_position = CalcPosition (Simulator::Now ());
  NotifyCourseChange ();

  if (m_precision > Seconds (0))
    {
      Simulator::Schedule (m_precision, &LeoCircularOrbitMobilityModel::Update, this);
    }

  return m_position;
}

Vector
LeoCircularOrbitMobilityModel::DoGetPosition (void) const
{
  return CartesianToTopocentric(DoGetGeocentricPosition(), GetCoordinateTranslationReferencePoint(), GeographicPositions::SPHERE);
}

void
LeoCircularOrbitMobilityModel::DoSetPosition (const Vector &position)
{
  // use first element of position vector as latitude, second for longitude
  // this works nicely with MobilityHelper and GetPostion will still get the
  // correct position, but be aware that it will not be the same as supplied to
  // SetPostion (see LeoCircularOrbitPostionAllocator to understand how it works)
  m_longitude = position.x;
  m_offset = position.y;
  Update ();
  // WARN this method is not standard compliant, it does not set the position
  // Could be considered to manage the allocation of satellite positions
  // in a different way than passing values in this way
}

double LeoCircularOrbitMobilityModel::GetAltitude () const
{
  return m_orbitHeight - GeographicPositions::EARTH_SPHERE_RADIUS/1000.0;
}

void LeoCircularOrbitMobilityModel::SetAltitude (double h)
{
  m_orbitHeight = GeographicPositions::EARTH_SPHERE_RADIUS/1000.0 + h;
  Update ();
}

double LeoCircularOrbitMobilityModel::GetInclination () const
{
  return RadiansToDegrees(m_inclination);
}

void LeoCircularOrbitMobilityModel::SetInclination (double incl)
{
  NS_ASSERT_MSG (incl != 0.0, "Plane must not be orthogonal to axis");
  m_inclination = DegreesToRadians(incl);
  Update ();
}

// GeocentricConstantPositionMobilityModel interface implementation
Vector
LeoCircularOrbitMobilityModel::DoGetGeographicPosition() const
{
  // Convert ECEF position to geographic coordinates
  Vector ecefPos = DoGetPosition();
  return GeographicPositions::CartesianToGeographicCoordinates(ecefPos, GeographicPositions::SPHERE);
}

void
LeoCircularOrbitMobilityModel::DoSetGeographicPosition(const Vector& latLonAlt)
{
  NS_ASSERT_MSG((latLonAlt.x >= -90) && (latLonAlt.x <= 90),
                "Latitude must be between -90 deg and +90 deg");
  NS_ASSERT_MSG(latLonAlt.z >= 0, "Altitude must be higher or equal than 0 meters");

  // Convert geographic to orbital parameters (simplified)
  m_longitude = DegreesToRadians(latLonAlt.y);
  m_offset = DegreesToRadians(latLonAlt.x);
  m_orbitHeight = GeographicPositions::EARTH_SPHERE_RADIUS/1000.0 + (latLonAlt.z / 1000.0); // altitude in km from center
  Update();
}

Vector
LeoCircularOrbitMobilityModel::DoGetGeocentricPosition() const
{
    if (m_precision == Time (0))
    {
      // Notice: NotifyCourseChange () will not be called
      return CalcPosition (Simulator::Now ());
    }
    return m_position;
}

void
LeoCircularOrbitMobilityModel::DoSetGeocentricPosition(const Vector& position)
{
  // Convert ECEF to geographic, then set
  Vector geographicCoordinates = GeographicPositions::CartesianToGeographicCoordinates(
    position, GeographicPositions::SPHERE);
  DoSetGeographicPosition(geographicCoordinates);
}

};
