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

#ifndef LEO_GROUND_MOBILITY_MODEL_H
#define LEO_GROUND_MOBILITY_MODEL_H

#include "ns3/vector.h"
#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"

/**
 * \file
 * \ingroup leo
 *
 * Declaration of GndConstantVelocityMobilityModel
 */

#define LEO_EARTH_RAD_KM 6371.0090
#define LEO_EARTH_GM_KM_E10 39.8600436

namespace ns3 {

/**
 * \ingroup leo
 * \brief Keep track of the position of a ground node with constant velocity.
 *
 * This uses simple circular orbits based on the first position of the node
 * and the direction of the velocity vector.
 * The node will move with constant speed in the direction of the velocity vector
 * around the Earth.
 */
class GndConstantVelocityMobilityModel : public MobilityModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  
  /// Default constructor  
  GndConstantVelocityMobilityModel ();
  
  /// Parameterized constructor
  GndConstantVelocityMobilityModel (
        double initialLatitude, double initialLongitude,
        double altitude, double azimuth, double velocity,
        Time precision = Seconds(1)
  );
  /// destructor
  virtual ~GndConstantVelocityMobilityModel ();

private:

  /**
   * Initial latitude in degrees
   */
  double m_initialLatitude;

  /**
   * Initial longitude in degrees
   */
  double m_initialLongitude;

  /** 
   * The altitude of the node in meters above the Earth's surface.
   */
  double m_altitude;
  
  /**
   * The direction of the velocity vector
   */
  double m_azimuth;

  /**
   * The velocity of the node in m/s
   */
   double m_velocity;

  /**
   * Current position
   */
  Vector m_position;

  /**
   * Time precision for positions
   */
  Time m_precision;

  /**
   * \return the current position.
   */
  virtual Vector DoGetPosition (void) const;
  /**
   * \param position the position to set.
   */
  virtual void DoSetPosition (const Vector &position);
  /**
   * \return the current velocity.
   */
  virtual Vector DoGetVelocity (void) const;

  /**
   * \brief Calculate the position at time t
   * \param t time
   * \return position at time t
   */
  Vector CalcPosition (Time t) const;

  /**
   * \brief Get the azimuth of the velocity vector
   * \return azimuth in degrees
   */
  double GetAzimuth () const;

  /**
   * \brief Set the azimuth of the velocity vector
   * \param azimuth in degrees
   */
  void SetAzimuth (double azimuth);

  /**
   * \brief Get the altitude of the node
   * \return altitude in meters above the Earth's surface
   */
  double GetAltitude () const;

  /**
   * \brief Set the altitude of the node
   * \param altitude in meters above the Earth's surface
   */
  void SetAltitude (double altitude);

    /**
   * \brief Get the velocity of the node
   * \return velocity in m/s
   */
  double GetVelocity () const;

  /**
   * \brief Set the velocity of the node
   * \param velocity in m/s
   */
  void SetVelocity (double velocity);

  /**
   * \brief Get the initial latitude
   * \return latitude in degrees
   */
  double GetInitialLatitude () const;

  /**
   * \brief Set the initial latitude
   * \param latitude in degrees
   */
  void SetInitialLatitude (double latitude);

  /**
   * \brief Get the initial longitude
   * \return longitude in degrees
   */
  double GetInitialLongitude () const;

  /**
   * \brief Set the initial longitude
   * \param longitude in degrees
   */
  void SetInitialLongitude (double longitude);

  /**
   * \brief Update the internal position of the mobility model
   * \return position that will be returned upon next call to DoGetPosition
   */
  Vector Update ();
};

}

#endif
