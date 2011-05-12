/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VISUALIZATION_MSGS_INTERACTIVE_MARKER_H
#define VISUALIZATION_MSGS_INTERACTIVE_MARKER_H

#include <visualization_msgs/InteractiveMarker.h>

namespace visualization_msgs
{

/// @brief fill in default values & insert default controls when none are specified
/// @param msg      interactive marker to be completed
void autoComplete( InteractiveMarker &msg );

/// @brief fill in default values & insert default controls when none are specified
/// @param msg      interactive marker which contains the control
/// @param control  the control to be completed
void autoComplete( const InteractiveMarker &msg, InteractiveMarkerControl &control );

/// --- marker helpers ---

/// @brief make a default-style arrow marker based on the properties of the given interactive marker
/// @param msg      the interactive marker that this will go into
/// @param control  the control where to insert the arrow marker
/// @param pos      how far from the center should the arrow be, and on which side
void makeArrow( const InteractiveMarker &msg, InteractiveMarkerControl &control, float pos );

/// @brief make a default-style disc marker (e.g for rotating) based on the properties of the given interactive marker
/// @param msg      the interactive marker that this will go into
/// @param width    width of the disc, relative to its inner radius
void makeDisc( const InteractiveMarker &msg, InteractiveMarkerControl &control, float width = 0.3 );

/// assign an RGB value to the given marker based on the given orientation
void assignDefaultColor(Marker &marker, const geometry_msgs::Quaternion &quat );

/// --- quaternion helpers ---

/// get the local x-axis of the given quaternion
void getXAxis( const geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &v );

/// make a quaternion from an angle / axis pair (the angle is given in radians)
void makeAngleAxis( const geometry_msgs::Vector3 &v, float angle, geometry_msgs::Quaternion &quat );

/// normalize quaternion to length 1
void normalize( geometry_msgs::Quaternion &quat );

/// apply quaternion to vector
void rotate( const geometry_msgs::Quaternion &quat, geometry_msgs::Point &p );

/// cross product (c=a*b)
geometry_msgs::Vector3 cross( const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b );

/// add vectors (c=a+b)
template< class Vector3T >
Vector3T add( const Vector3T& a, const Vector3T& b )
{
  Vector3T c;
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.z = a.z + b.z;
  return c;
}

/// scalar multiplication (in-place)
void mul( geometry_msgs::Vector3 &vec, float s );

/// get length
float len( const geometry_msgs::Vector3 &vec );

}

#endif
