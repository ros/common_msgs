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

#include "visualization_msgs/interactive_marker_tools.h"

#include <math.h>
#include <assert.h>

namespace visualization_msgs
{

void autoComplete( InteractiveMarker &msg )
{
  // this is a 'delete' message. no need for action.
  if ( msg.controls.empty() )
  {
    return;
  }

  // default size
  if ( msg.size == 0 )
  {
    msg.size = 1;
  }

  // correct empty orientation, normalize
  normalize( msg.pose.orientation );

  if ( msg.header.stamp.nsec == 0 && msg.header.stamp.sec == 0 )
  {
    msg.header.stamp = ros::Time::now();
  }

  // complete the controls
  for ( unsigned c=0; c<msg.controls.size(); c++ )
  {
    autoComplete( msg, msg.controls[c] );
  }
}

void autoComplete( const InteractiveMarker &msg, InteractiveMarkerControl &control )
{
  // correct empty orientation, normalize
  normalize( control.orientation );

  // add default control handles if there are none
  if ( control.markers.empty() )
  {
    switch ( control.mode )
    {
      case InteractiveMarkerControl::NONE:
        break;

      case InteractiveMarkerControl::MOVE_AXIS:
        control.markers.reserve(2);
        makeArrow( msg, control, 1.0 );
        makeArrow( msg, control, -1.0 );
        break;

      case InteractiveMarkerControl::MOVE_PLANE:
      case InteractiveMarkerControl::ROTATE_AXIS:
      case InteractiveMarkerControl::MOVE_ROTATE:
        makeDisc( msg, control );
        break;

      case InteractiveMarkerControl::BUTTON:
        break;

    }
  }

  // fill in missing pose information into the markers
  for ( unsigned m=0; m<control.markers.size(); m++ )
  {
    Marker &marker = control.markers[m];

    // correct empty orientation, normalize
    normalize( marker.pose.orientation );

    marker.frame_locked = false;
    marker.ns = msg.name;

    // if the marker has no header, we interpret it as being in local coordinates
    if ( marker.header.frame_id.empty() )
    {
      marker.header = msg.header;
      // we need to rotate the local position first, then add it to the parent position
      rotate( msg.pose.orientation, marker.pose.position );
      marker.pose.position = add ( msg.pose.position, marker.pose.position );
    }
  }
}

void makeArrow( const InteractiveMarker &msg, InteractiveMarkerControl &control, float pos )
{
  Marker marker;

  // rely on the auto-completion for the correct orientation
  marker.pose.orientation = control.orientation;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.size * 0.3;
  marker.scale.y = msg.size * 0.5;
  marker.scale.z = msg.size * 0.2;

  assignDefaultColor(marker, control.orientation);

  float dist = fabs(pos);
  float dir = pos > 0 ? 1 : -1;

  float inner = 0.5 * dist;
  float outer = inner + 0.4;

  marker.points.resize(2);
  marker.points[0].x = dir * msg.size * inner;
  marker.points[1].x = dir * msg.size * outer;

  control.markers.push_back( marker );
}

void makeDisc( const InteractiveMarker &msg, InteractiveMarkerControl &control, float width )
{
  Marker marker;

  // rely on the auto-completion for the correct orientation
  marker.pose.orientation = control.orientation;

  marker.type = Marker::TRIANGLE_LIST;
  marker.scale.x = msg.size;
  marker.scale.y = msg.size;
  marker.scale.z = msg.size;

  assignDefaultColor(marker, control.orientation);

  // compute points on a circle in the y-z plane
  int steps = 20;
  std::vector<geometry_msgs::Point> circle1, circle2;
  circle1.reserve(steps+1);
  circle2.reserve(steps+1);

  geometry_msgs::Point v1,v2;

  for ( int i=0; i<=steps; i++ )
  {
    float a = float(i)/float(steps) * M_PI * 2.0;

    v1.y = 0.5 * cos(a);
    v1.z = 0.5 * sin(a);

    v2.y = (1+width) * v1.y;
    v2.z = (1+width) * v1.z;

    circle1.push_back( v1 );
    circle2.push_back( v2 );
  }

  //construct disc from several segments, as otherwise z sorting won't work nicely
  control.markers.reserve( control.markers.size() + steps );
  marker.points.resize(6);

  for ( int i=0; i<steps; i++ )
  {
    marker.points[0] = circle1[i];
    marker.points[1] = circle2[i];
    marker.points[2] = circle1[i+1];

    marker.points[3] = circle2[i];
    marker.points[4] = circle2[i+1];
    marker.points[5] = circle1[i+1];

    control.markers.push_back(marker);
  }
}

void assignDefaultColor(Marker &marker, const geometry_msgs::Quaternion &quat )
{
  geometry_msgs::Vector3 v;
  getXAxis( quat, v );

  float x,y,z;
  x = fabs(v.x);
  y = fabs(v.y);
  z = fabs(v.z);

  float max_xy = x>y ? x : y;
  float max_yz = y>z ? y : z;
  float max_xyz = max_xy > max_yz ? max_xy : max_yz;

  marker.color.r = x / max_xyz;
  marker.color.g = y / max_xyz;
  marker.color.b = z / max_xyz;
  marker.color.a = 0.5;
}

void getXAxis( const geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &v )
{
  float t_y  = 2.0f*quat.y;
  float t_z  = 2.0f*quat.z;
  float t_wy = t_y*quat.w;
  float t_wz = t_z*quat.w;
  float t_xy = t_y*quat.x;
  float t_xz = t_z*quat.x;
  float t_yy = t_y*quat.y;
  float t_zz = t_z*quat.z;

  v.x=1.0f-(t_yy+t_zz);
  v.y=t_xy+t_wz;
  v.z=t_xz-t_wy;
}

void makeAngleAxis( const geometry_msgs::Vector3 &v, float angle, geometry_msgs::Quaternion &quat )
{
  float l = len( v );

  // The quaternion representing the rotation is
  //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

  float sinAngle = sin(angle*0.5);
  quat.w = cos(angle*0.5);
  quat.x = sinAngle * v.x / l;
  quat.y = sinAngle * v.y / l;
  quat.z = sinAngle * v.z / l;
}

float len( const geometry_msgs::Vector3 &vec )
{
  return sqrt( vec.x*vec.x + vec.y*vec.y + vec.z*vec.z );
}

void normalize( geometry_msgs::Quaternion &quat )
{
  if ( quat.w == 0 && quat.x == 0 && quat.y == 0 && quat.z == 0)
  {
    quat.w = 1;
  }

  float l = sqrt( quat.w*quat.w + quat.x*quat.x + quat.y*quat.y + quat.z+quat.z );
  quat.w /= l;
  quat.x /= l;
  quat.y /= l;
  quat.z /= l;
}

void rotate( const geometry_msgs::Quaternion &quat, geometry_msgs::Point &p )
{
  geometry_msgs::Vector3 uv, uuv, v;

  v.x = p.x;
  v.y = p.y;
  v.z = p.z;

  geometry_msgs::Vector3 qvec;
  qvec. x = quat.x;
  qvec. y = quat.y;
  qvec. z = quat.z;

  uv = cross(qvec,v);
  uuv = cross(qvec,uv);

  mul(uv, 2.0f * quat.w);
  mul(uuv, 2.0f);

  geometry_msgs::Vector3 result = add( add(v, uv), uuv );
  v.x = result.x;
  v.y = result.y;
  v.z = result.z;
}

geometry_msgs::Vector3 cross( const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b )
{
  geometry_msgs::Vector3 c;
  c.x = a.y*b.z - a.z*b.y;
  c.y = a.z*b.x - a.x*b.z;
  c.z = a.x*b.y - a.y*b.x;
  return c;
}

void mul( geometry_msgs::Vector3 &vec, float s )
{
  vec.x *= s;
  vec.y *= s;
  vec.z *= s;
}

}
