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

#include <visualization_msgs/InteractiveMarker.h>

#include <math.h>

namespace visualization_msgs
{

inline void assignDefaultColor(visualization_msgs::Marker &marker, float x, float y, float z)
{
  float max_xy = fabs(x)>fabs(y) ? fabs(x) : fabs(y);
  float max_yz = fabs(y)>fabs(z) ? fabs(y) : fabs(z);
  float max_xyz = max_xy > max_yz ? max_xy : max_yz;

  marker.color.r = fabs(x) / max_xyz;
  marker.color.g = fabs(y) / max_xyz;
  marker.color.b = fabs(z) / max_xyz;
  marker.color.a = 0.5;
}

inline visualization_msgs::Marker makeArrow( visualization_msgs::InteractiveMarker &msg, float x, float y, float z )
{
  visualization_msgs::Marker marker;

  if ( x==0 && y==0 && z==0 )
  {
    x = 1;
  }

  float l = sqrt(x*x+y*y+z*z);

  float x_norm = x / l;
  float y_norm = y / l;
  float z_norm = z / l;

  marker.header = msg.header;
  marker.pose = msg.pose;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = msg.size * 0.25;
  marker.scale.y = msg.size * 0.45;
  marker.scale.z = msg.size * 0.2;

  assignDefaultColor(marker,x,y,z);

  marker.points.resize(2);
  marker.points[0].x = x_norm * 0.5;
  marker.points[0].y = y_norm * 0.5;
  marker.points[0].z = z_norm * 0.5;
  marker.points[1].x = x_norm * 0.5 + msg.size * 0.4 * x_norm;
  marker.points[1].y = y_norm * 0.5 + msg.size * 0.4 * y_norm;
  marker.points[1].z = z_norm * 0.5 + msg.size * 0.4 * z_norm;

  return marker;
}

void makeDisc( visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control,
    float x, float y, float z )
{
  visualization_msgs::Marker marker;
  std::vector<visualization_msgs::Marker> markers;

  if ( x==0 && y==0 && z==0 )
  {
    x = 1;
  }

  float l = sqrt(x*x+y*y+z*z);

  float x0 = x / l;
  float y0 = y / l;
  float z0 = z / l;

  marker.header = msg.header;
  marker.pose = msg.pose;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.scale.z = 1;
  marker.scale.y = 1;
  marker.scale.x = 1;

  // make perpendicular vector
  // algorithm stolen from Ogre::Vector::perpendicular

  float x1,y1,z1,x2,y2,z2;

  // take cross product with (1,0,0), if unstable use (0,0,1)
  x1 = 0;
  y1 = z0;
  z1 = -y0;

  ROS_INFO_STREAM( x0*x1+y0*y1+z0*z1 );

  if (y1*y1 + z1*z1 < 1e-06 * 1e-06)
  {
    x1 = -z0;
    y1 = 0;
    z1 = x0;
  }

  // normalise
  l = sqrt( x1*x1 + y1*y1 + z1*z1 );
  x1 /= l;
  y1 /= l;
  z1 /= l;

  // construct 3rd perpendicular vector by taking cross product
  x2 = y0*z1 - z0*y1;
  y2 = z0*x1 - x0*z1;
  z2 = x0*y1 - y0*x1;

  // compute points on a circle
  int steps = 20;
  std::vector<geometry_msgs::Point> circle1, circle2;
  circle1.reserve(steps+1);
  circle2.reserve(steps+1);

  for ( int i=0; i<=steps; i++ )
  {
    float a = float(i)/float(steps) * M_PI * 2.0;

    geometry_msgs::Point v1,v2;

    v1.x = 0.5 * msg.size * (sin(a)*x1 + cos(a)*x2);
    v1.y = 0.5 * msg.size * (sin(a)*y1 + cos(a)*y2);
    v1.z = 0.5 * msg.size * (sin(a)*z1 + cos(a)*z2);

    v2.x = 1.3 * v1.x;
    v2.y = 1.3 * v1.y;
    v2.z = 1.3 * v1.z;

    circle1.push_back( v1 );
    circle2.push_back( v2 );
  }

  assignDefaultColor(marker,x,y,z);

  //construct disc from many pieces, as otherwise z sorting won't work with alpha
  control.markers.reserve( control.markers.size() + steps );
  marker.points.resize(6);

  for ( int i=0; i<steps; i++ )
  {
    marker.color.a = i%2 ? 0.7 : 0.4;

    marker.points[0] = circle1[i];
    marker.points[1] = circle2[i];
    marker.points[2] = circle1[i+1];

    marker.points[3] = circle2[i];
    marker.points[4] = circle2[i+1];
    marker.points[5] = circle1[i+1];

    control.markers.push_back(marker);
  }
}


inline visualization_msgs::InteractiveMarkerControl makeMoveAxisControl( visualization_msgs::InteractiveMarker &msg, float x, float y, float z )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.MOVE_AXIS;
  int_marker_control.vec.x = x;
  int_marker_control.vec.y = y;
  int_marker_control.vec.z = z;
  int_marker_control.markers.push_back( makeArrow( msg, x, y, z) );
  int_marker_control.markers.push_back( makeArrow( msg, -x, -y, -z) );

  return int_marker_control;
}


inline visualization_msgs::InteractiveMarkerControl makeMovePlaneControl( visualization_msgs::InteractiveMarker &msg, float x, float y, float z )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.MOVE_PLANE;
  int_marker_control.vec.x = x;
  int_marker_control.vec.y = y;
  int_marker_control.vec.z = z;
  makeDisc( msg, int_marker_control, x, y, z);

  return int_marker_control;
}


inline visualization_msgs::InteractiveMarkerControl makeRotatePlaneControl( visualization_msgs::InteractiveMarker &msg, float x, float y, float z )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.ROTATE_PLANE;
  int_marker_control.vec.x = x;
  int_marker_control.vec.y = y;
  int_marker_control.vec.z = z;
  makeDisc( msg, int_marker_control, x, y, z);

  ROS_INFO_STREAM(int_marker_control.markers[0].type);

  return int_marker_control;
}


inline visualization_msgs::InteractiveMarkerControl makeMoveRotatePlaneControl( visualization_msgs::InteractiveMarker &msg, float x, float y, float z )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.MOVE_ROTATE_PLANE;
  int_marker_control.vec.x = x;
  int_marker_control.vec.y = y;
  int_marker_control.vec.z = z;
  makeDisc( msg, int_marker_control, x, y, z);

  return int_marker_control;
}

}
