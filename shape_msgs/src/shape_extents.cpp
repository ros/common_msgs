/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <shape_msgs/shape_extents.h>
#include <limits>

void shape_msgs::getShapeExtents(const shape_msgs::Shape& shape_msg, double& x_extent, double& y_extent, double& z_extent)
{
  x_extent = y_extent = z_extent = 0.0;
  // this ignores PLANE shapes, as there are no limits to the extents
  if (shape_msg.type == shape_msgs::Shape::SPHERE)
  {
    if (shape_msg.dimensions.size() == 1) 
    {
      x_extent = y_extent = z_extent = shape_msg.dimensions[0] * 2.0;
    }
  } else if (shape_msg.type == shape_msgs::Shape::BOX)
  {
    if (shape_msg.dimensions.size() == 3)
    {
      x_extent = shape_msg.dimensions[0];
      y_extent = shape_msg.dimensions[1];
      z_extent = shape_msg.dimensions[2];
    }
  } else if (shape_msg.type == shape_msgs::Shape::CYLINDER || shape_msg.type == shape_msgs::Shape::CONE)
  {
    if (shape_msg.dimensions.size() == 2)
    {
      x_extent = y_extent = shape_msg.dimensions[0] * 2.0;
      z_extent = shape_msg.dimensions[1];
    }
  } else if (shape_msg.type == shape_msgs::Shape::MESH)
  {
    if(shape_msg.vertices.size() == 0) 
    {
      x_extent = y_extent = z_extent = 0.0;
    }
    else
    {
      double xmin = std::numeric_limits<double>::max(), ymin = std::numeric_limits<double>::max(), zmin = std::numeric_limits<double>::max();
      double xmax = -std::numeric_limits<double>::max(), ymax = -std::numeric_limits<double>::max(), zmax = -std::numeric_limits<double>::max();
      for(unsigned int i = 0; i < shape_msg.vertices.size(); ++i)
      {
        if(shape_msg.vertices[i].x > xmax) {
          xmax = shape_msg.vertices[i].x;
        }
        if(shape_msg.vertices[i].x < xmin) {
          xmin = shape_msg.vertices[i].x;
        }
        if(shape_msg.vertices[i].y > ymax) {
          ymax = shape_msg.vertices[i].y;
        }
        if(shape_msg.vertices[i].y < ymin) {
          ymin = shape_msg.vertices[i].y;
        }
        if(shape_msg.vertices[i].z > zmax) {
          zmax = shape_msg.vertices[i].z;
        }
        if(shape_msg.vertices[i].z < zmin) {
          zmin = shape_msg.vertices[i].z;
        }
      }
      x_extent = xmax-xmin;
      y_extent = ymax-ymin;
      z_extent = zmax-zmin;
    }
  }
}
