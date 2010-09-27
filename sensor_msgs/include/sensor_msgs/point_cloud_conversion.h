/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * $Id$
 *
 */

#ifndef SENSOR_MSGS_POINT_CLOUD_CONVERSION_H
#define SENSOR_MSGS_POINT_CLOUD_CONVERSION_H

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

/** 
  * \brief Convert between the old (sensor_msgs::PointCloud) and the new (sensor_msgs::PointCloud2) format.
  * \author Radu Bogdan Rusu
  */
namespace sensor_msgs
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \param points the the point cloud message
    * \param field_name the string defining the field name
    */
int getPointCloud2FieldIndex (const sensor_msgs::PointCloud2 &cloud, const std::string &field_name);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Convert a sensor_msgs::PointCloud message to a sensor_msgs::PointCloud2 message.
    * \param input the message in the sensor_msgs::PointCloud format
    * \param output the resultant message in the sensor_msgs::PointCloud2 format
    */ 
bool convertPointCloudToPointCloud2 (const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Convert a sensor_msgs::PointCloud2 message to a sensor_msgs::PointCloud message.
    * \param input the message in the sensor_msgs::PointCloud2 format
    * \param output the resultant message in the sensor_msgs::PointCloud format
    */ 
bool convertPointCloud2ToPointCloud (const sensor_msgs::PointCloud2 &input, sensor_msgs::PointCloud &output);
}
#endif// SENSOR_MSGS_POINT_CLOUD_CONVERSION_H
