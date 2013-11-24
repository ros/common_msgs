/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation, Inc.
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
 */

#ifndef SENSOR_MSGS_POINT_CLOUD2_PROXY_H
#define SENSOR_MSGS_POINT_CLOUD2_PROXY_H

#include <sensor_msgs/PointCloud2.h>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Offers tools to use a PointCloud2 as a container
 * This file provides two sets of utilities to modify and parse a PointCloud2
 * The first set allows you to conveniently set the fields by hand:
 *   #include <sensor_msgs/point_cloud2_proxy.h>
 *   // Create a PointCloud2
 *   sensor_msgs::PointCloud2 cloud_msg;
 *   // Fill some internals of the PoinCloud2 like the header/width/height ...
 *   cloud_msgs.height = 1;  cloud_msgs.width = 4;
 *   // Set the point fields to xyzrgb and resize the vector with the following command
 *   // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
 *   // the number of occurences of the type in the PointField, the type of the PointField
 *   setPointCloud2FieldsByString(cloud_msg, 4, "x", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "y", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "z", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "rgb", 1, sensor_msgs::PointField::FLOAT32);
 *   // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
 *   // 2 is for the number of fields to add
 *   setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
 *
 * The second set allow you to modify your PointCloud using a proxy class
 *   // Define some raw data we'll put in the PointCloud2
 *   float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
 *   uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
 *   // Create a sensor_msgs::PointCloud2 to fill with the data above
 *   sensor_msgs::PointCloud2 cloud;
 *   // Create a proxy to modify the PointCloud2
 *   sensor_msgs::PointCloud2Proxy<sensor_msgs::PointXYZRGB> cloud_proxy(cloud);
 *   // The constructor checks the fields of "cloud" automatically and sets them if not set (as
 *   // this is one of the two common types: sensor_msgs::PointXYZ and sensor_msgs::PointXYZRGB)
 *   // Fill the PointCloud2
 *   sensor_msgs::PointXYZRGB pt;
 *   cloud_proxy.reserve(n_points);
 *   for(size_t i=0; i<n_points; ++i) {
 *     pt.x = point_data[3*i+0];
 *     pt.y = point_data[3*i+1];
 *     pt.z = point_data[3*i+2];
 *     pt.r = color_data[3*i+0];
 *     pt.g = color_data[3*i+1];
 *     pt.b = color_data[3*i+2];
 *     cloud_proxy.push_back(pt);
 *   }
 *
 *
 *   // Alternative way of filling
 *   cloud_proxy.resize(n_points);
 *   for(size_t i=0; i<n_points; ++i) {
 *     sensor_msgs::PointXYZRGB &pt = cloud_proxy[i];
 *     pt.x = point_data[3*i+0];
 *     pt.y = point_data[3*i+1];
 *     pt.z = point_data[3*i+2];
 *     pt.r = color_data[3*i+0];
 *     pt.g = color_data[3*i+1];
 *     pt.b = color_data[3*i+2];
 *   }
 *
 *
 *   // Alternative way of filling (fastest, just like normal C++: pointers are faster than indexing)
 *   cloud_proxy.resize(n_points);
 *   sensor_msgs::PointXYZRGB *pt_ptr = &(cloud_proxy[0]);
 *   for(size_t i=0; i<n_points; ++i, ++pt_ptr) {
 *     pt_ptr->x = point_data[3*i+0];
 *     pt_ptr->y = point_data[3*i+1];
 *     pt_ptr->z = point_data[3*i+2];
 *     pt_ptr->r = color_data[3*i+0];
 *     pt_ptr->g = color_data[3*i+1];
 *     pt_ptr->b = color_data[3*i+2];
 *   }
 */

namespace sensor_msgs
{

/** Point type used in a sensor_msgs::PoinCloud2 that only contains x, y and z coordinates as floats */
struct PointXYZ {
  float x;
  float y;
  float z;
};

/** Point type used in a sensor_msgs::PoinCloud2 that contains x, y and z coordinates as floats,
 * and the colors RGB (or RGBA with alpha) packed in a float
 */
struct PointXYZRGB {
  float x;
  float y;
  float z;
  union {
    union {
      struct {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t a;
      };
      float rgb;
    };
    uint32_t rgba;
  };
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * E.g, you create your PointCloud2 message with XYZ/RGB as follows:
 *   setPointCloud2FieldsByString(cloud_msg, 4, "x", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "y", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "z", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "rgb", 1, sensor_msgs::PointField::FLOAT32);
 * For simple usual cases, the overloaded setPointCloud2FieldsByString is what you want.
 *
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as triplets: name of the field as char*,
 *          number of elements in the field, the datatype of the elements in the field
 * @return void
 */
void setPointCloud2Fields(PointCloud2& cloud_msg, int n_fields, ...);

/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * @param cloud_msg
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as strings: "xyz" (3 floats), "rgb" (3 uchar
 *            stacked in a float), "rgba" (4 uchar stacked in a float)
 * @return void
 */
void setPointCloud2FieldsByString(PointCloud2& cloud_msg, int n_fields, ...);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class enabling to read a sensor_msgs::PointCloud2 like a container
 * @param T the type of data each Point can be mapped to
 */
template<typename T>
class PointCloud2ConstProxy
{
public:
  /** Default constructor
   * @param cloud_msgs The sensor_msgs::PointCloud2 to read
   */
  PointCloud2ConstProxy(const PointCloud2& cloud_msg);

  /**
   * @return the number of T's in the original sensor_msgs::PointCloud2
   */
  size_t size() const;

  /**
   * @param i the index of the element T to access
   * @return a reference to the i^th T
   */
  const T& operator[](size_t i) const;

  /** Returns a reference to the element in the sensor_msgs::PointCloud2. No bounday checks are done
   * @param i the column of the element T to access
   * @param j the row of the element T to access
   * @return a reference to the T at the i^th column and j^th row
   */
  const T& operator()(size_t i, size_t j) const;
private:
  /** A reference to the original sensor_msgs::PointCloud2 that we read */
  const PointCloud2& cloud_msg_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class enabling to modify a sensor_msgs::PointCloud2 like a container
 * @param T the type of data each Point can be mapped to
 */
template<typename T>
class PointCloud2Proxy
{
public:
  /** Default constructor
   * @param cloud_msgs The sensor_msgs::PointCloud2 to modify
   */
  PointCloud2Proxy(PointCloud2& cloud_msg);

  /**
   * @return the number of T's in the original sensor_msgs::PointCloud2
   */
  size_t size() const;

  /**
   * @param size The number of T's to reserve in the original sensor_msgs::PointCloud2 for
   */
  void reserve(size_t size);

  /**
   * @param size The number of T's to change the size of the original sensor_msgs::PointCloud2 by
   */
  void resize(size_t size);

  /**
   * @brief remove all T's from the original sensor_msgs::PointCloud2
   */
  void clear();

  /**
   * @param i the index of the element T to access
   * @return a reference to the i^th T
   */
  T& operator[](size_t i);

  /**
   * @param i the column of the element T to access
   * @param j the row of the element T to access
   * @return a reference to the T at the i^th column and j^th row
   */
  T& operator()(size_t i, size_t j);

  /**
   * adds an element to the original sensor_msgs::PointCloud2. An error is thrown if the
   * original original sensor_msgs::PointCloud2 is ordered (with and height both > 1)
   * @param t the T element to copy at the end of the original sensor_msgs::PointCloud2
   */
  void push_back(const T& t);
private:
  /** A reference to the original sensor_msgs::PointCloud2 that we read */
  PointCloud2& cloud_msg_;
};
}

#include <sensor_msgs/impl/point_cloud2_proxy.h>

#endif// SENSOR_MSGS_POINT_CLOUD2_PROXY_H
