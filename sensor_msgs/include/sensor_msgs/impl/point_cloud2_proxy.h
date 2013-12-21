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

#ifndef SENSOR_MSGS_IMPL_POINT_CLOUD2_PROXY_H
#define SENSOR_MSGS_IMPL_POINT_CLOUD2_PROXY_H

#include <sensor_msgs/point_cloud2_proxy.h>
#include <sensor_msgs/PointCloud2.h>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Private implementation used by PointCloud2Proxy
 * \author Vincent Rabaud
 */


namespace
{
/** Returns the size of T with respect to the size of what makes a sensor_msgs::PointCloud2
 */
template<typename T>
size_t sizeofT()
{
  return sizeof(T) / sizeof(sensor_msgs::PointCloud2::_data_type::value_type);
}

/** Private function that adds a PointField to the "fields" member of a PointCloud2
* @param cloud_msg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointField that will be added to the PointCLoud2
 */
inline int addPointField(sensor_msgs::PointCloud2& cloud_msg, const std::string& name, int count, int datatype,
                         int offset)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  int point_field_size;
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
      point_field_size = sizeofT<int8_t>();
      break;
    case sensor_msgs::PointField::UINT8:
      point_field_size = sizeofT<uint8_t>();
      break;
    case sensor_msgs::PointField::INT16:
      point_field_size = sizeofT<int16_t>();
      break;
    case sensor_msgs::PointField::UINT16:
      point_field_size = sizeofT<uint16_t>();
      break;
    case sensor_msgs::PointField::INT32:
      point_field_size = sizeofT<int32_t>();
      break;
    case sensor_msgs::PointField::UINT32:
      point_field_size = sizeofT<uint32_t>();
      break;
    case sensor_msgs::PointField::FLOAT32:
      point_field_size = sizeofT<float>();
      break;
    default:
      point_field_size = sizeofT<double>();
  }
  return offset + point_field.count * point_field_size;
}
}

namespace sensor_msgs
{
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
inline void setPointCloud2Fields(PointCloud2& cloud_msg, int n_fields, ...)
{
  cloud_msg.fields.clear();
  cloud_msg.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i)
    // Create the corresponding PointField
    offset = addPointField(cloud_msg, std::string(va_arg(vl, char*)), va_arg(vl, int), va_arg(vl, int), offset);
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg.point_step = offset;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
}

/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * @param cloud_msg
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as strings: "xyz" (3 floats), "rgb" (3 uchar
 *            stacked in a float), "rgba" (4 uchar stacked in a float)
 * @return void
 */
inline void setPointCloud2FieldsByString(PointCloud2& cloud_msg, int n_fields, ...)
{
  cloud_msg.fields.clear();
  cloud_msg.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i) {
    // Create the corresponding PointFields
    std::string
    field_name = std::string(va_arg(vl, char*));
    if (field_name == "xyz") {
      sensor_msgs::PointField point_field;
      // Do x, y and z
      offset = addPointField(cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    } else
      if ((field_name == "rgb") || (field_name == "rgba")) {
        offset = addPointField(cloud_msg, field_name, 1, sensor_msgs::PointField::FLOAT32, offset);
      } else
        throw std::runtime_error("Field " + field_name + " does not exist");
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg.point_step = offset;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
inline PointCloud2ConstProxy<T>::PointCloud2ConstProxy(const PointCloud2& cloud_msg) : cloud_msg_(cloud_msg)
{
}

template<typename T>
inline size_t PointCloud2ConstProxy<T>::size() const
{
  return cloud_msg_.data.size() / sizeofT<T>();
}

template<typename T>
inline const T& PointCloud2ConstProxy<T>::operator[](size_t i) const
{
  return *reinterpret_cast<const T*>(&(cloud_msg_.data.front()) + i * sizeofT<T>());
}

template<typename T>
inline const T& PointCloud2ConstProxy<T>::operator()(size_t i, size_t j) const
{
  return *reinterpret_cast<const T*>(&(cloud_msg_.data.front()) + (j * cloud_msg_.width + i) * sizeofT<T>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
inline PointCloud2Proxy<T>::PointCloud2Proxy(PointCloud2& cloud_msg) : cloud_msg_(cloud_msg)
{
}

template<>
inline PointCloud2Proxy<sensor_msgs::PointXYZ>::PointCloud2Proxy(PointCloud2& cloud_msg) : cloud_msg_(cloud_msg)
{
  if (cloud_msg.fields.empty())
    setPointCloud2FieldsByString(cloud_msg, 1, "xyz");
}

template<>
inline PointCloud2Proxy<sensor_msgs::PointXYZRGB>::PointCloud2Proxy(PointCloud2& cloud_msg) : cloud_msg_(cloud_msg)
{
  if (cloud_msg.fields.empty())
    setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
}

template<typename T>
inline size_t PointCloud2Proxy<T>::size() const
{
  return cloud_msg_.data.size() / sizeofT<T>();
}

template<typename T>
inline T& PointCloud2Proxy<T>::operator[](size_t i)
{
  return *reinterpret_cast<T*>(&(cloud_msg_.data.front()) + i * sizeofT<T>());
}

template<typename T>
inline T& PointCloud2Proxy<T>::operator()(size_t i, size_t j)
{
  return *reinterpret_cast<T*>(&(cloud_msg_.data.front()) + (j * cloud_msg_.width + i) * sizeofT<T>());
}

template<typename T>
inline void PointCloud2Proxy<T>::reserve(size_t size)
{
  cloud_msg_.data.reserve(size * sizeofT<T>());
}

template<typename T>
inline void PointCloud2Proxy<T>::resize(size_t size)
{
  if ((cloud_msg_.height < 1) && (cloud_msg_.width < 1))
    throw std::runtime_error("Cannot resize a sensor_msgs::PointCloud2 that does not have a dimension <= to 1");

  cloud_msg_.data.resize(size * sizeofT<T>());

  // Update height/width
  if (cloud_msg_.height == 1)
    cloud_msg_.width = size;
  else
    if (cloud_msg_.width == 1)
      cloud_msg_.height = size;
    else {
      cloud_msg_.height = 1;
      cloud_msg_.width = size;
    }
}

template<typename T>
inline void PointCloud2Proxy<T>::clear()
{
  cloud_msg_.data.clear();

  // Update height/width
  if (cloud_msg_.height == 1)
    cloud_msg_.row_step = cloud_msg_.width = 0;
  else
    if (cloud_msg_.width == 1)
      cloud_msg_.height = 0;
    else
      cloud_msg_.row_step = cloud_msg_.width = cloud_msg_.height = 0;
}

template<typename T>
inline void PointCloud2Proxy<T>::push_back(const T& t)
{
  // Insert the element at the end of sensor_msgs::PointCloud2
  const sensor_msgs::PointCloud2::_data_type::value_type* ptr = reinterpret_cast<const sensor_msgs::PointCloud2::_data_type::value_type*>(&t);
  cloud_msg_.data.insert(cloud_msg_.data.end(), ptr, ptr + sizeofT<T>());

  // Update height/width
  if ((cloud_msg_.height == 0) && (cloud_msg_.width == 0)) {
    cloud_msg_.point_step = sizeofT<T>();
    cloud_msg_.height = cloud_msg_.width = 1;
  } else
    if (cloud_msg_.height == 1) {
      ++cloud_msg_.width;
      cloud_msg_.row_step += cloud_msg_.point_step;
    } else
      if (cloud_msg_.width == 1)
        ++cloud_msg_.height;
      else {
        cloud_msg_.height = 1;
        cloud_msg_.width = 1;
      }
}

}

#endif// SENSOR_MSGS_IMPL_POINT_CLOUD2_PROXY_H
