/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
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

#ifndef SENSOR_MSGS_IMPL_POINT_CLOUD_ITERATOR_H
#define SENSOR_MSGS_IMPL_POINT_CLOUD_ITERATOR_H

#include <sensor_msgs/PointCloud2.h>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Private implementation used by PointCloud2Iterator
 * \author Vincent Rabaud
 */

namespace
{
/** Private function that adds a PointField to the "fields" member of a PointCloud2
 * @param cloud_msg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointField that will be added to the PointCLoud2
 */
inline int addPointField(sensor_msgs::PointCloud2* cloud_msg, const std::string &name, int count, int datatype,
    int offset)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg->fields.push_back(point_field);

  // Update the offset
  int point_field_type;
  if (datatype == sensor_msgs::PointField::INT8)
    point_field_type = sizeof(int8_t);
  else if (datatype == sensor_msgs::PointField::UINT8)
    point_field_type = sizeof(uint8_t);
  else if (datatype == sensor_msgs::PointField::INT16)
    point_field_type = sizeof(int16_t);
  else if (datatype == sensor_msgs::PointField::UINT16)
    point_field_type = sizeof(uint16_t);
  else if (datatype == sensor_msgs::PointField::INT32)
    point_field_type = sizeof(int32_t);
  else if (datatype == sensor_msgs::PointField::UINT32)
    point_field_type = sizeof(uint32_t);
  else if (datatype == sensor_msgs::PointField::FLOAT32)
    point_field_type = sizeof(float);
  else
    point_field_type = sizeof(double);

  return offset + point_field.count * point_field_type / sizeof(char);
}
}

namespace sensor_msgs
{
/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * E.g, you create your PointClou2 message with XYZ/RGB as follows:
 *   setPointCloud2FieldsByString(cloud_msg, 4, "x", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "y", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "z", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "rgb", 1, sensor_msgs::PointField::FLOAT32);
 * For simple usual cases, the overloaded setPointCloud2FieldsByString is what you want.
 *
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as triplets: name of the field as char*,
 *          number of elements in the field, the datatype of the elements in the field
 * @return a reference to the original PointCloud2 but modified
 */
inline void setPointCloud2Fields(sensor_msgs::PointCloud2* cloud_msg, int n_fields, ...)
{
  cloud_msg->fields.clear();
  cloud_msg->fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i)
    // Create the corresponding PointField
    offset = addPointField(cloud_msg, std::string(va_arg(vl, char *)), va_arg(vl, int), va_arg(vl, int), offset);
    va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
}

/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * @param cloud_msg
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as strings: "xyz" (3 floats), "rgb" (3 uchar
 *            stacked in a float), "rgba" (4 uchar stacked in a float)
 * @return a reference to the original PointCloud2 but modified
 */
inline void setPointCloud2FieldsByString(sensor_msgs::PointCloud2* cloud_msg, int n_fields, ...)
{
  cloud_msg->fields.clear();
  cloud_msg->fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i)
  {
    // Create the corresponding PointFields
    std::string
    field_name = std::string(va_arg(vl, char *));
    if (field_name == "xyz")
    {
      sensor_msgs::PointField point_field;
      // Do x, y and z
      offset = addPointField(cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    }
    else if ((field_name == "rgb") || (field_name == "rgba"))
    {
      offset = addPointField(cloud_msg, field_name, 1, sensor_msgs::PointField::FLOAT32, offset);
    }
    else
      throw std::runtime_error("Field " + field_name + " does not exist");
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
}
}

namespace
{

/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename U>
void PointCloud2IteratorBase<T, U>::initialize(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  int offset = set_fields(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<T*>(data_char_);
  data_end_ = reinterpret_cast<T*>(&(cloud_msg.data.back()) + 1 + offset);
}

/** Const version of the above
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename U>
void PointCloud2IteratorBase<T, U>::initialize(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  int offset = set_fields(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<T*>(data_char_);
  data_end_ = reinterpret_cast<T*>(&(cloud_msg.data.back()) + 1 + offset);
}

/** Access the i th element starting at the current pointer (useful when a field has several elements of the same
 * type)
 * @param i
 * @return a reference to the i^th value from the current position
 */
template<typename T, typename U>
T& PointCloud2IteratorBase<T, U>::operator [](size_t i) const
{
  if (is_bigendian_)
    return *(data_ - i);
  else
    return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template<typename T, typename U>
T& PointCloud2IteratorBase<T, U>::operator *() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U>& PointCloud2IteratorBase<T, U>::operator ++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<T*>(data_char_);
  return *this;
}

/** Increase the iterator of a ceratin amount
 * @return a reference to the updated iterator
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U>& PointCloud2IteratorBase<T, U>::operator +(int i)
{
  data_char_ += i*point_step_;
  data_ = reinterpret_cast<T*>(data_char_);
  return *this;
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template<typename T, typename U>
bool PointCloud2IteratorBase<T, U>::operator !=(const PointCloud2IteratorBase<T, U>& iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U> PointCloud2IteratorBase<T, U>::end() const
{
  PointCloud2IteratorBase<T, U> res;
  res.data_ = data_end_;
  return res;
}

/** Common code to set the fiels of the point cloud
  * @param cloud_msg the PointCloud2 to modify
  * @param field_name the name of the field to iterate upon
  * @return the offset at which the field is found
  */
template<typename T, typename U>
int PointCloud2IteratorBase<T, U>::set_fields(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian;
  point_step_ = cloud_msg.point_step;
  // make sure the channel is valid
  std::vector<sensor_msgs::PointField>::const_iterator field_iter = cloud_msg.fields.begin(), field_end =
      cloud_msg.fields.end();
  while ((field_iter != field_end) && (field_iter->name != field_name))
    ++field_iter;

  if (field_iter == field_end)
    throw std::runtime_error("Field " + field_name + " does not exist");

  // continue filling the info
  field_count_ = field_iter->count;

  if (is_bigendian_)
    return point_step_ - field_iter->offset;
  else
    return field_iter->offset;
}

}

namespace sensor_msgs
{
/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T>
PointCloud2Iterator<T>::PointCloud2Iterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  PointCloud2IteratorBase<T, unsigned char>::initialize(cloud_msg, field_name);
};

/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T>
PointCloud2ConstIterator<T>::PointCloud2ConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  PointCloud2IteratorBase<const T, const unsigned char>::initialize(cloud_msg, field_name);
};
}

#endif// SENSOR_MSGS_IMPL_POINT_CLOUD_ITERATOR_H
