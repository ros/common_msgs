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

#ifndef SENSOR_MSGS_POINT_CLOUD_ITERATOR_H
#define SENSOR_MSGS_POINT_CLOUD_ITERATOR_H

#include <sensor_msgs/PointCloud2.h>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Offers an iterator over a PointCloud2
 * \author Vincent Rabaud
 */

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
void setPointCloud2Fields(sensor_msgs::PointCloud2* cloud_msg, int n_fields, ...);

/** Function setting some fields in a PointCloud and adjusting the internals of the PointCloud2
 * @param cloud_msg
 * @param cloud_msg the sensor_msgs::PointCloud2 message to modify
 * @param n_fields the number of fields to add. The fields are given as strings: "xyz" (3 floats), "rgb" (3 uchar
 *            stacked in a float), "rgba" (4 uchar stacked in a float)
 * @return a reference to the original PointCloud2 but modified
 */
void setPointCloud2FieldsByString(sensor_msgs::PointCloud2* cloud_msg, int n_fields, ...);
}

namespace
{
/** Private base class for PointCloud2Iterator and PointCloud2ConstIterator
 * T is the type of the value to be retrieved
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 */
template<typename T, typename U>
class PointCloud2IteratorBase
{
public:
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  void initialize(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** Const version of the above
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  void initialize(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  T& operator [](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  T& operator *() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  PointCloud2IteratorBase<T, U>& operator ++();

  /** Increase the iterator of a ceratin amount
   * @return a reference to the updated iterator
   */
  PointCloud2IteratorBase<T, U>& operator +(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator !=(const PointCloud2IteratorBase<T, U>& iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  PointCloud2IteratorBase<T, U> end() const;

private:
  /** Common code to set the fiels of the point cloud
   * @param cloud_msg the PointCloud2 to modify
   * @param field_name the name of the field to iterate upon
   * @return the offset at which the field is found
   */
  int set_fields(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** The "point_step" of the original cloud */
  int point_step_;
  /** The "count" of tha specific PointField */
  uint32_t field_count_;
  /** The raw data  in uchar* where the iterator is */
  U* data_char_;
  /** The cast data where the iterator is */
  T* data_;
  /** The end() pointer of the iterator */
  T* data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
};
}

namespace sensor_msgs
{
/** Class that can iterate over a PointCloud2
 * T type of the element being iterated upon
 * E.g, you create your PointClou2 message as follows:
 *   setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
 *
 * For iterating over XYZ, you do :
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive, you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointCloud2Iterator : public PointCloud2IteratorBase<T, unsigned char> {
public:
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2Iterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);
};

/** Same as a PointCloud2Iterator but for const data
 */
template<typename T>
class PointCloud2ConstIterator : public PointCloud2IteratorBase<const T, const unsigned char> {
public:
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2ConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);
};
}

#include <sensor_msgs/impl/point_cloud_iterator.h>

#endif// SENSOR_MSGS_POINT_CLOUD_ITERATOR_H
