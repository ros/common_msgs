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

#include <gtest/gtest.h>

#include <sensor_msgs/point_cloud_iterator.h>

TEST(sensor_msgs, PointCloud2Iterator)
{
  // Create a dummy PointCloud2
  int n_points = 4;
  sensor_msgs::PointCloud2 cloud_msg_1, cloud_msg_2;
  cloud_msg_1.height = n_points;
  cloud_msg_1.width = 1;
  setPointCloud2FieldsByString(&cloud_msg_1, 2, "xyz", "rgb");
  cloud_msg_2 = cloud_msg_1;

  // Fill 1 by hand
  float point_data_raw[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  std::vector<float> point_data(point_data_raw, point_data_raw + 3*n_points);
  uint8_t color_data_raw[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
  std::vector<uint8_t> color_data(color_data_raw, color_data_raw + 3*n_points);

  float *data = reinterpret_cast<float*>(&cloud_msg_1.data.front());
  for(size_t n=0, i=0, j=0; n<n_points; ++n) {
    for(; i<3*(n+1); ++i)
      *(data++) = point_data[i];
    uint8_t *rgb = reinterpret_cast<uint8_t*>(data++);
    for(; j<3*(n+1); ++j)
      *(rgb++) = color_data[j];
  }

  // Fill 2 using an iterator
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_2, "x");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg_2, "rgb");
  for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_rgb)
    for(size_t j=0; j<3; ++j) {
      iter_x[j] = point_data[j+3*i];
      iter_rgb[j] = color_data[j+3*i];
    }

  // Check the values using an iterator
  sensor_msgs::PointCloud2ConstIterator<float> iter_const_1_x(cloud_msg_1, "x"), iter_const_2_x(cloud_msg_2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_const_1_y(cloud_msg_1, "y"), iter_const_2_y(cloud_msg_2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_const_1_z(cloud_msg_1, "z"), iter_const_2_z(cloud_msg_2, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_const_1_rgb(cloud_msg_1, "rgb"), iter_const_2_rgb(cloud_msg_2, "rgb");

  size_t i=0;
  for(; iter_const_1_x != iter_const_1_x.end(); ++i, ++iter_const_1_x,
                               ++iter_const_2_x, ++iter_const_1_y, ++iter_const_2_y,++iter_const_1_z,
                               ++iter_const_2_z, ++iter_const_1_rgb, ++iter_const_2_rgb) {
    EXPECT_EQ(*iter_const_1_x, *iter_const_2_x);
    EXPECT_EQ(*iter_const_1_x, point_data[0+3*i]);
    EXPECT_EQ(*iter_const_1_y, *iter_const_2_y);
    EXPECT_EQ(*iter_const_1_y, point_data[1+3*i]);
    EXPECT_EQ(*iter_const_1_z, *iter_const_2_z);
    EXPECT_EQ(*iter_const_1_z, point_data[2+3*i]);
    for(size_t j=0; j<3; ++j) {
      EXPECT_EQ(iter_const_1_rgb[j], iter_const_2_rgb[j]);
      EXPECT_EQ(iter_const_1_rgb[j], color_data[j+3*i]);
    }
  }
  EXPECT_EQ(i, n_points);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
