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

#include <sensor_msgs/point_cloud2_proxy.h>

TEST(sensor_msgs, PointCloud2Proxy)
{
  // Create a dummy PointCloud2
  int n_points = 4;
  sensor_msgs::PointCloud2 cloud_msg_1, cloud_msg_2;
  cloud_msg_1.height = n_points;
  cloud_msg_1.width = 1;
  sensor_msgs::setPointCloud2FieldsByString(cloud_msg_1, 2, "xyz", "rgb");

  // Fill 1 by hand
  float point_data_raw[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  std::vector<float> point_data(point_data_raw, point_data_raw + 3 * n_points);
  uint8_t color_data_raw[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
  std::vector<uint8_t> color_data(color_data_raw, color_data_raw + 3 * n_points);

  cloud_msg_1.data.resize(n_points * sizeof(sensor_msgs::PointCloud2::_data_type::value_type));
  float* data = reinterpret_cast<float*>(&cloud_msg_1.data.front());
  for (size_t n = 0, i = 0, j = 0; n < n_points; ++n) {
    for (; i < 3 * (n + 1); ++i)
      *(data++) = point_data[i];
    uint8_t* rgb = reinterpret_cast<uint8_t*>(data++);
    for (; j < 3 * (n + 1); ++j)
      *(rgb++) = color_data[j];
  }

  // Fill 2 using a proxy
  sensor_msgs::PointCloud2Proxy<sensor_msgs::PointXYZRGB> cloud_msgs_2_proxy(cloud_msg_2);
  cloud_msgs_2_proxy.reserve(n_points + 1);
  sensor_msgs::PointXYZRGB pt;
  for (size_t i = 0; i < n_points; ++i) {
    pt.x = point_data[3 * i];
    pt.y = point_data[3 * i + 1];
    pt.z = point_data[3 * i + 2];
    pt.r = color_data[3 * i];
    pt.g = color_data[3 * i + 1];
    pt.b = color_data[3 * i + 2];
    cloud_msgs_2_proxy.push_back(pt);
  }
  // Check that reserves works properly
  EXPECT_EQ(cloud_msgs_2_proxy.size(), n_points);
  // Check that resize works properly
  cloud_msgs_2_proxy.resize(n_points + 10);
  cloud_msgs_2_proxy.resize(n_points);
  EXPECT_EQ(cloud_msgs_2_proxy.size(), n_points);

  // Check the values using a pointer
  sensor_msgs::PointXYZRGB* iter_1 = reinterpret_cast<sensor_msgs::PointXYZRGB*>(&cloud_msg_1.data.front());
  sensor_msgs::PointXYZRGB* iter_2 = &cloud_msgs_2_proxy[0];

  size_t i = 0;
  for (; i < n_points; ++i, ++iter_1, ++iter_2) {
    EXPECT_EQ(iter_1->x, iter_2->x);
    EXPECT_EQ(iter_1->x, point_data[0 + 3 * i]);
    EXPECT_EQ(iter_1->y, iter_2->y);
    EXPECT_EQ(iter_1->y, point_data[1 + 3 * i]);
    EXPECT_EQ(iter_1->z, iter_2->z);
    EXPECT_EQ(iter_1->z, point_data[2 + 3 * i]);
    // Compare RGB
    EXPECT_EQ(iter_1->r, iter_2->r);
    EXPECT_EQ(iter_1->r, color_data[0 + 3 * i]);
    EXPECT_EQ(iter_1->g, iter_2->g);
    EXPECT_EQ(iter_1->g, color_data[1 + 3 * i]);
    EXPECT_EQ(iter_1->b, iter_2->b);
    EXPECT_EQ(iter_1->b, color_data[2 + 3 * i]);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
