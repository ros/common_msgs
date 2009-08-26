#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('test_common_msgs')

import sys
import struct

import unittest

import rostest
import rosrecord
import rosbagmigration

import re
from cStringIO import StringIO
import os

import rospy

import math

def quaternion_from_euler(x, y, z):
  x /= 2.0
  y /= 2.0
  z /= 2.0
  ci = math.cos(x)
  si = math.sin(x)
  cj = math.cos(y)
  sj = math.sin(y)
  ck = math.cos(z)
  sk = math.sin(z)
  cc = ci*ck
  cs = ci*sk
  sc = si*ck
  ss = si*sk

  quaternion = [0.0,0.0,0.0,0.0]

  quaternion[0] = cj*sc - sj*cs
  quaternion[1] = cj*ss + sj*cc
  quaternion[2] = cj*cs - sj*sc
  quaternion[3] = cj*cc + sj*ss

  return quaternion



identity6x6 = [1.0] + 6*[0] + [1.0] + 6*[0] + [1.0] + 6*[0] + [1.0] + 6*[0] + [1.0] + 6*[0] + [1.0]


def repack(x):
  return struct.unpack('<f',struct.pack('<f',x))[0]

class TestCommonMsgsMigration(unittest.TestCase):


# (*) Acceleration.saved
# (*) AngularAcceleration.saved
# (*) AngularVelocity.saved
# (*) BatteryState.saved
# (*) ChannelFloat32.saved
# (*) DiagnosticMessage.saved
# (*) DiagnosticStatus.saved
# (*) Path.saved
# (*) Point32.saved
# (*) PointCloud.saved
# (*) Point.saved
# (*) PointStamped.saved
# (*) Polygon3D.saved
# (*) PoseDDot.saved
# (*) Pose.saved
# (*) PoseStamped.saved
# (*) PoseWithCovariance.saved
# (*) Quaternion.saved
# (*) QuaternionStamped.saved
# (*) Transform.saved
# (*) TransformStamped.saved
# ( ) TransformStamped_parent_id.saved
# (*) Twist.saved
# (*) Vector3.saved
# (*) Vector3Stamped.saved
# (*) Velocity.saved
# (*) Wrench.saved
# (*) DiagnosticString.saved
# (*) DiagnosticValue.saved
# (*) OccupancyGrid.saved
# (*) VOPose.saved
# (*) RobotBase2DOdom.saved


########### CamInfo ###############

  def get_old_cam_info(self):
    cam_info_classes = self.load_saved_classes('CamInfo.saved')

    cam_info = cam_info_classes['image_msgs/CamInfo']

    return cam_info(None,
                    480, 640,
                    (-0.45795000000000002, 0.29532999999999998, 0.0, 0.0, 0.0),
                    (734.37707999999998, 0.0, 343.25992000000002,
                     0.0, 734.37707999999998, 229.65119999999999,
                     0.0, 0.0, 1.0),
                    (0.99997999999999998, 0.0012800000000000001, -0.0057400000000000003,
                     -0.0012700000000000001, 1.0, 0.00148,
                     0.0057400000000000003, -0.00147, 0.99997999999999998),
                    (722.28197999999998, 0.0, 309.70123000000001, 0.0,
                     0.0, 722.28197999999998, 240.53899000000001, 0.0,
                     0.0, 0.0, 1.0, 0.0))

  def get_new_cam_info(self):
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import RegionOfInterest

    return CameraInfo(None,
                      480, 640,
                      RegionOfInterest(0,0,480,640),
                      (-0.45795000000000002, 0.29532999999999998, 0.0, 0.0, 0.0),
                      (734.37707999999998, 0.0, 343.25992000000002,
                       0.0, 734.37707999999998, 229.65119999999999,
                       0.0, 0.0, 1.0),
                      (0.99997999999999998, 0.0012800000000000001, -0.0057400000000000003,
                       -0.0012700000000000001, 1.0, 0.00148,
                       0.0057400000000000003, -0.00147, 0.99997999999999998),
                      (722.28197999999998, 0.0, 309.70123000000001, 0.0,
                       0.0, 722.28197999999998, 240.53899000000001, 0.0,
                       0.0, 0.0, 1.0, 0.0))

  def test_cam_info(self):
    self.do_test('cam_info', self.get_old_cam_info, self.get_new_cam_info)




########### CompressedImage ###############

  def get_old_compressed_image(self):
    compressed_image_classes = self.load_saved_classes('CompressedImage.saved')

    compressed_image = compressed_image_classes['sensor_msgs/CompressedImage']

    multi_array_layout = compressed_image_classes['std_msgs/MultiArrayLayout']
    multi_array_dimension = compressed_image_classes['std_msgs/MultiArrayDimension']
    uint8_multi_array = compressed_image_classes['std_msgs/UInt8MultiArray']

    import random
    r = random.Random(1234)

    # This is not a jpeg, but we don't really care.  It's just a binary blob.
    return compressed_image(None,
                            'image',
                            'mono',
                            'jpeg',
                            uint8_multi_array(multi_array_layout([multi_array_dimension('data', 1000, 1000)], 0),
                                              [r.randint(0,255) for x in xrange(0,1000)]))

  def get_new_compressed_image(self):
    from sensor_msgs.msg import CompressedImage

    import random
    r = random.Random(1234)

    return CompressedImage(None,
                            "jpeg",
                            [r.randint(0,255) for x in xrange(0,1000)])

  def test_compressed_image(self):
    self.do_test('compressed_image', self.get_old_compressed_image, self.get_new_compressed_image)




########### Image ###############

  def get_old_mono_image(self):
    image_classes = self.load_saved_classes('Image.saved')

    image = image_classes['image_msgs/Image']

    multi_array_layout = image_classes['std_msgs/MultiArrayLayout']
    multi_array_dimension = image_classes['std_msgs/MultiArrayDimension']

    uint8_multi_array = image_classes['std_msgs/UInt8MultiArray']
    int8_multi_array = image_classes['std_msgs/Int8MultiArray']
    uint16_multi_array = image_classes['std_msgs/UInt16MultiArray']
    int16_multi_array = image_classes['std_msgs/Int16MultiArray']
    uint32_multi_array = image_classes['std_msgs/UInt32MultiArray']
    int32_multi_array = image_classes['std_msgs/Int32MultiArray']
    uint64_multi_array = image_classes['std_msgs/UInt64MultiArray']
    int64_multi_array = image_classes['std_msgs/Int64MultiArray']

    float32_multi_array = image_classes['std_msgs/Float32MultiArray']
    float64_multi_array = image_classes['std_msgs/Float64MultiArray']    

    import random
    r = random.Random(1234)

    return image(None,
                 'image',
                 'mono',
                 'uint8',
                 uint8_multi_array(multi_array_layout([multi_array_dimension('height', 480, 307200),
                                                       multi_array_dimension('width', 640, 640),
                                                       multi_array_dimension('channel', 1, 1)
                                                       ], 0),
                                   [r.randint(0,255) for x in xrange(0,307200)]),
                 int8_multi_array(),
                 uint16_multi_array(),
                 int16_multi_array(),
                 uint32_multi_array(),
                 int32_multi_array(),
                 uint64_multi_array(),
                 int64_multi_array(),
                 float32_multi_array(),
                 float64_multi_array())

  def get_new_mono_image(self):
    from sensor_msgs.msg import Image

    import random
    r = random.Random(1234)

    return Image(None,
                 480,
                 640,
                 'mono8',
                 0,
                 640,
                 [r.randint(0,255) for x in xrange(0,307200)])

  def test_mono_image(self):
    self.do_test('mono_image', self.get_old_mono_image, self.get_new_mono_image)


  def get_old_rgb_image(self):
    image_classes = self.load_saved_classes('Image.saved')

    image = image_classes['image_msgs/Image']

    multi_array_layout = image_classes['std_msgs/MultiArrayLayout']
    multi_array_dimension = image_classes['std_msgs/MultiArrayDimension']

    uint8_multi_array = image_classes['std_msgs/UInt8MultiArray']
    int8_multi_array = image_classes['std_msgs/Int8MultiArray']
    uint16_multi_array = image_classes['std_msgs/UInt16MultiArray']
    int16_multi_array = image_classes['std_msgs/Int16MultiArray']
    uint32_multi_array = image_classes['std_msgs/UInt32MultiArray']
    int32_multi_array = image_classes['std_msgs/Int32MultiArray']
    uint64_multi_array = image_classes['std_msgs/UInt64MultiArray']
    int64_multi_array = image_classes['std_msgs/Int64MultiArray']

    float32_multi_array = image_classes['std_msgs/Float32MultiArray']
    float64_multi_array = image_classes['std_msgs/Float64MultiArray']    

    import random
    r = random.Random(1234)

    return image(None,
                 'image',
                 'rgb',
                 'uint8',
                 uint8_multi_array(multi_array_layout([multi_array_dimension('height', 480, 921600),
                                                       multi_array_dimension('width',  640, 1920),
                                                       multi_array_dimension('channel',  3, 3)
                                                       ], 0),
                                   [r.randint(0,255) for x in xrange(0,921600)]),
                 int8_multi_array(),
                 uint16_multi_array(),
                 int16_multi_array(),
                 uint32_multi_array(),
                 int32_multi_array(),
                 uint64_multi_array(),
                 int64_multi_array(),
                 float32_multi_array(),
                 float64_multi_array())

  def get_new_rgb_image(self):
    from sensor_msgs.msg import Image

    import random
    r = random.Random(1234)

    return Image(None,
                 480,
                 640,
                 'rgb8',
                 0,
                 1920,
                 [r.randint(0,255) for x in xrange(0,921600)])

  def test_rgb_image(self):
    self.do_test('rgb_image', self.get_old_rgb_image, self.get_new_rgb_image)



########### RobotBase2DOdom ###############

  def get_old_robot_base_2d_odom(self):
    robot_base_2d_odom_classes = self.load_saved_classes('RobotBase2DOdom.saved')

    robot_base_2d_odom = robot_base_2d_odom_classes['deprecated_msgs/RobotBase2DOdom']
    pose_2d_float32    = robot_base_2d_odom_classes['deprecated_msgs/Pose2DFloat32']
    
    return robot_base_2d_odom(None, pose_2d_float32(3.33, 2.22, 1.11), pose_2d_float32(.1,.2,.3), 0, 1)

  def get_new_robot_base_2d_odom(self):
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovariance
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    from geometry_msgs.msg import TwistWithCovariance
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import Vector3
    
    # We have to repack x,y values because they moved from float to double
    p = PoseWithCovariance(Pose(Point(repack(3.33), repack(2.22), 0), apply(Quaternion,quaternion_from_euler(0,0,repack(1.11)))), 36*[0.])
    t = TwistWithCovariance(Twist(Vector3(repack(.1), repack(.2), 0), Vector3(0, 0, repack(.3))), 36*[0.])

    return Odometry(None, 'base_footprint', p, t)

  def test_robot_base_2d_odom(self):
    self.do_test('robot_base_2d_odom', self.get_old_robot_base_2d_odom, self.get_new_robot_base_2d_odom)




########### OccupancyGrid ###############

  def get_old_occupancy_grid(self):
    occupancy_grid_classes = self.load_saved_classes('OccGrid.saved')

    occupancy_grid = occupancy_grid_classes['robot_msgs/OccGrid']
    map_meta_data  = occupancy_grid_classes['robot_msgs/MapMetaData']
    pose           = occupancy_grid_classes['robot_msgs/Pose']
    point          = occupancy_grid_classes['robot_msgs/Point']
    quaternion     = occupancy_grid_classes['robot_msgs/Quaternion']

    import random
    r = random.Random(1234)
    
    return occupancy_grid(map_meta_data(rospy.Time(123,456), 0.1, 100, 100, pose(point(1.23, 4.56, 7.89), quaternion(0,0,0,1))), [r.randint(-1,100) for x in xrange(0,1000)])

  def get_new_occupancy_grid(self):
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.msg import MapMetaData
    from roslib.msg import Header
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    import random
    r = random.Random(1234)
    
    return OccupancyGrid(Header(0, rospy.Time(0,0), "/map"), MapMetaData(rospy.Time(123,456), 0.1, 100, 100, Pose(Point(1.23, 4.56, 7.89), Quaternion(0,0,0,1))), [r.randint(-1,100) for x in xrange(0,1000)])

  def test_occupancy_grid(self):
    self.do_test('occupancy_grid', self.get_old_occupancy_grid, self.get_new_occupancy_grid)


########### Polygon3d ###############

  def get_old_polygon3d(self):
    polygon3d_classes = self.load_saved_classes('Polygon3D.saved')

    polygon3d = polygon3d_classes['robot_msgs/Polygon3D']
    point32   = polygon3d_classes['robot_msgs/Point32']
    color_rgba   = polygon3d_classes['std_msgs/ColorRGBA']

    return polygon3d([point32(1.23, 4.56, 7.89), point32(82.861, 31.028, 93.317), point32(87.569, 29.085, 33.415)], color_rgba(255, 100, 100, 0))

  def get_new_polygon3d(self):
    from geometry_msgs.msg import Polygon
    from geometry_msgs.msg import Point32

    return Polygon([Point32(1.23, 4.56, 7.89), Point32(82.861, 31.028, 93.317), Point32(87.569, 29.085, 33.415)])

  def test_polygon3d(self):
    self.do_test('polygon3d', self.get_old_polygon3d, self.get_new_polygon3d)


########### Acceleration ###############

  def get_old_acceleration(self):
    acceleration_classes = self.load_saved_classes('Acceleration.saved')

    acceleration = acceleration_classes['robot_msgs/Acceleration']

    return acceleration(1.23, 4.56, 7.89)

  def get_new_acceleration(self):
    from geometry_msgs.msg import Vector3

    return Vector3(1.23, 4.56, 7.89)

  def test_acceleration(self):
    self.do_test('acceleration', self.get_old_acceleration, self.get_new_acceleration)



########### AngularAcceleration ###############

  def get_old_angular_acceleration(self):
    angular_acceleration_classes = self.load_saved_classes('AngularAcceleration.saved')

    angular_acceleration = angular_acceleration_classes['robot_msgs/AngularAcceleration']

    return angular_acceleration(1.23, 4.56, 7.89)

  def get_new_angular_acceleration(self):
    from geometry_msgs.msg import Vector3

    return Vector3(1.23, 4.56, 7.89)

  def test_angular_acceleration(self):
    self.do_test('angular_acceleration', self.get_old_angular_acceleration, self.get_new_angular_acceleration)


########### DiagnosticValue ###############

  def get_old_diagnostic_value(self):
    diagnostic_value_classes = self.load_saved_classes('DiagnosticValue.saved')
    
    diagnostic_value  = diagnostic_value_classes['diagnostic_msgs/DiagnosticValue']

    return diagnostic_value(42.42, 'foo')

  def get_new_diagnostic_value(self):
    from diagnostic_msgs.msg import KeyValue

    return KeyValue('foo', str(repack(42.42)))


  def test_diagnostic_value(self):
    self.do_test('diagnostic_value', self.get_old_diagnostic_value, self.get_new_diagnostic_value)


########### DiagnosticString ###############

  def get_old_diagnostic_string(self):
    diagnostic_string_classes = self.load_saved_classes('DiagnosticString.saved')
    
    diagnostic_string  = diagnostic_string_classes['diagnostic_msgs/DiagnosticString']

    return diagnostic_string('xxxxx', 'bar')

  def get_new_diagnostic_string(self):
    from diagnostic_msgs.msg import KeyValue

    return KeyValue('bar', 'xxxxx')


  def test_diagnostic_string(self):
    self.do_test('diagnostic_string', self.get_old_diagnostic_string, self.get_new_diagnostic_string)


########### DiagnosticStatus ###############

  def get_old_diagnostic_status(self):
    diagnostic_status_classes = self.load_saved_classes('DiagnosticStatus.saved')
    
    diagnostic_status = diagnostic_status_classes['diagnostic_msgs/DiagnosticStatus']
    diagnostic_value  = diagnostic_status_classes['diagnostic_msgs/DiagnosticValue']
    diagnostic_string = diagnostic_status_classes['diagnostic_msgs/DiagnosticString']

    return diagnostic_status(0, "abcdef", "ghijkl", [diagnostic_value(42.42, 'foo')], [diagnostic_string('xxxxx', 'bar')])

  def get_new_diagnostic_status(self):
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_msgs.msg import KeyValue

    return DiagnosticStatus(0, "abcdef", "ghijkl", "NONE", [KeyValue('foo', str(repack(42.42))), KeyValue('bar', 'xxxxx')])


  def test_diagnostic_status(self):
    self.do_test('diagnostic_status', self.get_old_diagnostic_status, self.get_new_diagnostic_status)


########### DiagnosticMessage ###############

  def get_old_diagnostic_message(self):
    diagnostic_message_classes = self.load_saved_classes('DiagnosticMessage.saved')
    
    diagnostic_message = diagnostic_message_classes['diagnostic_msgs/DiagnosticMessage']
    diagnostic_status = diagnostic_message_classes['diagnostic_msgs/DiagnosticStatus']
    diagnostic_value  = diagnostic_message_classes['diagnostic_msgs/DiagnosticValue']
    diagnostic_string = diagnostic_message_classes['diagnostic_msgs/DiagnosticString']

    msg = diagnostic_message(None, [])
    msg.status.append(diagnostic_status(0, "abcdef", "ghijkl", [diagnostic_value(12.34, 'abc')], [diagnostic_string('ghbvf', 'jkl')]))
    msg.status.append(diagnostic_status(0, "mnop", "qrst", [diagnostic_value(56.78, 'def')], [diagnostic_string('klmnh', 'mno')]))
    msg.status.append(diagnostic_status(0, "uvw", "xyz", [diagnostic_value(90.12, 'ghi')], [diagnostic_string('erfcd', 'pqr')]))

    return msg

  def get_new_diagnostic_message(self):
    from diagnostic_msgs.msg import DiagnosticArray
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_msgs.msg import KeyValue

    msg = DiagnosticArray(None, [])
    msg.status.append(DiagnosticStatus(0, "abcdef", "ghijkl", "NONE", [KeyValue('abc', str(repack(12.34))), KeyValue('jkl', 'ghbvf')]))
    msg.status.append(DiagnosticStatus(0, "mnop", "qrst", "NONE", [KeyValue('def', str(repack(56.78))), KeyValue('mno', 'klmnh')]))
    msg.status.append(DiagnosticStatus(0, "uvw", "xyz", "NONE", [KeyValue('ghi', str(repack(90.12))), KeyValue('pqr', 'erfcd')]))

    return msg


  def test_diagnostic_message(self):
    self.do_test('diagnostic_message', self.get_old_diagnostic_message, self.get_new_diagnostic_message)



########### Vector3 ###############


  def get_old_vector3(self):
    vector3_classes = self.load_saved_classes('Vector3.saved')
    
    vector3  = vector3_classes['robot_msgs/Vector3']
    
    return vector3(1.23, 4.56, 7.89)

  def get_new_vector3(self):
    from geometry_msgs.msg import Vector3
    
    return Vector3(1.23, 4.56, 7.89)


  def test_vector3(self):
    self.do_test('vector3', self.get_old_vector3, self.get_new_vector3)


########### Vector3Stamped ###############


  def get_old_vector3_stamped(self):
    vector3_classes = self.load_saved_classes('Vector3.saved')
    vector3_stamped_classes = self.load_saved_classes('Vector3Stamped.saved')
    
    vector3  = vector3_classes['robot_msgs/Vector3']
    vector3_stamped  = vector3_stamped_classes['robot_msgs/Vector3Stamped']
    
    return vector3_stamped(None, vector3(1.23, 4.56, 7.89))

  def get_new_vector3_stamped(self):
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Vector3Stamped
    
    return Vector3Stamped(None, Vector3(1.23, 4.56, 7.89))


  def test_vector3_stamped(self):
    self.do_test('vector3_stamped', self.get_old_vector3_stamped, self.get_new_vector3_stamped)


########### Quaternion ###############


  def get_old_quaternion(self):
    quaternion_classes = self.load_saved_classes('Quaternion.saved')
    
    quaternion  = quaternion_classes['robot_msgs/Quaternion']
    
    return quaternion(1.23, 4.56, 7.89, 1.23)

  def get_new_quaternion(self):
    from geometry_msgs.msg import Quaternion
    
    return Quaternion(1.23, 4.56, 7.89, 1.23)


  def test_quaternion(self):
    self.do_test('quaternion', self.get_old_quaternion, self.get_new_quaternion)


########### QuaternionStamped ###############


  def get_old_quaternion_stamped(self):
    quaternion_classes = self.load_saved_classes('QuaternionStamped.saved')
    
    quaternion_stamped  = quaternion_classes['robot_msgs/QuaternionStamped']
    quaternion  = quaternion_classes['robot_msgs/Quaternion']
    
    return quaternion_stamped(None, quaternion(1.23, 4.56, 7.89, 1.23))

  def get_new_quaternion_stamped(self):
    from geometry_msgs.msg import QuaternionStamped
    from geometry_msgs.msg import Quaternion
    
    return QuaternionStamped(None, Quaternion(1.23, 4.56, 7.89, 1.23))


  def test_quaternion_stamped(self):
    self.do_test('quaternion_stamped', self.get_old_quaternion_stamped, self.get_new_quaternion_stamped)


########### Point ###############


  def get_old_point(self):
    point_classes = self.load_saved_classes('Point.saved')
    
    point  = point_classes['robot_msgs/Point']
    
    return point(1.23, 4.56, 7.89)

  def get_new_point(self):
    from geometry_msgs.msg import Point
    
    return Point(1.23, 4.56, 7.89)


  def test_point(self):
    self.do_test('point', self.get_old_point, self.get_new_point)


########### PointStamped ###############


  def get_old_point_stamped(self):
    point_classes = self.load_saved_classes('PointStamped.saved')
    
    point_stamped  = point_classes['robot_msgs/PointStamped']
    point  = point_classes['robot_msgs/Point']
    
    return point_stamped(None, point(1.23, 4.56, 7.89))

  def get_new_point_stamped(self):
    from geometry_msgs.msg import PointStamped
    from geometry_msgs.msg import Point
    
    return PointStamped(None, Point(1.23, 4.56, 7.89))


  def test_point_stamped(self):
    self.do_test('point_stamped', self.get_old_point_stamped, self.get_new_point_stamped)


########### Point32 ###############


  def get_old_point32(self):
    point32_classes = self.load_saved_classes('Point32.saved')
    
    point32  = point32_classes['robot_msgs/Point32']
    
    return point32(1.23, 4.56, 7.89)

  def get_new_point32(self):
    from geometry_msgs.msg import Point32
    
    return Point32(1.23, 4.56, 7.89)


  def test_point32(self):
    self.do_test('point32', self.get_old_point32, self.get_new_point32)


########### Transform ###############


  def get_old_transform(self):
    transform_classes = self.load_saved_classes('Transform.saved')
    
    transform  = transform_classes['robot_msgs/Transform']
    vector3  = transform_classes['robot_msgs/Vector3']
    quaternion  = transform_classes['robot_msgs/Quaternion']
    
    return transform(vector3(1.23, 4.56, 7.89), quaternion(0,0,0,1))

  def get_new_transform(self):
    from geometry_msgs.msg import Transform
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Quaternion
    
    return Transform(Vector3(1.23, 4.56, 7.89), Quaternion(0,0,0,1))


  def test_transform(self):
    self.do_test('transform', self.get_old_transform, self.get_new_transform)


########### TransformStamped ###############


  def get_old_transform_stamped(self):
    transform_classes = self.load_saved_classes('TransformStamped.saved')
    
    transform_stamped  = transform_classes['robot_msgs/TransformStamped']
    transform  = transform_classes['robot_msgs/Transform']
    vector3  = transform_classes['robot_msgs/Vector3']
    quaternion  = transform_classes['robot_msgs/Quaternion']
    
    ts = transform_stamped(None, "parent_frame", transform(vector3(1.23, 4.56, 7.89), quaternion(0,0,0,1)))
    ts.header.frame_id = "child_frame"
    return ts

  def get_old_transform_stamped_parent_id(self):
    transform_classes = self.load_saved_classes('TransformStamped_parent_id.saved')
    
    transform_stamped  = transform_classes['geometry_msgs/TransformStamped']
    transform  = transform_classes['geometry_msgs/Transform']
    vector3  = transform_classes['geometry_msgs/Vector3']
    quaternion  = transform_classes['geometry_msgs/Quaternion']
    header  = transform_classes['roslib/Header']

    ts = transform_stamped(None, "parent_frame", transform(vector3(1.23, 4.56, 7.89), quaternion(0,0,0,1)))
    ts.header.frame_id = "child_frame"
    return ts

  def get_new_transform_stamped(self):
    from geometry_msgs.msg import TransformStamped
    from geometry_msgs.msg import Transform
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Quaternion
    ts = TransformStamped(None, "child_frame", Transform(Vector3(1.23, 4.56, 7.89), Quaternion(0,0,0,1)))
    ts.header.frame_id = "parent_frame"
    return ts

  def test_transform_stamped(self):
    self.do_test('transform_stamped', self.get_old_transform_stamped, self.get_new_transform_stamped)

  def test_transform_stamped_parent_id(self):
    self.do_test('transform_stamped_parent_id', self.get_old_transform_stamped_parent_id, self.get_new_transform_stamped)


########### Pose ###############


  def get_old_pose(self):
    pose_classes = self.load_saved_classes('Pose.saved')
    
    pose  = pose_classes['robot_msgs/Pose']
    point  = pose_classes['robot_msgs/Point']
    quaternion  = pose_classes['robot_msgs/Quaternion']
    
    return pose(point(1.23, 4.56, 7.89), quaternion(0,0,0,1))

  def get_new_pose(self):
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    return Pose(Point(1.23, 4.56, 7.89), Quaternion(0,0,0,1))


  def test_pose(self):
    self.do_test('pose', self.get_old_pose, self.get_new_pose)



########### PoseStamped ###############


  def get_old_pose_stamped(self):
    pose_classes = self.load_saved_classes('PoseStamped.saved')
    
    pose_stamped  = pose_classes['robot_msgs/PoseStamped']
    pose  = pose_classes['robot_msgs/Pose']
    point  = pose_classes['robot_msgs/Point']
    quaternion  = pose_classes['robot_msgs/Quaternion']
    
    return pose_stamped(None, pose(point(1.23, 4.56, 7.89), quaternion(0,0,0,1)))

  def get_new_pose_stamped(self):
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    return PoseStamped(None, Pose(Point(1.23, 4.56, 7.89), Quaternion(0,0,0,1)))


  def test_pose_stamped(self):
    self.do_test('pose_stamped', self.get_old_pose_stamped, self.get_new_pose_stamped)



########### VOPose ###############


#  def get_old_vo_pose(self):
#    vo_pose_classes = self.load_saved_classes('VOPose.saved')
#    
#    vo_pose  = vo_pose_classes['robot_msgs/VOPose']
#    pose  = vo_pose_classes['robot_msgs/Pose']
#    point  = vo_pose_classes['robot_msgs/Point']
#    quaternion  = vo_pose_classes['robot_msgs/Quaternion']
#    
#    return vo_pose(None, pose(point(1.23, 4.56, 7.89), quaternion(0,0,0,1)), 123)
#
#  def get_new_vo_pose(self):
#    from deprecated_msgs.msg import VOPose
#    from geometry_msgs.msg import Pose
#    from geometry_msgs.msg import Point
#    from geometry_msgs.msg import Quaternion
#    
#    return VOPose(None, Pose(Point(1.23, 4.56, 7.89), Quaternion(0,0,0,1)), 123)
#
#
#  def test_vo_pose(self):
#    self.do_test('vo_pose', self.get_old_vo_pose, self.get_new_vo_pose)



########### PoseWithCovariance ###############


  def get_old_pose_with_covariance(self):
    pose_with_covariance_classes = self.load_saved_classes('PoseWithCovariance.saved')
    
    pose_with_covariance  = pose_with_covariance_classes['robot_msgs/PoseWithCovariance']
    pose  = pose_with_covariance_classes['robot_msgs/Pose']
    point  = pose_with_covariance_classes['robot_msgs/Point']
    quaternion  = pose_with_covariance_classes['robot_msgs/Quaternion']
    
    return pose_with_covariance(None, pose(point(1.23, 4.56, 7.89), quaternion(0,0,0,1)), identity6x6)

  def get_new_pose_with_covariance(self):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from geometry_msgs.msg import PoseWithCovariance
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    return PoseWithCovarianceStamped(None, PoseWithCovariance(Pose(Point(1.23, 4.56, 7.89), Quaternion(0,0,0,1)), identity6x6))


  def test_pose_with_covariance(self):
    self.do_test('pose_with_covariance', self.get_old_pose_with_covariance, self.get_new_pose_with_covariance)



########### PoseDot ###############


  def get_old_pose_dot(self):
    pose_dot_classes = self.load_saved_classes('PoseDot.saved')
    
    pose_dot          = pose_dot_classes['robot_msgs/PoseDot']
    velocity          = pose_dot_classes['robot_msgs/Velocity']
    angular_velocity  = pose_dot_classes['robot_msgs/AngularVelocity']
    
    return pose_dot(velocity(1.23, 4.56, 7.89), angular_velocity(9.87, 6.54, 3.21))

  def get_new_pose_dot(self):
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import Vector3
    
    return Twist(Vector3(1.23, 4.56, 7.89), Vector3(9.87, 6.54, 3.21))

  def test_pose_dot(self):
    self.do_test('pose_dot', self.get_old_pose_dot, self.get_new_pose_dot)




########### Particle Cloud ###############


  def get_old_particle_cloud(self):
    particle_cloud_classes = self.load_saved_classes('ParticleCloud.saved')
    
    particle_cloud  = particle_cloud_classes['robot_msgs/ParticleCloud']
    pose  = particle_cloud_classes['robot_msgs/Pose']
    point  = particle_cloud_classes['robot_msgs/Point']
    quaternion  = particle_cloud_classes['robot_msgs/Quaternion']
    
    return particle_cloud([pose(point(1.23, 4.56, 7.89), quaternion(1,0,0,1)),
                           pose(point(1.25, 4.58, 7.91), quaternion(0,1,0,1)),
                           pose(point(1.27, 4.60, 7.93), quaternion(0,0,1,1))])

  def get_new_particle_cloud(self):
    from geometry_msgs.msg import PoseArray
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    return PoseArray(None, [Pose(Point(1.23, 4.56, 7.89), Quaternion(1,0,0,1)),
                            Pose(Point(1.25, 4.58, 7.91), Quaternion(0,1,0,1)),
                            Pose(Point(1.27, 4.60, 7.93), Quaternion(0,0,1,1))])


  def test_particle_cloud(self):
    self.do_test('particle_cloud', self.get_old_particle_cloud, self.get_new_particle_cloud)



########### Path ###############


  def get_old_path(self):
    path_classes = self.load_saved_classes('Path.saved')
    


    path  = path_classes['robot_msgs/Path']
    header = path_classes['roslib/Header']
    pose_stamped  = path_classes['robot_msgs/PoseStamped']
    pose  = path_classes['robot_msgs/Pose']
    point  = path_classes['robot_msgs/Point']
    quaternion  = path_classes['robot_msgs/Quaternion']
    
    return path([pose_stamped(header(0,rospy.Time(0,0),'foo'),    pose(point(1.23, 4.56, 7.89), quaternion(1,0,0,1))),
                       pose_stamped(header(0,rospy.Time(0,1000),'foo'), pose(point(1.25, 4.58, 7.91), quaternion(0,1,0,1))),
                       pose_stamped(header(0,rospy.Time(0,2000),'foo'), pose(point(1.27, 4.60, 7.93), quaternion(0,0,1,1)))])

  def get_new_path(self):
    from nav_msgs.msg import Path
    from roslib.msg import Header
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    
    return Path(None, [PoseStamped(Header(0,rospy.Time(0,0),'foo'),    Pose(Point(1.23, 4.56, 7.89), Quaternion(1,0,0,1))),
                       PoseStamped(Header(0,rospy.Time(0,1000),'foo'), Pose(Point(1.25, 4.58, 7.91), Quaternion(0,1,0,1))),
                       PoseStamped(Header(0,rospy.Time(0,2000),'foo'), Pose(Point(1.27, 4.60, 7.93), Quaternion(0,0,1,1)))])


  def test_path(self):
    self.do_test('path', self.get_old_path, self.get_new_path)




########### Velocity ###############


  def get_old_velocity(self):
    velocity_classes = self.load_saved_classes('Velocity.saved')
    
    velocity  = velocity_classes['robot_msgs/Velocity']
    
    return velocity(1.23, 4.56, 7.89)

  def get_new_velocity(self):
    from geometry_msgs.msg import Vector3
    
    return Vector3(1.23, 4.56, 7.89)


  def test_velocity(self):
    self.do_test('velocity', self.get_old_velocity, self.get_new_velocity)


########### Angular Velocity ###############


  def get_old_angular_velocity(self):
    angular_velocity_classes = self.load_saved_classes('AngularVelocity.saved')
    
    angular_velocity  = angular_velocity_classes['robot_msgs/AngularVelocity']
    
    return angular_velocity(1.23, 4.56, 7.89)

  def get_new_angular_velocity(self):
    from geometry_msgs.msg import Vector3
    
    return Vector3(1.23, 4.56, 7.89)


  def test_angular_velocity(self):
    self.do_test('angular_velocity', self.get_old_angular_velocity, self.get_new_angular_velocity)


########### Twist ###############


  def get_old_twist(self):
    twist_classes = self.load_saved_classes('Twist.saved')
    
    twist    = twist_classes['robot_msgs/Twist']
    vector3  = twist_classes['robot_msgs/Vector3']
    
    return twist(None, vector3(1,2,3), vector3(4, 5, 6))

  def get_new_twist(self):
    from geometry_msgs.msg import TwistStamped
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import Vector3

    return TwistStamped(None, Twist(Vector3(1,2,3), Vector3(4, 5, 6)))


  def test_twist(self):
    self.do_test('twist', self.get_old_twist, self.get_new_twist)




########### Wrench ###############


  def get_old_wrench(self):
    wrench_classes = self.load_saved_classes('Wrench.saved')
    
    wrench    = wrench_classes['robot_msgs/Wrench']
    vector3   = wrench_classes['robot_msgs/Vector3']
    
    return wrench(None, vector3(1,2,3), vector3(4, 5, 6))

  def get_new_wrench(self):
    from geometry_msgs.msg import WrenchStamped
    from geometry_msgs.msg import Wrench
    from geometry_msgs.msg import Vector3

    return WrenchStamped(None, Wrench(Vector3(1,2,3), Vector3(4, 5, 6)))


  def test_wrench(self):
    self.do_test('wrench', self.get_old_wrench, self.get_new_wrench)






########### Channelfloat32 ###############


  def get_old_channelfloat32(self):
    channelfloat32_classes = self.load_saved_classes('ChannelFloat32.saved')
    
    channelfloat32  = channelfloat32_classes['robot_msgs/ChannelFloat32']
    
    return channelfloat32("myname", [1.23, 4.56, 7.89])

  def get_new_channelfloat32(self):
    from sensor_msgs.msg import ChannelFloat32
    
    return ChannelFloat32("myname", [1.23, 4.56, 7.89])


  def test_channelfloat32(self):
    self.do_test('channelfloat32', self.get_old_channelfloat32, self.get_new_channelfloat32)


########### PointCloud ###############


  def get_old_pointcloud(self):
    pointcloud_classes = self.load_saved_classes('PointCloud.saved')
    
    pointcloud  = pointcloud_classes['robot_msgs/PointCloud']
    point32  = pointcloud_classes['robot_msgs/Point32']
    channelfloat32  = pointcloud_classes['robot_msgs/ChannelFloat32']

    points = [point32(1,2,3), point32(4,5,6)]
    channels = [channelfloat32("myname1", [1.23, 4.56]),
                channelfloat32("myname2", [1.231, 4.561])]
    return pointcloud(None, points, channels)

  def get_new_pointcloud(self):
    from sensor_msgs.msg import PointCloud
    from geometry_msgs.msg import Point32
    from sensor_msgs.msg import ChannelFloat32

    Points = [Point32(1,2,3), Point32(4,5,6)]
    Channels = [ChannelFloat32("myname1", [1.23, 4.56]),
                ChannelFloat32("myname2", [1.231, 4.561])]

    
    return PointCloud(None, Points, Channels)


  def test_pointcloud(self):
    self.do_test('pointcloud', self.get_old_pointcloud, self.get_new_pointcloud)


########### Helper functions ###########


  def setUp(self):
    self.pkg_dir = roslib.packages.get_pkg_dir("test_common_msgs")


  def load_saved_classes(self,saved_msg):
    f = open("%s/test/saved/%s"%(self.pkg_dir,saved_msg), 'r')

    type_line = f.readline()
    pat = re.compile(r"\[(.*)]:")
    type_match = pat.match(type_line)

    self.assertTrue(type_match is not None, "Full definition file malformed.  First line should be: '[my_package/my_msg]:'")

    saved_type = type_match.groups()[0]
    saved_full_text = f.read()

    saved_classes = roslib.genpy.generate_dynamic(saved_type,saved_full_text)

    self.assertTrue(saved_classes is not None, "Could not generate class from full definition file.")
    self.assertTrue(saved_classes.has_key(saved_type), "Could not generate class from full definition file.")

    return saved_classes

  def do_test(self, name, old_msg, new_msg):
    # Name the bags
    oldbag = "%s/test/%s_old.bag"%(self.pkg_dir,name)
    newbag = "%s/test/%s_new.bag"%(self.pkg_dir,name)

    # Create an old message
    bag = rosrecord.Rebagger(oldbag)
    bag.add("topic", old_msg(), roslib.rostime.Time())
    bag.close()

    # Check and migrate
    res = rosbagmigration.checkbag(oldbag, [])
    self.assertTrue(res is None or res == [], 'Bag not ready to be migrated')
    res = rosbagmigration.fixbag(oldbag, newbag, [])
    self.assertTrue(res, 'Bag not converted successfully')

    # Pull the first message out of the bag
    msgs = [msg for msg in rosrecord.logplayer(newbag)]

    # Reserialize the new message so that floats get screwed up, etc.
    m = new_msg()
    buff = StringIO()
    m.serialize(buff)
    m.deserialize(buff.getvalue())
    
    #Compare
#    print "old"
#    print roslib.message.strify_message(msgs[0][1])
#    print "new"
#    print roslib.message.strify_message(m)

    # Strifying them helps make the comparison easier until I figure out why the equality operator is failing
    self.assertTrue(roslib.message.strify_message(msgs[0][1]) == roslib.message.strify_message(m))
#    self.assertTrue(msgs[0][1] == m)

    #Cleanup
    os.remove(oldbag)
    os.remove(newbag)





if __name__ == '__main__':
  rostest.unitrun('test_common_msgs', 'test_common_msgs_migration', TestCommonMsgsMigration, sys.argv)
