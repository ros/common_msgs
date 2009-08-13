/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include "sensor_msgs/image_encodings.h"

namespace sensor_msgs
{
  namespace image_encodings
  {
    const std::string RGB8 = "rgb8";
    const std::string RGBA8 = "rgba8";
    const std::string BGR8 = "bgr8";
    const std::string BGRA8 = "bgra8";
    const std::string MONO8="mono8";
    const std::string MONO16="mono16";

    // OpenCV CvMat encodings
    const std::string TYPE_8UC1="8UC1";
    const std::string TYPE_8UC2="8UC2";
    const std::string TYPE_8UC3="8UC3";
    const std::string TYPE_8UC4="8UC4";
    const std::string TYPE_8SC1="8SC1";
    const std::string TYPE_8SC2="8SC2";
    const std::string TYPE_8SC3="8SC3";
    const std::string TYPE_8SC4="8SC4";
    const std::string TYPE_16UC1="16UC1";
    const std::string TYPE_16UC2="16UC2";
    const std::string TYPE_16UC3="16UC3";
    const std::string TYPE_16UC4="16UC4";
    const std::string TYPE_16SC1="16SC1";
    const std::string TYPE_16SC2="16SC2";
    const std::string TYPE_16SC3="16SC3";
    const std::string TYPE_16SC4="16SC4";
    const std::string TYPE_32SC1="32SC1";
    const std::string TYPE_32SC2="32SC2";
    const std::string TYPE_32SC3="32SC3";
    const std::string TYPE_32SC4="32SC4";
    const std::string TYPE_32FC1="32FC1";
    const std::string TYPE_32FC2="32FC2";
    const std::string TYPE_32FC3="32FC3";
    const std::string TYPE_32FC4="32FC4";
    const std::string TYPE_64FC1="64FC1";
    const std::string TYPE_64FC2="64FC2";
    const std::string TYPE_64FC3="64FC3";
    const std::string TYPE_64FC4="64FC4";

    // Bayer encodings
    const std::string BAYER_RGGB8="bayer_rggb8";
    const std::string BAYER_BGGR8="bayer_bggr8";
    const std::string BAYER_GBRG8="bayer_gbrg8";
    const std::string BAYER_GRBG8="bayer_grbg8";
  }
}
