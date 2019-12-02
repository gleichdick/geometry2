from sys import argv

hdr = """/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TF2_IMPL_CROSS_CONVERT_H
#define TF2_IMPL_CROSS_CONVERT_H

#include <tf2/transform_functions.h>
#include <tf2/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

class btVector3;

namespace Eigen {
template<typename T, int i>
class Quaternion;
}

namespace KDL {
  class Vector;
} // namespace KDL

namespace tf2 {
  class Quaternion;

  template<>
  struct commonMsgType<tf2::Stamped<KDL::Vector>, tf2::Stamped<btVector3>> {
    using type = geometry_msgs::PointStamped;
  };

  template<typename T, int i>
  struct commonMsgType<Eigen::Quaternion<T, i>, tf2::Quaternion> {
    using type = geometry_msgs::Quaternion;
  };

} // namespace tf2

#endif // TF2_IMPL_CROSS_CONVERT_H

"""

if __name__ == "__main__":
    with open(argv[1],"w") as f:
        f.write(hdr)
