/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.
  
  Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that the
  following conditions are met:
  
* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#ifndef __LOG_AND_PLAYBACK__SPIN_READER_H
#define __LOG_AND_PLAYBACK__SPIN_READER_H


#include <queue>

#include <stdr_velodyne/message_filter.h>
#include <stdr_velodyne/pointcloud.h>

#include <log_and_playback/abstract_data_reader.h>
#include <log_and_playback/bag_tf_listener.h>

namespace log_and_playback
{

class SpinReader
{
public:

  /** Creates an empty spin reader.
   *
   * It needs to be filled with a DataReader, either by loading some data files
   * using the load function, or by hooking an external data reader with the
   * set data reader function.
   */
  SpinReader();

  ~SpinReader();

  /** Hook with an external data reader.
   *
   * Any previous data reader is discarded.
   */
  void setDataReader(AbstractDataReader &);

  /// Removes the data reader, deleting it if it's not an external one
  void unsetDataReader();

  /** Loads some data files.
   *
   * Constructs a new data reader internally.
   * If a data reader is already set it is discarded first.
   *
   * Optionally skip the first @param skip seconds.
   */
  void load(const std::vector< std::string > &logs, ros::Duration skip=ros::Duration(0));

  /// Returns the current spin
  stdr_velodyne::PointCloudConstPtr getSpin() const;

  bool prevSpin();
  bool nextSpin();

  const BagTFListener & tfListener() const { return tf_listener_; }
  BagTFListener & tfListener() { return tf_listener_; }


protected:
  bool do_I_own_the_data_reader_;
  AbstractDataReader * data_reader_;

  stdr_velodyne::SpinCollector spin_collector_; //< buffer to get full spins
  stdr_velodyne::PointCloud::Ptr current_spin_;   //< current spin

  BagTFListener tf_listener_;

  // a queue to hold the spin messages until a transform is available
  std::queue< stdr_velodyne::PointCloud::ConstPtr > spinQ_;

  stdr_velodyne::PacketToPcd packet2pcd_convertor_;

  /// Check whether spins in the Q can be transformed to the smooth frame.
  stdr_velodyne::PointCloud::Ptr processSpinQueue();

private:
  bool next();
};

}

#endif // __LOG_AND_PLAYBACK__SPIN_READER_H
