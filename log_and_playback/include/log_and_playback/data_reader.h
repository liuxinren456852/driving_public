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

#ifndef __LOG_AND_PLAYBACK__DATA_READER__H__
#define __LOG_AND_PLAYBACK__DATA_READER__H__


#include <log_and_playback/abstract_data_reader.h>


namespace log_and_playback
{

class DataReader : public AbstractDataReader
{
public:
  /// Load the logs. Optionally skip the first @param skip seconds.
  void load(const std::vector<std::string> & logs, ros::Duration skip=ros::Duration(0));
  bool next();
  bool ok() const { return ok_; }
  ros::Time time() const { return time_; }

  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const;
  stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const;
  stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const;
  velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const;
  stdr_velodyne::PointCloud::ConstPtr instantiateVelodyneSpin() const;
  stdr_msgs::LadybugImages::ConstPtr instantiateLadybugImages() const;
  stdr_msgs::LocalizePose::ConstPtr instantiateLocalizePose() const;
  stdr_msgs::EStopStatus::ConstPtr instantiateEStopStatus() const;
  stdr_msgs::PassatStatus::ConstPtr instantiatePassatStatus() const;
  stdr_msgs::Trajectory2D::ConstPtr instantiateTrajectory2D() const;

private:
  std::vector< boost::shared_ptr<AbstractDataReader> > readers_;
  bool ok_;
  ros::Time time_;
};

} //namespace log_and_playback

#endif /* __LOG_AND_PLAYBACK__DATA_READER__H__ */
