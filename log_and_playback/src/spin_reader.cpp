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

#include <stdr_lib/exception.h>
#include <stdr_velodyne/transform.h>

#include <log_and_playback/data_reader.h>
#include <log_and_playback/robot_model.h>
#include <log_and_playback/spin_reader.h>



namespace log_and_playback
{


SpinReader::SpinReader()
  : do_I_own_the_data_reader_(false)
  , data_reader_(0)
  , spin_collector_( ros::NodeHandle("/driving/velodyne") )
  , packet2pcd_convertor_( ros::NodeHandle("/driving/velodyne") )
{
  RobotModel model;
  ROS_ASSERT(ros::param::has("/driving/robot_description"));
  model.addParam("/driving/robot_description");
  BOOST_FOREACH(const tf::StampedTransform& t, model.getStaticTransforms()) {
    tf_listener_.addStaticTransform(t);
  }
}

SpinReader::~SpinReader()
{
  unsetDataReader();
}

void SpinReader::setDataReader(AbstractDataReader & reader)
{
  unsetDataReader();
  data_reader_ = &reader;
  do_I_own_the_data_reader_ = false;
}

void SpinReader::unsetDataReader()
{
  if( do_I_own_the_data_reader_ && data_reader_ ) {
    delete data_reader_;
    data_reader_ = NULL;
  }
}

void SpinReader::load(const std::vector< std::string > &logs, ros::Duration skip)
{
  unsetDataReader();
  data_reader_ = new DataReader();
  do_I_own_the_data_reader_ = true;
  ((DataReader *)data_reader_)->load(logs, skip);

  // read some data, until we get the tf static frames
  while( ros::ok() && !tf_listener_.initialized() && next() );
}

stdr_velodyne::PointCloudConstPtr SpinReader::getSpin() const
{
  return current_spin_;
}

stdr_velodyne::PointCloudPtr SpinReader::processSpinQueue()
{
  static const std::string target_frame = "smooth";
  stdr_velodyne::PointCloud::Ptr res_spin;

  if( spinQ_.empty() )
    return res_spin;

  try
  {
    stdr_velodyne::PointCloud::Ptr velo_smooth(new stdr_velodyne::PointCloud);
    stdr_velodyne::transform_scan(tf_listener_, target_frame, *spinQ_.front(), *velo_smooth);
    spinQ_.pop();
    res_spin = velo_smooth;
  }
  catch (tf::TransformException & e)
  {
    ROS_DEBUG_STREAM("transform_scan failed: " <<e.what());
  }
  catch(...)
  {
    ROS_DEBUG("transform_scan failed and threw an unexpected exception");
  }

  // limit the size of the queue. Keep the last 500ms of data
  while( !spinQ_.empty() &&
         (pcl_conversions::fromPCL(spinQ_.back()->header).stamp
          - pcl_conversions::fromPCL(spinQ_.front()->header).stamp).toSec()>.5 )
    spinQ_.pop();

  if( res_spin )
    current_spin_ = res_spin;
  return res_spin;
}

bool SpinReader::prevSpin()
{
  ROS_ERROR("prevSpin not yet implemented.");
  return false;
}

bool SpinReader::nextSpin()
{
  current_spin_.reset();
  stdr_velodyne::PointCloudPtr spin = processSpinQueue();
  while( !spin && ros::ok() )
  {
    if( !next() )
      return false;
    spin = processSpinQueue();
  }
  return spin;
}

bool SpinReader::next()
{
  if( !data_reader_->next() )
    return false;

  if( const stdr_msgs::ApplanixPose::ConstPtr applanix = data_reader_->instantiateApplanixPose() ) {
    ROS_DEBUG("Adding applanix pose t=%.3f", applanix->header.stamp.toSec());
    tf_listener_.addApplanixPose(*applanix);
  }
  else if( const stdr_msgs::LocalizePose::ConstPtr localize_pose = data_reader_->instantiateLocalizePose() ) {
    tf_listener_.addLocalizePose(*localize_pose);
  }
  else if( const stdr_velodyne::PointCloud::ConstPtr pcd = data_reader_->instantiateVelodyneSpin() ) {
    spinQ_.push(pcd);
  }
  else if( const velodyne_msgs::VelodyneScan::ConstPtr scans = data_reader_->instantiateVelodyneScans() ) {
    ROS_DEBUG("got raw scan t=%.3f", scans->header.stamp.toSec());
    BOOST_FOREACH(const velodyne_msgs::VelodynePacket & pkt, scans->packets) {
      stdr_velodyne::PointCloud pcd;
      packet2pcd_convertor_.processPacket(pkt, pcd);
      std_msgs::Header h(scans->header);
      h.stamp = pkt.stamp;
      pcl_conversions::toPCL(h, pcd.header);

      if( stdr_velodyne::PointCloud::ConstPtr spin = spin_collector_.add(pcd) ) {
        ROS_DEBUG("got whole spin");
        spinQ_.push(spin);
      }
    }
  }
  return true;
}

}
