//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>

#include <boost/algorithm/string.hpp>


#include "nav_msgs/GetMap.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"

#include <Eigen/Geometry>

#include <QtGui/QApplication>

#include <hector_map_tools/HectorMapTools.h>

#include <hector_geotiff/geotiff_writer.h>
#include <hector_geotiff/map_writer_plugin_interface.h>

#include <hector_nav_msgs/GetRobotTrajectory.h>

using namespace std;

namespace hector_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator
{
public:
  MapGenerator()
    : geotiff_writer_(false)
    , pn_("~")    
    , plugin_loader_(0)
    , running_saved_map_num_(0)
  {
    pn_.param("map_file_path", p_map_file_path_, std::string("."));
    geotiff_writer_.setMapFilePath(p_map_file_path_);
    geotiff_writer_.setUseUtcTimeSuffix(true);

    pn_.param("map_file_base_name", p_map_file_base_name_, std::string());

    pn_.param("draw_background_checkerboard", p_draw_background_checkerboard_, true);
    pn_.param("draw_free_space_grid", p_draw_free_space_grid_, true);

    sys_cmd_sub_ = n_.subscribe("syscommand", 1, &MapGenerator::sysCmdCallback, this);

    map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("map");
    //object_service_client_ = n_.serviceClient<worldmodel_msgs::GetObjectModel>("worldmodel/get_object_model");
    path_service_client_ = n_.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");


    double p_geotiff_save_period = 0.0;
    pn_.param("geotiff_save_period", p_geotiff_save_period, 0.0);

    if(p_geotiff_save_period > 0.0){
      //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, false);
      //publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);
      map_save_timer_ = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, this, false );
    }


    pn_.param("plugins", p_plugin_list_, std::string(""));

    std::vector<std::string> plugin_list;
    boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));

    //We always have at least one element containing "" in the string list
    if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
      plugin_loader_ = new pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>("hector_geotiff", "hector_geotiff::MapWriterPluginInterface");

      for (size_t i = 0; i < plugin_list.size(); ++i){
        try
        {
          boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> tmp (plugin_loader_->createInstance(plugin_list[i]));
          tmp->initialize(plugin_loader_->getName(plugin_list[i]));
          plugin_vector_.push_back(tmp);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }
    }else{
      ROS_INFO("No plugins loaded for geotiff node");
    }

    ROS_INFO("Geotiff node started");
  }

  ~MapGenerator()
  {
    if (plugin_loader_){
      delete plugin_loader_;
    }
  }

  void writeGeotiff()
  {
    ros::Time start_time (ros::Time::now());

    std::stringstream ssStream;

    nav_msgs::GetMap srv_map;
    if (map_service_client_.call(srv_map))
    {
      ROS_INFO("GeotiffNode: Map service called successfully");
      const nav_msgs::OccupancyGrid& map (srv_map.response.map);

      std::string map_file_name = p_map_file_base_name_;
      std::string competition_name;
      std::string team_name;
      std::string mission_name;
      std::string postfix;
      if (n_.getParamCached("/competition", competition_name) && !competition_name.empty()) map_file_name = map_file_name + "_" + competition_name;
      if (n_.getParamCached("/team", team_name)               && !team_name.empty())        map_file_name = map_file_name + "_" + team_name;
      if (n_.getParamCached("/mission", mission_name)         && !mission_name.empty())     map_file_name = map_file_name + "_" + mission_name;
      if (pn_.getParamCached("map_file_postfix", postfix)     && !postfix.empty())          map_file_name = map_file_name + "_" + postfix;
      if (map_file_name.substr(0, 1) == "_") map_file_name = map_file_name.substr(1);
      if (map_file_name.empty()) map_file_name = "GeoTiffMap";
      geotiff_writer_.setMapFileName(map_file_name);
      bool transformSuccess = geotiff_writer_.setupTransforms(map);

      if(!transformSuccess){
        ROS_INFO("Couldn't set map transform");
        return;
      }

      geotiff_writer_.setupImageSize();

      if (p_draw_background_checkerboard_){
        geotiff_writer_.drawBackgroundCheckerboard();
      }

      geotiff_writer_.drawMap(map, p_draw_free_space_grid_);
      geotiff_writer_.drawCoords();

      //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call map service");
      return;
    }

    for (size_t i = 0; i < plugin_vector_.size(); ++i){
      plugin_vector_[i]->draw(&geotiff_writer_);
    }

/*void draw(MapWriterInterface *interface)
  {
    if (!initialized_) return;

    hector_worldmodel_msgs::GetObjectModel data;
    if (!service_client_.call(data)) {
      ROS_ERROR_NAMED(name_, "Cannot draw victims, service %s failed", service_client_.getService().c_str());
      return;
    }

    std::string team_name;
    std::string country;
    std::string mission_name;
    nh_.getParamCached("/team", team_name);
    nh_.getParamCached("/country", country);
    nh_.getParamCached("/mission", mission_name);

    boost::posix_time::ptime now = ros::Time::now().toBoost();
    boost::gregorian::date now_date(now.date());
    boost::posix_time::time_duration now_time(now.time_of_day().hours(), now.time_of_day().minutes(), now.time_of_day().seconds(), 0);

    std::ofstream description_file((interface->getBasePathAndFileName() + "_victims.csv").c_str());
    if (description_file.is_open()) {
      description_file << "\"victims\"" << std::endl;
      description_file << "\"1.0\"" << std::endl;
      if (!team_name.empty()) description_file << "\"" << team_name << "\"" << std::endl;
      if (!country.empty()) description_file << "\"" << country << "\"" << std::endl;
      description_file << "\"" << now_date << "\"" << std::endl;
      description_file << "\"" << now_time << "\"" << std::endl;
      if (!mission_name.empty()) description_file << "\"" << mission_name << "\"" << std::endl;
      description_file << std::endl;
      description_file << "id,time,name,x,y,z" << std::endl;
    }

    int counter = 0;
    for(hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = data.response.model.objects.begin(); it != data.response.model.objects.end(); ++it) {
      const hector_worldmodel_msgs::Object& object = *it;
      if (!draw_all_objects_ && object.state.state != hector_worldmodel_msgs::ObjectState::CONFIRMED) continue;
      if (!class_id_.empty() && object.info.class_id != class_id_) continue;

      Eigen::Vector2f coords;
      coords.x() = object.pose.pose.position.x;
      coords.y() = object.pose.pose.position.y;
      interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(++counter), MapWriterInterface::Color(240,10,10));

      if (description_file.is_open()) {
        boost::posix_time::time_duration time_of_day(object.header.stamp.toBoost().time_of_day());
        boost::posix_time::time_duration time(time_of_day.hours(), time_of_day.minutes(), time_of_day.seconds(), 0);
        description_file << counter << "," << time << "," << object.info.object_id << "," << object.pose.pose.position.x << "," << object.pose.pose.position.y << "," << object.pose.pose.position.z << std::endl;
      }
    }
  }*/

    /**
      * No Victims for now, first  agree on a common standard for representation
      */
    /*
    if (req_object_model_){
      worldmodel_msgs::GetObjectModel srv_objects;
      if (object_service_client_.call(srv_objects))
      {
        ROS_INFO("GeotiffNode: Object service called successfully");

        const worldmodel_msgs::ObjectModel& objects_model (srv_objects.response.model);

        size_t size = objects_model.objects.size();


        unsigned int victim_num  = 1;

        for (size_t i = 0; i < size; ++i){
          const worldmodel_msgs::Object& object (objects_model.objects[i]);

          if (object.state.state == worldmodel_msgs::ObjectState::CONFIRMED){
            geotiff_writer_.drawVictim(Eigen::Vector2f(object.pose.pose.position.x,object.pose.pose.position.y),victim_num);
            victim_num++;
          }
        }
      }
      else
      {
        ROS_ERROR("Failed to call objects service");
      }
    }
    */

    /*
    hector_nav_msgs::GetRobotTrajectory srv_path;

    if (path_service_client_.call(srv_path))
    {
      ROS_INFO("GeotiffNode: Path service called successfully");

      std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

      size_t size = traj_vector.size();

      std::vector<Eigen::Vector2f> pointVec;
      pointVec.resize(size);

      for (size_t i = 0; i < size; ++i){
        const geometry_msgs::PoseStamped& pose (traj_vector[i]);

        pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
      }

      if (size > 0){
        //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
        Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
        geotiff_writer_.drawPath(startVec, pointVec);
      }
    }
    else
    {
      ROS_ERROR("Failed to call path service");
    }
    */


    geotiff_writer_.writeGeotiffImage();
    running_saved_map_num_++;

    ros::Duration elapsed_time (ros::Time::now() - start_time);

    ROS_INFO("GeoTiff created in %f seconds", elapsed_time.toSec());
  }

  void timerSaveGeotiffCallback(const ros::TimerEvent& e)
  {
    this->writeGeotiff();
  }

  void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if ( !(sys_cmd.data == "savegeotiff")){
      return;
    }

    this->writeGeotiff();
  }

  std::string p_map_file_path_;
  std::string p_map_file_base_name_;
  std::string p_plugin_list_;
  bool p_draw_background_checkerboard_;
  bool p_draw_free_space_grid_;

  //double p_geotiff_save_period_;

  GeotiffWriter geotiff_writer_;

  ros::ServiceClient map_service_client_;// = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  ros::ServiceClient object_service_client_;
  ros::ServiceClient path_service_client_;

  ros::Subscriber sys_cmd_sub_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  std::vector<boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> > plugin_vector_;

  pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>* plugin_loader_;

  ros::Timer map_save_timer_;

  unsigned int running_saved_map_num_;
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "geotiff_node");

  hector_geotiff::MapGenerator mg;

  //ros::NodeHandle pn_;
  //double p_geotiff_save_period = 60.0f;
  //pn_.param("geotiff_save_period", p_geotiff_save_period, 60.0);
  //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, &mg, false);

  ros::spin();

  return 0;
}

