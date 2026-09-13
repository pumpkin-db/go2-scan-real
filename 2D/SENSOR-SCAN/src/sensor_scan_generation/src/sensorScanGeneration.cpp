#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

nav_msgs::Odometry odometryIn;
ros::Publisher *pubOdometryPointer = NULL;
tf::StampedTransform transformToMap;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

bool lockMappingZ = false;
bool mappingZReady = false;
double mappingZ = 0.0;
int mappingZSampleCount = 20;
double mappingZMaxSpan = 0.03;
std::deque<double> mappingZSamples;

ros::Publisher pubLaserCloud;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  odometryIn = *odometry;

  if (lockMappingZ && !mappingZReady)
  {
    const double z = odometryIn.pose.pose.position.z;
    if (!std::isfinite(z))
    {
      ROS_WARN_THROTTLE(1.0, "[sensor_scan] Waiting for a finite Z sample");
      return;
    }

    mappingZSamples.push_back(z);
    while (static_cast<int>(mappingZSamples.size()) > mappingZSampleCount)
      mappingZSamples.pop_front();

    if (static_cast<int>(mappingZSamples.size()) < mappingZSampleCount)
    {
      ROS_INFO_THROTTLE(1.0, "[sensor_scan] Collecting stable Z samples: %zu/%d",
                        mappingZSamples.size(), mappingZSampleCount);
      return;
    }

    const auto limits = std::minmax_element(mappingZSamples.begin(), mappingZSamples.end());
    const double span = *limits.second - *limits.first;
    if (span > mappingZMaxSpan)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[sensor_scan] Waiting for stable Z: span=%.3f m, limit=%.3f m",
                        span, mappingZMaxSpan);
      return;
    }

    double sum = 0.0;
    for (const double sample : mappingZSamples)
      sum += sample;
    mappingZ = sum / mappingZSamples.size();
    mappingZReady = true;
    ROS_INFO("[sensor_scan] Locked 2-D mapping Z at %.3f m from %d stable samples "
             "(span %.3f m)", mappingZ, mappingZSampleCount, span);
  }

  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf::Vector3 vec;

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transformToMap.inverse() * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  // In a single-floor 2-D map, remove common-mode odometry Z drift without
  // changing X/Y or attitude.  Points are first recovered in the physical
  // sensor frame with the full pose above; OctoMap then reconstructs them with
  // the same pose except for a fixed Z origin.  The cloud endpoints and ray
  // origin therefore stay mutually consistent.
  if (lockMappingZ)
  {
    transformToMap.setOrigin(tf::Vector3(odometryIn.pose.pose.position.x,
                                        odometryIn.pose.pose.position.y,
                                        mappingZ));
    odometryIn.pose.pose.position.z = mappingZ;
  }

  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = "map";
  transformToMap.child_frame_id_ = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformToMap);

  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.param("lock_mapping_z", lockMappingZ, false);
  nhPrivate.param("lock_z_samples", mappingZSampleCount, 20);
  nhPrivate.param("lock_z_max_span", mappingZMaxSpan, 0.03);
  if (mappingZSampleCount < 1)
  {
    ROS_WARN("[sensor_scan] lock_z_samples=%d is invalid; using 1", mappingZSampleCount);
    mappingZSampleCount = 1;
  }
  if (mappingZMaxSpan < 0.0)
  {
    ROS_WARN("[sensor_scan] lock_z_max_span=%.3f is invalid; using 0", mappingZMaxSpan);
    mappingZMaxSpan = 0.0;
  }
  ROS_INFO("[sensor_scan] lock_mapping_z=%s samples=%d max_span=%.3f m",
           lockMappingZ ? "true" : "false", mappingZSampleCount, mappingZMaxSpan);

  // ROS message filters
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, "/state_estimation", 1);
  subLaserCloud.subscribe(nh, "/registered_scan", 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 2);

  ros::spin();

  return 0;
}
