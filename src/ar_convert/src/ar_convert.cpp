#include "ar_convert.h"

ARConvert::ARConvert(ros::NodeHandle nh)
    : nh_(nh)
{
    tag_sub_ = nh_.subscribe("/ar_pose_marker", 10, &ARConvert::tagCallback, this);
    tag_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/tags", 1);
}

ARConvert::~ARConvert()
{
}

void ARConvert::tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
    std::unique_lock<std::mutex>lock(tag_data_.mtx_);
    tag_data_.tags_.poses.clear();
    tag_data_.tag_ = false;
    if (msg->markers.size() > 0)
    {
        tag_data_.tag_ = true;
        for (int i = 0; i < msg->markers.size(); i++)
        {
            tag_data_.tags_.poses.push_back(msg->markers.at(i).pose.pose);
        }

        //Sort from lowest to highest id since id is not being sent
        // std::sort(tag_data_.tags_.poses.begin(), tag_data_.tags_.poses.end()); //Doesnt work
    }
    lock.unlock();
    tag_data_.cond_.notify_all();
}

void ARConvert::publishTag(void)
{
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(tag_data_.mtx_);
        tag_data_.cond_.wait(lock);
        tag_pub_.publish(tag_data_.tags_);
        lock.unlock();
    }
}