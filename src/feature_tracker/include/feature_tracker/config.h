#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <feature_tracker/my_include.h>

namespace config
{
    Mat Get_Vector16_Transform(const ros::NodeHandle &n, const std::string &key)
    {
        std::vector<double> v;
        n.getParam(key, v);
        if (v.size() != 16)
        {
            ROS_INFO_STREAM(" invalid vector16! ");
        }
        Mat T = Mat(v).clone().reshape(1, 4);
        return T;
    }
}
#endif