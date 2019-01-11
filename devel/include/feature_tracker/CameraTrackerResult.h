// Generated by gencpp from file feature_tracker/CameraTrackerResult.msg
// DO NOT EDIT!


#ifndef FEATURE_TRACKER_MESSAGE_CAMERATRACKERRESULT_H
#define FEATURE_TRACKER_MESSAGE_CAMERATRACKERRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <feature_tracker/FeatureTrackerResult.h>

namespace feature_tracker
{
template <class ContainerAllocator>
struct CameraTrackerResult_
{
  typedef CameraTrackerResult_<ContainerAllocator> Type;

  CameraTrackerResult_()
    : header()
    , num_of_features(0)
    , features()  {
    }
  CameraTrackerResult_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , num_of_features(0)
    , features(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int64_t _num_of_features_type;
  _num_of_features_type num_of_features;

   typedef std::vector< ::feature_tracker::FeatureTrackerResult_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::feature_tracker::FeatureTrackerResult_<ContainerAllocator> >::other >  _features_type;
  _features_type features;





  typedef boost::shared_ptr< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> const> ConstPtr;

}; // struct CameraTrackerResult_

typedef ::feature_tracker::CameraTrackerResult_<std::allocator<void> > CameraTrackerResult;

typedef boost::shared_ptr< ::feature_tracker::CameraTrackerResult > CameraTrackerResultPtr;
typedef boost::shared_ptr< ::feature_tracker::CameraTrackerResult const> CameraTrackerResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::feature_tracker::CameraTrackerResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace feature_tracker

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'feature_tracker': ['/home/tg/slam_mono/src/feature_tracker/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "588b69a3b8ad6dc432d4f61228bb89fd";
  }

  static const char* value(const ::feature_tracker::CameraTrackerResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x588b69a3b8ad6dc4ULL;
  static const uint64_t static_value2 = 0x32d4f61228bb89fdULL;
};

template<class ContainerAllocator>
struct DataType< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "feature_tracker/CameraTrackerResult";
  }

  static const char* value(const ::feature_tracker::CameraTrackerResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
int64 num_of_features\n\
FeatureTrackerResult[] features\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: feature_tracker/FeatureTrackerResult\n\
int64 id\n\
int64 cnt\n\
int64 seq\n\
float64 u0\n\
float64 v0\n\
float64 u1\n\
float64 v1\n\
";
  }

  static const char* value(const ::feature_tracker::CameraTrackerResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.num_of_features);
      stream.next(m.features);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CameraTrackerResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::feature_tracker::CameraTrackerResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::feature_tracker::CameraTrackerResult_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "num_of_features: ";
    Printer<int64_t>::stream(s, indent + "  ", v.num_of_features);
    s << indent << "features[]" << std::endl;
    for (size_t i = 0; i < v.features.size(); ++i)
    {
      s << indent << "  features[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::feature_tracker::FeatureTrackerResult_<ContainerAllocator> >::stream(s, indent + "    ", v.features[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FEATURE_TRACKER_MESSAGE_CAMERATRACKERRESULT_H