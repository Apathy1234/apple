// Generated by gencpp from file feature_tracker/FeatureTrackerResule.msg
// DO NOT EDIT!


#ifndef FEATURE_TRACKER_MESSAGE_FEATURETRACKERRESULE_H
#define FEATURE_TRACKER_MESSAGE_FEATURETRACKERRESULE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace feature_tracker
{
template <class ContainerAllocator>
struct FeatureTrackerResule_
{
  typedef FeatureTrackerResule_<ContainerAllocator> Type;

  FeatureTrackerResule_()
    : Header()
    , id(0)
    , lifeTime(0)
    , u0(0.0)
    , v0(0.0)
    , u1(0.0)
    , v1(0.0)  {
    }
  FeatureTrackerResule_(const ContainerAllocator& _alloc)
    : Header(_alloc)
    , id(0)
    , lifeTime(0)
    , u0(0.0)
    , v0(0.0)
    , u1(0.0)
    , v1(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _Header_type;
  _Header_type Header;

   typedef int64_t _id_type;
  _id_type id;

   typedef int64_t _lifeTime_type;
  _lifeTime_type lifeTime;

   typedef double _u0_type;
  _u0_type u0;

   typedef double _v0_type;
  _v0_type v0;

   typedef double _u1_type;
  _u1_type u1;

   typedef double _v1_type;
  _v1_type v1;





  typedef boost::shared_ptr< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> const> ConstPtr;

}; // struct FeatureTrackerResule_

typedef ::feature_tracker::FeatureTrackerResule_<std::allocator<void> > FeatureTrackerResule;

typedef boost::shared_ptr< ::feature_tracker::FeatureTrackerResule > FeatureTrackerResulePtr;
typedef boost::shared_ptr< ::feature_tracker::FeatureTrackerResule const> FeatureTrackerResuleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace feature_tracker

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'feature_tracker': ['/home/tg/slam_mono/src/feature_tracker/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab82e194225e796f26c8c2c76b2bd2bb";
  }

  static const char* value(const ::feature_tracker::FeatureTrackerResule_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab82e194225e796fULL;
  static const uint64_t static_value2 = 0x26c8c2c76b2bd2bbULL;
};

template<class ContainerAllocator>
struct DataType< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
{
  static const char* value()
  {
    return "feature_tracker/FeatureTrackerResule";
  }

  static const char* value(const ::feature_tracker::FeatureTrackerResule_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header Header\n\
int64 id\n\
int64 lifeTime\n\
float64 u0\n\
float64 v0\n\
float64 u1\n\
float64 v1\n\
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
";
  }

  static const char* value(const ::feature_tracker::FeatureTrackerResule_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Header);
      stream.next(m.id);
      stream.next(m.lifeTime);
      stream.next(m.u0);
      stream.next(m.v0);
      stream.next(m.u1);
      stream.next(m.v1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FeatureTrackerResule_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::feature_tracker::FeatureTrackerResule_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::feature_tracker::FeatureTrackerResule_<ContainerAllocator>& v)
  {
    s << indent << "Header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.Header);
    s << indent << "id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.id);
    s << indent << "lifeTime: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lifeTime);
    s << indent << "u0: ";
    Printer<double>::stream(s, indent + "  ", v.u0);
    s << indent << "v0: ";
    Printer<double>::stream(s, indent + "  ", v.v0);
    s << indent << "u1: ";
    Printer<double>::stream(s, indent + "  ", v.u1);
    s << indent << "v1: ";
    Printer<double>::stream(s, indent + "  ", v.v1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FEATURE_TRACKER_MESSAGE_FEATURETRACKERRESULE_H
