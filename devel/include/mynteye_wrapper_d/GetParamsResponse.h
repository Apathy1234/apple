// Generated by gencpp from file mynteye_wrapper_d/GetParamsResponse.msg
// DO NOT EDIT!


#ifndef MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSRESPONSE_H
#define MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mynteye_wrapper_d
{
template <class ContainerAllocator>
struct GetParamsResponse_
{
  typedef GetParamsResponse_<ContainerAllocator> Type;

  GetParamsResponse_()
    : value()  {
    }
  GetParamsResponse_(const ContainerAllocator& _alloc)
    : value(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetParamsResponse_

typedef ::mynteye_wrapper_d::GetParamsResponse_<std::allocator<void> > GetParamsResponse;

typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsResponse > GetParamsResponsePtr;
typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsResponse const> GetParamsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mynteye_wrapper_d

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'mynteye_wrapper_d': ['/home/tg/slam_mono/src/mynteye_wrapper_d/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "64e58419496c7248b4ef25731f88b8c3";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x64e58419496c7248ULL;
  static const uint64_t static_value2 = 0xb4ef25731f88b8c3ULL;
};

template<class ContainerAllocator>
struct DataType< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mynteye_wrapper_d/GetParamsResponse";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string value\n\
\n\
";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetParamsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mynteye_wrapper_d::GetParamsResponse_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSRESPONSE_H