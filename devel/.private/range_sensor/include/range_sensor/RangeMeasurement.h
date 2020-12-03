// Generated by gencpp from file range_sensor/RangeMeasurement.msg
// DO NOT EDIT!


#ifndef RANGE_SENSOR_MESSAGE_RANGEMEASUREMENT_H
#define RANGE_SENSOR_MESSAGE_RANGEMEASUREMENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace range_sensor
{
template <class ContainerAllocator>
struct RangeMeasurement_
{
  typedef RangeMeasurement_<ContainerAllocator> Type;

  RangeMeasurement_()
    : header()
    , id(0)
    , range(0.0)  {
    }
  RangeMeasurement_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , range(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _id_type;
  _id_type id;

   typedef double _range_type;
  _range_type range;





  typedef boost::shared_ptr< ::range_sensor::RangeMeasurement_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::range_sensor::RangeMeasurement_<ContainerAllocator> const> ConstPtr;

}; // struct RangeMeasurement_

typedef ::range_sensor::RangeMeasurement_<std::allocator<void> > RangeMeasurement;

typedef boost::shared_ptr< ::range_sensor::RangeMeasurement > RangeMeasurementPtr;
typedef boost::shared_ptr< ::range_sensor::RangeMeasurement const> RangeMeasurementConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::range_sensor::RangeMeasurement_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::range_sensor::RangeMeasurement_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::range_sensor::RangeMeasurement_<ContainerAllocator1> & lhs, const ::range_sensor::RangeMeasurement_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.range == rhs.range;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::range_sensor::RangeMeasurement_<ContainerAllocator1> & lhs, const ::range_sensor::RangeMeasurement_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace range_sensor

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::range_sensor::RangeMeasurement_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::range_sensor::RangeMeasurement_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::range_sensor::RangeMeasurement_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d36c99edc955a5c530e4f3ab55de9616";
  }

  static const char* value(const ::range_sensor::RangeMeasurement_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd36c99edc955a5c5ULL;
  static const uint64_t static_value2 = 0x30e4f3ab55de9616ULL;
};

template<class ContainerAllocator>
struct DataType< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "range_sensor/RangeMeasurement";
  }

  static const char* value(const ::range_sensor::RangeMeasurement_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"int32 id       # ID of anchor\n"
"float64 range  # distance to anchor\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::range_sensor::RangeMeasurement_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.range);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RangeMeasurement_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::range_sensor::RangeMeasurement_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::range_sensor::RangeMeasurement_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "range: ";
    Printer<double>::stream(s, indent + "  ", v.range);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RANGE_SENSOR_MESSAGE_RANGEMEASUREMENT_H
