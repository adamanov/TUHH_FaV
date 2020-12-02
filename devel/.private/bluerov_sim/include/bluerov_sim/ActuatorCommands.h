// Generated by gencpp from file bluerov_sim/ActuatorCommands.msg
// DO NOT EDIT!


#ifndef BLUEROV_SIM_MESSAGE_ACTUATORCOMMANDS_H
#define BLUEROV_SIM_MESSAGE_ACTUATORCOMMANDS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bluerov_sim
{
template <class ContainerAllocator>
struct ActuatorCommands_
{
  typedef ActuatorCommands_<ContainerAllocator> Type;

  ActuatorCommands_()
    : header()
    , roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , thrust(0.0)
    , lateral_thrust(0.0)
    , vertical_thrust(0.0)  {
    }
  ActuatorCommands_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , thrust(0.0)
    , lateral_thrust(0.0)
    , vertical_thrust(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _roll_type;
  _roll_type roll;

   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _thrust_type;
  _thrust_type thrust;

   typedef float _lateral_thrust_type;
  _lateral_thrust_type lateral_thrust;

   typedef float _vertical_thrust_type;
  _vertical_thrust_type vertical_thrust;





  typedef boost::shared_ptr< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> const> ConstPtr;

}; // struct ActuatorCommands_

typedef ::bluerov_sim::ActuatorCommands_<std::allocator<void> > ActuatorCommands;

typedef boost::shared_ptr< ::bluerov_sim::ActuatorCommands > ActuatorCommandsPtr;
typedef boost::shared_ptr< ::bluerov_sim::ActuatorCommands const> ActuatorCommandsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bluerov_sim::ActuatorCommands_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bluerov_sim::ActuatorCommands_<ContainerAllocator1> & lhs, const ::bluerov_sim::ActuatorCommands_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.roll == rhs.roll &&
    lhs.pitch == rhs.pitch &&
    lhs.yaw == rhs.yaw &&
    lhs.thrust == rhs.thrust &&
    lhs.lateral_thrust == rhs.lateral_thrust &&
    lhs.vertical_thrust == rhs.vertical_thrust;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bluerov_sim::ActuatorCommands_<ContainerAllocator1> & lhs, const ::bluerov_sim::ActuatorCommands_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bluerov_sim

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6deaf3fbb15f1ee6b5555ba450666513";
  }

  static const char* value(const ::bluerov_sim::ActuatorCommands_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6deaf3fbb15f1ee6ULL;
  static const uint64_t static_value2 = 0xb5555ba450666513ULL;
};

template<class ContainerAllocator>
struct DataType< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bluerov_sim/ActuatorCommands";
  }

  static const char* value(const ::bluerov_sim::ActuatorCommands_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"float32 roll\n"
"float32 pitch\n"
"float32 yaw\n"
"float32 thrust\n"
"float32 lateral_thrust\n"
"float32 vertical_thrust\n"
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

  static const char* value(const ::bluerov_sim::ActuatorCommands_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.yaw);
      stream.next(m.thrust);
      stream.next(m.lateral_thrust);
      stream.next(m.vertical_thrust);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActuatorCommands_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bluerov_sim::ActuatorCommands_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bluerov_sim::ActuatorCommands_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "thrust: ";
    Printer<float>::stream(s, indent + "  ", v.thrust);
    s << indent << "lateral_thrust: ";
    Printer<float>::stream(s, indent + "  ", v.lateral_thrust);
    s << indent << "vertical_thrust: ";
    Printer<float>::stream(s, indent + "  ", v.vertical_thrust);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BLUEROV_SIM_MESSAGE_ACTUATORCOMMANDS_H