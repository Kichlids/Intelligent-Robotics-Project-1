// Generated by gencpp from file robot_teleop/Teleop.msg
// DO NOT EDIT!


#ifndef ROBOT_TELEOP_MESSAGE_TELEOP_H
#define ROBOT_TELEOP_MESSAGE_TELEOP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_teleop
{
template <class ContainerAllocator>
struct Teleop_
{
  typedef Teleop_<ContainerAllocator> Type;

  Teleop_()
    : is_teleop(false)
    , command()  {
    }
  Teleop_(const ContainerAllocator& _alloc)
    : is_teleop(false)
    , command(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _is_teleop_type;
  _is_teleop_type is_teleop;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _command_type;
  _command_type command;





  typedef boost::shared_ptr< ::robot_teleop::Teleop_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_teleop::Teleop_<ContainerAllocator> const> ConstPtr;

}; // struct Teleop_

typedef ::robot_teleop::Teleop_<std::allocator<void> > Teleop;

typedef boost::shared_ptr< ::robot_teleop::Teleop > TeleopPtr;
typedef boost::shared_ptr< ::robot_teleop::Teleop const> TeleopConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_teleop::Teleop_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_teleop::Teleop_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_teleop

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'robot_teleop': ['/home/kichlids/Project1/Intelligent-Robotics-Project-1/robot_ws/src/robot_teleop/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_teleop::Teleop_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_teleop::Teleop_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_teleop::Teleop_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_teleop::Teleop_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_teleop::Teleop_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_teleop::Teleop_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_teleop::Teleop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c9b3c6bb662db54412007781db0793c9";
  }

  static const char* value(const ::robot_teleop::Teleop_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc9b3c6bb662db544ULL;
  static const uint64_t static_value2 = 0x12007781db0793c9ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_teleop::Teleop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_teleop/Teleop";
  }

  static const char* value(const ::robot_teleop::Teleop_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_teleop::Teleop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool is_teleop\n\
string command\n\
";
  }

  static const char* value(const ::robot_teleop::Teleop_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_teleop::Teleop_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_teleop);
      stream.next(m.command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Teleop_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_teleop::Teleop_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_teleop::Teleop_<ContainerAllocator>& v)
  {
    s << indent << "is_teleop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_teleop);
    s << indent << "command: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_TELEOP_MESSAGE_TELEOP_H