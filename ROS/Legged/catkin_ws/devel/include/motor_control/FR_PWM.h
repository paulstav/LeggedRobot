// Generated by gencpp from file motor_control/FR_PWM.msg
// DO NOT EDIT!


#ifndef MOTOR_CONTROL_MESSAGE_FR_PWM_H
#define MOTOR_CONTROL_MESSAGE_FR_PWM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motor_control
{
template <class ContainerAllocator>
struct FR_PWM_
{
  typedef FR_PWM_<ContainerAllocator> Type;

  FR_PWM_()
    : pwm_duty(0)  {
    }
  FR_PWM_(const ContainerAllocator& _alloc)
    : pwm_duty(0)  {
    }



   typedef int8_t _pwm_duty_type;
  _pwm_duty_type pwm_duty;




  typedef boost::shared_ptr< ::motor_control::FR_PWM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motor_control::FR_PWM_<ContainerAllocator> const> ConstPtr;

}; // struct FR_PWM_

typedef ::motor_control::FR_PWM_<std::allocator<void> > FR_PWM;

typedef boost::shared_ptr< ::motor_control::FR_PWM > FR_PWMPtr;
typedef boost::shared_ptr< ::motor_control::FR_PWM const> FR_PWMConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motor_control::FR_PWM_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motor_control::FR_PWM_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motor_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'motor_control': ['/home/legged/Legged/catkin_ws/src/motor_control/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motor_control::FR_PWM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motor_control::FR_PWM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_control::FR_PWM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_control::FR_PWM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_control::FR_PWM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_control::FR_PWM_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motor_control::FR_PWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc1c76e848affc91996664a93666ea97";
  }

  static const char* value(const ::motor_control::FR_PWM_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc1c76e848affc91ULL;
  static const uint64_t static_value2 = 0x996664a93666ea97ULL;
};

template<class ContainerAllocator>
struct DataType< ::motor_control::FR_PWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motor_control/FR_PWM";
  }

  static const char* value(const ::motor_control::FR_PWM_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motor_control::FR_PWM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 pwm_duty\n\
";
  }

  static const char* value(const ::motor_control::FR_PWM_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motor_control::FR_PWM_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pwm_duty);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct FR_PWM_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motor_control::FR_PWM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motor_control::FR_PWM_<ContainerAllocator>& v)
  {
    s << indent << "pwm_duty: ";
    Printer<int8_t>::stream(s, indent + "  ", v.pwm_duty);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOR_CONTROL_MESSAGE_FR_PWM_H
