// Generated by gencpp from file ucar_controller/SetSensorTFRequest.msg
// DO NOT EDIT!


#ifndef UCAR_CONTROLLER_MESSAGE_SETSENSORTFREQUEST_H
#define UCAR_CONTROLLER_MESSAGE_SETSENSORTFREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ucar_controller
{
template <class ContainerAllocator>
struct SetSensorTFRequest_
{
  typedef SetSensorTFRequest_<ContainerAllocator> Type;

  SetSensorTFRequest_()
    : pose_x(0.0)
    , pose_y(0.0)
    , pose_z(0.0)
    , euler_r(0.0)
    , euler_p(0.0)
    , euler_y(0.0)  {
    }
  SetSensorTFRequest_(const ContainerAllocator& _alloc)
    : pose_x(0.0)
    , pose_y(0.0)
    , pose_z(0.0)
    , euler_r(0.0)
    , euler_p(0.0)
    , euler_y(0.0)  {
  (void)_alloc;
    }



   typedef double _pose_x_type;
  _pose_x_type pose_x;

   typedef double _pose_y_type;
  _pose_y_type pose_y;

   typedef double _pose_z_type;
  _pose_z_type pose_z;

   typedef double _euler_r_type;
  _euler_r_type euler_r;

   typedef double _euler_p_type;
  _euler_p_type euler_p;

   typedef double _euler_y_type;
  _euler_y_type euler_y;





  typedef boost::shared_ptr< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetSensorTFRequest_

typedef ::ucar_controller::SetSensorTFRequest_<std::allocator<void> > SetSensorTFRequest;

typedef boost::shared_ptr< ::ucar_controller::SetSensorTFRequest > SetSensorTFRequestPtr;
typedef boost::shared_ptr< ::ucar_controller::SetSensorTFRequest const> SetSensorTFRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator1> & lhs, const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pose_x == rhs.pose_x &&
    lhs.pose_y == rhs.pose_y &&
    lhs.pose_z == rhs.pose_z &&
    lhs.euler_r == rhs.euler_r &&
    lhs.euler_p == rhs.euler_p &&
    lhs.euler_y == rhs.euler_y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator1> & lhs, const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ucar_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c539823f1bf23f7b686643f4bd7617b3";
  }

  static const char* value(const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc539823f1bf23f7bULL;
  static const uint64_t static_value2 = 0x686643f4bd7617b3ULL;
};

template<class ContainerAllocator>
struct DataType< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucar_controller/SetSensorTFRequest";
  }

  static const char* value(const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64  pose_x\n"
"float64  pose_y\n"
"float64  pose_z\n"
"float64  euler_r\n"
"float64  euler_p\n"
"float64  euler_y\n"
;
  }

  static const char* value(const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose_x);
      stream.next(m.pose_y);
      stream.next(m.pose_z);
      stream.next(m.euler_r);
      stream.next(m.euler_p);
      stream.next(m.euler_y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetSensorTFRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucar_controller::SetSensorTFRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucar_controller::SetSensorTFRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose_x: ";
    Printer<double>::stream(s, indent + "  ", v.pose_x);
    s << indent << "pose_y: ";
    Printer<double>::stream(s, indent + "  ", v.pose_y);
    s << indent << "pose_z: ";
    Printer<double>::stream(s, indent + "  ", v.pose_z);
    s << indent << "euler_r: ";
    Printer<double>::stream(s, indent + "  ", v.euler_r);
    s << indent << "euler_p: ";
    Printer<double>::stream(s, indent + "  ", v.euler_p);
    s << indent << "euler_y: ";
    Printer<double>::stream(s, indent + "  ", v.euler_y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCAR_CONTROLLER_MESSAGE_SETSENSORTFREQUEST_H
