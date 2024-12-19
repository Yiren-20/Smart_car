// Generated by gencpp from file ucar_controller/GetBatteryInfoResponse.msg
// DO NOT EDIT!


#ifndef UCAR_CONTROLLER_MESSAGE_GETBATTERYINFORESPONSE_H
#define UCAR_CONTROLLER_MESSAGE_GETBATTERYINFORESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/BatteryState.h>

namespace ucar_controller
{
template <class ContainerAllocator>
struct GetBatteryInfoResponse_
{
  typedef GetBatteryInfoResponse_<ContainerAllocator> Type;

  GetBatteryInfoResponse_()
    : battery_state()  {
    }
  GetBatteryInfoResponse_(const ContainerAllocator& _alloc)
    : battery_state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::BatteryState_<ContainerAllocator>  _battery_state_type;
  _battery_state_type battery_state;





  typedef boost::shared_ptr< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetBatteryInfoResponse_

typedef ::ucar_controller::GetBatteryInfoResponse_<std::allocator<void> > GetBatteryInfoResponse;

typedef boost::shared_ptr< ::ucar_controller::GetBatteryInfoResponse > GetBatteryInfoResponsePtr;
typedef boost::shared_ptr< ::ucar_controller::GetBatteryInfoResponse const> GetBatteryInfoResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator1> & lhs, const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator2> & rhs)
{
  return lhs.battery_state == rhs.battery_state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator1> & lhs, const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ucar_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0f586d9a0cd4ac8575cd2b82e8d112d3";
  }

  static const char* value(const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0f586d9a0cd4ac85ULL;
  static const uint64_t static_value2 = 0x75cd2b82e8d112d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucar_controller/GetBatteryInfoResponse";
  }

  static const char* value(const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/BatteryState battery_state\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/BatteryState\n"
"\n"
"# Constants are chosen to match the enums in the linux kernel\n"
"# defined in include/linux/power_supply.h as of version 3.7\n"
"# The one difference is for style reasons the constants are\n"
"# all uppercase not mixed case.\n"
"\n"
"# Power supply status constants\n"
"uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0\n"
"uint8 POWER_SUPPLY_STATUS_CHARGING = 1\n"
"uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2\n"
"uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3\n"
"uint8 POWER_SUPPLY_STATUS_FULL = 4\n"
"\n"
"# Power supply health constants\n"
"uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0\n"
"uint8 POWER_SUPPLY_HEALTH_GOOD = 1\n"
"uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2\n"
"uint8 POWER_SUPPLY_HEALTH_DEAD = 3\n"
"uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4\n"
"uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5\n"
"uint8 POWER_SUPPLY_HEALTH_COLD = 6\n"
"uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7\n"
"uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8\n"
"\n"
"# Power supply technology (chemistry) constants\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5\n"
"uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6\n"
"\n"
"Header  header\n"
"float32 voltage          # Voltage in Volts (Mandatory)\n"
"float32 current          # Negative when discharging (A)  (If unmeasured NaN)\n"
"float32 charge           # Current charge in Ah  (If unmeasured NaN)\n"
"float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)\n"
"float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)\n"
"float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)\n"
"uint8   power_supply_status     # The charging status as reported. Values defined above\n"
"uint8   power_supply_health     # The battery health metric. Values defined above\n"
"uint8   power_supply_technology # The battery chemistry. Values defined above\n"
"bool    present          # True if the battery is present\n"
"\n"
"float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack\n"
"                         # If individual voltages unknown but number of cells known set each to NaN\n"
"string location          # The location into which the battery is inserted. (slot number or plug)\n"
"string serial_number     # The best approximation of the battery serial number\n"
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

  static const char* value(const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.battery_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetBatteryInfoResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucar_controller::GetBatteryInfoResponse_<ContainerAllocator>& v)
  {
    s << indent << "battery_state: ";
    s << std::endl;
    Printer< ::sensor_msgs::BatteryState_<ContainerAllocator> >::stream(s, indent + "  ", v.battery_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCAR_CONTROLLER_MESSAGE_GETBATTERYINFORESPONSE_H
