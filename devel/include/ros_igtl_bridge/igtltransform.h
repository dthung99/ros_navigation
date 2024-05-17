// Generated by gencpp from file ros_igtl_bridge/igtltransform.msg
// DO NOT EDIT!


#ifndef ROS_IGTL_BRIDGE_MESSAGE_IGTLTRANSFORM_H
#define ROS_IGTL_BRIDGE_MESSAGE_IGTLTRANSFORM_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Transform.h>

namespace ros_igtl_bridge
{
template <class ContainerAllocator>
struct igtltransform_
{
  typedef igtltransform_<ContainerAllocator> Type;

  igtltransform_()
    : name()
    , transform()  {
    }
  igtltransform_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , transform(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;

   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _transform_type;
  _transform_type transform;





  typedef boost::shared_ptr< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> const> ConstPtr;

}; // struct igtltransform_

typedef ::ros_igtl_bridge::igtltransform_<std::allocator<void> > igtltransform;

typedef boost::shared_ptr< ::ros_igtl_bridge::igtltransform > igtltransformPtr;
typedef boost::shared_ptr< ::ros_igtl_bridge::igtltransform const> igtltransformConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_igtl_bridge::igtltransform_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_igtl_bridge::igtltransform_<ContainerAllocator1> & lhs, const ::ros_igtl_bridge::igtltransform_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.transform == rhs.transform;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_igtl_bridge::igtltransform_<ContainerAllocator1> & lhs, const ::ros_igtl_bridge::igtltransform_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_igtl_bridge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5b37f5c9159d09ed0eb53ad625b51f59";
  }

  static const char* value(const ::ros_igtl_bridge::igtltransform_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5b37f5c9159d09edULL;
  static const uint64_t static_value2 = 0x0eb53ad625b51f59ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_igtl_bridge/igtltransform";
  }

  static const char* value(const ::ros_igtl_bridge::igtltransform_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#---Transform Message\n"
" \n"
"string name\n"
"geometry_msgs/Transform transform\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::ros_igtl_bridge::igtltransform_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.transform);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct igtltransform_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_igtl_bridge::igtltransform_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_igtl_bridge::igtltransform_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
    s << indent << "transform: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.transform);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_IGTL_BRIDGE_MESSAGE_IGTLTRANSFORM_H
