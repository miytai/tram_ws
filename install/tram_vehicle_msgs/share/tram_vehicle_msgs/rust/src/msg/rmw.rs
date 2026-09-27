#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "tram_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tram_vehicle_msgs__msg__VelocitySensor() -> *const std::ffi::c_void;
}

#[link(name = "tram_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn tram_vehicle_msgs__msg__VelocitySensor__init(msg: *mut VelocitySensor) -> bool;
    fn tram_vehicle_msgs__msg__VelocitySensor__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocitySensor>, size: usize) -> bool;
    fn tram_vehicle_msgs__msg__VelocitySensor__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocitySensor>);
    fn tram_vehicle_msgs__msg__VelocitySensor__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocitySensor>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocitySensor>) -> bool;
}

// Corresponds to tram_vehicle_msgs__msg__VelocitySensor
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocitySensor {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: f64,

}



impl Default for VelocitySensor {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tram_vehicle_msgs__msg__VelocitySensor__init(&mut msg as *mut _) {
        panic!("Call to tram_vehicle_msgs__msg__VelocitySensor__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocitySensor {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__VelocitySensor__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__VelocitySensor__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__VelocitySensor__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocitySensor {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocitySensor where Self: Sized {
  const TYPE_NAME: &'static str = "tram_vehicle_msgs/msg/VelocitySensor";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tram_vehicle_msgs__msg__VelocitySensor() }
  }
}


#[link(name = "tram_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tram_vehicle_msgs__msg__DriverControllerCommand() -> *const std::ffi::c_void;
}

#[link(name = "tram_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn tram_vehicle_msgs__msg__DriverControllerCommand__init(msg: *mut DriverControllerCommand) -> bool;
    fn tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriverControllerCommand>, size: usize) -> bool;
    fn tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriverControllerCommand>);
    fn tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriverControllerCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<DriverControllerCommand>) -> bool;
}

// Corresponds to tram_vehicle_msgs__msg__DriverControllerCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Driver controller command
/// Reconstructed from the recorded rosbag wire format (verified byte-for-byte
/// against /vehicle/driver_position_cmd across multiple bags and both vehicles).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriverControllerCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Driver controller position (notch):
    ///   0        -> neutral
    ///   +1..+15  -> traction (acceleration)
    ///   -1..-15  -> braking
    pub position: i8,

}



impl Default for DriverControllerCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tram_vehicle_msgs__msg__DriverControllerCommand__init(&mut msg as *mut _) {
        panic!("Call to tram_vehicle_msgs__msg__DriverControllerCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriverControllerCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriverControllerCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriverControllerCommand where Self: Sized {
  const TYPE_NAME: &'static str = "tram_vehicle_msgs/msg/DriverControllerCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tram_vehicle_msgs__msg__DriverControllerCommand() }
  }
}


