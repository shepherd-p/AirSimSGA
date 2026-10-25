// Copyright (C) Microsoft Corporation. 
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include "core_sim/message/collision_info_message.hpp"

#include <memory>
#include <sstream>

#include "core_sim/physics_common_types.hpp"
#include "message/common_utils.hpp"
#include "message_impl.hpp"
#include "msgpack.hpp"

namespace microsoft {
namespace projectairsim {

// -----------------------------------------------------------------------------
// Forward declarations

class CollisionInfoMessage::Impl : public MessageImpl {
 public:
  Impl();

  Impl(const CollisionInfo collision_info);

  ~Impl() override {}

  std::string Serialize() override;

  void Deserialize(const std::string& buffer) override;

  MSGPACK_DEFINE_MAP(time_stamp, object_name, segmentation_id, position,
                     impact_point, normal, penetration_depth, has_collided,
                     component_name, other_pose, other_linear_velocity,
                     other_angular_velocity, contact_velocity, support_managed,
                     support_valid, support_gap, support_friction);

 private:
  TimeNano time_stamp = 0;
  std::string object_name;
  int segmentation_id = -1;
  Vector3Msgpack position;
  Vector3Msgpack impact_point;
  Vector3Msgpack normal;
  float penetration_depth = 0.0f;
  bool has_collided = false;
  std::string component_name;
  PoseMsgpack other_pose;
  Vector3Msgpack other_linear_velocity, other_angular_velocity, contact_velocity;
  bool support_managed = false, support_valid = false;
  float support_gap = 0.0f, support_friction = 0.5f;
};

// -----------------------------------------------------------------------------
// class CollisionInfoMessage

CollisionInfoMessage::CollisionInfoMessage()
    : Message(std::make_shared<CollisionInfoMessage::Impl>()) {}

CollisionInfoMessage::CollisionInfoMessage(const CollisionInfo collision_info)
    : Message(std::make_shared<CollisionInfoMessage::Impl>(collision_info)) {}

CollisionInfoMessage::~CollisionInfoMessage() {}

std::string CollisionInfoMessage::Serialize() const {
  return static_cast<CollisionInfoMessage::Impl*>(pimpl_.get())->Serialize();
}

void CollisionInfoMessage::Deserialize(const std::string& buffer) {
  static_cast<CollisionInfoMessage::Impl*>(pimpl_.get())->Deserialize(buffer);
}

// -----------------------------------------------------------------------------
// class PoseStampedMessage::Impl

CollisionInfoMessage::Impl::Impl() : MessageImpl(MessageType::kCollisionInfo) {}

CollisionInfoMessage::Impl::Impl(const CollisionInfo collision_info)
    : MessageImpl(MessageType::kCollisionInfo) {
  normal = Vector3Msgpack(collision_info.normal);
  impact_point = Vector3Msgpack(collision_info.impact_point);
  position = Vector3Msgpack(collision_info.position);
  penetration_depth = collision_info.penetration_depth;
  time_stamp = collision_info.time_stamp;
  object_name = collision_info.object_name;
  segmentation_id = collision_info.segmentation_id;
  has_collided = collision_info.has_collided;
  component_name = collision_info.component_name;
  other_pose = PoseMsgpack(collision_info.other_pose);
  other_linear_velocity = Vector3Msgpack(collision_info.other_linear_velocity);
  other_angular_velocity = Vector3Msgpack(collision_info.other_angular_velocity);
  contact_velocity = Vector3Msgpack(collision_info.contact_velocity);
  support_managed = collision_info.support_managed;
  support_valid = collision_info.support_valid;
  support_gap = collision_info.support_gap;
  support_friction = collision_info.support_friction;
}

std::string CollisionInfoMessage::Impl::Serialize() {
  std::stringstream stream;
  msgpack::packer<std::stringstream> packer(stream);
  this->msgpack_pack(packer);
  return stream.str();
}

void CollisionInfoMessage::Impl::Deserialize(const std::string& buffer) {
  auto handle = msgpack::unpack(buffer.data(), buffer.size());
  auto object = handle.get();
  this->msgpack_unpack(object);
}

}  // namespace projectairsim
}  // namespace microsoft
