#include "core_sim/actor/vessel_actor.hpp"
#include "actor_impl.hpp"
#include "constant.hpp"

#include <stdexcept>
#include <algorithm>

namespace microsoft {
namespace projectairsim {

VesselActor::VesselActor(const std::string& id, const Transform& origin,
                         const Logger& logger, const TopicManager& topic_manager,
                         const std::string& parent_topic_path,
                         const ServiceManager& service_manager,
                         const StateManager& state_manager)
    : Actor(std::make_shared<ActorImpl>(ActorType::kVessel, id, origin,
                                        Constant::Component::robot, logger,
                                        topic_manager, parent_topic_path,
                                        service_manager, state_manager)) {
    
    // 初始化默认状态
    current_state_ = {};
    env_forces_ = {};
    
    // Logging intentionally omitted in minimal integration path.
}

VesselActor::~VesselActor() {}

void VesselActor::Load(ConfigJson config_json) {
    try {
        // 解析配置
        config_ = ParseVesselConfig(config_json);
        
        // 初始化物理引擎
        InitializePhysicsEngine(config_.physics_params);
        
        // 初始化推进器系统
        InitializeThrusters(config_.thrusters);
        
        // 初始化传感器
        InitializeSensors(config_.sensors);
        
        // 初始化环境交互
        InitializeEnvironment(config_.environment);
        
        is_initialized_ = true;
        
    } catch (const std::exception& e) {
        throw;
    }
}

void VesselActor::Update(float delta_time) {
    if (!IsLoaded() || !is_initialized_) return;
    
    try {
        // 更新环境力
        UpdateEnvironmentalForces();
        
        // 更新物理仿真
        UpdatePhysics(delta_time);
        
        // 更新传感器
        UpdateSensors(delta_time);
        
        // 更新船舶状态
        UpdateVesselState();
        
    } catch (const std::exception& e) {
      (void)e;
    }
}

bool VesselActor::SetThrusterForces(const std::vector<float>& forces) {
    auto validated_forces = forces;
    ValidateThrusterForces(validated_forces);

    // 更新推进器状态
    for (size_t i = 0; i < std::min(validated_forces.size(), thrusters_.size()); ++i) {
        thrusters_[i].current_force = validated_forces[i];
    }
    return true;
}

bool VesselActor::SetThrusterAzimuths(const std::vector<float>& azimuths) {
    // 更新推进器方位角
    for (size_t i = 0; i < std::min(azimuths.size(), thrusters_.size()); ++i) {
        thrusters_[i].current_azimuth = std::clamp(
            azimuths[i],
            thrusters_[i].azimuth_min,
            thrusters_[i].azimuth_max
        );
    }
    return true;
}

bool VesselActor::SetEnvironmentalForces(const Vector3& wind, const Vector3& current) {
    env_forces_.wind_velocity = wind;
    env_forces_.current_velocity = current;
    return true;
}

VesselState VesselActor::GetVesselState() const {
    return current_state_;
}

std::vector<float> VesselActor::GetThrusterForces() const {
    std::vector<float> forces;
    for (const auto& thruster : thrusters_) {
        forces.push_back(thruster.current_force);
    }
    return forces;
}

EnvironmentalForces VesselActor::GetEnvironmentalForces() const {
    return env_forces_;
}

std::shared_ptr<Sensor> VesselActor::GetSensor(const std::string& sensor_name) {
    auto it = sensors_.find(sensor_name);
    if (it != sensors_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<std::string> VesselActor::GetSensorNames() const {
    std::vector<std::string> names;
    for (const auto& [name, sensor] : sensors_) {
        names.push_back(name);
    }
    return names;
}

void VesselActor::EnableDynamicPositioning(bool enable) {
    dynamic_positioning_enabled_ = enable;
    
    (void)enable;
}

void VesselActor::SetTargetPosition(const Vector3& position, float heading) {
    target_position_ = position;
    target_heading_ = heading;
    
    if (dynamic_positioning_enabled_) {
        // 实现简单的位置控制逻辑
        auto current_state = GetVesselState();
        Vector3 position_error = target_position_ - current_state.position;
        float heading_error = target_heading_ - current_state.orientation(2);
        
        // 简单的PD控制器
        float kp_pos = 100.0f;
        float kp_heading = 500.0f;
        
        std::vector<float> control_forces(thrusters_.size(), 0.0f);
        
        // 计算所需的控制力（简化实现）
        if (thrusters_.size() >= 3) {
            // 假设有弓推、左主推、右主推
            control_forces[0] = kp_pos * position_error(1);  // 弓推控制横向
            control_forces[1] = kp_pos * position_error(0) + kp_heading * heading_error;  // 左主推
            control_forces[2] = kp_pos * position_error(0) - kp_heading * heading_error;  // 右主推
        }
        
        SetThrusterForces(control_forces);
    }
}

void VesselActor::SetTargetVelocity(const Vector3& velocity, float angular_velocity) {
    target_velocity_ = velocity;
    target_angular_velocity_ = angular_velocity;
    
    // 实现速度控制逻辑
    auto current_state = GetVesselState();
    Vector3 velocity_error = target_velocity_ - current_state.linear_velocity;
    float angular_error = target_angular_velocity_ - current_state.angular_velocity(2);
    
    // 简单的速度控制器
    float kv_linear = 200.0f;
    float kv_angular = 300.0f;
    
    std::vector<float> control_forces(thrusters_.size(), 0.0f);
    
    if (thrusters_.size() >= 3) {
        control_forces[0] = kv_linear * velocity_error(1);  // 弓推
        control_forces[1] = kv_linear * velocity_error(0) + kv_angular * angular_error;
        control_forces[2] = kv_linear * velocity_error(0) - kv_angular * angular_error;
    }
    
    SetThrusterForces(control_forces);
}

// 私有方法实现

void VesselActor::InitializePhysicsEngine(const FossenParameters& params) {
    (void)params;
}

void VesselActor::InitializeThrusters(const std::vector<ThrusterConfig>& thruster_configs) {
    thrusters_ = thruster_configs;
}

void VesselActor::InitializeSensors(const std::vector<SensorConfig>& sensor_configs) {
    (void)sensor_configs;
    // Minimal bridge stage: vessel dynamics first, sensor plumbing later.
    sensors_.clear();
}

void VesselActor::InitializeEnvironment(const EnvironmentConfig& env_config) {
    env_forces_.wave_height = env_config.wave_height;
    env_forces_.wind_velocity = env_config.wind_velocity;
    env_forces_.current_velocity = env_config.current_velocity;
    env_forces_.wave_interaction = env_config.wave_interaction;
    env_forces_.current_interaction = env_config.current_interaction;
    env_forces_.wind_interaction = env_config.wind_interaction;
    
}

void VesselActor::UpdateEnvironmentalForces() {
    // Minimal mode keeps environment values in env_forces_ only.
}

void VesselActor::UpdateSensors(float delta_time) {
    (void)delta_time;
}

void VesselActor::UpdateVesselState() {
    // Minimal mode: state is advanced in UpdatePhysics().
}

void VesselActor::UpdatePhysics(float delta_time) {
    if (thrusters_.empty() || delta_time <= 0.0f) {
        return;
    }

    float surge_force = 0.0f;
    for (size_t i = 0; i < thrusters_.size(); ++i) {
        surge_force += thrusters_[i].current_force;
    }

    const float inv_mass = 1.0f / std::max(config_.physics_params.mass, 1.0f);
    current_state_.linear_velocity(0) += surge_force * inv_mass * delta_time;

    // Simple damping to keep motion bounded in minimal mode.
    current_state_.linear_velocity(0) *= 0.98f;
    current_state_.position(0) += current_state_.linear_velocity(0) * delta_time;
}

VesselConfig VesselActor::ParseVesselConfig(const ConfigJson& config) {
    (void)config;
    VesselConfig vessel_config;
    // Minimal bridge stage: keep defaults and avoid depending on wrapper helper APIs.
    return vessel_config;
}

FossenParameters VesselActor::ParseFossenParameters(const ConfigJson& config) {
    (void)config;
    FossenParameters params;
    return params;
}

std::vector<ThrusterConfig> VesselActor::ParseThrusterConfigs(const ConfigJson& config) {
    (void)config;
    return {};
}

std::vector<SensorConfig> VesselActor::ParseSensorConfigs(const ConfigJson& config) {
    (void)config;
    return {};
}

EnvironmentConfig VesselActor::ParseEnvironmentConfig(const ConfigJson& config) {
    (void)config;
    EnvironmentConfig env_config;
    return env_config;
}

void VesselActor::ValidateThrusterForces(std::vector<float>& forces) {
    // 确保推进器数量匹配
    if (forces.size() != thrusters_.size()) {
        forces.resize(thrusters_.size(), 0.0f);
    }
    
    // 限制推进器力在允许范围内
    for (size_t i = 0; i < forces.size(); ++i) {
        forces[i] = std::clamp(
            forces[i],
            -thrusters_[i].max_force,
            thrusters_[i].max_force
        );
    }
}

} // namespace projectairsim
} // namespace microsoft