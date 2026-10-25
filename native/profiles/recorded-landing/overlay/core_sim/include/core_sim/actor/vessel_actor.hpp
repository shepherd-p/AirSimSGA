#ifndef CORE_SIM_INCLUDE_CORE_SIM_ACTOR_VESSEL_ACTOR_HPP_
#define CORE_SIM_INCLUDE_CORE_SIM_ACTOR_VESSEL_ACTOR_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

#include "core_sim/actor.hpp"
#include "core_sim/config_json.hpp"
#include "core_sim/transforms/transform.hpp"
#include "core_sim/math_utils.hpp"

namespace microsoft {
namespace projectairsim {

// 前向声明
class Sensor;
class ConfigJson;
class Logger;
class TopicManager;
class ServiceManager;
class StateManager;

// 推进器配置结构
struct ThrusterConfig {
    std::string name;
    Vector3 position;
    Vector3 orientation;
    float max_force;
    std::string type;  // "fixed", "azimuth", "tunnel"
    float azimuth_min = -180.0f;
    float azimuth_max = 180.0f;
    float current_force = 0.0f;
    float current_azimuth = 0.0f;
};

// Fossen船舶物理参数
struct FossenParameters {
    float mass = 1000.0f;
    Matrix3x3 inertia_matrix;
    Vector3 center_of_mass;
    Vector3 center_of_buoyancy;
    
    // 6x6矩阵用于3DOF扩展
    struct HydrodynamicCoeffs {
        float xu_dot = -25.0f;    // 附加质量
        float yv_dot = -240.0f;
        float nr_dot = -50.0f;
        
        float xu = -70.0f;        // 线性阻尼
        float yv = -100.0f;
        float nr = -50.0f;
        
        float xu_abs_u = -135.0f; // 二次阻尼
        float yv_abs_v = -2000.0f;
        float nr_abs_r = -50.0f;
    } hydro_coeffs;
};

// 船舶状态
struct VesselState {
    Vector3 position;
    Vector3 orientation;  // roll, pitch, yaw
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    std::vector<float> thruster_forces;
    float wave_height = 0.0f;
    Vector3 current_velocity;
    Vector3 wind_velocity;
};

// 环境力
struct EnvironmentalForces {
    Vector3 wind_velocity;
    Vector3 current_velocity;
    float wave_height = 0.0f;
    bool wave_interaction = true;
    bool current_interaction = true;
    bool wind_interaction = true;
};

// 传感器配置
struct SensorConfig {
    std::string name;
    std::string type;
    bool enabled = true;
    Vector3 position;
    Vector3 orientation;
    json parameters = json::object();
};

// 环境配置
struct EnvironmentConfig {
    float wave_height = 0.5f;
    Vector3 wind_velocity;
    Vector3 current_velocity;
    bool wave_interaction = true;
    bool current_interaction = true;
    bool wind_interaction = true;
};

// 船舶配置
struct VesselConfig {
    FossenParameters physics_params;
    std::vector<ThrusterConfig> thrusters;
    std::vector<SensorConfig> sensors;
    EnvironmentConfig environment;
};

class VesselActor : public Actor {
public:
    VesselActor(const std::string& id, const Transform& origin,
                const Logger& logger, const TopicManager& topic_manager,
                const std::string& parent_topic_path,
                const ServiceManager& service_manager,
                const StateManager& state_manager);
    virtual ~VesselActor();
    
    // 重写Actor基类方法
    void Load(ConfigJson config_json) override;
    void Update(float delta_time);
    
    // 船舶控制接口
    bool SetThrusterForces(const std::vector<float>& forces);
    bool SetThrusterAzimuths(const std::vector<float>& azimuths);
    bool SetEnvironmentalForces(const Vector3& wind, const Vector3& current);
    
    // 状态查询接口
    VesselState GetVesselState() const;
    std::vector<float> GetThrusterForces() const;
    EnvironmentalForces GetEnvironmentalForces() const;
    
    // 传感器接口
    std::shared_ptr<Sensor> GetSensor(const std::string& sensor_name);
    std::vector<std::string> GetSensorNames() const;
    
    // 控制模式
    void EnableDynamicPositioning(bool enable);
    void SetTargetPosition(const Vector3& position, float heading);
    void SetTargetVelocity(const Vector3& velocity, float angular_velocity);

private:
    // 初始化方法
    void InitializePhysicsEngine(const FossenParameters& params);
    void InitializeThrusters(const std::vector<ThrusterConfig>& thruster_configs);
    void InitializeSensors(const std::vector<SensorConfig>& sensor_configs);
    void InitializeEnvironment(const EnvironmentConfig& env_config);
    
    // 更新方法
    void UpdateEnvironmentalForces();
    void UpdateSensors(float delta_time);
    void UpdateVesselState();
    void UpdatePhysics(float delta_time);
    
    // 配置解析
    VesselConfig ParseVesselConfig(const ConfigJson& config);
    FossenParameters ParseFossenParameters(const ConfigJson& config);
    std::vector<ThrusterConfig> ParseThrusterConfigs(const ConfigJson& config);
    std::vector<SensorConfig> ParseSensorConfigs(const ConfigJson& config);
    EnvironmentConfig ParseEnvironmentConfig(const ConfigJson& config);
    
    // 辅助方法
    void ValidateThrusterForces(std::vector<float>& forces);
private:
    // Minimal integration mode: no dedicated vessel physics engine yet.
    std::vector<ThrusterConfig> thrusters_;
    std::unordered_map<std::string, std::shared_ptr<Sensor>> sensors_;
    
    // 状态数据
    VesselState current_state_;
    EnvironmentalForces env_forces_;
    
    // 控制状态
    bool dynamic_positioning_enabled_ = false;
    Vector3 target_position_;
    float target_heading_ = 0.0f;
    Vector3 target_velocity_;
    float target_angular_velocity_ = 0.0f;
    
    // 配置
    VesselConfig config_;
    bool is_initialized_ = false;
};

} // namespace projectairsim
} // namespace microsoft

#endif // CORE_SIM_INCLUDE_CORE_SIM_ACTOR_VESSEL_ACTOR_HPP_