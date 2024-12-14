#pragma once

#include <random>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <hardware_interfaces/types.h>

class PerturbationGenerator {
 public:
  struct PerturbationGeneratorConfig {
    double force_magnitude{0.0};
    double force_stddev{0.0};
    double torque_magnitude{0.0};
    double torque_stddev{0.0};
    double perturbation_duration_ms{0.0};
    double interval_duration_ms{0.0};
    double interval_stddev{0.0};

    RandomType magnitude_type{RandomType::NONE};
    RandomType duration_type{RandomType::NONE};

    bool deserialize(const YAML::Node& node) {
      try {
        force_magnitude = node["force_magnitude"].as<double>();
        force_stddev = node["force_stddev"].as<double>();
        torque_magnitude = node["torque_magnitude"].as<double>();
        torque_stddev = node["torque_stddev"].as<double>();
        perturbation_duration_ms =
            node["perturbation_duration_ms"].as<double>();
        interval_duration_ms = node["interval_duration_ms"].as<double>();
        interval_stddev = node["interval_stddev"].as<double>();
        magnitude_type = string_to_enum<RandomType>(
            node["magnitude_type"].as<std::string>());
        duration_type =
            string_to_enum<RandomType>(node["duration_type"].as<std::string>());
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  PerturbationGenerator() {};
  ~PerturbationGenerator() {};

  void init(const PerturbationGeneratorConfig& config);

  bool generate_perturbation(RUT::VectorXd& perturbation);

 private:
  PerturbationGeneratorConfig _config;
  RUT::Timer _timer;

  double _current_duration_ms{0.0};
  RUT::VectorXd _perturbation;
  enum class InternalStatus { PERTURBATION, NO_PERTURBATION };
  InternalStatus _status{InternalStatus::NO_PERTURBATION};

  std::default_random_engine _rand_generator;
  std::normal_distribution<double> _rand_dist;
};
