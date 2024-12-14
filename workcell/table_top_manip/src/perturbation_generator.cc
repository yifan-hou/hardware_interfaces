#include "table_top_manip/perturbation_generator.h"

void PerturbationGenerator::init(const PerturbationGeneratorConfig& config) {
  _config = config;

  _perturbation = RUT::VectorXd::Zero(6);
  _rand_dist = std::normal_distribution<double>(0.0, 1.0);
  _status = InternalStatus::NO_PERTURBATION;

  _timer.tic();
}

bool PerturbationGenerator::generate_perturbation(RUT::VectorXd& perturbation) {
  assert(perturbation.size() == 6);
  double elapsed_time = _timer.toc_ms();

  if (_status == InternalStatus::NO_PERTURBATION) {
    if (elapsed_time < _current_duration_ms) {
      // still no perturbation
      return false;
    }
    // start a new perturbation
    // Perturbation direction
    RUT::Vector3d force_direction = RUT::Vector3d::Random().normalized();
    RUT::Vector3d torque_direction = RUT::Vector3d::Random().normalized();
    // Perturbation magnitude
    double force_mag = 0.0;
    double torque_mag = 0.0;
    switch (_config.magnitude_type) {
      case RandomType::CONSTANT:
        force_mag = _config.force_magnitude;
        torque_mag = _config.torque_magnitude;
        break;
      case RandomType::UNIFORM:
        force_mag = _config.force_magnitude * rand() / RAND_MAX;
        torque_mag = _config.torque_magnitude * rand() / RAND_MAX;
        break;
      case RandomType::GAUSSIAN:
        force_mag = _config.force_magnitude +
                    _config.force_stddev * _rand_dist(_rand_generator);
        torque_mag = _config.torque_magnitude +
                     _config.torque_stddev * _rand_dist(_rand_generator);
        break;
      default:
        std::cout << "Invalid magnitude RandomType: "
                  << to_string(_config.magnitude_type) << std::endl;
        throw std::runtime_error("Invalid magnitude RandomType");
        break;
    }

    perturbation.head<3>() = force_mag * force_direction;
    perturbation.tail<3>() = torque_mag * torque_direction;
    _perturbation = perturbation;

    // decide duration till next perturbation
    switch (_config.duration_type) {
      case RandomType::CONSTANT:
        _current_duration_ms = _config.interval_duration_ms;
        break;
      case RandomType::UNIFORM:
        _current_duration_ms = _config.interval_duration_ms * rand() / RAND_MAX;
        break;
      case RandomType::GAUSSIAN:
        _current_duration_ms =
            _config.interval_duration_ms +
            _config.interval_stddev * _rand_dist(_rand_generator);
        break;
      default:
        std::cout << "Invalid duration RandomType: "
                  << to_string(_config.duration_type) << std::endl;
        throw std::runtime_error("Invalid duration RandomType");
        break;
    }

    _status = InternalStatus::PERTURBATION;
    // std::cout << "---- Adding new perturbation: " << perturbation.transpose()
    //           << std::endl;
    // std::cout << "Next perturbation in " << _current_duration_ms << " ms"
    //           << std::endl;
    _timer.tic();
    return true;
  } else {
    if (elapsed_time < _config.perturbation_duration_ms) {
      // still in perturbation
      perturbation = _perturbation;
      return true;
    }
    // end of perturbation
    _status = InternalStatus::NO_PERTURBATION;
    _timer.tic();
    // std::cout << "---- End of perturbation. Entering No Perturbation mode."
    //           << std::endl;
    return false;
  }
}
