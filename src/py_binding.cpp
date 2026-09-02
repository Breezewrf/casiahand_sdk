#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <casia_hand_m.h>

namespace py = pybind11;

namespace {

constexpr std::size_t kJointCount = HANDM_DOF_NUM * casia::HandM::HANDM_NUM;
constexpr std::size_t kHandCount = casia::HandM::HANDM_NUM;

class CasiaHand {
 public:
  CasiaHand(int left_hand_id, int right_hand_id, int baudrate, std::string port_name) {
    if (left_hand_id < 0 || left_hand_id > 255 || right_hand_id < 0 || right_hand_id > 255) {
      throw std::invalid_argument("hand IDs must be in [0, 255]");
    }
    if (left_hand_id == right_hand_id) throw std::invalid_argument("left and right hand IDs must differ");
    if (baudrate <= 0) throw std::invalid_argument("baudrate must be positive");
    if (port_name.empty()) throw std::invalid_argument("port_name must not be empty");
    hand_ = std::make_unique<casia::HandM::CasiaHandM>(
        left_hand_id, right_hand_id, baudrate, std::move(port_name));
  }

  bool init() {
    ensure_open();
    if (initialized_) return true;
    initialized_ = hand_->init();
    return initialized_;
  }

  std::vector<float> set_joint_positions(
      const std::vector<float>& positions,
      const std::vector<float>& powers,
      const std::vector<float>& speeds) {
    ensure_initialized();
    validate_vector(positions, kJointCount, "positions");
    if (!powers.empty()) validate_vector(powers, kJointCount, "powers");
    if (!speeds.empty()) validate_vector(speeds, kHandCount, "speeds");

    casia::HandM::handm_target_set_t target{};
    std::vector<float> applied(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::size_t joint_index = i % HANDM_DOF_NUM;
      const float maximum = joint_index == 0 ? HAND_JOINT_0_POS_MAX
                            : joint_index == 1 ? HAND_JOINT_1_POS_MAX
                                               : HAND_JOINT_2_9_POS_MAX;
      applied[i] = std::clamp(positions[i], 0.0F, maximum);
      target.handm_angle[i] = applied[i];
      const float default_power = joint_index < 6 ? 15.0F : 3.0F;
      const float maximum_power = joint_index < 6 ? 27.0F : 6.0F;
      target.handm_power[i] = std::clamp(powers.empty() ? default_power : powers[i], 0.0F, maximum_power);
    }
    for (std::size_t i = 0; i < kHandCount; ++i) {
      target.handm_speed[i] = std::clamp(speeds.empty() ? 0.6F : speeds[i], 0.0F, 1.0F);
    }
    hand_->setHandTargetoQueue(target);
    return applied;
  }

  bool try_get_joint_positions(std::vector<float>& positions) {
    ensure_initialized();
    casia::HandM::handm_state_get_t state{};
    if (!hand_->getHandStateFromQueue(state)) return false;
    positions.assign(std::begin(state.handm_angle), std::end(state.handm_angle));
    return true;
  }

  void close() {
    initialized_ = false;
    hand_.reset();
  }

 private:
  static void validate_vector(const std::vector<float>& values, std::size_t expected, const char* label) {
    if (values.size() != expected) {
      throw std::invalid_argument(std::string(label) + " must contain " + std::to_string(expected) + " values");
    }
    if (!std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); })) {
      throw std::invalid_argument(std::string(label) + " must contain only finite values");
    }
  }

  void ensure_open() const {
    if (!hand_) throw std::runtime_error("CASIA hand is closed");
  }

  void ensure_initialized() const {
    ensure_open();
    if (!initialized_) throw std::runtime_error("CASIA hand is not initialized");
  }

  std::unique_ptr<casia::HandM::CasiaHandM> hand_;
  bool initialized_ = false;
};

}  // namespace

PYBIND11_MODULE(_native, module) {
  module.doc() = "Bindings for dual CASIA Hand-M control and measured joint feedback";

  py::class_<CasiaHand>(module, "CasiaHand")
      .def(
          py::init<int, int, int, std::string>(),
          py::arg("left_hand_id") = 2,
          py::arg("right_hand_id") = 0x20,
          py::arg("baudrate") = 115200,
          py::arg("port_name") = "/dev/ttyUSB0")
      .def("init", &CasiaHand::init, py::call_guard<py::gil_scoped_release>())
      .def(
          "set_joint_positions",
          &CasiaHand::set_joint_positions,
          py::arg("positions"),
          py::arg("powers") = std::vector<float>{},
          py::arg("speeds") = std::vector<float>{},
          py::call_guard<py::gil_scoped_release>())
      .def(
          "try_get_joint_positions",
          [](CasiaHand& self) -> py::object {
            std::vector<float> positions;
            bool received = false;
            {
              py::gil_scoped_release release;
              received = self.try_get_joint_positions(positions);
            }
            if (!received) return py::none();
            return py::cast(std::move(positions));
          })
      .def("close", &CasiaHand::close, py::call_guard<py::gil_scoped_release>());
}
