/*********************************************************************************************/ /**
* ins_hand_control.h
*

* September 2015
* Author:Hanson Du

* *********************************************************************************************/
#include <serial/serial.h>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <boost/optional.hpp>
#include "casia_grasp_probuf.h"
#include "casia_hand_m_global.h"
#include <atomic>
#include <queue>

namespace casia
{
  namespace HandM
  {
    template <typename T>
    class HandMDataQueue
    {
    public:
      void push(T new_value)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        data_ = std::move(new_value);
        data_cond_.notify_one();
      }

      boost::optional<T> pop(int timeout_ms)
      {
        std::unique_lock<std::mutex> lock(mtx_);
        if (data_cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]()
                                { return bool(data_); }))
        {
          auto result = std::move(*data_);
          data_.reset();
          return result;
        }
        else
        {
          return boost::none;
        }
      }

      bool empty() const
      {
        std::lock_guard<std::mutex> lock(mtx_);
        return !data_;
      }

      boost::optional<T> query() const
      {
        std::lock_guard<std::mutex> lock(mtx_);
        return data_;
      }

    private:
      mutable std::mutex mtx_;
      std::condition_variable data_cond_;
      boost::optional<T> data_;
    };

    class CasiaHandMControl
    {
    public:
      CasiaHandMControl(int l_handm_id, int r_handm_id, int baudrate, std::string port_name = "/dev/ttyUSB0");
      ~CasiaHandMControl();
      bool Init();
      bool StartHandMControlThread();
      void SetHandMTargetoQueue(casia::HandM::handm_target_set_t &target);
      bool GetHandMStateFromQueue(casia::HandM::handm_state_get_t &state);

    private:
      bool getHandMTemp(handm_index_t index, float *cur_temp);
      bool getHandMTemp(float *cur_temp);
      bool getHandMSoftVersion(handm_index_t index, float *soft_version);
      bool getHandMSoftVersion(float *soft_version);
      bool getHandMHardwareVersion(handm_index_t index, float *hardware_version);
      bool getHandMHardwareVersion(float *hardware_version);
      bool getHandMState(handm_index_t index, uint16_t *cur_state);
      bool getHandMState(uint16_t *cur_state);
      bool getHandMPosPowerState(handm_index_t index, float *cur_angle, float *cur_power);
      bool getHandMPosPowerState(float *cur_angle, float *cur_power);
      bool setHandMTargetPosPowerSpeed(handm_index_t index, float *angle_buf, float *power_buf, float *speed_buf, float *cur_angle, float *cur_power);
      bool setHandMTargetPosPowerSpeed(float *angle_buf, float *power_buf, float *speed_buf, float *cur_angle, float *cur_power);
      std::vector<std::shared_ptr<casia::CasiaHandMProbuf>> handm_list_;
      int l_handm_dev_id_;
      int r_handm_dev_id_;
      int l_handm_dev_dof_;
      int r_handm_dev_dof_;
      bool l_handm_connected_ = false;
      bool r_handm_connected_ = false;
      std::string port_name_;
      int baudrate_ = 921600;
      serial::Serial *com_port_ = nullptr;
      bool com_port_connected_ = false;

    private:
      std::thread handm_control_thread_;
      std::atomic<bool> run_flag_{false};
      casia::HandM::handm_param_get_t handm_param_get{};
      casia::HandM::handm_target_set_t handm_target_set{};
      casia::HandM::handm_state_get_t handm_state_get{};
      HandMDataQueue<casia::HandM::handm_target_set_t> handm_target_set_fifo;
      HandMDataQueue<casia::HandM::handm_state_get_t> handm_state_fifo_get;
      void HandMControlThread();
    };
  }
}
