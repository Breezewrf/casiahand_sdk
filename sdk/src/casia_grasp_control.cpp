#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include "casia_grasp_control.h"
#include "casia_grasp_probuf.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <limits.h>

using namespace casia::HandM;
namespace casia
{
  CasiaHandMControl::CasiaHandMControl(int l_handm_id, int r_handm_id,int baudrate,std::string port_name):
  l_handm_dev_id_(l_handm_id), r_handm_dev_id_(r_handm_id), baudrate_(baudrate),port_name_(port_name){
  handm_list_.resize(HANDM_NUM);
}


CasiaHandMControl::~CasiaHandMControl(){
  Shutdown();
  delete com_port_;
  com_port_ = nullptr;
}

void CasiaHandMControl::Shutdown(){
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (shutdown_complete_)
  {
    return;
  }

  run_flag_ = false;
  if (handm_control_thread_.joinable())
  {
    handm_control_thread_.join();
  }

  // Protocol objects must stop referring to the shared serial object before
  // its file descriptor is closed.
  handm_list_.clear();

  if (com_port_ != nullptr && com_port_->isOpen())
  {
    // No worker can be in a transaction after the join above. Discard any
    // bytes left in the tty queues instead of carrying a partial RS485 frame
    // into the driver close path. A flush failure must not prevent close().
    try
    {
      com_port_->flushInput();
      com_port_->flushOutput();
    }
    catch (const std::exception &error)
    {
      std::cerr << "CasiaHandMControl: serial flush during shutdown failed: "
                << error.what() << std::endl;
    }

    try
    {
      com_port_->close();
    }
    catch (const std::exception &error)
    {
      // The vendored serial implementation invalidates its local descriptor
      // before reporting close errors, so it cannot close a recycled fd later.
      std::cerr << "CasiaHandMControl: serial close reported an error: "
                << error.what() << std::endl;
    }
  }

  com_port_connected_ = false;
  l_handm_connected_ = false;
  r_handm_connected_ = false;
  shutdown_complete_ = true;
}

bool CasiaHandMControl::Init(double startup_timeout_s)
{
  const auto startup_deadline = startup_timeout_s > 0
      ? std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(startup_timeout_s))
      : std::chrono::steady_clock::time_point{};
  std::vector<serial::PortInfo> port_info =  serial::list_ports();
  char resolved_port[PATH_MAX];
  if (realpath(port_name_.c_str(), resolved_port) != nullptr) {
    port_name_ = resolved_port;
  }
  std::string port_name = port_name_;
#ifdef LIST_SERIAL_PORT
  for(int i = 0; i < port_info.size(); i++){
    printf("find serial port name: %s\r\n", port_info[i].port.c_str());
  }
#endif
  auto iter = std::find_if(port_info.begin(), port_info.end(), [port_name](const serial::PortInfo &port_info){
    return (port_name == port_info.port);
  });
  if(iter == port_info.end()){
    com_port_connected_ = false;
    printf(FONT_RED "CasiaHandMControl, can not find port : %s\r\n" FONT_CLEAR, port_name.c_str());
    return false;
  }

  try{
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(2000));
  }catch(serial::PortNotOpenedException &e){
    std::cout<<e.what()<<std::endl;
    return false;
  }catch(serial::IOException &e){
    std::cout<<e.what()<<std::endl;
    return false;
  }catch(std::invalid_argument &e){
    std::cout<<e.what()<<std::endl;
    return false;
  }
  if (com_port_->isOpen()){
    int left_error = 0;  int right_error = 0;
    com_port_connected_ = true;
    printf(FONT_GREEN "CasiaHandMControl: Serial port %s openned\r\n" FONT_CLEAR, port_name_.c_str());
    // CH341 may expose an open tty before RX is ready after a previous close.
    // Discard partial replies and allow the adapter a short quiet period before
    // probing either hand. This wait remains inside the native startup budget.
    try {
      com_port_->flush();
    } catch (const std::exception &error) {
      std::cerr << "CasiaHandMControl: startup serial flush failed: " << error.what() << std::endl;
      return false;
    }
    auto quiet_period = std::chrono::steady_clock::duration(std::chrono::milliseconds(250));
    if (startup_timeout_s > 0) {
      const auto remaining = startup_deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) return false;
      quiet_period = std::min(quiet_period, remaining);
    }
    std::this_thread::sleep_for(quiet_period);
    if (startup_timeout_s > 0 && std::chrono::steady_clock::now() >= startup_deadline) return false;
    handm_list_[LEFT_HANDM_INDEX].reset(new(CasiaHandMProbuf)(l_handm_dev_id_, com_port_));
    handm_list_[RIGHT_HANDM_INDEX].reset(new(CasiaHandMProbuf)(r_handm_dev_id_, com_port_));
    
    for (auto &hand : handm_list_) hand->SetStartupDeadline(startup_deadline);
    left_error = handm_list_[LEFT_HANDM_INDEX]->start();

    if(left_error != 0){
        printf(FONT_RED "CasiaHandMControl starting error, lost left_handm device.\r\n" FONT_CLEAR);
        l_handm_connected_=false;}
    else{
        l_handm_connected_=true;}

    right_error = handm_list_[RIGHT_HANDM_INDEX]->start();

    if(right_error != 0){
      printf(FONT_RED "CasiaHandMControl starting error, lost right_handm device.\r\n" FONT_CLEAR);
      r_handm_connected_=false;}
    else{
      r_handm_connected_=true;}
      
    // This SDK instance represents one atomic dual-hand device. Starting with
    // only one hand would make a 20-joint sample partially uninitialized and
    // would incorrectly look fresh to Recorder.
    if(!l_handm_connected_ || !r_handm_connected_)
    {
      return false;
    }
    else
    {

    if(!getHandMSoftVersion(handm_state_get.handm_sys_soft_version)){
       LogTransportError("CasiaHandMControl, getHandMSoftVersion failed.");
       return false;
    }else{
      if(l_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_left,handm_soft_version: %f.\r\n" FONT_CLEAR, handm_state_get.handm_sys_soft_version[0]);
      }
      if(r_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_right,handm_soft_version: %f.\r\n" FONT_CLEAR,handm_state_get.handm_sys_soft_version[1]);
      }
     } 
    if(!getHandMHardwareVersion(handm_state_get.handm_sys_hardware_version)){
       LogTransportError("CasiaHandMControl, getHandMHardwareVersion failed.");
       return false;
    }else{
      if(l_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_left,handm_hardware_version: %f.\r\n" FONT_CLEAR, handm_state_get.handm_sys_hardware_version[0]);
      }
      if(r_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_right,grasp_hardware_version: %f.\r\n" FONT_CLEAR, handm_state_get.handm_sys_hardware_version[1]);
      }
     } 

    do{
     if(!getHandMTemp(handm_state_get.handm_temp)){
       LogTransportError("CasiaHandMControl, getHandMTemp failed.");
       return false;
     }else{
      if(l_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_left,frist_run,hand_temp: %f.\r\n" FONT_CLEAR, handm_state_get.handm_temp[0]);
      }
      if(r_handm_connected_){ 
         printf(FONT_YELLOW "CasiaHandM_right,frist_run,hand_temp: %f.\r\n" FONT_CLEAR, handm_state_get.handm_temp[1]);
      }
     } 
     
    if(!getHandMState(handm_state_get.handm_sys_state)){
       LogTransportError("CasiaHandMControl, getHandMState failed.");
       return false;
     }else{
      if(l_handm_connected_){ 
         if(handm_state_get.handm_sys_state[0]!=casia::HandMC_SYS_OK) {
           printf(FONT_RED "CasiaHandM_left,frist_run,hand_run error .\r\n" FONT_CLEAR);
           return false;
         }    
         else 
              printf(FONT_GREEN "CasiaHandM_right,frist_run,hand_run ok.\r\n" FONT_CLEAR);

      }
      if(r_handm_connected_){ 
          if(handm_state_get.handm_sys_state[1]!=casia::HandMC_SYS_OK){
            printf(FONT_RED "CasiaHandM_left,frist_run,hand_run error .\r\n" FONT_CLEAR);
            return false;
          }     
         else 
            printf(FONT_GREEN "CasiaHandM_right,frist_run,hand_run ok.\r\n" FONT_CLEAR);
      }
     } 
    }while (0);



    if (startup_timeout_s > 0 && std::chrono::steady_clock::now() >= startup_deadline) return false;
    for (auto &hand : handm_list_) hand->SetStartupDeadline({});
    if (!StartHandMControlThread())
    {
      printf(FONT_RED "CasiaGraspControl thread starting error.\r\n" FONT_CLEAR);
      return false;
    }
    printf(FONT_GREEN "CasiaGraspControl starting succefully.\r\n" FONT_CLEAR);
    return true;
    }
  }
  else
  {
    printf(FONT_RED "CasiaGraspControl: Serial port  %s not opened\r\n" FONT_CLEAR, port_name_.c_str());
    return false;
  }
  return false;
  }

  bool CasiaHandMControl::getHandMTemp(handm_index_t index,float* cur_temp){
    float temp_val;
    if(!com_port_connected_){return false;} 
    if(handm_list_[index]->IsDevOnline()){
      if(handm_list_[index]->getSystemTemp(&temp_val)){
        cur_temp[0]=temp_val*HANDM_SYS_TEMP;
        return true;
      }else{
        LogTransportError("getGraspTemp failed, ack timeout.");
        return false;
      }
    }else{
      LogTransportError(std::string("getGraspTemp failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
      return false;
    }
    return true;
  }

  bool CasiaHandMControl::getHandMTemp(float* cur_temp){
    if(!com_port_connected_){
      LogTransportError("getGraspTemp failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
    if(l_handm_connected_){
     ret_val_l = getHandMTemp(casia::HandM::LEFT_HANDM_INDEX,  cur_temp);}
    if(r_handm_connected_){     
     ret_val_r = getHandMTemp(casia::HandM::RIGHT_HANDM_INDEX, &cur_temp[1]);}
    return (ret_val_l & ret_val_r);
  }

  bool CasiaHandMControl::getHandMSoftVersion(handm_index_t index,float* soft_version){
    if(!com_port_connected_){return false;} 
    if(handm_list_[index]->IsDevOnline()){
      if(handm_list_[index]->getSystemSoftVersion(soft_version)){
        return true;
      }else{
        LogTransportError("getGraspSoftVersion failed, ack timeout.");
        return false;
      }
    }else{
      LogTransportError(std::string("getGraspSoftVersion failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
      return false;
    }
    return true;
  }

  bool CasiaHandMControl::getHandMSoftVersion(float* soft_version){
    if(!com_port_connected_){
      LogTransportError("getHandSoftVersion failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
    if(l_handm_connected_){
     ret_val_l = getHandMSoftVersion(casia::HandM::LEFT_HANDM_INDEX,  soft_version);}
    if(r_handm_connected_){     
     ret_val_r = getHandMSoftVersion(casia::HandM::RIGHT_HANDM_INDEX, &soft_version[1]);}
    return (ret_val_l & ret_val_r);
  }

  bool CasiaHandMControl::getHandMHardwareVersion(handm_index_t index,float* hardware_version){
    if(!com_port_connected_){return false;} 
    if(handm_list_[index]->IsDevOnline()){
      if(handm_list_[index]->getSystemHardwareVersion(hardware_version)){
        return true;
      }else{
        LogTransportError("getHandMHardwareVersion failed, ack timeout.");
        return false;
      }
    }else{
      LogTransportError(std::string("getHandMHardwareVersion failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
      return false;
    }
    return true;
  }

  bool CasiaHandMControl::getHandMHardwareVersion(float* hardware_version){
    if(!com_port_connected_){
      LogTransportError("getHandMHardwareVersion failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
    if(l_handm_connected_){
     ret_val_l = getHandMHardwareVersion(casia::HandM::LEFT_HANDM_INDEX,  hardware_version);}
    if(r_handm_connected_){     
     ret_val_r = getHandMHardwareVersion(casia::HandM::RIGHT_HANDM_INDEX, &hardware_version[1]);}
    return (ret_val_l & ret_val_r);
  }

  bool CasiaHandMControl::getHandMState(handm_index_t index,uint16_t* cur_state){
    if(!com_port_connected_){return false;} 
    if(handm_list_[index]->IsDevOnline()){
      if(handm_list_[index]->getError(cur_state)){
        return true;
      }else{
        LogTransportError("getGraspState failed, ack timeout.");
        return false;
      }
    }else{
      LogTransportError(std::string("getGraspState failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
      return false;
    }
    return true; 
  }

  bool CasiaHandMControl::getHandMState(uint16_t* cur_state){
    if(!com_port_connected_){
      LogTransportError("getHandState failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
    if(l_handm_connected_){
     ret_val_l = getHandMState(casia::HandM::LEFT_HANDM_INDEX,  cur_state);}
    if(r_handm_connected_){     
     ret_val_r = getHandMState(casia::HandM::RIGHT_HANDM_INDEX, &cur_state[1]);}
    return (ret_val_l & ret_val_r);
  }

  bool CasiaHandMControl::setHandMTargetPosPowerSpeed(handm_index_t index, float* angle_buf,float* power_buf,float* speed_buf,float* cur_angle,float* cur_power)
  {
     float angle_tmp    [HANDM_DOF_NUM];
     float power_tmp    [HANDM_DOF_NUM]; 
     float speed_tmp; 
     float cur_pos_tmp  [HANDM_DOF_NUM];
     float cur_power_tmp[HANDM_DOF_NUM];
     float cur_state_tmp[HANDM_DOF_NUM];

     angle_tmp[0] = angle_buf[0]/HAND_JOINT_0_POS_MAX*2048.0;
     angle_tmp[1] = angle_buf[1]/HAND_JOINT_1_POS_MAX*2048.0;
     for(int i = 2; i < HANDM_DOF_NUM; i++){
        angle_tmp[i] = angle_buf[i]/HAND_JOINT_2_9_POS_MAX*2048.0;
     }
     speed_tmp = speed_buf[0]*HANDM_SPEED_PARAM_NORMAL_RES;

     for(int i = 0; i < 6; i++){
        power_tmp[i] = power_buf[i]/27.0*2048.0;
     } 
     for(int i = 6; i < HANDM_DOF_NUM; i++){
        power_tmp[i] = power_buf[i]/6.0*2048.0;
     }  
    if(!com_port_connected_){return false;}
    if(handm_list_[index]->IsDevOnline())
    {
      if(handm_list_[index]->setTargetPosPowerSpeed(angle_tmp,power_tmp,speed_tmp,cur_pos_tmp,cur_power_tmp,HANDM_DOF_NUM,0x01)){
      
      cur_angle[0] = cur_pos_tmp[0]*HAND_JOINT_0_POS_MAX/2048.0;
      cur_angle[1] = cur_pos_tmp[1]*HAND_JOINT_1_POS_MAX/2048.0;
       for(int i = 2; i < HANDM_DOF_NUM; i++){
        cur_angle[i] = cur_pos_tmp[i]*HAND_JOINT_2_9_POS_MAX/2048.0;
       }



       for(int i =0; i < 6; i++){
          cur_power[i] = cur_power_tmp[i]*27.0/2048.0;
        }
       for(int i =6; i < HANDM_DOF_NUM; i++){
          cur_power[i] = cur_power_tmp[i]*6.0/2048.0;
        }  
        return true;
      }else {
        LogTransportError(std::string("setGraspTargetPosPowerSpeed failed, ack timeout, device id: ") + std::to_string(handm_list_[index]->GetDevId()));
        return false;
    }
   }else{
    LogTransportError(std::string("setGraspTargetPosPowerSpeed failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
    return false;
  }
  return true;
  }
  
  bool CasiaHandMControl::setHandMTargetPosPowerSpeed(float* angle_buf,float* power_buf,float* speed_buf,float* cur_angle,float* cur_power)
  {
    if(!com_port_connected_){
      LogTransportError("setGraspTargetPosPowerSpeed failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
      if(l_handm_connected_){
      ret_val_l = setHandMTargetPosPowerSpeed(casia::HandM::LEFT_HANDM_INDEX,  angle_buf,power_buf,speed_buf,cur_angle,cur_power);}
    if(r_handm_connected_){     
     ret_val_r = setHandMTargetPosPowerSpeed(casia::HandM::RIGHT_HANDM_INDEX, &angle_buf[HANDM_DOF_NUM],&power_buf[HANDM_DOF_NUM],&speed_buf[RIGHT_HANDM_INDEX],
                                                                       &cur_angle[HANDM_DOF_NUM],&cur_power[HANDM_DOF_NUM]);}
    return (ret_val_l & ret_val_r);
  }


  bool CasiaHandMControl::getHandMPosPowerState(handm_index_t index,float* cur_angle,float* cur_power)
   {
     float cur_pos_tmp  [HANDM_DOF_NUM];
     float cur_power_tmp[HANDM_DOF_NUM];
     float cur_state_tmp[HANDM_DOF_NUM];
    if(!com_port_connected_){return false;}
    if(handm_list_[index]->IsDevOnline())
    {
      if(handm_list_[index]->getPosPowerState(cur_pos_tmp, cur_power_tmp,HANDM_DOF_NUM)){
       
       
       cur_angle[0] = cur_pos_tmp[0]*HAND_JOINT_0_POS_MAX/2048.0;
      cur_angle[1] = cur_pos_tmp[1]*HAND_JOINT_1_POS_MAX/2048.0;
       for(int i = 2; i < HANDM_DOF_NUM; i++){
        cur_angle[i] = cur_pos_tmp[i]*HAND_JOINT_2_9_POS_MAX/2048.0;
       }



      for(int i =0; i < 6; i++){
          cur_power[i] = cur_power_tmp[i]*27.0/2048.0;
        }
       for(int i =6; i < HANDM_DOF_NUM; i++){
          cur_power[i] = cur_power_tmp[i]*6.0/2048.0;
        }  
      
        return true;
      }else {
        LogTransportError(std::string("getHandMPosPowerState failed, ack timeout, device id: ") + std::to_string(handm_list_[index]->GetDevId()));
        return false;
    }
   }else{
    LogTransportError(std::string("getHandMPosPowerState failed, device id: ") + std::to_string(handm_list_[index]->GetDevId()) + " offline.");
    return false;
  }
    return true;
  }

  bool CasiaHandMControl::getHandMPosPowerState(float* cur_angle,float* cur_power)
  {
    if(!com_port_connected_){
      LogTransportError("getHandPosPowerState failed, com port not connected.");
      return false; }
    bool ret_val_l=true; bool ret_val_r = true;
    if(l_handm_connected_){
      ret_val_l = getHandMPosPowerState(casia::HandM::LEFT_HANDM_INDEX,  cur_angle,cur_power);
      }
    if(r_handm_connected_)
    {   
      ret_val_r = getHandMPosPowerState(casia::HandM::RIGHT_HANDM_INDEX, &cur_angle[HANDM_DOF_NUM],&cur_power[HANDM_DOF_NUM]);
     }
    return (ret_val_l & ret_val_r);

  }

  bool CasiaHandMControl::StartHandMControlThread()
  {
    run_flag_ = true;  
    handm_control_thread_ = std::thread(&CasiaHandMControl::HandMControlThread, this);
    pthread_setname_np(handm_control_thread_.native_handle(), "HandMControl");
    return true;
  }

  void CasiaHandMControl::LogTransportError(const std::string &message)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto found = last_error_log_.find(message);
    if (found == last_error_log_.end() || now - found->second >= std::chrono::seconds(1)) {
      last_error_log_[message] = now;
      std::cerr << message << std::endl;
    }
  }

  void CasiaHandMControl::HandMControlThread()
  {
    try {
      const auto half_period = std::chrono::milliseconds(
          static_cast<int>(std::ceil(1000.0 / HANDM_UPDATE_FREQ / 2)));
      while (run_flag_) {
        auto data = handm_target_set_fifo.pop(0);
        if (data && run_flag_) {
          auto target = *data;
          if (!setHandMTargetPosPowerSpeed(target.handm_angle, target.handm_power, target.handm_speed,
                                         handm_state_get.handm_angle, handm_state_get.handm_power)) {
            LogTransportError("CasiaHandMControl: update grasp_set_target fail");
          }
        }
        std::this_thread::sleep_for(half_period);
        if (!run_flag_) break;
        // Timestamp before polling either hand: a slow second reply must not
        // make an older first-hand measurement appear newly sampled.
        timespec sampled_at{};
        clock_gettime(CLOCK_MONOTONIC, &sampled_at);
        if (!getHandMPosPowerState(handm_state_get.handm_angle, handm_state_get.handm_power)) {
          LogTransportError("CasiaHandMControl: getHandPosForceAngle fail");
        } else {
          handm_state_get.sampled_at_ns = static_cast<int64_t>(sampled_at.tv_sec) * 1000000000LL
                                        + sampled_at.tv_nsec;
          handm_state_fifo_get.push(handm_state_get);
        }
        std::this_thread::sleep_for(half_period);
      }
    } catch (const std::exception &error) {
      transport_failed_ = true;
      run_flag_ = false;
      LogTransportError(std::string("CasiaHandMControl: serial worker stopped: ") + error.what());
    } catch (...) {
      transport_failed_ = true;
      run_flag_ = false;
      LogTransportError("CasiaHandMControl: serial worker stopped with an unknown exception");
    }
  }

   void CasiaHandMControl::SetHandMTargetoQueue(casia::HandM::handm_target_set_t &target)
   {
      for(int i = 0; i <HANDM_DOF_NUM*HANDM_NUM; i++){
          const int joint_index = i % HANDM_DOF_NUM;
          const float max_angle = joint_index == 0 ? HAND_JOINT_0_POS_MAX
                                : joint_index == 1 ? HAND_JOINT_1_POS_MAX
                                                   : HAND_JOINT_2_9_POS_MAX;
          target.handm_angle[i]=HANDM_SAT(target.handm_angle[i],max_angle,0) ;
          const float max_power = joint_index < 6 ? 27.0f : 6.0f;
          target.handm_power[i]=HANDM_SAT(target.handm_power[i],max_power,0) ;
      }
      target.handm_speed[0]=HANDM_SAT(target.handm_speed[0],1.0,0) ;//0.0,1.0
      target.handm_speed[1]=HANDM_SAT(target.handm_speed[1],1.0,0) ;//0.0,1.0
      handm_target_set_fifo.push(target);
   }
   
   bool CasiaHandMControl::GetHandMStateFromQueue(casia::HandM::handm_state_get_t &state)
   {
    auto _data = handm_state_fifo_get.pop(0);
    if (_data)
     {
      state = static_cast<casia::HandM::handm_state_get_t>(*_data);
    }
    else
    {
      return false;
    }
    return true;
  }
} 
