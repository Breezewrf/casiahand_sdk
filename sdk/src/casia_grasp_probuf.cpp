#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <unistd.h>
#include <iomanip>
#include "casia_grasp_probuf.h"

namespace casia
{
CasiaHandMProbuf::CasiaHandMProbuf(int hand_id, serial::Serial *com_port) :
  dev_id_(hand_id), com_port_(com_port){
  com_timeout_counter_ = 0;
}

CasiaHandMProbuf::~CasiaHandMProbuf(){

}

int CasiaHandMProbuf::start()
{
  std::vector<uint8_t> output;
  ReadParamRegister(output,casia::HandMP_COM_ID_VAL_INDEX);
  constexpr size_t frame_length = casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN;
  // Many CH34x/RS485 interfaces return the transmitted 9-byte request before
  // the hand's 9-byte reply. Read enough for both instead of mistaking the
  // local echo for the device response or leaving the reply queued.
  constexpr size_t probe_read_length = 2 * frame_length;

  for (uint32_t retry_counter = 0; retry_counter < 3; ++retry_counter) {
    std::vector<uint8_t> input;
    try {
      // A previous process or a timed-out transaction can leave bytes queued
      // in the USB/TTY receive path. Never use those bytes as this probe's
      // response.
      com_port_->flushInput();
    } catch (const std::exception &e) {
      serial_dev_exception_ = true;
      dev_online_ = false;
      std::cout << e.what() << std::endl;
      return -1;
    }

    const bool wrote = ComWriteData(output);
    const bool read = wrote && ComReadData(input, probe_read_length);
    bool found_device_response = false;

    // Scan rather than assuming an offset: adapters may suppress the echo,
    // return it, or prepend a small amount of stale data.
    if (read && input.size() >= frame_length) {
      for (size_t offset = 0; offset + frame_length <= input.size(); ++offset) {
        if (input[offset] != casia::HandM_RS485_CMD_HEAD_L ||
            input[offset + 1] != casia::HandM_RS485_CMD_HEAD_H ||
            input[offset + 2] != static_cast<uint8_t>(dev_id_)) {
          continue;
        }

        std::vector<uint8_t> frame(input.begin() + offset,
                                   input.begin() + offset + frame_length);
        const uint16_t returned_id =
            (static_cast<uint16_t>(frame[7]) << 8) | frame[6];
        if (returned_id == static_cast<uint16_t>(dev_id_) &&
            DevcheckSumCheck(frame)) {
          found_device_response = true;
          break;
        }
      }
    }

    if (found_device_response) {
      dev_online_ = true;
      return 0;
    }

    dev_online_ = false;
    if (input.empty()) {
      printf(FONT_RED "hand device id: %d not found.\r\n" FONT_CLEAR, dev_id_);
    } else if (input == output) {
      printf(FONT_RED "hand device id: %d received only the local TX echo.\r\n"
             FONT_CLEAR, dev_id_);
    } else {
      printf(FONT_RED "hand device id: %d has no valid reply in %zu received bytes.\r\n"
             FONT_CLEAR, dev_id_, input.size());
    }
  }

  return -1;
}

bool CasiaHandMProbuf::setPortId(uint16_t id) {

  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteParamRegister(output,casia::HandMP_COM_ID_VAL_INDEX,id);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, setPortId timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, setPortId CheckSum error.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  return true;
}

bool CasiaHandMProbuf::setPortBaudrate(uint16_t baudrate){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteParamRegister(output,casia::HandMP_COM_BAUDRATE_VAL_INDEX,(uint16_t)baudrate);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setPortBaudrate timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setPortBaudrate CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}

bool CasiaHandMProbuf::setSystemOverTemp(float* over_temp){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteParamRegister(output,casia::HandMP_OVER_TEMP_VAL_INDEX,(uint16_t)over_temp[0]);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setSystemOverTemp timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  } 
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setSystemOverTemp CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}

bool CasiaHandMProbuf::setHandType(uint16_t hand_type){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteParamRegister(output,casia::HandMP_HAND_TYPE_VAL_INDEX,(uint16_t)hand_type);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setHandType timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  } 
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setHandType CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}
bool CasiaHandMProbuf::getHandType(uint16_t* hand_type){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  ReadParamRegister(output,casia::HandMP_HAND_TYPE_VAL_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, getHandType timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  } 
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setHandType CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  hand_type[0]=((input[7]<< 8) & 0xff00) + input[6]; 
  return true;
}

bool CasiaHandMProbuf::getPortId(uint16_t* id)
{ 
 std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadParamRegister(output,casia::HandMP_COM_ID_VAL_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getPortId timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getPortId CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }

  id[0]=((input[7]<< 8) & 0xff00) + input[6]; 
  return true; 

}

bool CasiaHandMProbuf::getPortBaudrate(uint16_t* baudrate)
{
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadParamRegister(output,casia::HandMP_COM_BAUDRATE_VAL_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getPortBaudrate timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
   }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getPortBaudrate CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  baudrate[0]=((input[7]<< 8) & 0xff00) + input[6]; 
  return true; 

}

bool CasiaHandMProbuf::getSystemTempLimit(float* temp_limit_buf)
{
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadParamRegister(output,casia::HandMP_OVER_TEMP_VAL_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_PARAM_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getSystemTempLimit timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
    }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getSystemTempLimit CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  temp_limit_buf[0]=(float(((input[7]<< 8) & 0xff00) + input[6])); 
  return true; 
}


bool CasiaHandMProbuf::setCleanError()
{
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_CLEAR_ERR_INDEX,HandMC_CMD_ENABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setCleanError timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setCleanError CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}       

bool CasiaHandMProbuf::setSaveParam(){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_SAVE_PARAM_INDEX,HandMC_CMD_ENABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setSaveParam timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setSaveParam CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}

bool CasiaHandMProbuf::setResetParam(){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_RSENT_PARAM_INDEX,HandMC_CMD_ENABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setResetParam timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setResetParam CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}

bool CasiaHandMProbuf::setSaveZeroPos(){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_SAVE_ZERO_POS_INDEX,HandMC_CMD_ENABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setSaveZeroPos timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setSaveZeroPos CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}
            
bool CasiaHandMProbuf::setPosSoftTestOpen(){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_POS_SOFT_TEST_INDEX,HandMC_CMD_ENABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setPosSoftReset timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setPosSoftReset CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}
bool CasiaHandMProbuf::setPosSoftTestClose(){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input;  input.clear();
  WriteCmdRegister(output,casia::HandMC_POS_SOFT_TEST_INDEX,HandMC_CMD_DISABLE);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_RW_CMD_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_RW_CMD_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, setPosSoftReset timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, setPosSoftReset CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  return true;
}


bool CasiaHandMProbuf::getSystemTemp(float* board_temp){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadStateRegister(output,casia::HandMS_TEMP_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_R_STATE_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_R_STATE_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getSystemTemp timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getSystemTemp CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  board_temp[0]=float(((input[7]<< 8) & 0xff00) + input[6]); 
  return true;
}

bool CasiaHandMProbuf::getSystemSoftVersion(float* sysytem_soft_version){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadStateRegister(output,casia::HandMS_SOFT_VERSION_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_R_STATE_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_R_STATE_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getSystemSoftVersion timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getSystemSoftVersion CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  sysytem_soft_version[0]=(float)(input[7])+ 0.1f*(float)input[6]; 
  return true;
}

bool CasiaHandMProbuf::getSystemHardwareVersion(float* sysytem_hardware_version){
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadStateRegister(output,casia::HandMS_HARDWARE_VERSION_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_R_STATE_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_R_STATE_DATA_LEN)){
    printf(FONT_RED "hand dev id: %d, getSystemSoftVersion timeout.\r\n" FONT_CLEAR, dev_id_);
    return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, getSystemSoftVersion CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }

  sysytem_hardware_version[0]=(float)(input[7])+ 0.1f*(float)input[6]; 
  return true;
}

bool CasiaHandMProbuf::getError(uint16_t* cur_error)
{
  std::vector<uint8_t> output; output.clear();
  std::vector<uint8_t> input; input.clear();
  ReadStateRegister(output,casia::HandMS_ERROR_INDEX);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)casia::HandM_RS485_CMD_R_STATE_DATA_LEN);
  if(!DevOnlineCheck(input.size(), casia::HandM_RS485_CMD_R_STATE_DATA_LEN)){
  printf(FONT_RED "hand dev id: %d, getError timeout.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  if(!DevcheckSumCheck(input)){
  printf(FONT_RED "hand dev id: %d, getError CheckSum error.\r\n" FONT_CLEAR, dev_id_);
  return false;
  }
  cur_error[0]=((input[7]<< 8) & 0xff00)+ input[6]; 
  return true; 
}


bool CasiaHandMProbuf::setTargetPosPowerSpeed(float* pos_buf,float* power_buf,float  sync_speed_buf,float* cur_angle,float* cur_power,uint8_t dof_num_val,uint8_t param_index){
  std::vector<uint8_t> output;  output.clear();
  std::vector<uint8_t> input;   input.clear();
  std::vector<int16_t> reg_buf; reg_buf.clear();
  uint8_t grasp_dof_num =dof_num_val;
  for (int j = 0; j < grasp_dof_num; j++){reg_buf.push_back(int16_t(pos_buf[j]));}
  for (int j = 0; j < grasp_dof_num; j++){reg_buf.push_back(int16_t(power_buf[j]));}
  reg_buf.push_back(int16_t(sync_speed_buf));
  WriteRegister(output,reg_buf,reg_buf.size(),param_index,grasp_dof_num);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)HandM_WRITE_CONTROL_PARAM_CMD_VAILD_LEN(2,grasp_dof_num)+5);
  if(!DevOnlineCheck(input.size(),HandM_WRITE_CONTROL_PARAM_CMD_VAILD_LEN(2,grasp_dof_num)+5)){
     printf(FONT_RED "hand dev id: %d, setTargetPosForceSpeed timeout.\r\n" FONT_CLEAR, dev_id_);
      return false;
  }
  if(!DevcheckSumCheck(input)){
    printf(FONT_RED "hand dev id: %d, setTargetPosForceSpeed CheckSum error.\r\n" FONT_CLEAR, dev_id_);
   return false;
  } 
  for (int j = 0; j < grasp_dof_num; j++){   
       cur_angle[j]=float(int16_t(((input[7+ j * 2]<< 8) & 0xff00) + input[6+ j * 2]));
   } 

  for (int j = grasp_dof_num; j < (2*grasp_dof_num); j++){   
      cur_power[j-grasp_dof_num]=float(int16_t(((input[7+ j * 2]<< 8) & 0xff00) + input[6+ j * 2]));
  }  
  return true;
}



 bool CasiaHandMProbuf::getPosPowerState(float* cur_angle, float* cur_power,uint8_t dof_num_val)
 {
  uint8_t check_sum;
  std::vector<uint8_t> output;  output.clear();
  std::vector<uint8_t> input;   input.clear();
  uint8_t grasp_dof_num =dof_num_val;
  output.push_back(casia::HandM_RS485_CMD_HEAD_L);
	output.push_back(casia::HandM_RS485_CMD_HEAD_H);
  output.push_back(dev_id_);
	output.push_back(HandM_RS485_CMD_R_STATE_DATA_VAILD_LEN);
  output.push_back(HandM_RS485_CMD_READ_CONTROL_STATE_ID);
  output.push_back(0x01);
  output.push_back(casia::HandMC_CMD_ENABLE & 0xFF);
  output.push_back((casia::HandMC_CMD_ENABLE >>8) & 0xFF);  
  checkSumCalculate(output,check_sum);
  output.push_back(check_sum);
  if(!dev_online_) {return false;}
  ComWriteData(output);
  ComReadData(input, (size_t)HandM_READ_CONTROL_STATE_ACK_CMD_VAILD_LEN(2,grasp_dof_num)+5);
  if(!DevOnlineCheck(input.size(),HandM_READ_CONTROL_STATE_ACK_CMD_VAILD_LEN(2,grasp_dof_num)+5)){
     printf(FONT_RED "hand dev id: %d, getPosForceState timeout.\r\n" FONT_CLEAR, dev_id_);
      return false;
  }
  if(!DevcheckSumCheck(input))
  {
    printf(FONT_RED "hand dev id: %d, getPosForceState CheckSum error.\r\n" FONT_CLEAR, dev_id_);
    


   return false;
  } 
  for (int j = 0; j < grasp_dof_num; j++){   
       cur_angle[j]=float(int16_t(((input[7+ j * 2]<< 8) & 0xff00) + input[6+ j * 2]));
   } 

  for (int j = grasp_dof_num; j < (2*grasp_dof_num); j++){   
      cur_power[j-grasp_dof_num]=float(int16_t(((input[7+ j * 2]<< 8) & 0xff00) + input[6+ j * 2]));
  }  
  return true;
 }

bool CasiaHandMProbuf::IsDevOnline()
{
  return dev_online_;
}

int CasiaHandMProbuf::GetDevId()
{
  return dev_id_;
}

bool CasiaHandMProbuf::DevOnlineCheck(int recv_data_len, int expect_data_len){
  if(recv_data_len != expect_data_len)
  {
    com_timeout_counter_++;
    if(com_timeout_counter_ > HandM_DEV_COM_TIMEOUT_MAX){
      dev_online_ = false;
    }
    return false;
  }
  else if(com_timeout_counter_)
  {
    com_timeout_counter_--;
  }
  return true;
}


void CasiaHandMProbuf::WriteRegister(std::vector<uint8_t> &data,std::vector<int16_t> RegData,int16_t RegLen,uint8_t index,uint8_t dof_num_val)
{
  uint8_t check_sum;
	data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
  data.push_back(HandM_WRITE_CONTROL_PARAM_CMD_VAILD_LEN(2,dof_num_val)+2);
  data.push_back(casia::HandM_RS485_CMD_WRITE_CONTROL_PARAM_ID);
  data.push_back(index);
  for (int j = 0; j < RegLen; j++)  
  { 
    data.push_back(RegData[j] & 0xFF);
    data.push_back((RegData[j]>>8)&0xFF);
  }
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum);
}



void CasiaHandMProbuf::WriteCmdRegister(std::vector<uint8_t> &data,uint8_t RegAddress,uint16_t RegData)
{
  uint8_t check_sum;
	data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
  data.push_back(casia::HandM_RS485_CMD_RW_CMD_DATA_VAILD_LEN);
  data.push_back(casia::HandM_RS485_CMD_WRITE_USER_CMD_ID);
	data.push_back(RegAddress);
  data.push_back(RegData & 0xFF);
  data.push_back((RegData>>8) & 0xFF);
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum);
}

void CasiaHandMProbuf::ReadCmdRegister(std::vector<uint8_t> &data,uint8_t RegAddress)
{
  uint8_t check_sum;
	data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
  data.push_back(casia::HandM_RS485_CMD_RW_CMD_DATA_VAILD_LEN);
  data.push_back(casia::HandM_RS485_CMD_READ_USER_CMD_ID);
	data.push_back(RegAddress);
  data.push_back(casia::HandMC_CMD_ENABLE & 0xFF);
  data.push_back((casia::HandMC_CMD_ENABLE >>8) & 0xFF);  
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum);
}

void CasiaHandMProbuf::ReadParamRegister(std::vector<uint8_t> &data,uint8_t RegAddress)
{
  uint8_t check_sum;
  data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
	data.push_back(casia::HandM_RS485_CMD_RW_PARAM_DATA_VAILD_LEN);
  data.push_back(casia::HandM_RS485_CMD_READ_USER_PARAM_ID);
  data.push_back(RegAddress);
  data.push_back(casia::HandMC_CMD_ENABLE & 0xFF);
  data.push_back((casia::HandMC_CMD_ENABLE >>8) & 0xFF);  
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum);
}

void CasiaHandMProbuf::WriteParamRegister(std::vector<uint8_t> &data,uint8_t RegAddress,uint16_t RegData)
{
  uint8_t check_sum;
  data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
	data.push_back(casia::HandM_RS485_CMD_RW_PARAM_DATA_VAILD_LEN);
  data.push_back(casia::HandM_RS485_CMD_WRITE_USER_PARAM_ID);
	data.push_back(RegAddress);
  data.push_back(RegData & 0xFF);
  data.push_back((RegData>>8) & 0xFF);
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum); 
}

void CasiaHandMProbuf::ReadStateRegister(std::vector<uint8_t> &data,uint8_t RegAddress)
{
  uint8_t check_sum;
  data.push_back(casia::HandM_RS485_CMD_HEAD_L);
	data.push_back(casia::HandM_RS485_CMD_HEAD_H);
  data.push_back(dev_id_);
  data.push_back(casia::HandM_RS485_CMD_R_STATE_DATA_VAILD_LEN);
  data.push_back(casia::HandM_RS485_CMD_READ_STATUS_PARAM_ID);
  data.push_back(RegAddress);
  data.push_back(casia::HandMC_CMD_ENABLE & 0xFF);
  data.push_back((casia::HandMC_CMD_ENABLE >>8) & 0xFF);  
  checkSumCalculate(data,check_sum);
  data.push_back(check_sum);
}

void CasiaHandMProbuf::checkSumCalculate(uint8_t *data ,uint8_t length,uint8_t &check_sum){
	unsigned int check_num = 0;
	for (int i = 0; i <length; i++)
	   check_num = check_num + data[i];
	check_sum=~(check_num & 0xFF);
}

void CasiaHandMProbuf::checkSumCalculate(std::vector<uint8_t> data,uint8_t &check_sum){
	unsigned int check_num = 0;
	for (int i = (int)casia::HandM_RS485_CMD_CHECK_START_INDEX; i <(int)data.size(); i++)
	   check_num = check_num + data[i];
	check_sum=~(check_num & 0xFF);
}

bool CasiaHandMProbuf::DevcheckSumCheck(std::vector<uint8_t> &input_data){
  uint8_t cal_checksum;
  uint8_t read_check_sum;
  uint8_t size_len=input_data.size();
  read_check_sum  =input_data[size_len-HandM_RS485_CMD_CHECK_LEN];
  checkSumCalculate(input_data.data()+casia::HandM_RS485_CMD_CHECK_START_INDEX,size_len-casia::HandM_RS485_CMD_CHECK_LEN-casia::HandM_RS485_CMD_CHECK_START_INDEX,cal_checksum);
  if(cal_checksum!=read_check_sum){
     return false;
  }
  return true; 
}

void CasiaHandMProbuf::crcCalculate(std::vector<uint8_t> data, uint8_t &crcHigh, uint8_t &crcLow){
  uint8_t uchCRCHi = 0xFF ; 
  uint8_t uchCRCLo = 0xFF ; 
  int32_t uIndex ;

  for(size_t i =0;i<data.size();i++)
  {
    uIndex   = uchCRCLo ^ data[i];
    uchCRCLo = uchCRCHi ^ auchCRCLo[uIndex] ;
    uchCRCHi = auchCRCHi[uIndex] ;
  }
  crcHigh = uchCRCHi;
  crcLow = uchCRCLo;
}

void CasiaHandMProbuf::crcCalculate(uint8_t *data,uint8_t length,uint8_t &crcHigh, uint8_t &crcLow){
	uint8_t j;
	uint16_t reg_crc=0xffff;
	while(length--)
	{
		reg_crc^=*data++;
		for(j=0;j<8;j++)
		{
			if(reg_crc & 0x01)
				reg_crc=(reg_crc>>1)^0xa001;
			else
				reg_crc=(reg_crc>>1);
		}
	}
	crcLow=(uint8_t)(reg_crc & 0x00ff);
	crcHigh=(uint8_t)((reg_crc>>8) & 0x00ff);
}

bool CasiaHandMProbuf::DevCrcCheck(std::vector<uint8_t> &input_data){
  uint8_t crcHlgh,crcLow;
  uint8_t read_crch,read_crcl;
  uint8_t size_d=input_data.size();
  read_crch     =input_data[(size_d-1)];
  read_crcl     =input_data[(size_d-2)];
  std::vector<uint8_t> data_;
  for(uint8_t i=0;i<(input_data.size()-2);i++){
    data_.push_back(input_data[i]);
  }  
  crcCalculate(data_,crcHlgh,crcLow);
  if(read_crcl!=crcLow||read_crch!=crcHlgh){
    return false;
  }
  return true;
}

bool CasiaHandMProbuf::ComWriteData(std::vector<uint8_t> &output){
  if( serial_dev_exception_ )
  {
    return false;
  }
  try
  {
    com_port_->write(output);
  }
  catch (serial::SerialException &e)
  {
    serial_dev_exception_ = true;
    dev_online_ = false;
    std::cout<<e.what()<<std::endl;
    return false;
  }
  catch (serial::IOException &e)
  {
    serial_dev_exception_ = true;
    dev_online_ = false;
    std::cout<<e.what()<<std::endl;
    return false;
  }
  catch (serial::PortNotOpenedException &e)
  {
    std::cout<<e.what()<<std::endl;
    serial_dev_exception_ = true;
    dev_online_ = false;
    return false;
  }
#ifdef SDK_DEBUG
  DebugPrintf("Write: ",output); 
#endif
  return true;
}


bool CasiaHandMProbuf::ComReadData(std::vector<uint8_t> &input, uint32_t read_len){
   
  uint8_t error_size=0;
   std::vector<uint8_t> input_read;input_read.clear();
  std::vector<uint8_t> input_error;input_error.clear();
  if(serial_dev_exception_ )
  {
    return false;
  }
  try
  {
    com_port_->read(input, (size_t)read_len);
  }
  catch (serial::PortNotOpenedException &e)
  {
    serial_dev_exception_ = true;
    dev_online_ = false;
    std::cout<<e.what()<<std::endl;
    return false;
  }
  catch (serial::SerialException &e)
  {
    serial_dev_exception_ = true;
    dev_online_ = false;
    std::cout<<e.what()<<std::endl;
    return false;
  }
  catch (serial::IOException &e)
  {
    serial_dev_exception_ = true;
    dev_online_ = false;
    std::cout<<e.what()<<std::endl;
    return false;
  }
#ifdef SDK_DEBUG
  DebugPrintf("Read: ",input);
#endif
  return true;
}

void CasiaHandMProbuf::DebugPrintf(std::string prefixStr,std::vector<uint8_t> &data){
  std::stringstream os; 
  std::string debugStr; 
  for(uint8_t i=0; i<data.size();i++)
  {  
      os<<std::to_string(data[i])+" ";
  }
  debugStr=prefixStr + os.str();
  std::cout<<debugStr<<std::endl;
}

}
