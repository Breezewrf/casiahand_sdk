#ifndef CASIA_HAND_M_GLOBAL_H_
#define CASIA_HAND_M_GLOBAL_H_
#include <string>
#include <vector>

#define LEFT_HANDM_DEV_ID 1
#define RIGHT_HANDM_DEV_ID 2
#define HANDM_UPDATE_FREQ 200
#define UPDATE_HANDM_TCV_FREQ 1

#define HANDM_SYS_TEMP_RES_MAX 125.0f
#define HANDM_SYS_TEMP_RES 4095.0f
#define HANDM_SYS_TEMP HANDM_SYS_TEMP_RES_MAX / HANDM_SYS_TEMP_RES

#define HANDM_JOINT_TORQUE_RES 0.1f
#define HANDM_JOINT_TORQUE_RES_INT 10.0f

#define HANDM_SPEED_PARAM_NORMAL_RES 255
#define HANDM_DOF_NUM 10
#define HANDM_SAT(val, max, min) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

#define HAND_JOINT_POS_RES 2048
#define HAND_JOINT_TORQUE_RES 2048

#define HAND_JOINT_0_POS_MAX 1.57
#define HAND_JOINT_1_POS_MAX 1.57 / 9.0 * 7.0
#define HAND_JOINT_2_9_POS_MAX 1.57

namespace casia
{
    namespace HandM
    {

        typedef enum
        {
            LEFT_HANDM_INDEX = 0,
            RIGHT_HANDM_INDEX,
            HANDM_NUM
        } handm_index_t;

        typedef enum
        {
            HANDM_ANGLE_RAD_TYPE_INDEX = 1,
            HANDM_ANGLE_NORMAL_TYPE_INDEX = 2,
        } handm_angle_type_t;

        typedef struct
        {
            float handm_speed[HANDM_NUM];
            float handm_angle[HANDM_DOF_NUM * HANDM_NUM];
            float handm_power[HANDM_DOF_NUM * HANDM_NUM];
        } handm_target_set_t;

        typedef struct
        {
            float handm_angle[HANDM_DOF_NUM * HANDM_NUM];
            float handm_power[HANDM_DOF_NUM * HANDM_NUM];
            float handm_temp[HANDM_NUM];
            uint16_t handm_sys_state[HANDM_NUM];
            float handm_sys_soft_version[HANDM_NUM];
            float handm_sys_hardware_version[HANDM_NUM];
        } handm_state_get_t;

        typedef struct
        {
            float handm_angle_limit[HANDM_DOF_NUM * HANDM_NUM];
            float handm_power_limit[HANDM_DOF_NUM * HANDM_NUM];
            float handm_temp_limit[HANDM_NUM];
            float handm_port_baudrate[HANDM_NUM];
            float handm_type[HANDM_NUM];
        } handm_param_get_t;
    }
}
#endif
