#ifndef CASIA_HAND_M_H_
#define CASIA_HAND_M_H_

#include "casia_hand_m_global.h"
namespace casia
{
    namespace HandM
    {
        class CasiaHandMControl;
        class CasiaHandM
        {
        public:
            CasiaHandM(int l_hand_id, int r_hand_id, int baudrate, std::string port_name = "/dev/ttyUSB0");
            ~CasiaHandM();
            CasiaHandM(const CasiaHandM &) = delete;
            CasiaHandM &operator=(const CasiaHandM &) = delete;
            bool init(double startup_timeout_s = 0.0);
            void shutdown();
            bool transportFailed() const;
            void clearHandTargets();
            void setHandTargetoQueue(casia::HandM::handm_target_set_t &target);
            bool getHandStateFromQueue(casia::HandM::handm_state_get_t &state);
        private:
            CasiaHandMControl *hand_control_;
        };
    }
}

#endif
