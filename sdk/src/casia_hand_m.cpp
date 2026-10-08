#include "casia_hand_m.h"
#include "casia_grasp_control.h"
namespace casia
{
    namespace HandM
    {
        CasiaHandM::CasiaHandM(int l_hand_id, int r_hand_id, int baudrate, std::string port_name)
        {
            hand_control_ = new CasiaHandMControl(l_hand_id, r_hand_id, baudrate, port_name);
        }
        CasiaHandM::~CasiaHandM()
        {
            shutdown();
            delete hand_control_;
            hand_control_ = nullptr;
        }
        bool CasiaHandM::init(double startup_timeout_s)
        {
            return hand_control_->Init(startup_timeout_s);
        }
        void CasiaHandM::shutdown()
        {
            if (hand_control_ != nullptr)
            {
                hand_control_->Shutdown();
            }
        }
        bool CasiaHandM::transportFailed() const
        {
            return hand_control_->TransportFailed();
        }
        void CasiaHandM::clearHandTargets()
        {
            hand_control_->ClearHandMTargets();
        }
        void CasiaHandM::setHandTargetoQueue(casia::HandM::handm_target_set_t &target)
        {
            hand_control_->SetHandMTargetoQueue(target);
        }
        bool CasiaHandM::getHandStateFromQueue(casia::HandM::handm_state_get_t &state)
        {
            return hand_control_->GetHandMStateFromQueue(state);
        }
    }
}
