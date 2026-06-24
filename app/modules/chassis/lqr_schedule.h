#ifndef WBR_CONTROL_CORE_LQR_SCHEDULE_H_
#define WBR_CONTROL_CORE_LQR_SCHEDULE_H_

namespace wbr::v2 {

void EvaluateLqrGain(double leg_length, double gain[2][6]);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_LQR_SCHEDULE_H_

