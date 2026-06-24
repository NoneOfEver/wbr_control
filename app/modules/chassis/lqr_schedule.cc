#include "lqr_schedule.h"

#include <algorithm>

namespace wbr::v2 {
namespace {

constexpr double kMinScheduledLegLength = 0.10;
constexpr double kMaxScheduledLegLength = 0.40;
constexpr double kLegLengthCenter = 0.25;
constexpr double kLegLengthHalfRange = 0.15;

// Least-squares cubic fits of 16 discrete 1 ms LQR solutions over
// leg_length = 0.10:0.02:0.40 m. Coefficients use normalized leg length
// s = (leg_length - 0.25) / 0.15 and are ordered cubic to constant.
constexpr double kGainPolynomial[2][6][4] = {
    {
        {0.897170608310775, -2.32939317443538, 16.7059976957011,
         38.8526254507589},
        {-0.0228361753682143, 0.232404900551473, 4.32505880995283,
         6.274615308625},
        {-0.911051203574326, 1.13760895558692, -0.647190717200958,
         -20.3911440813467},
        {-0.593999716185213, 0.918710201253934, -2.2448131382717,
         -16.4061795638765},
        {-3.49476918809027, 6.13975253789063, -7.73473464900145,
         13.4763576720677},
        {-0.0831831375503744, 0.176157729076615, -0.293197669452077,
         0.627901172752307},
    },
    {
        {-0.268692451639343, -0.217585030560664, 1.84072646724177,
         -12.1191823100521},
        {-0.0282467037217746, 0.0327540224428837, -0.153261775822214,
         -2.11264770374926},
        {-1.50312292751761, 3.03115999832261, -4.37864307913938,
         7.70509411080729},
        {-0.88378147249717, 1.82503542518382, -2.78193085706165,
         5.92023088220833},
        {4.05692537881045, -5.0574068146009, 2.69758379391917,
         129.971426257738},
        {0.113791933805854, -0.161125613090861, 0.121015841120616,
         2.87982730880655},
    },
};

double EvaluateCubic(const double coefficients[4], double x) {
  return ((coefficients[0] * x + coefficients[1]) * x +
          coefficients[2]) * x + coefficients[3];
}

}  // namespace

void EvaluateLqrGain(double leg_length, double gain[2][6]) {
  const double scheduled_leg_length = std::clamp(
      leg_length, kMinScheduledLegLength, kMaxScheduledLegLength);
  const double normalized_leg_length =
      (scheduled_leg_length - kLegLengthCenter) / kLegLengthHalfRange;

  for (int input = 0; input < 2; ++input) {
    for (int state = 0; state < 6; ++state) {
      gain[input][state] = EvaluateCubic(
          kGainPolynomial[input][state], normalized_leg_length);
    }
  }
}

}  // namespace wbr::v2

