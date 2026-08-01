#ifndef BALL_MOTION_ESTIMATOR_CONFIG_H
#define BALL_MOTION_ESTIMATOR_CONFIG_H

#include <stdint.h>

/* Current three-point calibration: -50 mm=447 px, O=653 px, +50 mm=869 px. */
#define BALL_CALIBRATION_ORIGIN_PX              ((int32_t)653)
#define BALL_CALIBRATION_POS_SPAN_PX            ((int32_t)216)
#define BALL_CALIBRATION_NEG_SPAN_PX            ((int32_t)206)
#define BALL_CALIBRATION_SPAN_UM                 ((int32_t)50000)

#define BALL_ESTIMATOR_MIN_SCORE_MILLI           ((uint16_t)600U)
/* Current camera firmware uses 0 when confidence is unavailable. */
#define BALL_ESTIMATOR_ACCEPT_ZERO_SCORE          1U
#define BALL_ESTIMATOR_ALPHA_Q10                 ((int32_t)768)
#define BALL_ESTIMATOR_BETA_Q10                  ((int32_t)128)
#define BALL_ESTIMATOR_Q_SHIFT                   10
#define BALL_ESTIMATOR_MIN_FRAME_DT_MS           ((uint32_t)5U)
#define BALL_ESTIMATOR_MAX_FRAME_DT_MS           ((uint32_t)150U)
#define BALL_ESTIMATOR_MAX_PREDICT_DT_MS         ((uint32_t)50U)
#define BALL_ESTIMATOR_MAX_RESIDUAL_UM            ((int32_t)30000)

#endif /* BALL_MOTION_ESTIMATOR_CONFIG_H */
