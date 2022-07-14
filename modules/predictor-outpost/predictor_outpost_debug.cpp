
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "predictor_outpost_debug.h"

void OutpostPredictorDebug::Save() {
    parameter_maintain_.savePredictorOutpostParameters();
}

void OutpostPredictorDebug::addTrackbar() {
    debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_down:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_pitch_down_,
                                                    kMax_delta_pitch_down);

    debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_up:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_pitch_up_,
                                                    kMax_delta_pitch_up);

    debug::Trackbar<double>::Instance().AddTrackbar("delta_yaw_left:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_yaw_left_,
                                                    kMax_delta_yaw_left);

    debug::Trackbar<double>::Instance().AddTrackbar("delta_yaw_right:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_yaw_right_,
                                                    kMax_delta_yaw_right);

    debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.outpost_shoot_delay_,
                                                    kMax_shoot_delay_);
}
