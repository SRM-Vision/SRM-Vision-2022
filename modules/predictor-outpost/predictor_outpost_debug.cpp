
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "predictor_outpost_debug.h"

void OutpostPredictorDebug::Save() {
    parameter_maintain_.savePredictorOutpostParameters();
}

void OutpostPredictorDebug::addTrackbar() {
    debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_60cm6m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_pitch_60cm6m_,
                                                    kMax_delta_pitch_up);

    debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_20cm5m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_pitch_20cm5m_,
                                                    kMax_delta_pitch_up);

    debug::Trackbar<double>::Instance().AddTrackbar("delta_pitch_0cm5m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.delta_pitch_0cm5m_,
                                                    kMax_delta_pitch_up);


    debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay_60cm6m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.outpost_shoot_delay_60cm6m_,
                                                    kMax_shoot_delay_);

    debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay_20cm5m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.outpost_shoot_delay_20cm5m_,
                                                    kMax_shoot_delay_);

    debug::Trackbar<double>::Instance().AddTrackbar("shoot_delay_0cm5m:",
                                                    trackbar_windows_name_,
                                                    parameter_maintain_.outpost_shoot_delay_0cm5m_,
                                                    kMax_shoot_delay_);
}
