#include "predictor_rune_debug.h"

void RunePredictorDebug::addTrackbar() {
    /// --- delta_u_ ---
    debug::Trackbar<int>::Instance().AddTrackbar("delta_u (0-4000): ",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.delta_u_,
                                                 kMaxCompensation);

    /// --- delta_v_ ---
    debug::Trackbar<int>::Instance().AddTrackbar("delta_v (0-4000): ",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.delta_v_,
                                                 kMaxCompensation);

    /// --- compensate_time_ ---
    debug::Trackbar<int>::Instance().AddTrackbar("compensate_time (0-1000): ",
                                                 trackbar_window_name_,
                                                 parameter_maintain_.compensate_time_,
                                                 kMaxCompensation >> 2);
}
