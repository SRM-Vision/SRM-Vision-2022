#include <cassert>
#include <glog/logging.h>
#include "cmdline_arg_parser.h"

/**
 * \note DEFINE flags declared in .h file here. Refer to https://gflags.github.io/gflags/.
 * \warning Flags will be initialized before the program entering the main function!
 *   This means any error occurring here will not be caught unless you're using debugger.
 *   (Thus, do not use this variable in any other place and you should not modify it.)
 */
DEFINE_string(type, "", "controller type");
DEFINE_bool(camera, false, "run with camera");
DEFINE_bool(serial, false, "run with serial communication");
DEFINE_bool(record, true, "record videos");

DEFINE_int32(mode_chooser, 0, "controller running mode chooser");

// TODO Temporary flag for debug, will be removed in the future.
DEFINE_bool(rune, false, "run with rune, must under infantry controller");
DEFINE_bool(outpost, false, "run with outpost, must under hero controller");
DEFINE_bool(debug_image, true, "in debug mode show image");
DEFINE_bool(debug_trackbar, true, "in debug use trackbar");
DEFINE_bool(ekf, true, "use ekf");

void CmdlineArgParser::Parse(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Set GLog flags.
    // ================================================
    google::InitGoogleLogging(argv[0]);
    // STDERR settings.
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    // File settings.
    FLAGS_log_dir = "../log";
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_max_log_size = 16;

    run_with_camera_ = FLAGS_camera;
    run_with_serial_ = FLAGS_serial;
    controller_type_ = FLAGS_type;
    record_ = FLAGS_record;

    mode_chooser_ = FLAGS_mode_chooser;
    debug_show_image_ = FLAGS_debug_image;
    debug_use_trackbar_ = FLAGS_debug_trackbar;
    run_mode_rune_ = FLAGS_rune;

    run_mode_outpost_ = FLAGS_outpost;
    with_ekf_ = FLAGS_ekf;

    // Rune mode must be run in infantry controller.
    assert(!run_mode_rune_ || controller_type_ == "infantry");


    // Outpost mode must be run in infantry controller.
    assert(!run_mode_outpost_ || controller_type_ == "hero");

    LOG(INFO) << "Running " << (run_with_camera_ ? "with" : "without") << " camera.";
    LOG(INFO) << "Running " << (run_with_serial_ ? "with" : "without") << " serial communication.";
    LOG(INFO) << "Running " << (run_mode_rune_ ? "with" : "without") << " rune mode.";
    LOG(INFO) << "Running " << (run_mode_outpost_ ? "with" : "without") << " outpost mode.";
    LOG(INFO) << "Running " << (with_ekf_ ? "with" : "without") << " EKF.";
    LOG(INFO) << "Controller type: " << controller_type_;
}
