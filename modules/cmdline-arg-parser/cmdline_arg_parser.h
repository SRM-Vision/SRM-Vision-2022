/**
 * Command line parser header.
 * \author screw-44, trantuan-20048607
 * \date 2021.1.28
 * \details All arguments will be defined and parsed in this class.
 */

#ifndef CMDLINE_ARG_PARSER_H_
#define CMDLINE_ARG_PARSER_H_

#include <gflags/gflags.h>
#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/disable_constructor.h"

/**
 * \brief Global command line argument parser.
 * \note Use singleton pattern to make it global,
 *   refer to https://en.wikipedia.org/wiki/Singleton_pattern.
 */
class CmdlineArgParser : NO_COPY, NO_MOVE {
public:
    ATTR_READER(run_with_camera_, RunWithCamera)

    ATTR_READER(run_with_serial_, RunWithSerial)

    ATTR_READER(run_with_gimbal_, RunWithGimbal)

    ATTR_READER(run_mode_rune_, RuneModeRune)

    ATTR_READER(debug_use_trackbar_, DebugUseTrackbar)

    ATTR_READER_REF(controller_type_, ControllerType)

    CmdlineArgParser() :
            run_with_camera_(false),
            run_with_gimbal_(false),
            run_with_serial_(false),
            run_mode_rune_(false),
            mode_chooser_(0),
            debug_show_image_(false),
            debug_use_trackbar_(true) {}

    inline static CmdlineArgParser &Instance() {
        static CmdlineArgParser _;
        return _;
    }

    void Parse(int argc, char *argv[]);

private:
    bool run_with_camera_;         ///< Running with camera flag.
    bool run_with_gimbal_;         ///< Running with serial gimbal communication flag.
    bool run_with_serial_;         ///< Running with serial else communication flag.
    std::string controller_type_;  ///< Controller type, no default value.

    int mode_chooser_;         ///< TODO Controller mode chooser, will be implemented in the future.
    bool debug_show_image_;
    bool debug_use_trackbar_;
    bool run_mode_rune_;       ///< secondly controller mode.
};

#endif  // CMDLINE_ARG_PARSER_H_
