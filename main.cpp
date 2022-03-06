#include <iostream>
#include <csignal>
#include "cmdline-arg-parser/cmdline_arg_parser.h"
#include "controller-base/controller_factory.h"

bool Controller::exit_signal_ = false;

void SignalHandler(int signal) {
    if (signal == SIGINT) {
        LOG(WARNING) << "Caught interrupt signal. Attempting to exit...";
        Controller::exit_signal_ = true;
    } else if (signal == SIGTERM) {
        LOG(WARNING) << "Caught terminate signal. Attempting to exit...";
        Controller::exit_signal_ = true;
    }
}

int main(int argc, char *argv[]) {
    // Parse command line flags.
    CmdlineArgParser::Instance().Parse(argc, argv);

    // Create controller.
    Controller *controller = CREATE_CONTROLLER(CmdlineArgParser::Instance().ControllerType());

    if (controller == nullptr)
        return -1;

    controller->Initialize();

    std::signal(SIGINT, &SignalHandler);
    std::signal(SIGTERM, &SignalHandler);

    controller->Run();

    google::ShutdownGoogleLogging();
}
