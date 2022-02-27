/**
 * Controller factory header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \details Include this file to create controller objects.
 */

#ifndef CONTROLLER_FACTORY_H_
#define CONTROLLER_FACTORY_H_

#include <glog/logging.h>
#include "controller_base.h"

/// \brief A macro to create a controller of specified type name. Turn to class ControllerFactory for details.
#define CREATE_CONTROLLER(controller_type_name)  \
    ControllerFactory::Instance().CreateController(controller_type_name)

/**
 * \brief Base class of controller registry.
 * \warning You should use its subclass ControllerRegistry instead of this base class.
 */
class ControllerRegistryBase : NO_COPY, NO_MOVE {
public:
    virtual Controller *CreateController() = 0;

protected:
    ControllerRegistryBase() = default;

    virtual ~ControllerRegistryBase() = default;
};

/**
 * \brief Singleton controller factory.
 * \details
 *   For Singleton pattern, refer to https://en.wikipedia.org/wiki/Singleton_pattern.  \n
 *   For Factory pattern, refer to https://en.wikipedia.org/wiki/Factory_method_pattern.
 * \warning
 *   Controller factory will not check whether ControllerType is really subclass of Controller base class.  \n
 *   (Thus, you should ensure that all callings of ControllerRegistry constructor are completely under control.)
 */
class ControllerFactory : NO_COPY, NO_MOVE {
public:
    /**
     * \brief Get the only instance_ of controller factory.
     * \return A controller factory object.
     */
    inline static ControllerFactory &Instance() {
        static ControllerFactory _;
        return _;
    }

    /**
     * \brief Register a controller type.
     * \param [in] controller_type_name Type name of controller.
     * \param [in] registry A registry object of controller.
     * \warning You may call this function only when you're programming for a new type of controller.
     */
    inline void RegisterController(const std::string &controller_type_name, ControllerRegistryBase *registry) {
        controller_registry_[controller_type_name] = registry;
    }

    /**
     * \brief Create a controller whose type is registered to factory.
     * \param [in] controller_type_name Type name of controller.
     * \return A pointer to crated controller.
     * \note You may use macro CREATE_CONTROLLER(controller_type_name) instead of call this function.
     */
    inline Controller *CreateController(const std::string &controller_type_name) {
        if (controller_registry_.find(controller_type_name) != controller_registry_.end()) {
            DLOG(INFO) << "Created " << controller_type_name << " object.";
            return controller_registry_[controller_type_name]->CreateController();
        } else {
            LOG(ERROR) << "Controller type '" << controller_type_name << "' not found.";
            return nullptr;
        }
    }

private:
    ControllerFactory() = default;

    ~ControllerFactory() = default;

    std::unordered_map<std::string, ControllerRegistryBase *> controller_registry_;  ///< Controller registry map.
};

/**
 * \brief Templated controller registry class.
 * \tparam ControllerType Controller type inherited from base class Controller.
 * \attention Once object is constructed, this type of controller will immediately be registered to controller factory.  \n
 *   This means the constructed object is useless and should not appear in any other place.  \n
 *   (Thus, template class though this is, it's better to be treated as a function.)
 * \warning
 *   Controller factory will not check whether ControllerType is really subclass of Controller base class.  \n
 *   (Thus, you should ensure that all callings of ControllerRegistry constructor are completely under control.)
 */
template<class ControllerType>
class ControllerRegistry final : public ControllerRegistryBase {
public:
    /**
     * \brief Constructor of controller registry.
     * \param [in] controller_type_name Type name of controller.
     */
    [[maybe_unused]] explicit ControllerRegistry<ControllerType>(const std::string &controller_type_name) {
        ControllerFactory::Instance().RegisterController(controller_type_name, this);
    }

    /**
     * \brief Create a controller of this type.
     * \return A controller pointer.
     * \warning NEVER directly call this function. Instead, it should be called by controller factory.
     */
    inline Controller *CreateController() final { return new ControllerType(); }
};

#endif  // CONTROLLER_FACTORY_H_
