/**
 * Image provider factory header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \details Include this file to create image provider objects.
 */

#ifndef IMAGE_PROVIDER_FACTORY_H_
#define IMAGE_PROVIDER_FACTORY_H_

#include <glog/logging.h>
#include "image_provider_base.h"

/// \brief A macro to create an image provider of specified type name. For details, turn to class ImageProviderFactory.
#define CREATE_IMAGE_PROVIDER(ip_type_name)  \
    ImageProviderFactory::Instance().CreateImageProvider(ip_type_name)

/**
 * \brief Base class of image provider registry.
 * \warning You should use its subclass ImageProviderRegistry instead of this base class.
 */
class ImageProviderRegistryBase : NO_COPY, NO_MOVE {
public:
    virtual ImageProvider *CreateImageProvider() = 0;

protected:
    ImageProviderRegistryBase() = default;

    virtual ~ImageProviderRegistryBase() = default;
};

/**
 * \brief Singleton image provider factory.
 * \details
 *   For Singleton pattern, refer to https://en.wikipedia.org/wiki/Singleton_pattern.  \n
 *   For Factory pattern, refer to https://en.wikipedia.org/wiki/Factory_method_pattern.
 * \warning
 *   Image provider factory will not check whether IPType is really subclass of ImageProvider base class.  \n
 *   (Thus, you should ensure that all callings of ImageProviderRegistry constructor are completely under control.)
 */
class ImageProviderFactory : NO_COPY, NO_MOVE {
public:
    ImageProviderFactory(const ImageProviderFactory &) = delete;

    ImageProviderFactory &operator=(const ImageProviderFactory &) = delete;

    /**
     * \brief Get the only instance of image provider factory.
     * \return An image provider factory object.
     */
    inline static ImageProviderFactory &Instance() {
        static ImageProviderFactory _;
        return _;
    }

    /**
     * \brief Register an image provider type.
     * \param [in] ip_type_name Type name of image provider.
     * \param [in] registry A registry object of image provider.
     * \warning You may call this function only when you're programming for a new type of image provider.
     */
    inline void RegisterImageProvider(const std::string &ip_type_name,
                                      ImageProviderRegistryBase *registry) {
        ip_registry[ip_type_name] = registry;
    }

    /**
     * \brief Create an image provider whose type is registered to factory.
     * \param [in] ip_type_name Type name of image provider.
     * \return A pointer to crated image provider.
     * \note You may use macro CREATE_IMAGE_PROVIDER(ip_type_name) instead of call this function.
     */
    inline ImageProvider *CreateImageProvider(const std::string &ip_type_name) {
        if (ip_registry.find(ip_type_name) != ip_registry.end()) {
            DLOG(INFO) << "Created " << ip_type_name << " object.";
            return ip_registry[ip_type_name]->CreateImageProvider();
        } else {
            LOG(ERROR) << "Image provider type '" << ip_type_name << "' not found.";
            return nullptr;
        }
    }

private:
    ImageProviderFactory() = default;

    ~ImageProviderFactory() = default;

    std::unordered_map<std::string, ImageProviderRegistryBase *> ip_registry;  ///< Image provider registry map.
};

/**
 * \brief Templated image provider registry class.
 * \tparam IPType Image provider type inherited from base class ImageProvider.
 * \attention Once object is constructed, this type of image provider will immediately
 *     be registered to image provider factory.  \n
 *   This means the constructed object is useless and should not appear in any other place.  \n
 *   (Thus, template class though this is, it's better to be treated as a function.)
 * \warning
 *   Image provider factory will not check whether IPType is really subclass of ImageProvider base class.  \n
 *   (Thus, you should ensure that all callings of ImageProviderRegistry constructor are completely under control.)
 */
template<class IPType>
class ImageProviderRegistry final : public ImageProviderRegistryBase {
public:
    /**
     * \brief Constructor of image provider registry.
     * \param [in] ip_type_name Type name of image provider.
     */
    explicit ImageProviderRegistry<IPType>(const std::string &ip_type_name) {
        ImageProviderFactory::Instance().RegisterImageProvider(ip_type_name, this);
    }

    /**
     * \brief Create an image provider of this type.
     * \return An image provider pointer.
     * \warning NEVER directly call this function. Instead, it should be called by image provider factory.
     */
    inline ImageProvider *CreateImageProvider() final { return new IPType(); }
};

#endif  // IMAGE_PROVIDER_FACTORY_H_
