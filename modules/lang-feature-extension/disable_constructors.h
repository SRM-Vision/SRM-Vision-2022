/**
 * A quick tool to disable redundant constructors.
 * \author trantuan-20048607
 * \date 2022.2.18
 * \details Use constructor disabling tag as one of parent classes.
 */

#ifndef DISABLE_CONSTRUCTORS_H_
#define DISABLE_CONSTRUCTORS_H_

namespace disable_constructors {
    class DisableMoveConstructor {
    public:
        DisableMoveConstructor() = default;

        ~DisableMoveConstructor() = default;

        DisableMoveConstructor(DisableMoveConstructor &&) = delete;

        DisableMoveConstructor &operator=(DisableMoveConstructor &&) = delete;

        DisableMoveConstructor(const DisableMoveConstructor &) = default;

        DisableMoveConstructor &operator=(const DisableMoveConstructor &) = default;
    };

    class DisableCopyConstructor {
    public:
        DisableCopyConstructor() = default;

        ~DisableCopyConstructor() = default;

        DisableCopyConstructor(const DisableCopyConstructor &) = delete;

        DisableCopyConstructor &operator=(const DisableCopyConstructor &) = delete;

        DisableCopyConstructor(DisableCopyConstructor &&) = default;

        DisableCopyConstructor &operator=(DisableCopyConstructor &&) = default;
    };
}

/**
 * \brief Copy constructor disabler.
 * \details Use this tag like a normal class to inherit:
 * \code{.cpp}
 *   class Foo : NO_COPY {
 *     ...
 *   };
 * \endcode
 */
#define NO_COPY public disable_constructors::DisableCopyConstructor

/**
 * \brief Move constructor disabler.
 * \details Use this tag like a normal class to inherit:
 * \code{.cpp}
 *   class Foo : NO_MOVE {
 *     ...
 *   };
 * \endcode
 */
#define NO_MOVE public disable_constructors::DisableMoveConstructor

#endif  // DISABLE_CONSTRUCTORS_H_
