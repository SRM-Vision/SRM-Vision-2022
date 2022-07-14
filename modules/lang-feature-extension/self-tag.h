/**
 * Self tag for C++.
 * \author trantuan-20048607
 * \date 2022.1.20
 * \details Introduced from ruby and python, self tag is used to replace ugly *this
 *   to refer to object itself.
 */

#ifndef SELF_TAG_H_
#define SELF_TAG_H_

/**
 * \brief SELF represents for C++ object itself (simply defined as *this),
 *   corresponding "self" in ruby and python.
 */
#define SELF (*this)

#endif  // SELF_TAG_H_
