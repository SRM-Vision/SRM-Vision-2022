/**
 * Attribute reader method generator for C++.
 * \author trantuan-20048607
 * \date 2022.1.14
 * \details Introduced from ruby programming language, attribute reader generator is
 *   used to quickly generate a function to READ a private attribute in a class.
 */

#ifndef ATTR_READER_H_
#define ATTR_READER_H_

/**
 * \brief Use of ATTR_READER(var_name, func_name) will create a function named func_name
 *   which only returns var_name,
 *   corresponding "attr_reader" in ruby.
 * \details Use this macro in a class to generate a reader function for a private variable:
 * \code{.cpp}
 *   class Foo() {
 *   public:
 *       ATTR_READER(private_var_, PrivateVar)  // No need to add a semicolon.
 *   private:
 *       int private_var_{};
 *   }
 *
 *   int main() {
 *       Foo bar;
 *       int private_var = bar.PrivateVar();  // Get the value.
 *   }
 * \endcode
 */
#define ATTR_READER(_var, _func)  \
    inline auto _func() const { return _var; }

/**
 * \brief Use of ATTR_READER_REF(var_name, func_name) will create a function named func_name
 *   which only returns a read-only reference of var_name,
 *   corresponding "attr_reader" in ruby.
 * \details Use this macro in a class to generate a reader function for a private variable:
 * \code{.cpp}
 *   class Foo() {
 *   public:
 *       Foo() { private_var_.push_back(0); }
 *
 *       ATTR_READER(private_var_, PrivateVar)  // No need to add a semicolon.
 *   private:
 *       std::vector<int> private_var_;
 *   }
 *
 *   int main() {
 *       Foo bar;
 *       for (auto i: bar.PrivateVar())
 *           std::cout << i << std::endl;  // Allowed.  => 0
 *       bar.PrivateVar().push_back(0);  // Prohibited. This operation will change the value inside the object bar.
 *   }
 * \endcode
 */
#define ATTR_READER_REF(_var, _func)  \
    inline auto &_func() const { return _var; }

/// \brief Virtual attribute reader function generator. See ATTR_READER for details.
#define VIRTUAL_ATTR_READER(_type, _var, _func)  \
    virtual _type _func() const { return _var; }

/// \brief Overridden attribute reader function generator. See ATTR_READER for details.
#define OVERRIDE_ATTR_READER(_type, _var, _func)  \
    inline _type _func() const override { return _var; }

#endif  // ATTR_READER_H_
