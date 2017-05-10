#pragma once

#include <base/conversion.hpp>

#include <string>
#include <unordered_map>

namespace mvSLAM
{

/* ===== Config Format =====
 * [module_name]
 * variable_name = variable_value
 */


class ParameterManager
{
public:
    /// Convenience function
    template <typename T>
    static T get_value(const std::string &module_name,
                       const std::string &variable_name,
                       const T& default_value)
    {
        std::string string_value;
        if (!ParameterManager::get_value(module_name, variable_name, string_value))
        {
            return default_value;
        }
        else
        {
            return Conversion<T, std::string>::convert(string_value);
        }
    }

    /// return number of variables loaded; or -1 if failed
    static int load_from_file(const std::string &filename);

    /// return number of variables saved; or -1 if failed
    static int save_to_file(const std::string &filename);

    /** DEBUG ONLY!! set module parameters
     * @return true if overwritten; false if added.
     */
    static bool DEBUG_set_module_parameters(const std::string &name,
            const std::unordered_map<std::string, std::string> &variables);

private:
    /// if found the specified variable, update @p variable_value and return true.
    static bool get_value(const std::string &module_name,
                          const std::string &variable_name,
                          std::string &variable_value);
};


}
