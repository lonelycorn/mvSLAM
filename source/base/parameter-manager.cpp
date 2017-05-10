#include <base/parameter-manager.hpp>
#include <base/debug.hpp>
#include <base/string-manipulation.hpp>

#include <fstream>
#include <algorithm>
#include <cassert>

namespace mvSLAM
{
static Logger logger("[ParameterManager]", true);

/// (module name -> (variable name -> variable value))
static std::unordered_map<std::string,
    std::unordered_map<std::string, std::string> > m_module_parameters;

int
ParameterManager::load_from_file(const std::string &filename)
{
    logger.info("load from file: '", filename, "'");
    m_module_parameters.clear();

    std::fstream fin(filename, std::ios_base::in);
    if (!fin.is_open())
    {
        logger.error("cannot open file");
        assert(false);
    }

    std::string module_name;
    std::unordered_map<std::string, std::string> variable_values;
    while (!fin.eof())
    {
        // get new line
        std::string line_raw;
        std::getline(fin, line_raw);
        std::string line = string_trim_whitespaces(line_raw);

        // skip empty lines
        if (line.size() == 0)
        {
            continue;
        }

        if ((line.size() > 2) && ('[' == line.front()) && (']' == line.back())) // module name
        {
            if ((module_name.size() > 0) && (variable_values.size() > 0))
            {
                m_module_parameters.emplace(module_name, variable_values);
                module_name.clear();
                variable_values.clear();
            }

            module_name = line.substr(1, line.size() - 2);
            if (m_module_parameters.count(module_name) > 0)
            {
                logger.error("duplicated module '", module_name, "'");
                assert(false);
            }

            if (module_name.size() == 0)
            {
                logger.error("empty module name.");
                assert(false);
            }
        }
        else if ((line.size() > 2) && (std::count(line.begin(), line.end(), '=') == 1)) // variable
        {
            size_t equal_sign_pos = line.find('=');
            std::string variable(line.begin(), line.begin() + equal_sign_pos);
            std::string value(line.begin() + equal_sign_pos + 1, line.end());
            variable = string_trim_whitespaces(variable);
            value = string_trim_whitespaces(value);

            if (variable_values.count(variable) > 0)
            {
                logger.error("duplicate variable '", variable, "' in module '", module_name, "'");
                assert(false);
            }

            if (variable.size() ==  0)
            {
                logger.error("empty variable name in module '", module_name, "'");
                assert(false);
            }

            variable_values.emplace(variable, value);
        }
        else
        {
            logger.error("invalid line: '", line_raw, "'");
            assert(false);
        }
    }

    if ((module_name.size() > 0) && (variable_values.size() > 0))
    {
        m_module_parameters.emplace(module_name, variable_values);
        module_name.clear();
        variable_values.clear();
    }

    int variable_count = 0;
    for (const auto &mp : m_module_parameters)
    {
        variable_count += mp.second.size();
    }
    logger.info("loaded ", variable_count, " variables in ", m_module_parameters.size(), " modules");
    return variable_count;
}

int
ParameterManager::save_to_file(const std::string &filename)
{
    logger.info("save to file: '", filename, "'");
    int variable_count = 0;

    std::fstream fout(filename, std::ios_base::out);

    if (!fout.is_open())
    {
        logger.error("cannot open file");
        assert(false);
    }

    for (const auto &module_name_param_pair : m_module_parameters)
    {
        const auto &module_name = module_name_param_pair.first;
        const auto &module_params = module_name_param_pair.second;
        // module name
        fout << "[" << module_name << "]" << std::endl;

        for (const auto &variable_name_value_pair : module_params)
        {
            const auto &variable_name = variable_name_value_pair.first;
            const auto &variable_value = variable_name_value_pair.second;
            fout << variable_name << " = " << variable_value << std::endl;
            ++variable_count;
        }
        
        fout << std::endl;
    }
    return variable_count;
}

bool
ParameterManager::DEBUG_set_module_parameters(const std::string &name,
        const std::unordered_map<std::string, std::string> &variables)
{
    bool overwritten = (m_module_parameters.count(name) > 0);
    m_module_parameters[name] = variables;
    return overwritten;
}

bool
ParameterManager::get_value(const std::string &module_name,
                            const std::string &variable_name,
                            std::string &variable_value)
{
    auto it = m_module_parameters.find(module_name);
    if ((it != m_module_parameters.end()) &&
        (it->second.count(variable_name) > 0))
    {
        variable_value = it->second.at(variable_name);
        return true;
    }
    return false;
}

}
