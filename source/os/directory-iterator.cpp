#include <os/directory-iterator.hpp>
#include <cassert>
#include <iostream>

namespace mvSLAM
{
DirectoryIterator::DirectoryIterator(const std::string &directory,
                                     const std::string &extension):
    m_valid(false), m_ext(extension), m_dir(nullptr),
    m_dir_entry(nullptr), m_dir_entry_next(nullptr)
{
    m_dir = opendir(directory.c_str());
    assert(m_dir);

    load_next_entry();
    next();
}

DirectoryIterator::~DirectoryIterator()
{
    if (m_valid)
    {
        int result = closedir(m_dir);
        assert(result == 0);
    }
}

bool
DirectoryIterator::next()
{
    if (m_dir_entry_next == nullptr)
    {
        return false;
    }

    m_dir_entry = m_dir_entry_next;
    load_next_entry();
    return true;
}

std::string
DirectoryIterator::get_file_name() const
{
    if (m_dir_entry == nullptr)
    {
        return std::string("");
    }
    return std::string(m_dir_entry->d_name);
}

void
DirectoryIterator::load_next_entry()
{
    assert(m_dir != nullptr);

    while (true)
    {
        m_dir_entry_next = readdir(m_dir);
        if (m_dir_entry_next == nullptr) // no more entries
        {
            break;
        }

        std::string filename(m_dir_entry_next->d_name);
        size_t fn_len = filename.size();
        size_t ext_len = m_ext.size();
        if (ext_len == 0) // any file is okay
        {
            break;
        }
        if ((fn_len > ext_len + 1) && // note the extra '.'
            (filename[fn_len - ext_len - 1] == '.') &&
            (filename.substr(fn_len - ext_len, ext_len) == m_ext))
        {
            break;
        }
    }
}

}
