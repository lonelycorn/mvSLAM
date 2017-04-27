#pragma once
#include <string>
#include <sys/types.h>
#include <dirent.h>

namespace mvSLAM
{

/// iterator to files in a specified directory. does NOT guarantee any ordering.
class DirectoryIterator
{
public:
    /** Ctor.
     * Need to call @ref next() to start iteration.
     * @param [in] directory    directory to iterate through.
     * @param [in] extension    file extension to consider. leave blank for all files.
     */
    DirectoryIterator(const std::string &directory,
                      const std::string &extension = "");
    /// Dtor
    ~DirectoryIterator();

    /** Move the iterator forward.
     * @return false if there is no more entry in this directory.
     */
    bool next();

    /// Check to see if successfully opened the directory.
    bool is_valid() const;

    /// Get the file name that the iterator is pointing to.
    std::string get_file_name() const;

private:
    /** Load the next entry in the directory.
     */
    void load_next_entry();

    bool m_valid;
    std::string m_ext;
    DIR *m_dir;
    struct dirent *m_dir_entry;
    struct dirent *m_dir_entry_next;
};

}
