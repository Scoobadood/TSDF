#include <fstream>

#include "FileUtilities.hpp"
/**
 * Process file content line by line.
 * @param file_name The name of the file to be read
 * @param processor Pointer to function taking a single string argument to process
 * @return true if processing was successful
 */
bool process_file_by_lines( std::string file_name, std::function<void(const std::string & )> processor) {
    // Open file
    std::ifstream f( file_name );

    // After this attempt to open a file, we can safely use perror() only
    // in case f.is_open() returns False.
    if (!f.is_open())
        perror(("error while opening file " + file_name).c_str());

    // Read the file via std::getline(). Rules obeyed:
    //   - first the I/O operation, then error check, then data processing
    //   - failbit and badbit prevent data processing, eofbit does not
    std::string line;
    while(getline(f, line)) {
        processor(line);
    }

    // Only in case of set badbit we are sure that errno has been set in
    // the current context. Use perror() to print error details.
    if (f.bad())
        perror(("error while reading file " + file_name).c_str());

    f.close();
    return true;
}
