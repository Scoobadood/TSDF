#include <string>
#include <functional>
/**
 * Process file content line by line.
 * @param file_name The name of the file to be read
 * @param processor Pointer to function taking a single string argument to process
 * @return true if processing was successful
 */
bool process_file_by_lines( const std::string& file_name, std::function<void(const std::string & )>);


/**
 * @param file_name The file (or directory name) to test for
 * @param is_directory Set to true if it's a directory
 * @return true if the file exists
 */
bool file_exists( const std::string& file_name, bool & is_directory );
