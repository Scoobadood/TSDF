#include <string>
#include <functional>
/**
 * Process file content line by line.
 * @param file_name The name of the file to be read
 * @param processor Pointer to function taking a single string argument to process
 * @return true if processing was successful
 */
bool process_file_by_lines( std::string file_name, std::function<void(const std::string & )>);
