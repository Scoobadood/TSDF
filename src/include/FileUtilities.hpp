#include <string>
#include <functional>
#include <vector>

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


/**
 * Return the list of files in a directory which meet a spefici criteria
 * which is labelled
 * @param directory The directory to search
 * @param files A vector to be filled by file names meeting the filter
 * @param filter The function which filters the files. Should return true if a file name matches the filter.
 */
void files_in_directory( const std::string& directory, std::vector<std::string>& files, std::function< bool( const char * ) > filter );