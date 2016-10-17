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

/**
 * Return true if a string matches the given template
 * The template is of the form <prexifx>_<nnn...>.<suffix>
 * Where prefix is a string, there are a number of digits and a suffix
 * This is a utility function to get around the fact that despite appearances to the contrary, 
 * gcc 4.8.n does NOT support regexes
 * We use this function to match input colour, depth and scene flow file names
 * @param prefix The prefix
 * @param num_digits The number of digits to match
 * @param suffix The file extension
 * @param test_string The candidate string to match
 * @return true if it matches else false
 */
bool match_file_name( const std::string& prefix, int num_digits, const std::string& suffix, const std::string& test_string );
