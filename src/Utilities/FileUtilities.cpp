#include "FileUtilities.hpp"

#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <dirent.h>

/**
 * Process file content line by line.
 * @param file_name The name of the file to be read
 * @param processor Pointer to function taking a single string argument to process
 * @return true if processing was successful
 */
bool process_file_by_lines( const std::string& file_name, std::function<void(const std::string & )> processor) {
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
    while (getline(f, line)) {
        processor(line);
    }

    // Only in case of set badbit we are sure that errno has been set in
    // the current context. Use perror() to print error details.
    if (f.bad())
        perror(("error while reading file " + file_name).c_str());

    f.close();
    return true;
}

/**
 * @param file_name The file (or directory name) to test for
 * @param is_directory Set to true if it's a directory
 * @return true if the file exists
 */
bool file_exists( const std::string& file_name, bool & is_directory ) {
    bool exists = false;

    struct stat fileStat;
    if ( ! stat( file_name.c_str(), &fileStat) ) {
        exists = true;
        if ( S_ISREG(fileStat.st_mode) ) {
            is_directory = false;
        } else if ( S_ISDIR( fileStat.st_mode)  ) {
            is_directory = true;
        }
    }

    return exists;
}

/**
 * Return the list of files in a directory which meet a spefici criteria
 * which is labelled
 * @param directory The directory to search
 * @param files A vector to be filled by file names meeting the filter
 * @param filter The function which filters the files. Should return true if a file name matches the filter.
 */
void files_in_directory( const std::string& directory, std::vector<std::string>& files, std::function< bool( const char * ) > filter ) {
    // Get a directory listing#include <string>
    DIR *dir =  opendir ( directory.c_str() );
    if ( dir != NULL ) {
        struct dirent *ent;

        // Iterate over all the file names
        while ( ( ent = readdir ( dir ) ) != NULL ) {

            // Call the filter with each
            if( filter( ent->d_name ) ) {
                // Add matching file name
                files.push_back( ent->d_name );
            }
        }
        closedir (dir);
    } else {
        // Could not open directory
        std::cerr << "Problem reading directory " << directory << std::endl;
    }
}