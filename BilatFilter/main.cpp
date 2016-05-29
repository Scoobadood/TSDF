//
//  main.cpp
//  BilateralFiltering
//
//  Created by Dave on 30/04/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "BilateralFilter.hpp"
#include "TestHelpers.hpp"

typedef struct {
    float       sigma_colour;
    float       sigma_space;
    std::string file_name;
    std::string output_file_name;
} bilat_args_t;



void parse_arguments( int argc, const char * argv[], bilat_args_t & args ) {
    using namespace boost::program_options;
    
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
    ("sigma_colour", value<float>( &args.sigma_colour )->default_value(2.0f), "Sigma for similarity in colour space[2.0]")
    ("sigma_space", value<float>( &args.sigma_space )->default_value(2.0f) , "Sigma for closeness in distance [2.0]")
    ("input_file,i", value< std::vector<std::string> >(), "input image file")
    ("output_file,o", value< std::vector<std::string> >(), "output image file")
    ;
    
    
    // Allow file name without argument tag
    positional_options_description p;
    p.add("input_file", -1);
    
    variables_map vm;
    store( command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    notify(vm);
    
    
    if (vm.count("input_file")) {
        args.file_name = vm["input_file"].as< std::vector< std::string>>()[0];
    }
    
    
    if (vm.count("output_file")) {
        args.output_file_name = vm["output_file"].as< std::vector< std::string>>()[0];
    } else if ( args.file_name.length() > 0 ) {
        boost::filesystem::path in_path{ args.file_name };
        
        std::string in_file_extension = in_path.filename().extension().string();
        std::string in_file = in_path.filename().stem().string();
        std::string path = in_path.remove_filename().string();
        
        args.output_file_name = path.append("/").append(in_file).append("_out").append(in_file_extension);
    }
}



int main(int argc, const char * argv[]) {
    
    // Parse arguments looking for sigmaR, sigmaD and image name
    bilat_args_t args;
    
    parse_arguments( argc, argv, args );
    
    // Load the image and obtain dimensions
    uint32_t width = 0;
    uint32_t height= 0;
    uint16_t * pixels = read_tum_depth_map(args.file_name, width, height);
    
    if( pixels ) {
        
        // Create Bilateral Filter
        BilateralFilter bl{ args.sigma_colour, args.sigma_space };
        
        // Apply it
        bl.filter( pixels, width, height );
        
        // Save output image
        save_depth_map( args.output_file_name, width, height, pixels );
        
        delete [] pixels;
        
    } else {
        std::cerr << "Couldn't load file " << args.file_name << std::endl;
    }
    std::cout << "Done" << std::endl;
    
    return 0;
}
