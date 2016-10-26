#include "../include/ply.hpp"

#include <iostream>
#include <fstream>

void write_to_ply( const std::string& file_name, const std::vector<float3>& vertices, const std::vector<int3>& triangles ) {
	std::ofstream f { file_name };
	if( f.is_open() ) {
		f << "ply" << std::endl;
		f << "format ascii 1.0" << std::endl;

		f << "element vertex " << vertices.size() << std::endl;
		f << "property float x" << std::endl;
		f << "property float y" << std::endl;
		f << "property float z" << std::endl;

		f << "element face " << triangles.size() << std::endl;
		f << "property list uchar int vertex_indices" << std::endl;
		f << "end_header" << std::endl;

		for ( int v = 0; v < vertices.size(); v++ ) {
			f << vertices[v].x << " " << vertices[v].y << " " << vertices[v].z << std::endl;
		}
		for ( int t = 0; t < triangles.size(); t++ ) {
			f << "3 " << triangles[t].x << " " << triangles[t].y << " " << triangles[t].z << std::endl;
		}
	} else {
		std::cout << "Problem opening file for write " << file_name << std::endl;
	}	
}

