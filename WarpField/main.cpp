//
//  main.cpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <iostream>
#include "WarpField.hpp"


std::vector<Eigen::Vector3d> makeInputPoints( ) {
    using namespace Eigen;
    
    // Construct a regular 11x11x11 grid of points
    std::vector<Vector3d> points_in;
    
    for( double z=-3.0; z<= 3.0; z+= 0.5 ) {
        for( double y=-3.0; y<= 3.0; y+= 0.5 ) {
            for( double x=-3.0; x<= 3.0; x+= 0.5 ) {
                points_in.push_back( Vector3d( x, y, z ) );
            }
        }
    }
    
    return points_in;
}



int main( int argc, char *argv[] ) {
    using namespace phd;
    using namespace Eigen;
 
    WarpField wf{};
    
    DeformationNode dn1{Vector3d( 1,1, 0), 0, Vector3d(0.0,0.0,0.0), Vector3d(1, 1, 1), 1};
    dn1.setMaxEffectiveRange(3.0);
    wf.addNode(dn1);
    
    
    DeformationNode dn2{Vector3d( 1, -1, 0), -M_PI_4, Vector3d(0.0,0.0,0.0), Vector3d(1, -1, 1), 1};
    dn2.setMaxEffectiveRange(3.0);
    wf.addNode(dn2);

    DeformationNode dn3{Vector3d(-1, 1, 0), M_PI_4, Vector3d(0.0,0.0,0.0), Vector3d(-1, 1,1), 1};
    dn3.setMaxEffectiveRange(3.0);
    wf.addNode(dn3);
    
    DeformationNode dn4{Vector3d(-1, -1, 0), M_PI_4, Vector3d(0.0,0.0,0.0), Vector3d(-1,-1,1), 1};
    dn4.setMaxEffectiveRange(3.0);
    wf.addNode(dn4);
    
    
    std::vector<Eigen::Vector3d> in         = makeInputPoints();
    std::vector<Eigen::Vector3d> out;
    
    // Iterate over all grid points to find transformation
    for( auto pPoint = in.begin(); pPoint != in.end(); pPoint++ ) {
        Vector3d pout = wf.warp(*pPoint);
        
        out.push_back(pout);
    }
    
    
    // Convert the warp field into a translation and rotation map
    std::cout << "pt_out=[";
    for( auto pPt = out.begin(); pPt != out.end(); pPt++ ) {
        std::cout << pPt->x() << ", " << pPt->y() << ", " <<  pPt->z() << ";";
    }
    std::cout << "];"<<std::endl;

    return 0;
}