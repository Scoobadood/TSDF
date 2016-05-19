//
//  Cubic.hpp
//  KinFu
//
//  Created by Dave on 13/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Cubic_hpp
#define Cubic_hpp

#include <stdio.h>

/**
 * Solve for ax^3 + bx^2 + cx + d = 0
 * Solutions in s
 * Number of roots returned
 */
int solve_cubic( double a, double b, double c, double d, double s[3] );

#endif /* Cubic_hpp */
