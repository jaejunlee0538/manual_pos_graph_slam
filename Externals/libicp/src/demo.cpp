/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libicp.
Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libicp can be used

#include <iostream>
#include <libicp/icpPointToPlane.h>
#include <libicp/icpPointToPoint.h>
using namespace std;
using namespace libicp;

int main (int argc, char** argv) {

  libicp::Matrix m(3,3);
  m(0,0) = 0.7071068; m(0,1) = -0.7071068; m(0,2) = 0;
  m(1,0) = 0.7071068; m(1,1) = 0.7071068; m(1,2) = 0;
  m(2,0) = m(2,1) = 0; m(2,2)=1;
  std::cerr<<m.getMat(0,0,2,2)<<std::endl;
  double qx, qy,qz, qw;
  libicp::toQuaternion(m,qx, qy,qz,qw);
  std::cerr<<qx<<","<<qx<<","<<qz<<","<<qw<<std::endl;
  return 0;

  // define a 3 dim problem with 10000 model points
  // and 10000 template points:
  int32_t dim = 2;
  int32_t num = 10000;

  // allocate model and template memory
  double* M = (double*)calloc(dim*num,sizeof(double));
  double* T = (double*)calloc(dim*num,sizeof(double));

  // set model and template points
  cout << endl << "Creating model with 10000 points ..." << endl;
  cout << "Creating template by shifting model by (1,1,1) ..." << endl;
  int32_t k=0;
  for (double x=-2; x<2; x+=0.04) {
    for (double y=-2; y<2; y+=0.04) {
      M[k*dim+0] = x;
      M[k*dim+1] = y;

      T[k*dim+0] = x-1;
      T[k*dim+1] = y-1;

      k++;
    }
  }

  // start with identity as initial transformation
  // in practice you might want to use some kind of prediction here
  Matrix R = Matrix::eye(dim);
  Matrix t(dim,1);
  R(1,0) = 10;

  // run point-to-plane ICP (-1 = no outlier threshold)
  cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
  Icp * picp = new IcpPointToPoint(M,num,dim);
//  IcpPointToPlane icp(M,num,dim);
//  picp->fit(T,num,R,t,-1);
//
//  // results
//  cout << endl << "Transformation results:" << endl;
//  cout << "R:" << endl << R << endl << endl;
//  cout << "t:" << endl << t << endl << endl;

  delete picp;
  // free memory
  free(M);
  free(T);

  // success
  return 0;
}

