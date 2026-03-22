#include "mat3.hpp"

Mat3::Mat3(){
    for(int i=0; i<3; ++i)
      for(int j=0; j<3;++j)
        m[i][j]=0.0f;

}
Mat3 Mat3::identity(){
    Mat3 mat;
    mat.m[0][0]=1.0f;
    mat.m[1][1]=1.0f;
    mat.m[2][2]=1.0f;
    return mat;
}
Mat3 Mat3::transpose() const {
    
}
