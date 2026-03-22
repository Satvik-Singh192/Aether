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
    Mat3 result;
    for(int i=0; i<3; ++i){
        for(int j=0;j<3;++j){
            result.m[i][j]=m[j][i];
        }

    }
    return result;
}
Vec3 Mat3::operator*(const Vec3& v) const {
    return Vec3(
        m[0][0]*v.x+m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        );
}
Mat3 Mat3::operator*(const Mat3& rhs) const {
    Mat3 result;
    for(int i=0;i<3; i++){
        for(int j=0;j<3; j++){
            result.m[i][j]=0.0f;
            for(int k=0; k<3; ++k) {
                result.m[i][j]+=m[i][k]*rhs.m[k][j];
            }
        }
    }
    return result;
}
