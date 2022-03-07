#include "iostream"
#include <eigen3/Eigen/Dense>

using namespace std;

inline Eigen::Quaterniond ToEigen(float* eulur)
{
  Eigen::Matrix3d matrix_tmp;
  matrix_tmp = (Eigen::AngleAxisd(eulur[0],Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(eulur[1],Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(eulur[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Quaterniond q(matrix_tmp);
  return q;
}

inline Eigen::Vector3d ToEulur(const Eigen::Quaterniond& q) //q(w,x,y,z)
{
  Eigen::Vector3d eul = q.matrix().eulerAngles(2,1,0);  //yawl pitch roll  
  return eul;
}

Eigen::Quaterniond QBL[15];
Eigen::Vector3d temp_eulur[15];

int main(int argc, char const *argv[])
{
    QBL[0] = Eigen::Quaterniond( 0.00297, 0.00699, -0.01041, 0.99992);
    QBL[1] = Eigen::Quaterniond( 0.00270, 0.00549, -0.00858, 0.99994);
    QBL[2] = Eigen::Quaterniond( 0.00240, 0.00357, -0.00608, 0.99997);
    QBL[3] = Eigen::Quaterniond( 0.00215, 0.00248, -0.00459, 0.99998);
    QBL[4] = Eigen::Quaterniond( 0.00193, 0.00174, -0.00354, 0.99999);
    QBL[5] = Eigen::Quaterniond( 0.00183, 0.00144, -0.00312, 0.99999);
    QBL[6] = Eigen::Quaterniond( 0.00168, 0.00129, -0.00294, 0.99999);
    QBL[7] = Eigen::Quaterniond( 0.00163, 0.00123, -0.00284, 0.99999);
    QBL[8] = Eigen::Quaterniond( 0.00149, 0.00120, -0.00278, 0.99999);
    QBL[9] = Eigen::Quaterniond( 0.00147, 0.00103, -0.00262, 0.99999);
    QBL[10] = Eigen::Quaterniond(0.00134, 0.00090, -0.00235, 1.00000);
    QBL[11] = Eigen::Quaterniond(0.00121, 0.00034, -0.00148, 1.00000);
    QBL[12] = Eigen::Quaterniond(0.00103, -0.00005, -0.00100, 1.00000);
    QBL[13] = Eigen::Quaterniond(0.00080, -0.00163, 0.00146, 1.00000);
    QBL[14] = Eigen::Quaterniond(0.36164, -0.00246, 0.00260, 0.93231);
    // 四元数转欧拉角
    for (int i = 0; i < 15; ++i)
    {
        temp_eulur[i] = ToEulur(QBL[i]);
        cout << i << " " << temp_eulur[i][0] << " " << temp_eulur[i][1] << " " << temp_eulur[i][2] << endl;
    }
    
    return 0;
}
