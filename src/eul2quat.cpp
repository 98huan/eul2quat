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

Eigen::Quaterniond QBL[11];
Eigen::Vector3d temp_eulur[11];

int main(int argc, char const *argv[])
{
    QBL[0] = Eigen::Quaterniond( 0.00187, 0.00179, -0.00336, 0.99999);
    QBL[1] = Eigen::Quaterniond( 0.00219, 0.00141, -0.00288, 0.99999);
    QBL[2] = Eigen::Quaterniond( 0.00161, 0.00172, -0.00288, 0.99999);
    QBL[3] = Eigen::Quaterniond( 0.00112, 0.00136, -0.00324, 0.99999);
    QBL[4] = Eigen::Quaterniond( 0.00200, 0.00188, -0.00310, 0.99999);
    QBL[5] = Eigen::Quaterniond( 0.00163, 0.00124, -0.00285, 0.99999);
    QBL[6] = Eigen::Quaterniond( 0.00151, 0.00132, -0.00278, 0.99999);
    QBL[7] = Eigen::Quaterniond( 0.00215, 0.00156, -0.00264, 0.99999);
    QBL[8] = Eigen::Quaterniond( 0.00124, 0.00195, -0.00380, 0.99999);
    QBL[9] = Eigen::Quaterniond( 0.00153, 0.00377, -0.00695, 0.99997);
    QBL[10] = Eigen::Quaterniond(0.00076, 0.00107, -0.00289, 0.99999);
    // 四元数转欧拉角
    for (int i = 0; i < 11; ++i)
    {
        temp_eulur[i] = ToEulur(QBL[i]);
        cout << i << " " << temp_eulur[i][0] << " " << temp_eulur[i][1] << " " << temp_eulur[i][2] << endl;
    }
    
    return 0;
}
