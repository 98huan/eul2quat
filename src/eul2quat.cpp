#include "iostream"
#include "/home/zh/catkin_ws/src/eul2quat/include/eul2quat.h"
#include <cmath>
using namespace std;
Eigen::Quaterniond QBL[15];
Eigen::Vector3d temp_eulur[15];

int main(int argc, char const *argv[])
{
        QBL[0] = Eigen::Quaterniond(0.00297, 0.00699, -0.01041, 0.99992);
        QBL[1] = Eigen::Quaterniond(0.00270, 0.00549, -0.00858, 0.99994);
        QBL[2] = Eigen::Quaterniond(0.00240, 0.00357, -0.00608, 0.99997);
        QBL[3] = Eigen::Quaterniond(0.00215, 0.00248, -0.00459, 0.99998);
        QBL[4] = Eigen::Quaterniond(0.00193, 0.00174, -0.00354, 0.99999);
        QBL[5] = Eigen::Quaterniond(0.00183, 0.00144, -0.00312, 0.99999);
        QBL[6] = Eigen::Quaterniond(0.00168, 0.00129, -0.00294, 0.99999);
        QBL[7] = Eigen::Quaterniond(0.00163, 0.00123, -0.00284, 0.99999);
        QBL[8] = Eigen::Quaterniond(0.00149, 0.00120, -0.00278, 0.99999);
        QBL[9] = Eigen::Quaterniond(0.00147, 0.00103, -0.00262, 0.99999);
        QBL[10] = Eigen::Quaterniond(0.00134, 0.00090, -0.00235, 1.00000);
        QBL[11] = Eigen::Quaterniond(0.00121, 0.00034, -0.00148, 1.00000);
        QBL[12] = Eigen::Quaterniond(0.00103, -0.00005, -0.00100, 1.00000);
        QBL[13] = Eigen::Quaterniond(0.00080, -0.00163, 0.00146, 1.00000);
        QBL[14] = Eigen::Quaterniond(0.36164, -0.00246, 0.00260, 0.93231);
        // 四元数转欧拉角
        for (int i = 0; i < 15; ++i)
        {
                temp_eulur[i] = ToEulur(QBL[i]);
                cout << i << " " << 57.3 * temp_eulur[i][0] << " " << 57.3 * temp_eulur[i][1] << " " << 57.3 * temp_eulur[i][2] << endl;
        }
        // 欧拉角转四元数
        float eul[3] = {2.40154, 0.00646754, 0.00306881};
        cout << ToEigen(eul) << endl;

        cout << "atan(-1.0 / -1.0) = " << 57.3 * atan(-1.0 / -1.0) << endl;
        cout << "atan(-1.0 / 1.0) = " << 57.3 * atan(-1.0 / 1.0) << endl;
        cout << "atan2(-1.0, -1.0) = " << 57.3 * atan2(-1.0, -1.0) << endl;
        cout << "atan2(-1.0, 1.0) = " << 57.3 * atan2(-1.0, 1.0) << endl;

        return 0;
}
