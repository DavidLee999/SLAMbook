#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main()
{
    // data initialization
    Quaterniond q1 (0.35, 0.2, 0.3, 0.1);
    q1.normalize();
    cout << "q1 = " << q1.coeffs() << endl;

    Vector3d t2 (0.3, 0.1, 0.1);

    Quaterniond q2 (-0.5, 0.4, -0.1, 0.2);
    q2.normalize();
    cout << "q2 = " << q2.coeffs() << endl;

    Vector3d t (-0.1, 0.5, 0.3);

    Vector3d p1 (0.5, 0, 0.2);
    
    Isometry3d Tcw1 = Isometry3d::Identity();
    Tcw1.rotate(q1);
    Tcw1.pretranslate(t2);

    Isometry3d Tcw2 = Isometry3d::Identity();
    Tcw2.rotate(q2);
    Tcw2.pretranslate(t);

    // convertion
    // p1 to p-world
    Vector3d pw = Tcw1.inverse() * p1;;
    // p-world to p2
    Vector3d p2 = Tcw2 * pw;

    cout << "p2 = " << p2.transpose() << endl;

    return 0;
}
