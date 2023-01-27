#include<iostream>
#include <eigen3/Eigen/Dense>
using namespace std;

int main()
{
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

    Eigen::Vector3d a(0.5, 3, -0.4);

    Eigen::Vector3d Aa = A * a;

    double b = 0.5;
    Eigen::MatrixXd joint(2,2);
    Eigen::MatrixXd joint_velocity(2,2);
    Eigen::MatrixXd joint_velocity_step1(2,2);

    
  





    cout << "The multiplication of A * a is " << endl << Aa << endl;
    cout << "Vector3d a :" << endl << a << endl;


    Eigen::MatrixXd m(3,3);
    m << 2, 0, 0,
         0, 1, 0,
         0, 0, 1;
    cout << m << endl;

    cout << m.inverse() << endl;


    Eigen::MatrixXd m12(3,3);
    m12 << 1, 0, 1,
         0, 1, 0,
         0, 0, 1;
    cout << "Vector3d :"  << endl;

    cout << a.transpose()  << endl;

    cout << m12.transpose()  << endl;

    cout << "=========================Admittance Control======================"  << endl;


    Eigen::MatrixXd m2(2,2);
    m2 << 1, 1,
           2, -2;

    Eigen::Vector2d a2(1, 1);
    cout << "==============================linear============================="  << endl;

    joint = b*m2;
    cout << "joint :\n" << joint <<endl;
    joint_velocity = joint.inverse();
    cout << "joint_velocity :\n" << joint_velocity <<endl;
    // joint_velocity_step1 =  a2 * joint_velocity;
    joint_velocity_step1 = joint_velocity * a2;
    cout << "position : \n"<< joint_velocity_step1 << endl;


    double Fx = 40;
    double Dx = 10;
    double NUM1 = 1;
    double Mx = 10;
    double t = 0.1;

    double tau = Mx/Dx; //tau is Mass(M))/Damping(D).

    double Vxd = 0;



    cout << "exp : "<< exp(-5/2) << endl;
    cout << "exp_log : "<< log(exp(1)) << endl;

    //Admittance Control
    Vxd = (Fx / Dx)*(1-exp(-t/tau));
    cout << "Admittance calculater : " <<  Vxd  <<endl;


    

    






    // cout <<  b * m2.inverse() * a2 << endl;








    





    // M1.inverse()

    // std::cout << "r . s =\n" << r.dot(s) << std::endl;
    // std::cout << "r x s =\n" << r.cross(s) << std::endl;

}

//g++ -o Eigen eigen.cpp
// ./Eigen