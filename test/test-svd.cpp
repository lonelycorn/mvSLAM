#include <Eigen/Core>
#include <Eigen/SVD>
#include <base/math.hpp>
#include <iostream>
using MatrixType = Eigen::Matrix<mvSLAM::ScalarType, 2, 3>;

void test_svd(const MatrixType &A)
{
    Eigen::JacobiSVD<MatrixType> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout<<"Matrix A:\n"<<A<<std::endl;
    std::cout<<"S:\n"<<svd.singularValues()<<std::endl;
    std::cout<<"U:\n"<<svd.matrixU()<<std::endl;
    std::cout<<"V:\n"<<svd.matrixV()<<std::endl;
    std::cout<<"Ax = 0 => x:\n"<<svd.matrixV().block<3, 1>(0, 2)<<std::endl;
}
int main()
{
    {
        std::cout<<"=== Identity matrix ==="<<std::endl;
        MatrixType A = MatrixType::Identity();
        test_svd(A);
    }
    {
        std::cout<<"=== Some integral matrix ==="<<std::endl;
        MatrixType A;
        A << 1, 1, 0,
             0, 1, 1;
        test_svd(A);
    }
    {
        std::cout<<"=== Random matrix ==="<<std::endl;
        MatrixType A = MatrixType::Random();
        test_svd(A);
    }
    return 0;
};
