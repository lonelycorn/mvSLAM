#include <vision/fundamental-matrix.hpp>
#include <Eigen/Eigenvalues>
#include <cassert>
#include <algorithm>

#include <base/opencv.hpp>
namespace mvSLAM
{

static constexpr size_t POINT_COUNT_8POINT = 8;

/* find a scaling and a translation, such that the centroid is at
 * the origin and the RMS distance from the origin is sqrt(2)
 * This normalization step is critical for the following reasons:
 *  1. preconditioning to improve numerical stability and accuracy
 *  2. provide invariance of the algorithm in the presence of arbitrary transform
 */
static void 
find_normalization_transform(const std::vector<Vector3Type> &points,
                             std::vector<Vector3Type> &normalized_points,
                             Matrix3Type &T)
{
    const size_t point_count = points.size();
    const ScalarType point_count_inv = static_cast<ScalarType>(1.0 / point_count);
    normalized_points.resize(point_count);

    Vector3Type mean(Vector3Type::Zero());

    for (const auto &p : points)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            mean[i] += p[i];
        }
    }
    mean *= point_count_inv;

    ScalarType scale = 0;
    for (size_t i = 0; i < point_count; ++i)
    {
        normalized_points[i] = points[i] - mean;
        scale += normalized_points[i].norm();
    }
    scale *= point_count_inv; // now this is the average distance towards center
    assert(scale > epsilon);
    scale = static_cast<ScalarType>(sqrt(2.0) / scale);

    for (size_t i = 0; i < point_count; ++i)
        normalized_points[i] *= scale;

    T << scale, 0, -mean[0] * scale,
         0, scale, -mean[1] * scale,
         0, 0, 1;
}

static bool
find_fundamental_matrix_8point(const std::vector<Vector3Type> &normalized_p1,
                               const std::vector<Vector3Type> &normalized_p2,
                               Matrix3Type &F21)
{
    assert(normalized_p1.size() == POINT_COUNT_8POINT);
    assert(normalized_p2.size() == normalized_p1.size());

    /* F21 = [f11, f12, f13,
     *        f21, f22, f23,
     *        f31, f32, f33]
     * p2.T * F21 * p1.T == 0 expands to
     *      x2*x1*f11 + x2*y1*f12 + x2*f13 +
     *      y2*x1*f21 + y2*y1*f22 + y2*f23 +
     *      x1*f31 + y1*f32 + 1*f33 = 0
     * let f = [f11, f12, f13, f21, f22, f23, f31, f32, f33].T
     *     a = [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
     * then a * f == 0
     * construct A from a, one row for each point pair.
     */
    using MatrixAType = Eigen::Matrix<ScalarType, 8, 9>;
    MatrixAType A;
    for (size_t i = 0; i < POINT_COUNT_8POINT; ++i)
    {
        const Vector3Type &p1 = normalized_p1[i];
        const Vector3Type &p2 = normalized_p2[i];
        const ScalarType x1 = p1[0],
                         y1 = p1[1],
                         x2 = p2[0],
                         y2 = p2[1];
        A.row(i) << x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1;
    }
    std::cout<<"A =\n"<<A<<std::endl;

    // solve A * f == 0
    // A = U * S * V.T
    // We want to get square U and square V. Since A is 8-by-9, U would be 8-by-8,
    // V would be 9-by-9, and S would be 8-by-9, which means the last singular value is 0.
    // f then is the right signular vector corresponding to that singular value of 0.
    Eigen::Matrix<ScalarType, 9, 1> f;
    {
        // NOTE 1: Eigen::JacobiSVD() seems to suffer a lot from floating point inaccuracy.
        // we have to use the SVD routine provided by OpenCV
        // NOTE 2: SVD of A.T * A seems to have better accuracy than decomposing A, as 
        // shown by MATLAB
        // NOTE 3: OpenCV::run8Point() computes eigenvalues and eigenvectors, as opposed
        // to SVD, which gives very good (too good to believe) results in unit test. I
        // tried to do the same here, but the results are far worse.
        cv::Mat AT_A_Mat(9, 9, cv_Mat_traits<ScalarType>::DataType);
        for (size_t i = 0; i < 9; ++i)
            for (size_t j = 0; j < 9; ++j)
            {
                AT_A_Mat.at<ScalarType>(i, j) = 0;
                for (size_t k = 0; k < POINT_COUNT_8POINT; ++k)
                    AT_A_Mat.at<ScalarType>(i, j) += A(k, i) * A(k, j);
            }
        std::cout<<"AT_A_Mat = \n"<<AT_A_Mat<<std::endl;

        cv::Mat u, w, vt;
        cv::SVDecomp(AT_A_Mat, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        // column of V == row of V.transpose
        for (size_t i = 0; i < 9; ++i)
            f[i] = vt.at<ScalarType>(POINT_COUNT_8POINT, i);
    }

    // convert to Matrix3Type
    F21 << f[0], f[1], f[2],
           f[3], f[4], f[5],
           f[6], f[7], f[8];
    std::cout<<"F21 = \n"<<F21<<std::endl;
        
    // singular constraint
    {
        cv::Mat Fpre = Matrix3Type_to_Mat(F21);
        cv::Mat u, w, vt;
        cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        std::cout<<"Singular values of F:\n"<<w<<std::endl;
        w.at<ScalarType>(2)=0;
        cv::Mat F21_Mat = u * cv::Mat::diag(w) * vt;
        F21 = Mat_to_Matrix3Type(F21_Mat);
    }
    std::cout<<"After applying the singular constraint, F = \n"<<F21<<std::endl;

    return true;
}

/// code snippet from ORBSLAM
static bool
find_fundamental_matrix_orbslam(const std::vector<Vector3Type> &normalized_p1,
                                const std::vector<Vector3Type> &normalized_p2,
                                Matrix3Type &F21)
{
    std::vector<cv::Point_<double> > vP1, vP2;
    vP1.reserve(normalized_p1.size());
    for (const auto &p : normalized_p1)
        vP1.emplace_back(p[0], p[1]);
    vP2.reserve(normalized_p2.size());
    for (const auto &p : normalized_p2)
        vP2.emplace_back(p[0], p[1]);

    const int N = vP1.size();

    cv::Mat A(N,9,CV_64F);

    for(int i=0; i<N; i++)
    {   
        const double u1 = vP1[i].x;
        const double v1 = vP1[i].y;
        const double u2 = vP2[i].x;
        const double v2 = vP2[i].y;

        A.at<double>(i,0) = u2*u1;
        A.at<double>(i,1) = u2*v1;
        A.at<double>(i,2) = u2; 
        A.at<double>(i,3) = v2*u1;
        A.at<double>(i,4) = v2*v1;
        A.at<double>(i,5) = v2; 
        A.at<double>(i,6) = u1; 
        A.at<double>(i,7) = v1; 
        A.at<double>(i,8) = 1;
    }   

    std::cout<<"A =\n"<<A<<std::endl;

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    std::cout<<"U =\n"<<u<<std::endl;
    cv::Mat v;
    cv::transpose(vt, v);
    std::cout<<"V =\n"<<v<<std::endl;
    std::cout<<"w =\n"<<w<<std::endl;

    cv::Mat Fpre = vt.row(8).reshape(0, 3); 

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<double>(2)=0;

    cv::Mat F21_Mat = u*cv::Mat::diag(w)*vt;

    std::cout<<"F21_Mat = \n"<<F21_Mat<<std::endl;
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            F21(i, j) = F21_Mat.at<double>(i, j);
    return true;
}

bool
find_fundamental_matrix(const std::vector<Vector3Type> &sample_p1,
                        const std::vector<Vector3Type> &sample_p2,
                        Matrix3Type &F21)
{
    (void) find_fundamental_matrix_8point;
    (void) find_fundamental_matrix_orbslam;

    auto ffm = find_fundamental_matrix_8point;
    //auto ffm = find_fundamental_matrix_orbslam;

    /* For every image, the points are first normalized.
     * See Hartley and Zisserman, Multiple View Geometry in Computer Vision, p104
     */
    std::vector<Vector3Type> normalized_p1, normalized_p2;
    Matrix3Type T1, T2;
    find_normalization_transform(sample_p1, normalized_p1, T1);
    find_normalization_transform(sample_p2, normalized_p2, T2);

#if 0 
    {
        std::cout<<"normalized sample p1 = \n";
        for (size_t i = 0; i < POINT_COUNT_8POINT; ++i)
        {
            std::cout<<normalized_p1[i][0]<<", "<<normalized_p1[i][1]<<std::endl;
        }
        std::cout<<"normalized sample p2 = \n";
        for (size_t i = 0; i < POINT_COUNT_8POINT; ++i)
        {
            std::cout<<normalized_p2[i][0]<<", "<<normalized_p2[i][1]<<std::endl;
        }
    }
#endif

    if (!ffm(normalized_p1, normalized_p2, F21))
    {
        std::cout<<"Unable to find findamental matrix."<<std::endl;
        return false;
    }

    // De-normalize the result
    F21 = T2.transpose() * F21 * T1;

#if 0
    if (F21(2, 2) > epsilon) // FIXME: why do we need to do this?
    {
        ScalarType scale = static_cast<ScalarType>(1) / F21(2, 2);
        F21 *= scale;
    }
#endif

    std::cout<<"Finally, fundamental matrix =\n"<<F21<<std::endl;

#if 1
    // Check to see if the constraints are satisfied
    {
        for (size_t i = 0; i < POINT_COUNT_8POINT; ++i)
        {
            std::cout<<"p2.T * F21 * p1 = "<<sample_p2[i].transpose() * F21 * sample_p1[i]<<std::endl;
        }
    }
#endif
    return true;
}

}


