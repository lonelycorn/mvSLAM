#include <base/error.hpp>
#include <base/opencv.hpp>
#include <base/image.hpp>
#include <os/directory-iterator.hpp>
#include <vision/camera.hpp>

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

class ChessboardParams
{
public:
    ChessboardParams(std::string config_filename)
    {
        m_valid = read_from_file(config_filename);
    }
    bool is_valid() const
    {
        return m_valid;
    }
    const cv::Size &get_pattern_size() const
    {
        return m_pattern_size;
    }
    float get_cell_size() const
    {
        return m_cell_size;
    }
    bool read_from_file(const std::string &filename)
    {
        std::ifstream fin(filename);
        if (!fin.is_open() || !fin.good())
        {
            std::cout << "Cannot open file: '" << filename << "'" << std::endl;
            return false;
        }
        int col, row;
        fin >> col >> row;
        if ((col <= 0) || (row <= 0))
        {
            std::cout << "Invalid chessboard pattern." << std::endl;
            return false;
        }

        float cell_size;
        fin >> cell_size;
        if (cell_size <= 0.0f)
        {
            std::cout << "Invalid cell size." << std::endl;
            return false;
        }

        m_pattern_size = cv::Size(col, row);
        m_cell_size = cell_size;

        return true;
    }
private:
    bool m_valid;
    cv::Size m_pattern_size;
    float m_cell_size;
};

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <directory> <output> <visualization>\n", cmdline);
    std::printf("\tCompute pinhole camera intrinsics.\n");
    std::printf("\t<directory>  directory containing all necessary information:\n");
    std::printf("\t\t<chessboard.txt>   columns and rows, and cell size in mm.\n");
    std::printf("\t\t<*.jpg>    images of the chessboard.\n");
    std::printf("\t<output>     where the calibration result will be written to.\n");
    std::printf("\t<visualization>  pass 1 to visualize result.\n");
}

int main(int argc, char **argv)
{
    const std::string chessboard_param_fn("chessboard.txt");
    const std::string image_extension("jpg");
    constexpr int MIN_IMAGE_COUNT = 10;
    constexpr int FIND_CHESSBOARD_CORNER_FLAGS = 
            cv::CALIB_CB_ADAPTIVE_THRESH | 
            cv::CALIB_CB_NORMALIZE_IMAGE;
            //cv::CALIB_CB_FAST_CHECK;

    if (argc != 4)
    {
        print_help(argv[0]);
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }

    std::string directory_fn(argv[1]);
    std::string output_fn(argv[2]);
    std::string visualize_result(argv[3]);

    // load config
    ChessboardParams cbp(directory_fn + "/" + chessboard_param_fn);
    if (!cbp.is_valid())
    {
        std::cerr << "Unable to load chessboard params." << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::BAD_DATA);
    }
    const cv::Size &pattern_size = cbp.get_pattern_size();
    const float cell_size = cbp.get_cell_size();

    // load images
    mvSLAM::DirectoryIterator di(directory_fn, image_extension);
    std::vector<std::vector<cv::Point2f> > image_points;
    cv::Size image_size(0, 0);
    while (di.next())
    {
        std::string image_fn = di.get_file_name();
        std::cout << "Loading image: '" << image_fn << "'" << std::endl;
        auto image = mvSLAM::load_image_rgb(directory_fn + "/" + image_fn);

        // check image size
        if ((image_size.height == 0) && (image_size.width == 0)) // initialize
        {
            image_size.height = image.rows;
            image_size.width = image.cols;
        }
        else if ((image_size.height != image.rows) ||
                (image_size.width != image.cols))
        {
            std::cerr << "Image size mismatch: (" << image.rows << ", "
                    << image.cols << ") != (" << image_size.height << ", "
                    << image_size.width << ")" << std::endl;;
            return static_cast<int>(mvSLAM::ApplicationErrorCode::BAD_DATA);
        }

        std::vector<cv::Point2f> corners;
        auto found = cv::findChessboardCorners(image,
                pattern_size,
                corners,
                FIND_CHESSBOARD_CORNER_FLAGS);
        if (!found)
        {
            std::cerr << "Cannot find chessboard corners in '"
                    << image_fn << "'"<<std::endl;
            return static_cast<int>(mvSLAM::ApplicationErrorCode::BAD_DATA);
        }
        image_points.push_back(std::move(corners));
    }

    if (image_points.size() < MIN_IMAGE_COUNT)
    {
        std::cerr << "Too few images: " << image_points.size()
                << " < " << MIN_IMAGE_COUNT << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::BAD_DATA);
    }

    // generate chessboard world points
    std::vector<cv::Point3f> chessboard_corners;
    chessboard_corners.reserve(pattern_size.height * pattern_size.width);
    for (int i = 0; i < pattern_size.height; ++i)
    {
        for (int j = 0; j < pattern_size.width; ++j)
        {
            chessboard_corners.emplace_back(
                    static_cast<float>(j * cell_size),
                    static_cast<float>(i * cell_size),
                    0.0f);
        }
    }
    std::vector<std::vector<cv::Point3f > > object_points(
            image_points.size(), chessboard_corners);

    // solve for calibration
    cv::Mat intrinsics_matrix;
    std::vector<double> dist_coeff;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    int flags = 0;
    double reprojection_error = calibrateCamera(object_points,
            image_points,
            image_size,
            intrinsics_matrix,
            dist_coeff,
            rvecs,
            tvecs,
            flags);

    std::cout << "intrinsics =\n" << intrinsics_matrix << std::endl;
    std::cout << "distortion coefficients:";
    for (auto c : dist_coeff)
    {
        std::cout << " " << c;
    }
    std::cout << std::endl;
    std::cout << "reprojection error = " << reprojection_error << std::endl;

    // save results
    auto K = mvSLAM::Mat_to_Matrix<mvSLAM::Matrix3Type>(intrinsics_matrix);
    mvSLAM::PinholeCamera pc(K, mvSLAM::SE3());
    pc.save_to_file(output_fn);

    // visualize results
    if (visualize_result == "1")
    {
        mvSLAM::DirectoryIterator it(directory_fn, image_extension);
        while (it.next())
        {
            std::string image_fn = it.get_file_name();
            std::cout << "Rectifying image: '" << image_fn << "'" << std::endl;
            auto image = mvSLAM::load_image_rgb(directory_fn + "/" + image_fn);
            cv::Mat image_rectified;
            cv::undistort(image, image_rectified, intrinsics_matrix, dist_coeff);
            cv::imshow("rectified image", image_rectified);
            while (cv::waitKey(-1) != 27);
        }
    }
    
    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}
