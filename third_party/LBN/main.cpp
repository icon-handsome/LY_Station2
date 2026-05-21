#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/Frame2DAnd3D.h"
#include "area_scan_3d_camera/api_util.h"
#include "UserSetManager.h"
#include "FastGeoHash.h"

//#if defined(_MSC_VER) && (_MSC_VER >= 1900)
//#pragma execution_character_set("utf-8")
//#endif

namespace eye = mmind::eye;

namespace 
{

struct Pixel {
    double x = 0.0;
    double y = 0.0;
};

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Options {
    std::string cameraIp;
    std::string parameterFile;
    std::string userSetName;
    int scanCount = 1;
    int searchRadius = 20;
    std::vector<Pixel> rightQueries;
    std::vector<Point3D> xyzQueries;
};

struct GrayImage {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels;

    uint8_t at(int row, int col) const { return pixels[static_cast<size_t>(row) * width + col]; }
    uint8_t& at(int row, int col) { return pixels[static_cast<size_t>(row) * width + col]; }
};

bool isFinitePoint(const eye::PointXYZ& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool isFinitePoint(const eye::PointXYZBGR& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

int clampInt(int value, int low, int high)
{
    return std::max(low, std::min(value, high));
}

std::string nowTag()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

bool ensureDirectory(const std::string& path)
{
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

GrayImage frameToGrayImage(const eye::Frame2D& frame)
{
    const eye::GrayScale2DImage gray = frame.getGrayScaleImage();
    GrayImage out;
    out.width = static_cast<int>(gray.width());
    out.height = static_cast<int>(gray.height());
    out.pixels.resize(static_cast<size_t>(out.width) * out.height);
    for (int r = 0; r < out.height; ++r) {
        for (int c = 0; c < out.width; ++c)
            out.pixels[static_cast<size_t>(r) * out.width + c] = gray.at(r, c).gray;
    }
    return out;
}

bool saveGrayBmp(const GrayImage& img, const std::string& fileName)
{
    if (img.width <= 0 || img.height <= 0 || img.pixels.empty())
        return false;

    const int rowStride = ((img.width * 3 + 3) / 4) * 4;
    const int imageSize = rowStride * img.height;
    const int fileSize = 54 + imageSize;

    std::ofstream os(fileName, std::ios::binary);
    if (!os)
        return false;

    auto writeU16 = [&os](uint16_t v) {
        os.put(static_cast<char>(v & 0xff));
        os.put(static_cast<char>((v >> 8) & 0xff));
    };
    auto writeU32 = [&os](uint32_t v) {
        os.put(static_cast<char>(v & 0xff));
        os.put(static_cast<char>((v >> 8) & 0xff));
        os.put(static_cast<char>((v >> 16) & 0xff));
        os.put(static_cast<char>((v >> 24) & 0xff));
    };

    writeU16(0x4d42);
    writeU32(static_cast<uint32_t>(fileSize));
    writeU16(0);
    writeU16(0);
    writeU32(54);
    writeU32(40);
    writeU32(static_cast<uint32_t>(img.width));
    writeU32(static_cast<uint32_t>(img.height));
    writeU16(1);
    writeU16(24);
    writeU32(0);
    writeU32(static_cast<uint32_t>(imageSize));
    writeU32(2835);
    writeU32(2835);
    writeU32(0);
    writeU32(0);

    std::vector<uint8_t> row(static_cast<size_t>(rowStride), 0);
    for (int r = img.height - 1; r >= 0; --r) {
        std::fill(row.begin(), row.end(), 0);
        for (int c = 0; c < img.width; ++c) {
            const uint8_t v = img.at(r, c);
            row[static_cast<size_t>(c) * 3 + 0] = v;
            row[static_cast<size_t>(c) * 3 + 1] = v;
            row[static_cast<size_t>(c) * 3 + 2] = v;
        }
        os.write(reinterpret_cast<const char*>(row.data()), row.size());
    }
    return true;
}

void drawCross(GrayImage& img, const Pixel& p, int radius = 18)
{
    const int x = static_cast<int>(std::round(p.x));
    const int y = static_cast<int>(std::round(p.y));
    if (x < 0 || y < 0 || x >= img.width || y >= img.height)
        return;

    for (int dx = -radius; dx <= radius; ++dx) {
        const int xx = x + dx;
        if (xx >= 0 && xx < img.width)
            img.at(y, xx) = 255;
    }
    for (int dy = -radius; dy <= radius; ++dy) {
        const int yy = y + dy;
        if (yy >= 0 && yy < img.height)
            img.at(yy, x) = 255;
    }

    for (int dy = -3; dy <= 3; ++dy) {
        for (int dx = -3; dx <= 3; ++dx) {
            const int xx = x + dx;
            const int yy = y + dy;
            if (xx >= 0 && yy >= 0 && xx < img.width && yy < img.height)
                img.at(yy, xx) = 0;
        }
    }
}

void writeMatrix3x3(std::ostream& os, const char* name, const eye::CameraMatrix& k)
{
    os << name << "_fx=" << k.fx << "\n";
    os << name << "_fy=" << k.fy << "\n";
    os << name << "_cx=" << k.cx << "\n";
    os << name << "_cy=" << k.cy << "\n";
    os << name << "_matrix=\n";
    os << k.fx << " 0 " << k.cx << "\n";
    os << "0 " << k.fy << " " << k.cy << "\n";
    os << "0 0 1\n";
}

void writeDistortion(std::ostream& os, const char* name, const eye::CameraDistortion& d)
{
    os << name << "_distortion_k1=" << d.k1 << "\n";
    os << name << "_distortion_k2=" << d.k2 << "\n";
    os << name << "_distortion_p1=" << d.p1 << "\n";
    os << name << "_distortion_p2=" << d.p2 << "\n";
    os << name << "_distortion_k3=" << d.k3 << "\n";
}

void writeTransform(std::ostream& os, const char* name, const eye::Transformation& t)
{
    os << name << "_rotation=\n";
    for (int r = 0; r < 3; ++r)
        os << t.rotation[r][0] << " " << t.rotation[r][1] << " " << t.rotation[r][2] << "\n";
    os << name << "_translation_mm=" << t.translation[0] << " " << t.translation[1] << " "
       << t.translation[2] << "\n";
}

bool saveIntrinsics(const eye::CameraIntrinsics& intrinsics, const std::string& fileName)
{
    std::ofstream os(fileName);
    if (!os)
        return false;

    os << std::setprecision(12);
    writeMatrix3x3(os, "texture", intrinsics.texture.cameraMatrix);
    writeDistortion(os, "texture", intrinsics.texture.cameraDistortion);
    writeMatrix3x3(os, "depth", intrinsics.depth.cameraMatrix);
    writeDistortion(os, "depth", intrinsics.depth.cameraDistortion);
    writeTransform(os, "depth_to_texture", intrinsics.depthToTexture);
    return true;
}

bool queryPointInterpolated(const eye::PointCloud& cloud, 
                            double x,
                            double y,
                            eye::PointXYZ& out,
                            int radius = 4)
{
    if (cloud.isEmpty())
        return false;

    const int width = static_cast<int>(cloud.width());
    const int height = static_cast<int>(cloud.height());
    if (x < 0 || y < 0 || x > width - 1 || y > height - 1)
        return false;

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const double dx = x - x0;
    const double dy = y - y0;

    const eye::PointXYZ p00 = cloud.at(y0, x0);
    const eye::PointXYZ p10 = cloud.at(y0, x1);
    const eye::PointXYZ p01 = cloud.at(y1, x0);
    const eye::PointXYZ p11 = cloud.at(y1, x1);
    if (isFinitePoint(p00) && isFinitePoint(p10) && isFinitePoint(p01) && isFinitePoint(p11)) {
        const double w00 = (1.0 - dx) * (1.0 - dy);
        const double w10 = dx * (1.0 - dy);
        const double w01 = (1.0 - dx) * dy;
        const double w11 = dx * dy;
        out.x = static_cast<float>(w00 * p00.x + w10 * p10.x + w01 * p01.x + w11 * p11.x);
        out.y = static_cast<float>(w00 * p00.y + w10 * p10.y + w01 * p01.y + w11 * p11.y);
        out.z = static_cast<float>(w00 * p00.z + w10 * p10.z + w01 * p01.z + w11 * p11.z);
        return true;
    }

    double sumW = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    for (int yy = clampInt(y0 - radius, 0, height - 1); yy <= clampInt(y0 + radius, 0, height - 1); ++yy)
    {
        for (int xx = clampInt(x0 - radius, 0, width - 1); xx <= clampInt(x0 + radius, 0, width - 1); ++xx) 
        {
            const eye::PointXYZ p = cloud.at(yy, xx);
            if (!isFinitePoint(p))
            {
                continue;
            }
            const double dist2 = (xx - x) * (xx - x) + (yy - y) * (yy - y);
            const double w = 1.0 / std::max(dist2, 1e-6);
            sumW += w;
            sx += w * p.x;
            sy += w * p.y;
            sz += w * p.z;
        }
    }

    if (sumW <= 0.0)
        return false;

    out.x = static_cast<float>(sx / sumW);
    out.y = static_cast<float>(sy / sumW);
    out.z = static_cast<float>(sz / sumW);
    return true;
}

bool queryPointNearest(const eye::PointCloud& cloud, double x, double y, eye::PointXYZ& out)
{
    if (cloud.isEmpty())
        return false;

    const int col = static_cast<int>(std::round(x));
    const int row = static_cast<int>(std::round(y));
    if (col < 0 || row < 0 || col >= static_cast<int>(cloud.width()) ||
        row >= static_cast<int>(cloud.height()))
        return false;

    out = cloud.at(row, col);
    return isFinitePoint(out);
}

bool queryPointInterpolated(const eye::TexturedPointCloud& cloud, double x, double y,
                            eye::PointXYZBGR& out, int radius = 4)
{
    if (cloud.isEmpty())
        return false;

    const int width = static_cast<int>(cloud.width());
    const int height = static_cast<int>(cloud.height());
    if (x < 0 || y < 0 || x > width - 1 || y > height - 1)
        return false;

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const double dx = x - x0;
    const double dy = y - y0;

    const eye::PointXYZBGR p00 = cloud.at(y0, x0);
    const eye::PointXYZBGR p10 = cloud.at(y0, x1);
    const eye::PointXYZBGR p01 = cloud.at(y1, x0);
    const eye::PointXYZBGR p11 = cloud.at(y1, x1);
    if (isFinitePoint(p00) && isFinitePoint(p10) && isFinitePoint(p01) && isFinitePoint(p11)) {
        const double w00 = (1.0 - dx) * (1.0 - dy);
        const double w10 = dx * (1.0 - dy);
        const double w01 = (1.0 - dx) * dy;
        const double w11 = dx * dy;
        out.x = static_cast<float>(w00 * p00.x + w10 * p10.x + w01 * p01.x + w11 * p11.x);
        out.y = static_cast<float>(w00 * p00.y + w10 * p10.y + w01 * p01.y + w11 * p11.y);
        out.z = static_cast<float>(w00 * p00.z + w10 * p10.z + w01 * p01.z + w11 * p11.z);
        return true;
    }

    double sumW = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    for (int yy = clampInt(y0 - radius, 0, height - 1); yy <= clampInt(y0 + radius, 0, height - 1); ++yy) {
        for (int xx = clampInt(x0 - radius, 0, width - 1); xx <= clampInt(x0 + radius, 0, width - 1); ++xx) {
            const eye::PointXYZBGR p = cloud.at(yy, xx);
            if (!isFinitePoint(p))
                continue;
            const double dist2 = (xx - x) * (xx - x) + (yy - y) * (yy - y);
            const double w = 1.0 / std::max(dist2, 1e-6);
            sumW += w;
            sx += w * p.x;
            sy += w * p.y;
            sz += w * p.z;
        }
    }

    if (sumW <= 0.0)
        return false;

    out.x = static_cast<float>(sx / sumW);
    out.y = static_cast<float>(sy / sumW);
    out.z = static_cast<float>(sz / sumW);
    return true;
}

bool queryPointNearest(const eye::TexturedPointCloud& cloud, double x, double y,
                       eye::PointXYZBGR& out)
{
    if (cloud.isEmpty())
        return false;

    const int col = static_cast<int>(std::round(x));
    const int row = static_cast<int>(std::round(y));
    if (col < 0 || row < 0 || col >= static_cast<int>(cloud.width()) ||
        row >= static_cast<int>(cloud.height()))
        return false;

    out = cloud.at(row, col);
    return isFinitePoint(out);
}

bool findNearestValidPixelAround(const eye::TexturedPointCloud& cloud,
                                 double x,
                                 double y,
                                 int radius,
                                 Pixel& pixel,
                                 eye::PointXYZBGR& out,
                                 double& pixelDistance)
{
    if (cloud.isEmpty())
        return false;

    const int width = static_cast<int>(cloud.width());
    const int height = static_cast<int>(cloud.height());
    const int centerCol = static_cast<int>(std::round(x));
    const int centerRow = static_cast<int>(std::round(y));
    if (centerCol < 0 || centerRow < 0 || centerCol >= width || centerRow >= height)
        return false;

    double bestDist2 = std::numeric_limits<double>::max();
    int bestCol = -1;
    int bestRow = -1;
    eye::PointXYZBGR bestPoint;
    const int safeRadius = std::max(0, radius);
    const double maxDist2 = static_cast<double>(safeRadius) * safeRadius;

    for (int row = clampInt(centerRow - safeRadius, 0, height - 1);
         row <= clampInt(centerRow + safeRadius, 0, height - 1);
         ++row) {
        for (int col = clampInt(centerCol - safeRadius, 0, width - 1);
             col <= clampInt(centerCol + safeRadius, 0, width - 1);
             ++col) {
            const double dx = col - x;
            const double dy = row - y;
            const double dist2 = dx * dx + dy * dy;
            if (dist2 > maxDist2)
                continue;

            const eye::PointXYZBGR p = cloud.at(row, col);
            if (!isFinitePoint(p))
                continue;

            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestCol = col;
                bestRow = row;
                bestPoint = p;
            }
        }
    }

    if (bestCol < 0)
        return false;

    pixel.x = bestCol;
    pixel.y = bestRow;
    out = bestPoint;
    pixelDistance = std::sqrt(bestDist2);
    return true;
}

bool findNearestPixelByXYZ(const eye::PointCloud& cloud, const Point3D& target, Pixel& pixel,
                           eye::PointXYZ& nearest, double& distance)
{
    if (cloud.isEmpty())
        return false;

    double bestDist2 = std::numeric_limits<double>::max();
    int bestCol = -1;
    int bestRow = -1;
    eye::PointXYZ bestPoint;

    for (int row = 0; row < static_cast<int>(cloud.height()); ++row) {
        for (int col = 0; col < static_cast<int>(cloud.width()); ++col) {
            const eye::PointXYZ p = cloud.at(row, col);
            if (!isFinitePoint(p))
                continue;

            const double dx = p.x - target.x;
            const double dy = p.y - target.y;
            const double dz = p.z - target.z;
            const double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestCol = col;
                bestRow = row;
                bestPoint = p;
            }
        }
    }

    if (bestCol < 0)
        return false;

    pixel.x = bestCol;
    pixel.y = bestRow;
    nearest = bestPoint;
    distance = std::sqrt(bestDist2);
    return true;
}

bool findNearestPixelByXYZ(const eye::TexturedPointCloud& cloud, const Point3D& target, Pixel& pixel,
                           eye::PointXYZBGR& nearest, double& distance)
{
    if (cloud.isEmpty())
        return false;

    double bestDist2 = std::numeric_limits<double>::max();
    int bestCol = -1;
    int bestRow = -1;
    eye::PointXYZBGR bestPoint;

    for (int row = 0; row < static_cast<int>(cloud.height()); ++row) {
        for (int col = 0; col < static_cast<int>(cloud.width()); ++col) {
            const eye::PointXYZBGR p = cloud.at(row, col);
            if (!isFinitePoint(p))
                continue;

            const double dx = p.x - target.x;
            const double dy = p.y - target.y;
            const double dz = p.z - target.z;
            const double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                bestCol = col;
                bestRow = row;
                bestPoint = p;
            }
        }
    }

    if (bestCol < 0)
        return false;

    pixel.x = bestCol;
    pixel.y = bestRow;
    nearest = bestPoint;
    distance = std::sqrt(bestDist2);
    return true;
}

void printUsage()
{
    std::cout << "Usage:\n"
              << "  MechEyeNanoUltraQuery.exe --ip <camera_ip> [--params <json_file>] "
                 "[--user-set <name>] [--search-radius <pixels>] --right <x> <y>\n\n"
              << "Examples:\n"
              << "  MechEyeNanoUltraQuery.exe\n"
              << "  MechEyeNanoUltraQuery.exe --ip 192.168.1.100 --right 640 360\n"
              << "  MechEyeNanoUltraQuery.exe --ip 192.168.1.100 --search-radius 50 "
                 "--right 1342 1087\n"
              << "  MechEyeNanoUltraQuery.exe --ip 192.168.1.100 --params "
                 "\"C:\\Users\\lenovo\\Desktop\\99_RUM3525CA510C098_parameter_group.json\" "
                 "--user-set default --right 640 360\n"
              << "  MechEyeNanoUltraQuery.exe --ip 192.168.1.100 --xyz -1.765 32.212 347.267\n";
}

bool connectCamera(eye::Camera& camera, const std::string& ip)
{
    if (!ip.empty()) {
        const eye::ErrorStatus status = camera.connect(ip);
        if (!status.isOK()) {
            showError(status);
            return false;
        }
        std::cout << "Connected to camera: " << ip << "\n";
        return true;
    }
    return findAndConnect(camera);
}

bool loadCameraParameters(eye::Camera& camera, const Options& options)
{
    if (options.parameterFile.empty() && options.userSetName.empty())
        return true;

    eye::UserSetManager& userSetManager = camera.userSetManager();

    if (!options.parameterFile.empty()) {
        std::cout << "Loading camera parameter groups from JSON: " << options.parameterFile << "\n";
        std::cout << "Note: SDK loadFromFile imports user sets to the camera and overwrites "
                     "same-name user sets on the device.\n";
        const eye::ErrorStatus status = userSetManager.loadFromFile(options.parameterFile);
        if (!status.isOK()) {
            showError(status);
            return false;
        }
        std::cout << "Loaded camera parameter groups successfully.\n";
    }

    if (!options.userSetName.empty()) {
        std::cout << "Selecting camera user set: " << options.userSetName << "\n";
        const eye::ErrorStatus status = userSetManager.selectUserSet(options.userSetName);
        if (!status.isOK()) {
            showError(status);
            return false;
        }
    }

    std::string currentName;
    const eye::ErrorStatus nameStatus = userSetManager.currentUserSet().getName(currentName);
    if (nameStatus.isOK())
        std::cout << "Current camera user set: " << currentName << "\n";
    else
        showError(nameStatus);

    return true;
}

bool parseArgs(int argc, char* argv[], Options& options)
{
    int arg = 1;
    while (arg < argc) {
        const std::string token = argv[arg];
        if (token == "--help") {
            printUsage();
            return false;
        }
        if (token == "--count" && arg + 1 < argc) {
            options.scanCount = std::max(1, std::atoi(argv[arg + 1]));
            arg += 2;
            continue;
        }
        if (token == "--search-radius" && arg + 1 < argc) {
            options.searchRadius = std::max(0, std::atoi(argv[arg + 1]));
            arg += 2;
            continue;
        }
        if (token == "--ip" && arg + 1 < argc) {
            options.cameraIp = argv[arg + 1];
            arg += 2;
            continue;
        }
        if ((token == "--params" || token == "--parameter-file" || token == "--config") &&
            arg + 1 < argc) 
        {
            options.parameterFile = argv[arg + 1];
            arg += 2;
            continue;
        }
        if (token == "--user-set" && arg + 1 < argc) {
            options.userSetName = argv[arg + 1];
            arg += 2;
            continue;
        }
        if (token == "--right" && arg + 2 < argc) {
            options.rightQueries.push_back({std::atof(argv[arg + 1]), std::atof(argv[arg + 2])});
            arg += 3;
            continue;
        }
        if (token == "--xyz" && arg + 3 < argc) {
            options.xyzQueries.push_back({std::atof(argv[arg + 1]), std::atof(argv[arg + 2]),
                                          std::atof(argv[arg + 3])});
            arg += 4;
            continue;
        }

        break;
    }

    if (arg < argc) {
        options.cameraIp = argv[arg++];
    }
    if (argc - arg >= 2) {
        options.rightQueries.push_back({std::atof(argv[arg]), std::atof(argv[arg + 1])});
        arg += 2;
    }
    return true;
}

int frame2dToCvMat(const eye::Frame2D& textureFrame, cv::Mat& cvImg)
{
    const eye::GrayScale2DImage gray = textureFrame.getGrayScaleImage();

    if (gray.isEmpty())
        return 1;

    const int w = static_cast<int>(gray.width());
    const int h = static_cast<int>(gray.height());

    if (w <= 0 || h <= 0)
        return 2;

    cvImg.create(h, w, CV_8UC1);
    for (int row = 0; row < h; ++row) {
        uint8_t* dst = cvImg.ptr<uint8_t>(row);
        for (int col = 0; col < w; ++col)
            dst[col] = gray.at(row, col).gray;
    }

    return 0;
}

} // namespace



int main(int argc, char* argv[])
{
    std::vector<cv::Point3f> mark_centers_3D;
    mark_centers_3D.reserve(MARK_POINT_SIZE_MAX);







    // 3D camera setting
    Options options;
    if (!parseArgs(argc, argv, options))
        return 0;

    std::cout << "options.parameterFile:   " << options.parameterFile << std::endl;
    std::cout << "options.userSetName:   " << options.userSetName << std::endl;

    eye::Camera camera;
    if (!connectCamera(camera, options.cameraIp))
        return 1;

    if (!loadCameraParameters(camera, options))
    {
        camera.disconnect();
        return 1;
    }

    showError(camera.setPointCloudUnit(eye::CoordinateUnit::Millimeter));

    eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    eye::CameraIntrinsics intrinsics;
    showError(camera.getCameraIntrinsics(intrinsics));
    printCameraIntrinsics(intrinsics);

    const std::string outDir = "output";
    if (!ensureDirectory(outDir)) {
        std::cerr << "Failed to create output directory: " << outDir << "\n";
        camera.disconnect();
        return 2;
    }

    // 3D camera scan
    eye::Frame2D textureFrame;
    eye::Frame3D frame3D;
    eye::TexturedPointCloud texturedPointCloud;
    GrayImage textureGray;
    std::ostringstream prefix;
    prefix << outDir << "\\scan_" << std::setw(4) << std::setfill('0') << "_" << nowTag();
    {
        eye::Frame2DAnd3D frame2DAnd3D;
        showError(camera.capture2DAnd3D(frame2DAnd3D, 10000));

        textureFrame       = frame2DAnd3D.frame2D();
        // frame3D            = frame2DAnd3D.frame3D();
        texturedPointCloud = frame2DAnd3D.getTexturedPointCloud(); // 很关键，点云和图像已经对齐了
        std::cout << "Textured point cloud size: " << texturedPointCloud.width() << " x "
                  << texturedPointCloud.height() << "\n";
         
        textureGray = frameToGrayImage(textureFrame);
        std::cout << "Texture image size: " << textureGray.width << " x " << textureGray.height
            << "\n";
        const std::string textureFile = prefix.str() + "_texture_aligned.bmp";
        const std::string cloudFile = prefix.str() + "_textured_point_cloud.ply";
        const std::string intrinsicsFile = prefix.str() + "_camera_intrinsics.txt";

        saveGrayBmp(textureGray, textureFile);
        showError(frame2DAnd3D.saveTexturedPointCloud(eye::FileFormat::PLY, cloudFile, true));
        if (!saveIntrinsics(intrinsics, intrinsicsFile))
            std::cerr << "Failed to save intrinsics: " << intrinsicsFile << "\n";

        std::cout << "Saved texture-aligned image: " << textureFile << "\n";
        std::cout << "Saved organized textured point cloud: " << cloudFile << "\n";
        std::cout << "Saved intrinsics/extrinsics: " << intrinsicsFile << "\n";
        std::cout << "Right/texture query search radius: " << options.searchRadius << " pixels\n";
    }








    // texture-aligned image search 2D center
    cv::Mat              cvImg_from_mec;
    int stat = frame2dToCvMat(textureFrame,            // Mech image
                              cvImg_from_mec);
    if (stat != 0)
    {
        std::cout << "ERRs from frame2dToCvMat: " << stat << std::endl;
        return stat;
    }

    // search 2D centers
    MarkPointDetector   right_img_mark_detec;
    std::vector<cv::Point2f>  mark_centers;      // marker points
    mark_centers.reserve(MARK_POINT_SIZE_MAX);
    bool stat_bool = right_img_mark_detec.ProcessFrame(cvImg_from_mec,
                                                       mark_centers );
    if(stat_bool != true)
    {
        std::cout<<"ERRs from ProcessFrame: "<<stat_bool<<std::endl;
        return stat_bool;
    }

    // find 2D center to 3D Center
    for (const cv::Point2f& p_2d : mark_centers)
    {
        Pixel q;
        q.x = p_2d.x;
        q.y = p_2d.y;
        GrayImage marked = textureGray;
        drawCross(marked, q);
        const std::string markedFile = prefix.str() + "_right_query_" +
                                       std::to_string(static_cast<int>(std::round(q.x))) + "_" +
                                       std::to_string(static_cast<int>(std::round(q.y))) +
                                       ".bmp";
        // saveGrayBmp(marked, markedFile);
        // std::cout << "Saved marked right/texture query image: " << markedFile << "\n";

  /*    eye::PointXYZBGR nearest;
        if (queryPointNearest(texturedPointCloud, q.x, q.y, nearest)) 
        {
            std::cout << "Right/texture pixel nearest (" << std::round(q.x) << ", " << std::round(q.y)
                      << ") -> XYZ(mm) = " << nearest.x << ", " << nearest.y << ", "
                      << nearest.z << "\n";
        } 
        else
        {
            std::cout << "Right/texture pixel nearest (" << std::round(q.x) << ", " << std::round(q.y)
                      << ") has no valid direct 3D point.\n";

            Pixel fallbackPixel;
            eye::PointXYZBGR fallbackPoint;
            double pixelDistance = 0.0;
            if (findNearestValidPixelAround(texturedPointCloud, q.x, q.y, options.searchRadius,
                                            fallbackPixel, fallbackPoint, pixelDistance)) {
                std::cout << "Nearest valid point within radius " << options.searchRadius
                          << " px is at right/texture pixel (" << fallbackPixel.x << ", "
                          << fallbackPixel.y << "), pixel distance = " << pixelDistance
                          << ", XYZ(mm) = " << fallbackPoint.x << ", " << fallbackPoint.y
                          << ", " << fallbackPoint.z << "\n";
            } 
            else
            {
                std::cout << "WARNING: No valid 3D point found within search radius "
                          << options.searchRadius << " px around right/texture pixel ("
                          << q.x << ", " << q.y << ").\n";
            }
        }*/

        eye::PointXYZBGR interpolated;
        if (queryPointInterpolated(texturedPointCloud, q.x, q.y, interpolated,
                                   options.searchRadius))
        {
            std::cout << "Right/texture pixel interpolated (" << q.x << ", " << q.y
                      << ") -> XYZ(mm) = " << interpolated.x << ", " << interpolated.y
                      << ", " << interpolated.z << "\n";
            mark_centers_3D.push_back(cv::Point3f(interpolated.x, interpolated.y, interpolated.z));
        } 
        else 
        {
            std::cout << "WARNING: Right/texture pixel (" << q.x << ", " << q.y
                      << ") has no valid 3D points for interpolation within search radius "
                      << options.searchRadius << " px.\n";
        }
    }

    // Hash Map searching
    float cosTolerance = 0.015f;
    float minPercent = 0.5f;
    FastGeoHash Geo_Hash(650.0f, 30.0f);   // Max dist between markers, Min dist for feature calculation and query
    int status = Geo_Hash.read_template_pnts("D:/3 Data/4 Track_Match/template-3D-ALL-Shift-Cut-Cut.txt");
    if (status != 0)
    {
        return status;
    }
    Geo_Hash.build();

	// pose is Rt_global
    status = Geo_Hash.Get_Track_Pose(mark_centers_3D,
                                     cosTolerance,
                                     minPercent);
    if (status != 0)
    {
        return status;
    }







    camera.disconnect();
    std::cout << "Disconnected from the camera successfully.\n";
    return 0;
}
