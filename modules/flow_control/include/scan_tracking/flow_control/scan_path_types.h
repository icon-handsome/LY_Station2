#pragma once

#include <QString>
#include <array>
#include <memory>
#include <vector>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace flow_control {

/**
 * @brief 单个扫描点位的结果
 * 
 * 包含一个点位的所有采集数据和位姿矩阵。
 * 采用流式处理策略：点云数据保存为 PLY 文件，只缓存文件路径。
 */
struct ScanPointResult {
    // 点位标识
    int pathId;                  // 所属路径 ID
    int pointIndex;              // 点位索引（从 1 开始）

    // 点云数据（流式处理：只存文件路径，不缓存点云数据）
    QString pointCloudPath;      // PLY 文件路径
    
    // 海康图像路径（落盘至 hik_mono/camera_a|camera_b/*.bmp 后仅存路径，像素在 bundle 内已释放）
    QString hikMonoPathA;
    QString hikMonoPathB;
    
    // 位姿矩阵（轻量级，可以缓存）
    std::array<float, 16> calibrationMatrix;      // 标定矩阵 T0' 或 T0''（4×4，行优先）
    std::array<float, 16> stereoTrackingMatrix;   // 双目跟踪矩阵 T_N（4×4，行优先）
    
    // 元数据
    bool needRotation;           // 是否需要转动转盘
    qint64 timestampMs;          // 采集时间戳（毫秒）
    qint64 elapsedMs;            // 采集耗时（毫秒）
    
    /**
     * @brief 默认构造函数
     */
    ScanPointResult()
        : pathId(-1)
        , pointIndex(-1)
        , needRotation(false)
        , timestampMs(0)
        , elapsedMs(0)
    {
        // 初始化标定矩阵为单位矩阵
        calibrationMatrix.fill(0.0f);
        calibrationMatrix[0] = 1.0f;
        calibrationMatrix[5] = 1.0f;
        calibrationMatrix[10] = 1.0f;
        calibrationMatrix[15] = 1.0f;
        
        // 初始化双目跟踪矩阵为单位矩阵
        stereoTrackingMatrix.fill(0.0f);
        stereoTrackingMatrix[0] = 1.0f;
        stereoTrackingMatrix[5] = 1.0f;
        stereoTrackingMatrix[10] = 1.0f;
        stereoTrackingMatrix[15] = 1.0f;
    }
    
    /**
     * @brief 检查点位结果是否有效
     * 
     * @return 点云文件路径非空则认为有效
     */
    bool isValid() const {
        return !pointCloudPath.isEmpty();
    }
};

/**
 * @brief 单条扫描路径的结果
 * 
 * 包含一条路径的所有点位结果。
 */
struct ScanPathResult {
    int pathId;                  // 路径 ID
    std::vector<ScanPointResult> pointResults;  // 所有点位的结果
    bool success;                // 路径是否成功完成
    QString errorMessage;        // 失败时的错误信息
    qint64 totalElapsedMs;       // 路径总耗时（毫秒）
    
    /**
     * @brief 默认构造函数
     */
    ScanPathResult()
        : pathId(-1)
        , success(false)
        , totalElapsedMs(0)
    {
    }
    
    /**
     * @brief 获取路径的点位数量
     * 
     * @return 点位数量
     */
    int pointCount() const {
        return static_cast<int>(pointResults.size());
    }
    
    /**
     * @brief 检查路径结果是否有效
     * 
     * @return 成功且至少有一个点位结果
     */
    bool isValid() const {
        return success && !pointResults.empty();
    }
};

/**
 * @brief 多路径扫描的总结果
 * 
 * 包含所有路径的结果和统计信息。
 */
struct MultiPathScanResult {
    std::vector<ScanPathResult> pathResults;  // 所有路径的结果
    bool allSuccess;             // 是否所有路径都成功
    int totalPoints;             // 总点位数
    qint64 totalElapsedMs;       // 总耗时（毫秒）
    QString summaryMessage;      // 总结信息
    
    /**
     * @brief 默认构造函数
     */
    MultiPathScanResult()
        : allSuccess(false)
        , totalPoints(0)
        , totalElapsedMs(0)
    {
    }
    
    /**
     * @brief 获取路径数量
     * 
     * @return 路径数量
     */
    int pathCount() const {
        return static_cast<int>(pathResults.size());
    }
    
    /**
     * @brief 获取成功的路径数量
     * 
     * @return 成功的路径数量
     */
    int successPathCount() const {
        int count = 0;
        for (const auto& path : pathResults) {
            if (path.success) {
                ++count;
            }
        }
        return count;
    }
    
    /**
     * @brief 检查多路径扫描结果是否有效
     * 
     * @return 至少有一条路径成功
     */
    bool isValid() const {
        return successPathCount() > 0;
    }
    
    /**
     * @brief 生成总结信息
     * 
     * @return 总结字符串，如 "2/3 路径成功，14 个点位，耗时 87.6 秒"
     */
    QString generateSummary() const {
        const int successCount = successPathCount();
        const int totalCount = pathCount();
        const double elapsedSeconds = totalElapsedMs / 1000.0;
        
        return QStringLiteral("%1/%2 路径成功，%3 个点位，耗时 %4 秒")
			.arg(successCount)  
            .arg(totalCount)
            .arg(totalPoints)
            .arg(elapsedSeconds, 0, 'f', 1);
    }
};

/**
 * @brief 扫描点位的上下文信息
 * 
 * 用于在扫描过程中跟踪当前点位的状态。
 */
struct ScanPointContext {
    int pathId;                  // 当前路径 ID
    int pathIndex;               // 当前路径索引（从 0 开始）
    int pointIndex;              // 当前点位索引（从 1 开始）
    bool needRotation;           // 是否需要转动
    qint64 startTimestampMs;     // 开始时间戳
    
    /**
     * @brief 默认构造函数
     */
    ScanPointContext()
        : pathId(-1)    
        , pathIndex(-1)
        , pointIndex(-1)
        , needRotation(false)
        , startTimestampMs(0)
    {
    }
    
    /**
     * @brief 检查上下文是否有效
     * 
     * @return 路径 ID 和点位索引都有效
     */
    bool isValid() const {
        return pathId >= 0 && pointIndex > 0;
    }
    
    /**
     * @brief 重置上下文
     */
    void reset() {
        pathId = -1;
        pathIndex = -1;
        pointIndex = -1;
        needRotation = false;
        startTimestampMs = 0;
    }
};

}  // namespace flow_control
}  // namespace scan_tracking
