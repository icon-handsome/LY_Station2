#pragma once

#include <QtCore/QString>

namespace scan_tracking::common {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultCaptureCacheRoot();

/// 配置为空时使用默认根目录，否则使用配置路径（绝对化）
QString resolveCaptureCacheRoot(const QString& configuredRoot);

/// 确保目录存在；失败返回空字符串
QString ensureDirectoryExists(const QString& directoryPath);

/// 点子目录：<root>/pointcloud
QString captureCachePointCloudDir(const QString& root);

/// 点子目录：<root>/hik_mono
QString captureCacheHikMonoDir(const QString& root);

/// 同一次分段落盘共用的时间戳：yyyyMMdd_HHmmss_zzz
QString buildCaptureTimestamp();

}  // namespace scan_tracking::common
