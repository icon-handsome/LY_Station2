#pragma once

#include <QtCore/QString>
#include <QtCore/QtMessageHandler>

#include <cstdio>
#include <mutex>
#include <string>

namespace scan_tracking::common {

// 每次进程启动新建：
//   logs/scan_tracking_yyyy-MM-dd_HHmmss_zzz.txt              （全量运行日志）
//   logs/algorithm_scan_tracking_yyyy-MM-dd_HHmmss_zzz.txt    （算法日志，同时间戳）
// 同一次运行始终写入这对文件，不按自然日切换。
class Logger {
public:
    static void initialize(const QString& log_dir = QStringLiteral("logs"));
    static void cleanup();

    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static Logger* instance();

    void setMinLevel(QtMsgType level);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    explicit Logger(const QString& log_dir);
    ~Logger();

    void openLogFile();
    void log(QtMsgType type, const QMessageLogContext& context, const QString& msg);
    static bool isAlgorithmLogCategory(const char* category);
    static void writeLineToFile(FILE* file, const std::string& line);

    static const char* getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    std::string log_dir_;
    std::string log_file_path_;
    std::string algorithm_log_file_path_;
    FILE* log_file_ = nullptr;
    FILE* algorithm_log_file_ = nullptr;
    std::mutex mutex_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
};

}  // namespace scan_tracking::common
