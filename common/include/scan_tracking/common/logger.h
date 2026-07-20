#pragma once

#include <QtCore/QString>
#include <QtCore/QtMessageHandler>

#include <cstdio>
#include <mutex>
#include <string>

namespace scan_tracking::common {

// 每次进程启动新建一个 txt：logs/scan_tracking_yyyy-MM-dd_HHmmss_zzz.txt；
// 同一次运行始终写入该文件，不按自然日切换。
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

    static const char* getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    std::string log_dir_;
    std::string log_file_path_;
    FILE* log_file_ = nullptr;
    std::mutex mutex_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
};

}  // namespace scan_tracking::common
