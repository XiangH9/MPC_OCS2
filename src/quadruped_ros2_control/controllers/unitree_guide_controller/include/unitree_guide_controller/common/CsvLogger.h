#ifndef UNITREE_GUIDE_CONTROLLER_COMMON_CSVLOGGER_H
#define UNITREE_GUIDE_CONTROLLER_COMMON_CSVLOGGER_H

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvLogger {
public:
    CsvLogger() = default;

    CsvLogger(const std::string& file_path, const std::vector<std::string>& header) {
        open(file_path, header);
    }

    void open(const std::string& file_path, const std::vector<std::string>& header) {
        if (stream_.is_open()) {
            return;
        }

        const std::filesystem::path path(file_path);
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }

        const bool write_header = !std::filesystem::exists(path);
        stream_.open(file_path, std::ios::out | std::ios::app);
        if (!stream_.is_open()) {
            return;
        }

        if (write_header) {
            for (std::size_t i = 0; i < header.size(); ++i) {
                stream_ << header[i];
                if (i + 1 < header.size()) {
                    stream_ << ",";
                }
            }
            stream_ << "\n";
            stream_.flush();
        }
    }

    template <typename... Args>
    void writeRow(const Args&... args) {
        if (!stream_.is_open()) {
            return;
        }

        std::ostringstream oss;
        writeValues(oss, args...);
        oss << "\n";
        stream_ << oss.str();
    }

    void flush() {
        if (stream_.is_open()) {
            stream_.flush();
        }
    }

    bool isOpen() const {
        return stream_.is_open();
    }

private:
    void writeValues(std::ostringstream&) {}

    template <typename T, typename... Rest>
    void writeValues(std::ostringstream& oss, const T& value, const Rest&... rest) {
        oss << value;
        if constexpr (sizeof...(rest) > 0) {
            oss << ",";
            writeValues(oss, rest...);
        }
    }

    std::ofstream stream_;
};

#endif  // UNITREE_GUIDE_CONTROLLER_COMMON_CSVLOGGER_H
