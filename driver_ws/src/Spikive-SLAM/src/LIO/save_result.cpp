#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr int kSchemaVersion = 1;
const char* kStateIdle = "idle";
const char* kStateRecording = "recording";

ros::Publisher g_status_pub;
rosbag::Bag g_bag;
std::mutex g_bag_mutex;

std::string g_drone_id{"2"};
std::string g_save_dir;
std::string g_imu_topic{"/livox/imu/"};
std::string g_visual_odom_topic;

bool g_recording{false};
std::uint64_t g_message_count{0};
std::string g_active_path;
std::string g_final_path;

void skipWhitespace(const std::string& text, size_t& index) {
    while (index < text.size() &&
           std::isspace(static_cast<unsigned char>(text[index])) != 0) {
        ++index;
    }
}

bool parseJsonString(const std::string& text, size_t& index, std::string& output,
                     std::string& error) {
    if (index >= text.size() || text[index] != '"') {
        error = "expected string";
        return false;
    }
    ++index;
    output.clear();
    while (index < text.size()) {
        const char c = text[index++];
        if (c == '"') {
            return true;
        }
        if (c == '\\') {
            if (index >= text.size()) {
                error = "unterminated escape sequence";
                return false;
            }
            const char esc = text[index++];
            switch (esc) {
                case '"':
                case '\\':
                case '/':
                    output.push_back(esc);
                    break;
                case 'b':
                    output.push_back('\b');
                    break;
                case 'f':
                    output.push_back('\f');
                    break;
                case 'n':
                    output.push_back('\n');
                    break;
                case 'r':
                    output.push_back('\r');
                    break;
                case 't':
                    output.push_back('\t');
                    break;
                case 'u':
                    if (index + 4 > text.size()) {
                        error = "invalid unicode escape";
                        return false;
                    }
                    for (size_t i = 0; i < 4; ++i) {
                        if (std::isxdigit(static_cast<unsigned char>(text[index + i])) == 0) {
                            error = "invalid unicode escape";
                            return false;
                        }
                    }
                    // Commands only need ASCII request IDs; keep unicode escapes valid but compact.
                    output.push_back('?');
                    index += 4;
                    break;
                default:
                    error = "invalid escape sequence";
                    return false;
            }
            continue;
        }
        if (static_cast<unsigned char>(c) < 0x20) {
            error = "control character in string";
            return false;
        }
        output.push_back(c);
    }
    error = "unterminated string";
    return false;
}

bool skipJsonValue(const std::string& text, size_t& index, std::string& error);

bool skipLiteral(const std::string& text, size_t& index, const std::string& literal,
                 std::string& error) {
    if (text.compare(index, literal.size(), literal) != 0) {
        error = "invalid literal";
        return false;
    }
    index += literal.size();
    return true;
}

bool skipJsonNumber(const std::string& text, size_t& index, std::string& token,
                    std::string& error) {
    const size_t begin = index;
    if (index < text.size() && text[index] == '-') {
        ++index;
    }
    if (index >= text.size() || std::isdigit(static_cast<unsigned char>(text[index])) == 0) {
        error = "invalid number";
        return false;
    }
    if (text[index] == '0') {
        ++index;
    } else {
        while (index < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[index])) != 0) {
            ++index;
        }
    }
    if (index < text.size() && text[index] == '.') {
        ++index;
        if (index >= text.size() ||
            std::isdigit(static_cast<unsigned char>(text[index])) == 0) {
            error = "invalid fraction";
            return false;
        }
        while (index < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[index])) != 0) {
            ++index;
        }
    }
    if (index < text.size() && (text[index] == 'e' || text[index] == 'E')) {
        ++index;
        if (index < text.size() && (text[index] == '+' || text[index] == '-')) {
            ++index;
        }
        if (index >= text.size() ||
            std::isdigit(static_cast<unsigned char>(text[index])) == 0) {
            error = "invalid exponent";
            return false;
        }
        while (index < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[index])) != 0) {
            ++index;
        }
    }
    token = text.substr(begin, index - begin);
    return true;
}

bool skipJsonObject(const std::string& text, size_t& index, std::string& error) {
    if (index >= text.size() || text[index] != '{') {
        error = "expected object";
        return false;
    }
    ++index;
    skipWhitespace(text, index);
    if (index < text.size() && text[index] == '}') {
        ++index;
        return true;
    }
    while (index < text.size()) {
        std::string key;
        if (!parseJsonString(text, index, key, error)) {
            return false;
        }
        skipWhitespace(text, index);
        if (index >= text.size() || text[index] != ':') {
            error = "expected ':'";
            return false;
        }
        ++index;
        if (!skipJsonValue(text, index, error)) {
            return false;
        }
        skipWhitespace(text, index);
        if (index < text.size() && text[index] == '}') {
            ++index;
            return true;
        }
        if (index >= text.size() || text[index] != ',') {
            error = "expected ',' or '}'";
            return false;
        }
        ++index;
        skipWhitespace(text, index);
    }
    error = "unterminated object";
    return false;
}

bool skipJsonArray(const std::string& text, size_t& index, std::string& error) {
    if (index >= text.size() || text[index] != '[') {
        error = "expected array";
        return false;
    }
    ++index;
    skipWhitespace(text, index);
    if (index < text.size() && text[index] == ']') {
        ++index;
        return true;
    }
    while (index < text.size()) {
        if (!skipJsonValue(text, index, error)) {
            return false;
        }
        skipWhitespace(text, index);
        if (index < text.size() && text[index] == ']') {
            ++index;
            return true;
        }
        if (index >= text.size() || text[index] != ',') {
            error = "expected ',' or ']'";
            return false;
        }
        ++index;
        skipWhitespace(text, index);
    }
    error = "unterminated array";
    return false;
}

bool skipJsonValue(const std::string& text, size_t& index, std::string& error) {
    skipWhitespace(text, index);
    if (index >= text.size()) {
        error = "expected value";
        return false;
    }
    if (text[index] == '"') {
        std::string ignored;
        return parseJsonString(text, index, ignored, error);
    }
    if (text[index] == '{') {
        return skipJsonObject(text, index, error);
    }
    if (text[index] == '[') {
        return skipJsonArray(text, index, error);
    }
    if (text[index] == 't') {
        return skipLiteral(text, index, "true", error);
    }
    if (text[index] == 'f') {
        return skipLiteral(text, index, "false", error);
    }
    if (text[index] == 'n') {
        return skipLiteral(text, index, "null", error);
    }
    std::string ignored_number;
    return skipJsonNumber(text, index, ignored_number, error);
}

bool parseCommandPayload(const std::string& payload, std::string& request_id,
                         std::string& error) {
    size_t index = 0;
    int schema_version = -1;
    request_id.clear();

    skipWhitespace(payload, index);
    if (index >= payload.size() || payload[index] != '{') {
        error = "command must be a JSON object";
        return false;
    }
    ++index;
    skipWhitespace(payload, index);

    if (index < payload.size() && payload[index] == '}') {
        error = "command object is empty";
        return false;
    }

    while (index < payload.size()) {
        std::string key;
        if (!parseJsonString(payload, index, key, error)) {
            return false;
        }
        skipWhitespace(payload, index);
        if (index >= payload.size() || payload[index] != ':') {
            error = "expected ':' after key";
            return false;
        }
        ++index;
        skipWhitespace(payload, index);

        if (key == "schema_version") {
            std::string number;
            if (!skipJsonNumber(payload, index, number, error)) {
                return false;
            }
            if (number != "1") {
                error = "schema_version must be 1";
                return false;
            }
            schema_version = 1;
        } else if (key == "request_id") {
            if (!parseJsonString(payload, index, request_id, error)) {
                return false;
            }
            if (request_id.empty()) {
                error = "request_id is required";
                return false;
            }
        } else if (!skipJsonValue(payload, index, error)) {
            return false;
        }

        skipWhitespace(payload, index);
        if (index < payload.size() && payload[index] == '}') {
            ++index;
            skipWhitespace(payload, index);
            if (index != payload.size()) {
                error = "trailing characters after JSON object";
                return false;
            }
            if (schema_version != kSchemaVersion) {
                error = "schema_version is required";
                return false;
            }
            if (request_id.empty()) {
                error = "request_id is required";
                return false;
            }
            return true;
        }
        if (index >= payload.size() || payload[index] != ',') {
            error = "expected ',' or '}'";
            return false;
        }
        ++index;
        skipWhitespace(payload, index);
    }

    error = "unterminated JSON object";
    return false;
}

std::string jsonEscape(const std::string& input) {
    std::ostringstream out;
    for (const char c : input) {
        switch (c) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\b':
                out << "\\b";
                break;
            case '\f':
                out << "\\f";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out << "?";
                } else {
                    out << c;
                }
                break;
        }
    }
    return out.str();
}

std::string defaultSaveDir() {
    const char* home = std::getenv("HOME");
    if (home == nullptr || std::string(home).empty()) {
        return "Spikive_save";
    }
    return std::string(home) + "/Spikive_save";
}

std::string expandUserPath(const std::string& path) {
    if (path == "~") {
        const char* home = std::getenv("HOME");
        return home == nullptr ? path : std::string(home);
    }
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        const char* home = std::getenv("HOME");
        return home == nullptr ? path : std::string(home) + path.substr(1);
    }
    return path;
}

bool isDirectory(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool ensureDirectory(const std::string& path, std::string& error) {
    if (path.empty()) {
        error = "save directory is empty";
        return false;
    }
    if (isDirectory(path)) {
        return true;
    }

    std::string current;
    size_t index = 0;
    if (path[0] == '/') {
        current = "/";
        index = 1;
    }
    while (index <= path.size()) {
        const size_t next = path.find('/', index);
        const std::string part =
            path.substr(index, next == std::string::npos ? std::string::npos : next - index);
        if (!part.empty()) {
            if (!current.empty() && current.back() != '/') {
                current += "/";
            }
            current += part;
            if (!isDirectory(current)) {
                if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                    error = "failed to create " + current + ": " + std::strerror(errno);
                    return false;
                }
                if (!isDirectory(current)) {
                    error = current + " exists but is not a directory";
                    return false;
                }
            }
        }
        if (next == std::string::npos) {
            break;
        }
        index = next + 1;
    }
    return true;
}

std::string joinPath(const std::string& dir, const std::string& file) {
    if (dir.empty() || dir.back() == '/') {
        return dir + file;
    }
    return dir + "/" + file;
}

std::string sanitizeToken(const std::string& value, size_t max_len) {
    std::string out;
    for (const char c : value) {
        if (std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_' || c == '-') {
            out.push_back(c);
            if (out.size() >= max_len) {
                break;
            }
        }
    }
    return out.empty() ? "request" : out;
}

std::string makeBagStem(const std::string& request_id) {
    std::ostringstream stem;
    stem << "drone_" << sanitizeToken(g_drone_id, 16) << "_" << ros::Time::now().toNSec()
         << "_" << sanitizeToken(request_id, 12);
    return stem.str();
}

std::string buildStatusJsonLocked(const std::string& request_id, const std::string& phase,
                                  bool ok, const std::string& error_code,
                                  const std::string& error_message) {
    std::ostringstream out;
    out << "{";
    out << "\"schema_version\":" << kSchemaVersion << ",";
    out << "\"request_id\":\"" << jsonEscape(request_id) << "\",";
    out << "\"drone_id\":\"" << jsonEscape(g_drone_id) << "\",";
    out << "\"phase\":\"" << jsonEscape(phase) << "\",";
    out << "\"ok\":" << (ok ? "true" : "false") << ",";
    out << "\"state\":\"" << (g_recording ? kStateRecording : kStateIdle) << "\",";
    out << "\"bag_path\":\"" << jsonEscape(g_final_path) << "\",";
    out << "\"active_path\":\"" << jsonEscape(g_active_path) << "\",";
    out << "\"message_count\":" << g_message_count << ",";
    out << "\"capabilities\":{";
    out << "\"can_start_record\":" << (!g_recording ? "true" : "false") << ",";
    out << "\"can_stop_record\":" << (g_recording ? "true" : "false");
    out << "},";
    if (ok) {
        out << "\"error\":null";
    } else {
        out << "\"error\":{\"code\":\"" << jsonEscape(error_code) << "\",\"message\":\""
            << jsonEscape(error_message) << "\"}";
    }
    out << "}";
    return out.str();
}

void publishStatus(const std::string& request_id, const std::string& phase, bool ok,
                   const std::string& error_code = "", const std::string& error_message = "") {
    std_msgs::String msg;
    {
        std::lock_guard<std::mutex> lock(g_bag_mutex);
        msg.data = buildStatusJsonLocked(request_id, phase, ok, error_code, error_message);
    }
    g_status_pub.publish(msg);
}

bool closeAndFinalizeLocked(std::string& error) {
    const std::string active_path = g_active_path;
    const std::string final_path = g_final_path;

    try {
        g_bag.close();
    } catch (const std::exception& exc) {
        g_recording = false;
        error = std::string("failed to close bag: ") + exc.what();
        return false;
    }

    g_recording = false;
    if (std::rename(active_path.c_str(), final_path.c_str()) != 0) {
        g_final_path.clear();
        g_active_path = active_path;
        error = "failed to finalize bag: " + std::string(std::strerror(errno));
        return false;
    }

    g_active_path.clear();
    g_final_path = final_path;
    return true;
}

void startRecordCbk(const std_msgs::String::ConstPtr& msg) {
    std::string request_id;
    std::string error;
    std::string failure_code;
    std::string failure_message;
    if (!parseCommandPayload(msg->data, request_id, error)) {
        publishStatus(request_id, "record_start", false, "invalid_command", error);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_bag_mutex);
        if (g_recording) {
            failure_code = "already_recording";
            failure_message = "bag recording is already active";
        }
    }
    if (!failure_code.empty()) {
        publishStatus(request_id, "record_start", false, failure_code, failure_message);
        return;
    }

    if (!ensureDirectory(g_save_dir, error)) {
        publishStatus(request_id, "record_start", false, "storage_error", error);
        return;
    }

    const std::string stem = makeBagStem(request_id);
    const std::string active_path = joinPath(g_save_dir, stem + ".active.bag");
    const std::string final_path = joinPath(g_save_dir, stem + ".bag");

    {
        std::lock_guard<std::mutex> lock(g_bag_mutex);
        if (g_recording) {
            failure_code = "already_recording";
            failure_message = "bag recording is already active";
        } else {
            try {
                g_bag.open(active_path, rosbag::bagmode::Write);
            } catch (const std::exception& exc) {
                g_active_path.clear();
                g_final_path.clear();
                failure_code = "open_failed";
                failure_message = exc.what();
            }
        }
        if (failure_code.empty()) {
            g_recording = true;
            g_message_count = 0;
            g_active_path = active_path;
            g_final_path = final_path;
        }
    }

    if (!failure_code.empty()) {
        publishStatus(request_id, "record_start", false, failure_code, failure_message);
        return;
    }

    ROS_INFO("save_result started bag recording: %s", active_path.c_str());
    publishStatus(request_id, "record_start", true);
}

void stopRecordCbk(const std_msgs::String::ConstPtr& msg) {
    std::string request_id;
    std::string error;
    std::string failure_code;
    std::string failure_message;
    if (!parseCommandPayload(msg->data, request_id, error)) {
        publishStatus(request_id, "record_stop", false, "invalid_command", error);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_bag_mutex);
        if (!g_recording) {
            failure_code = "not_recording";
            failure_message = "bag recording is not active";
        } else if (!closeAndFinalizeLocked(error)) {
            ROS_ERROR("save_result failed to stop bag recording: %s", error.c_str());
            failure_code = "stop_failed";
            failure_message = error;
        }
    }

    if (!failure_code.empty()) {
        publishStatus(request_id, "record_stop", false, failure_code, failure_message);
        return;
    }

    ROS_INFO("save_result finalized bag recording: %s", g_final_path.c_str());
    publishStatus(request_id, "record_stop", true);
}

template <typename MessageT>
void writeBagMessage(const std::string& topic, const ros::Time& stamp, const MessageT& msg) {
    std::lock_guard<std::mutex> lock(g_bag_mutex);
    if (!g_recording) {
        return;
    }
    try {
        g_bag.write(topic, stamp, msg);
        ++g_message_count;
    } catch (const std::exception& exc) {
        ROS_ERROR_THROTTLE(2.0, "save_result failed to write %s: %s", topic.c_str(), exc.what());
    }
}

void cloudBodyCbk(const sensor_msgs::PointCloud2ConstPtr& msg) {
    writeBagMessage("/cloud_registered_body", msg->header.stamp, *msg);
}

void odomTransCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    writeBagMessage("/Odometry_trans", msg->header.stamp, *msg);
}

void visualOdomCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    writeBagMessage(g_visual_odom_topic, msg->header.stamp, *msg);
}

void imuCbk(const sensor_msgs::Imu::ConstPtr& msg) {
    writeBagMessage(g_imu_topic, msg->header.stamp, *msg);
}

void shutdownCbk() {
    std::string error;
    bool finalized = false;
    std::string final_path;
    {
        std::lock_guard<std::mutex> lock(g_bag_mutex);
        if (!g_recording) {
            return;
        }
        finalized = closeAndFinalizeLocked(error);
        final_path = g_final_path;
    }
    if (finalized) {
        ROS_INFO("save_result finalized active bag during shutdown: %s", final_path.c_str());
    } else {
        ROS_ERROR("save_result kept active bag during shutdown: %s", error.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_result");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    nh.param<std::string>("common/kDroneId", g_drone_id, g_drone_id);
    nh.param<std::string>("kDroneId", g_drone_id, g_drone_id);
    private_nh.param<std::string>("kDroneId", g_drone_id, g_drone_id);
    nh.param<std::string>("drone_id", g_drone_id, g_drone_id);
    private_nh.param<std::string>("drone_id", g_drone_id, g_drone_id);
    nh.param<std::string>("common/imu_topic", g_imu_topic, g_imu_topic);

    const std::string default_visual_odom = "/drone_" + g_drone_id + "_visual_slam/odom";
    private_nh.param<std::string>("visual_odom_topic", g_visual_odom_topic, default_visual_odom);
    nh.param<std::string>("bag_record/visual_odom_topic", g_visual_odom_topic,
                          g_visual_odom_topic);

    std::string save_dir_param;
    private_nh.param<std::string>("save_dir", save_dir_param, defaultSaveDir());
    nh.param<std::string>("bag_record/save_dir", save_dir_param, save_dir_param);
    g_save_dir = expandUserPath(save_dir_param);

    const std::string start_topic = "/drone_" + g_drone_id + "_bag_record_start";
    const std::string stop_topic = "/drone_" + g_drone_id + "_bag_record_stop";
    const std::string status_topic = "/drone_" + g_drone_id + "_bag_record_status";

    g_status_pub = nh.advertise<std_msgs::String>(status_topic, 1, true);
    ros::Subscriber sub_start = nh.subscribe(start_topic, 10, startRecordCbk);
    ros::Subscriber sub_stop = nh.subscribe(stop_topic, 10, stopRecordCbk);
    ros::Subscriber sub_cloud_body = nh.subscribe("/cloud_registered_body", 1000, cloudBodyCbk);
    ros::Subscriber sub_odom_trans = nh.subscribe("/Odometry_trans", 1000, odomTransCbk);
    ros::Subscriber sub_visual_odom = nh.subscribe(g_visual_odom_topic, 1000, visualOdomCbk);
    ros::Subscriber sub_imu = nh.subscribe(g_imu_topic, 1000, imuCbk);

    publishStatus("", "status", true);

    ROS_INFO("save_result bag recorder ready: start=%s stop=%s status=%s save_dir=%s",
             start_topic.c_str(), stop_topic.c_str(), status_topic.c_str(), g_save_dir.c_str());
    ROS_INFO("save_result recording topics: /cloud_registered_body, /Odometry_trans, %s, %s",
             g_visual_odom_topic.c_str(), g_imu_topic.c_str());

    ros::spin();
    shutdownCbk();
    return 0;
}

/*
Legacy save_result behavior is intentionally disabled for the Select Object bag recorder.
The previous node mixed ENU/RTK initialization, pub_pcl_map accumulation, and startup-time bag
recording controlled by pcd_save/save_result_en. That code is kept here as a comment so the old
logic is recoverable, but none of it participates in the current frontend-driven rosbag flow.

Original runtime surfaces that are no longer active in this node:

ros::Publisher pub_trans_odom, pub_enu_rtk, pub_pcl_map;
std::string kSavePath, kSaveName;
rosbag::Bag scan_bag;
std::ofstream pose_file;
OdomStructd diff_odom, lio_odom, rtk_odom, trans_odom, origin_rot_odom;
std::string curr_signal;
bool kSaveResult, kEnuResults;
bool initial_pose_init{false};

void lioOdomCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    lio_odom.T = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);
    lio_odom.Q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // if (kSaveResult) {
    // pose_file << std::fixed << std::setprecision(20) << msg->header.stamp << " "
    //           << lio_odom.T.x() << " " << lio_odom.T.y() << " " << lio_odom.T.z() << " "
    //           << lio_odom.Q.x() << " " << lio_odom.Q.y() << " " << lio_odom.Q.z() << " "
    //           << lio_odom.Q.w() << std::endl;
    // scan_bag.write("/Odometry", msg->header.stamp, *msg);
    // }

    if (initial_pose_init) {
        if (kEnuResults) {
            trans_odom.setFromSE3(origin_rot_odom.M() * diff_odom.M() * lio_odom.M());
        } else {
            trans_odom.setFromSE3(diff_odom.M() * lio_odom.M());
        }
        nav_msgs::Odometry odom_msg;
        odom_msg.header = msg->header;
        odom_msg.child_frame_id = curr_signal;
        setPoseStamp(odom_msg.pose, trans_odom.T, trans_odom.Q);
        pub_trans_odom.publish(odom_msg);
        if (kSaveResult) {
            if (kEnuResults)
                scan_bag.write("/Odometry_enu", odom_msg.header.stamp, odom_msg);
            else
                scan_bag.write("/Odometry_trans", odom_msg.header.stamp, odom_msg);
        }
    }
}

void udPclCbk(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (kSaveResult)
        scan_bag.write("/cloud_registered_body", msg->header.stamp, *msg);
}

void initPoseCbk(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    if (!initial_pose_init) {
        diff_odom.T = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
        diff_odom.Q =
            Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        initial_pose_init = true;
    }
}

void rtkCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    rtk_odom.T = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);
    rtk_odom.Q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    rtk_odom.setFromSE3(origin_rot_odom.M() * rtk_odom.M());
    nav_msgs::Odometry rtk_msg;
    rtk_msg = *msg;
    setPoseStamp(rtk_msg.pose, rtk_odom.T, rtk_odom.Q);
    pub_enu_rtk.publish(rtk_msg);

    if (kSaveResult) {
        if (kEnuResults)
            scan_bag.write("/rtk_odom_enu", rtk_msg.header.stamp, rtk_msg);
        else
            scan_bag.write("/rtk_odom", msg->header.stamp, *msg);
    }
}

pcl::PointCloud<pcl::PointXYZI> pcl_map;
void pclCbk(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI> pcl;
    pcl::fromROSMsg(*msg, pcl);
    pcl_map += pcl;

    static int iii = 0;
    if (iii++ % 10 == 0) {
        down_sampling_voxel(pcl_map, 0.4);
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(pcl_map, laserCloudmsg);
        laserCloudmsg.header = msg->header;
        pub_pcl_map.publish(laserCloudmsg);
    }
}

int legacyMain(int argc, char** argv) {
    ros::init(argc, argv, "save_result");
    ros::NodeHandle nh;

    nh.param<bool>("pcd_save/save_result_en", kSaveResult, false);
    nh.param<bool>("pcd_save/enu_results", kEnuResults, false);
    nh.param<std::string>("pcd_save/save_path", kSavePath, std::string(ROOT_DIR) + "Lsdc_Repub");
    nh.param<std::string>("pcd_save/save_name", kSaveName, "lio");
    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 100000, lioOdomCbk);
    ros::Subscriber sub_scan = nh.subscribe("/cloud_registered_body", 100000, udPclCbk);
    ros::Subscriber sub_cloud = nh.subscribe("/cloud_registered", 100000, pclCbk);
    ros::Subscriber sub_initial_pose = nh.subscribe("/initial_pose", 100000, initPoseCbk);
    ros::Subscriber sub_rtk = nh.subscribe("/rtk_odom", 100000, rtkCbk);
    pub_trans_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_enu", 100000);
    pub_enu_rtk = nh.advertise<nav_msgs::Odometry>("/rtk_odom_enu", 100000);
    pub_pcl_map = nh.advertise<sensor_msgs::PointCloud2>("/pcl_map", 100000);

    std::string tmp_name = kSavePath + (kSavePath.back() == '/' ? "" : "/") + kSaveName;
    if (kSaveResult) {
        scan_bag.open(tmp_name + "_cloud.bag", rosbag::bagmode::Write);
        // pose_file.open(tmp_name + "_pose.txt");
    }
    std::cout << "[SLAM]: " << "Init save_result Success" << std::endl;

    // 初始旋转调整enu坐标系
    lsdc::LsdcGeographicLib lgl;
    lgl.setRtkParamFromCfg(nh);
    origin_rot_odom.Q = lsdc::kRot90 * lgl.origin_Q;
    origin_rot_odom.T = Eigen::Vector3d(0, 0, 0);

    curr_signal = to_string(ros::Time::now().toNSec());

    ros::spin();

    if (kSaveResult) {
        scan_bag.close();
        // pose_file.close();
    }
    return 0;
}
*/
