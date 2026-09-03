#include "casia_hand_m.h"
#include <zmq.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <cstring>
#include <atomic>

// Keep physical-hand ports aligned with dex_teleop's CASIA sim2real channels.
constexpr int CASIA_REAL_LEFT_PORT = 5555;
constexpr int CASIA_REAL_RIGHT_PORT = 5556;

/**
 * Simple JSON parser for ZMQ message format:
 * {
 *   "timestamp": 1234567890.123,
 *   "qpos": [0.1, 0.2, 0.3, ...],
 *   "joint_names": ["joint1", "joint2", "joint3", ...]
 * }
 */
class SimpleJsonParser
{
public:
    static bool parseMessage(const std::string &json_str,
                            double &timestamp,
                            std::vector<double> &qpos,
                            std::vector<std::string> &joint_names)
    {
        try
        {
            // Extract timestamp
            size_t ts_pos = json_str.find("\"timestamp\"");
            if (ts_pos == std::string::npos)
                return false;
            ts_pos = json_str.find(":", ts_pos);
            if (ts_pos == std::string::npos)
                return false;
            timestamp = std::stod(json_str.substr(ts_pos + 1));

            // Extract qpos array
            size_t qpos_pos = json_str.find("\"qpos\"");
            if (qpos_pos == std::string::npos)
                return false;
            qpos_pos = json_str.find("[", qpos_pos);
            size_t qpos_end = json_str.find("]", qpos_pos);
            if (qpos_end == std::string::npos)
                return false;

            std::string qpos_str = json_str.substr(qpos_pos + 1, qpos_end - qpos_pos - 1);
            qpos = parseArray(qpos_str);

            // Extract joint_names array (optional)
            size_t names_pos = json_str.find("\"joint_names\"");
            if (names_pos != std::string::npos)
            {
                names_pos = json_str.find("[", names_pos);
                size_t names_end = json_str.find("]", names_pos);
                if (names_end != std::string::npos)
                {
                    std::string names_str = json_str.substr(names_pos + 1, names_end - names_pos - 1);
                    joint_names = parseStringArray(names_str);
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Parser] Error: " << e.what() << std::endl;
            return false;
        }
    }

private:
    static std::vector<double> parseArray(const std::string &str)
    {
        std::vector<double> result;
        std::istringstream iss(str);
        std::string token;

        while (std::getline(iss, token, ','))
        {
            // Trim whitespace
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);

            if (!token.empty())
            {
                try
                {
                    result.push_back(std::stod(token));
                }
                catch (...)
                {
                    // Skip invalid numbers
                }
            }
        }
        return result;
    }

    static std::vector<std::string> parseStringArray(const std::string &str)
    {
        std::vector<std::string> result;
        size_t start = 0;

        while (true)
        {
            size_t quote_start = str.find('"', start);
            if (quote_start == std::string::npos)
                break;

            size_t quote_end = str.find('"', quote_start + 1);
            if (quote_end == std::string::npos)
                break;

            result.push_back(str.substr(quote_start + 1, quote_end - quote_start - 1));
            start = quote_end + 1;
        }

        return result;
    }
};

/**
 * Hand Teleoperator Class
 * Controls the dual hand system via ZMQ messages
 */
class HandTeleoperator
{
private:
    std::shared_ptr<casia::HandM::CasiaHandM> hand_;
    void *zmq_context_;
    void *zmq_socket_left_;
    void *zmq_socket_right_;
    std::atomic<bool> running_;
    std::thread left_subscriber_thread_;
    std::thread right_subscriber_thread_;
    std::thread state_thread_;
    std::thread command_thread_;
    std::mutex state_mutex_;
    casia::HandM::handm_target_set_t current_target_;
    casia::HandM::handm_state_get_t current_state_;
    std::string zmq_endpoint_left_;
    std::string zmq_endpoint_right_;

    // Hand configuration parameters
    float default_power_left_[HANDM_DOF_NUM];
    float default_power_right_[HANDM_DOF_NUM];
    float default_speed_[2];

public:
    HandTeleoperator(int left_hand_id, int right_hand_id, int baudrate,
                                        const std::string &port_name,
                                        const std::string &zmq_endpoint_left,
                                        const std::string &zmq_endpoint_right)
                : zmq_endpoint_left_(zmq_endpoint_left),
                    zmq_endpoint_right_(zmq_endpoint_right),
                    running_(false)
    {
        hand_ = std::make_shared<casia::HandM::CasiaHandM>(left_hand_id, right_hand_id,
                                                           baudrate, port_name);

        // Initialize ZMQ
        zmq_context_ = zmq_ctx_new();
                zmq_socket_left_ = zmq_socket(zmq_context_, ZMQ_SUB);
                zmq_socket_right_ = zmq_socket(zmq_context_, ZMQ_SUB);

                // Subscribe to all messages on both channels
                zmq_setsockopt(zmq_socket_left_, ZMQ_SUBSCRIBE, "", 0);
                zmq_setsockopt(zmq_socket_right_, ZMQ_SUBSCRIBE, "", 0);

        // Set socket options
        int timeout = 1000; // 1 second timeout
                zmq_setsockopt(zmq_socket_left_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
                zmq_setsockopt(zmq_socket_right_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

        // Initialize target
        memset(&current_target_, 0, sizeof(current_target_));
        memset(&current_state_, 0, sizeof(current_state_));

        // Set default power and speed values
        // Left hand
        float power_left[HANDM_DOF_NUM] = {15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 3.0, 3.0, 3.0, 3.0};
        memcpy(default_power_left_, power_left, sizeof(default_power_left_));

        // Right hand
        float power_right[HANDM_DOF_NUM] = {15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 3.0, 3.0, 3.0, 3.0};
        memcpy(default_power_right_, power_right, sizeof(default_power_right_));

        // Speed for both hands
        float speed[2] = {0.6, 0.6};
        memcpy(default_speed_, speed, sizeof(default_speed_));
    }

    ~HandTeleoperator()
    {
        stop();
        if (zmq_socket_left_)
            zmq_close(zmq_socket_left_);
        if (zmq_socket_right_)
            zmq_close(zmq_socket_right_);
        if (zmq_context_)
            zmq_ctx_destroy(zmq_context_);
    }

    bool initialize()
    {
        if (!hand_->init())
        {
            std::cerr << "[ERROR] Failed to initialize hand" << std::endl;
            return false;
        }

        std::cout << "[INFO] Hand initialized successfully" << std::endl;

        // Connect to left-hand ZMQ publisher
        int connect_result_left = zmq_connect(zmq_socket_left_, zmq_endpoint_left_.c_str());
        if (connect_result_left != 0)
        {
            std::cerr << "[ERROR] Failed to connect to LEFT ZMQ endpoint: "
                      << zmq_endpoint_left_ << std::endl;
            return false;
        }

        // Connect to right-hand ZMQ publisher
        int connect_result_right = zmq_connect(zmq_socket_right_, zmq_endpoint_right_.c_str());
        if (connect_result_right != 0)
        {
            std::cerr << "[ERROR] Failed to connect to RIGHT ZMQ endpoint: "
                      << zmq_endpoint_right_ << std::endl;
            return false;
        }

        std::cout << "[INFO] Connected LEFT ZMQ endpoint:  " << zmq_endpoint_left_ << std::endl;
        std::cout << "[INFO] Connected RIGHT ZMQ endpoint: " << zmq_endpoint_right_ << std::endl;

        running_ = true;
        return true;
    }

    void start()
    {
        // Subscriber threads - receive commands from dedicated ZMQ ports
        left_subscriber_thread_ = std::thread([this]()
                                              { zmq_subscriber_loop(true); });

        right_subscriber_thread_ = std::thread([this]()
                                               { zmq_subscriber_loop(false); });

        // State publisher thread - reads and displays hand state
        state_thread_ = std::thread([this]()
                                    { hand_state_loop(); });

        // Hand command thread - sends targets to hand
        command_thread_ = std::thread([this]()
                                      { hand_command_loop(); });
    }

    void stop()
    {
        running_ = false;
        join_thread(left_subscriber_thread_);
        join_thread(right_subscriber_thread_);
        join_thread(state_thread_);
        join_thread(command_thread_);

        if (hand_)
        {
            // Releasing the final SDK owner stops and joins its serial worker,
            // then closes the serial port. This runs from normal control flow,
            // never from a signal handler.
            hand_.reset();

            // OmniHand's serial reconnect path leaves a two-second quiet
            // interval between close() and the next open(). Enforce the same
            // recovery window before returning control to the shell so a
            // Jetson user cannot immediately reopen the CH341 adapter.
            std::cout << "[Main] Serial closed; waiting 2 seconds for adapter recovery..."
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

private:
    static void join_thread(std::thread &thread)
    {
        if (thread.joinable())
            thread.join();
    }

    /**
     * Main subscriber loop - receives JSON commands from ZMQ
     */
    void zmq_subscriber_loop(bool is_left_hand)
    {
        void *socket = is_left_hand ? zmq_socket_left_ : zmq_socket_right_;
        const std::string &tag = is_left_hand ? "LEFT" : "RIGHT";

        std::cout << "[ZMQ Subscriber " << tag << "] Started listening for messages..." << std::endl;

        while (running_)
        {
            // Receive message (non-blocking with timeout)
            char buffer[4096] = {0};
            int size = zmq_recv(socket, buffer, 4095, 0);

            if (size == -1)
            {
                // Timeout - no message received
                continue;
            }

            std::string json_message(buffer, size);
            std::cout << "[ZMQ " << tag << "] Received message (" << size << " bytes)" << std::endl;

            // Parse JSON message
            double timestamp;
            std::vector<double> qpos;
            std::vector<std::string> joint_names;

            if (SimpleJsonParser::parseMessage(json_message, timestamp, qpos, joint_names)
                && qpos.size() == HANDM_DOF_NUM
                && std::all_of(qpos.begin(), qpos.end(), [](double value) { return std::isfinite(value); }))
            {
                std::cout << "[ZMQ " << tag << "] Successfully parsed:" << std::endl;
                std::cout << "      Timestamp: " << std::fixed << std::setprecision(3) << timestamp << std::endl;
                std::cout << "      Joint count: " << qpos.size() << std::endl;

                // Lock and update target
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    update_hand_target(qpos, joint_names, is_left_hand);
                }
            }
            else
            {
                std::cerr << "[ERROR] Rejected " << tag
                          << " message: expected exactly 10 finite qpos values" << std::endl;
            }
        }

        std::cout << "[ZMQ Subscriber " << tag << "] Stopped" << std::endl;
    }

    /**
     * Hand command loop - sends target commands to hand
     */
    void hand_command_loop()
    {
        std::cout << "[Hand Command] Started command thread..." << std::endl;

        while (running_)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                hand_->setHandTargetoQueue(current_target_);
            }

            // 50 Hz command rate (20 ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "[Hand Command] Stopped" << std::endl;
    }

    /**
     * Hand state loop - reads and displays hand state feedback
     */
    void hand_state_loop()
    {
        std::cout << "[Hand State] Started state feedback thread..." << std::endl;

        while (running_)
        {
            if (hand_->getHandStateFromQueue(current_state_))
            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                std::cout << "\n[Hand State Update]" << std::endl;

                // Left hand state
                std::cout << "Left Hand:" << std::endl;
                for (int i = 0; i < HANDM_DOF_NUM; i++)
                {
                    std::cout << "  DOF" << (i + 1) << " - Angle: " << std::fixed
                             << std::setprecision(4) << current_state_.handm_angle[i]
                             << " rad, Power: " << current_state_.handm_power[i] << std::endl;
                }

                // Right hand state
                std::cout << "Right Hand:" << std::endl;
                for (int i = HANDM_DOF_NUM; i < 2 * HANDM_DOF_NUM; i++)
                {
                    std::cout << "  DOF" << (i - HANDM_DOF_NUM + 1) << " - Angle: "
                             << std::fixed << std::setprecision(4)
                             << current_state_.handm_angle[i] << " rad, Power: "
                             << current_state_.handm_power[i] << std::endl;
                }
            }

            // 2 Hz state feedback rate (500 ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::cout << "[Hand State] Stopped" << std::endl;
    }

    /**
     * Update hand target from received qpos
     * Maps left/right joint positions to the corresponding hand index range
     */
    void update_hand_target(const std::vector<double> &qpos,
                           const std::vector<std::string> &joint_names,
                           bool is_left_hand)
    {
        // Copy power values
        memcpy(&current_target_.handm_power[0], default_power_left_, sizeof(default_power_left_));
        memcpy(&current_target_.handm_power[HANDM_DOF_NUM], default_power_right_,
               sizeof(default_power_right_));

        // Copy speed values
        memcpy(&current_target_.handm_speed[0], default_speed_, sizeof(default_speed_));

        // Left hand writes [0, HANDM_DOF_NUM), right hand writes [HANDM_DOF_NUM, 2*HANDM_DOF_NUM)
        const size_t index_base = is_left_hand ? 0 : static_cast<size_t>(HANDM_DOF_NUM);
        const size_t num_joints = std::min(qpos.size(), static_cast<size_t>(HANDM_DOF_NUM));
        const std::string &tag = is_left_hand ? "LEFT" : "RIGHT";

        std::cout << "      Updating " << tag << " " << num_joints << " joint targets:" << std::endl;

        for (size_t i = 0; i < num_joints; i++)
        {
            const size_t target_index = index_base + i;
            current_target_.handm_angle[target_index] = static_cast<float>(qpos[i]);

            // Log joint name if available
            if (i < joint_names.size())
            {
                std::cout << "      [" << joint_names[i] << "] -> target[" << target_index
                          << "] = " << qpos[i] << " rad" << std::endl;
            }
            else
            {
                std::cout << "      [joint_" << i << "] -> target[" << target_index
                          << "] = " << qpos[i] << " rad" << std::endl;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    // Hand configuration
    int left_hand_id = 2;
    int right_hand_id = 0x20;
    int baudrate = 115200;
    std::cout << "version: 1.0.0" << std::endl;
    std::string port_name = "/dev/ttyUSB0";
    std::string zmq_endpoint_left = "tcp://localhost:" + std::to_string(CASIA_REAL_LEFT_PORT);
    std::string zmq_endpoint_right = "tcp://localhost:" + std::to_string(CASIA_REAL_RIGHT_PORT);

    // Parse command line arguments
    if (argc > 1)
    {
        zmq_endpoint_left = argv[1];
    }
    if (argc > 2)
    {
        zmq_endpoint_right = argv[2];
    }
    if (argc > 3)
    {
        port_name = argv[3];
    }
    try
    {
        if (argc > 4) left_hand_id = std::stoi(argv[4], nullptr, 0);
        if (argc > 5) right_hand_id = std::stoi(argv[5], nullptr, 0);
        if (argc > 6) baudrate = std::stoi(argv[6], nullptr, 0);
    }
    catch (const std::exception &error)
    {
        std::cerr << "[FATAL] Invalid device ID or baudrate: " << error.what() << std::endl;
        return 2;
    }

    std::cout << "\n========== Hand Teleoperator (ZMQ Subscriber) ==========" << std::endl;
    std::cout << "Left Hand ID:  " << left_hand_id << " (0x" << std::hex << left_hand_id << std::dec << ")" << std::endl;
    std::cout << "Right Hand ID: " << right_hand_id << " (0x" << std::hex << right_hand_id << std::dec << ")" << std::endl;
    std::cout << "Serial Port:   " << port_name << std::endl;
    std::cout << "Baudrate:      " << baudrate << std::endl;
    std::cout << "ZMQ Left:      " << zmq_endpoint_left << std::endl;
    std::cout << "ZMQ Right:     " << zmq_endpoint_right << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "\nUsage: ./casia_zmq_teleop [left_endpoint] [right_endpoint]"
                 " [serial_port] [left_id] [right_id] [baudrate]" << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  ./casia_zmq_teleop                    (uses all defaults)" << std::endl;
    std::cout << "  ./casia_zmq_teleop tcp://localhost:8888 tcp://localhost:8889 /dev/ttyUSB1 2 0x20 115200" << std::endl;
    std::cout << "========================================================\n" << std::endl;

    // Create and initialize teleoperator
    HandTeleoperator teleop(left_hand_id, right_hand_id, baudrate, port_name,
                            zmq_endpoint_left, zmq_endpoint_right);

    if (!teleop.initialize())
    {
        std::cerr << "[FATAL] Failed to initialize teleoperator" << std::endl;
        return -1;
    }

    // Start controller threads
    teleop.start();

    std::cout << "[Main] Teleoperator running. Type q then press Enter to exit cleanly."
              << std::endl;

    // Deliberately avoid SIGINT for the normal shutdown path: a terminal input
    // cannot interrupt an in-flight CASIA serial transaction in another
    // thread. EOF also requests an orderly shutdown.
    std::string input;
    while (std::getline(std::cin, input))
    {
        if (input == "q" || input == "quit")
            break;
    }

    std::cout << "[Main] Shutdown requested; joining workers and closing serial..."
              << std::endl;
    teleop.stop();
    std::cout << "[Main] Clean shutdown complete." << std::endl;

    return 0;
}
