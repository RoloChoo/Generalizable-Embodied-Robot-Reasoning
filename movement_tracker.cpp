#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <optional>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/options/find.hpp>

#ifdef _WIN32
#include <windows.h>
#include <wininet.h>
#pragma comment(lib, "wininet.lib")
#else
#include <curl/curl.h>
#endif

using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;

class DarwinOpController {
private:
    std::string robot_ip;
    int robot_port;
    std::string last_action;
    
    std::map<std::string, std::string> action_mapping = {
        {"forward", "move_forward"},
        {"backward", "move_backward"},
        {"left", "turn_left"},
        {"right", "turn_right"}
    };

public:
    DarwinOpController(const std::string& ip, int port = 8080) 
        : robot_ip(ip), robot_port(port) {
        std::cout << "[ROBOT] Darwin-OP Target: http://" << robot_ip << ":" << robot_port << std::endl;
    }
    
    bool send_http_command(const std::string& command) {
#ifdef _WIN32
        HINTERNET hInternet = InternetOpenA("MovementTracker", INTERNET_OPEN_TYPE_DIRECT, NULL, NULL, 0);
        if (!hInternet) {
            std::cout << "[HTTP] Failed to initialize internet" << std::endl;
            return false;
        }
        
        std::string url = "http://" + robot_ip + ":" + std::to_string(robot_port) + "/?command=" + command;
        
        HINTERNET hConnect = InternetOpenUrlA(hInternet, url.c_str(), NULL, 0, 
            INTERNET_FLAG_RELOAD | INTERNET_FLAG_NO_CACHE_WRITE, 0);
        
        bool success = false;
        if (hConnect) {
            char buffer[1024];
            DWORD bytesRead;
            if (InternetReadFile(hConnect, buffer, sizeof(buffer) - 1, &bytesRead)) {
                buffer[bytesRead] = '\0';
                if (strstr(buffer, "200 OK") || bytesRead > 0) {
                    std::cout << "[HTTP] Command sent successfully: " << command << std::endl;
                    success = true;
                } else {
                    std::cout << "[HTTP] Response error: " << command << std::endl;
                }
            }
            InternetCloseHandle(hConnect);
        } else {
            std::cout << "[HTTP] Connection failed: " << command << std::endl;
        }
        
        InternetCloseHandle(hInternet);
        return success;
        
#else
        CURL* curl = curl_easy_init();
        if (!curl) return false;
        
        std::string url = "http://" + robot_ip + ":" + std::to_string(robot_port) + "/?command=" + command;
        std::string response;
        
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, 
            [](void* contents, size_t size, size_t nmemb, std::string* s) -> size_t {
                s->append((char*)contents, size * nmemb);
                return size * nmemb;
            });
        
        CURLcode res = curl_easy_perform(curl);
        bool success = (res == CURLE_OK);
        
        if (success) {
            std::cout << "[HTTP] Command sent successfully: " << command << std::endl;
        } else {
            std::cout << "[HTTP] Command send failed: " << command << std::endl;
        }
        
        curl_easy_cleanup(curl);
        return success;
#endif
    }
    
    bool execute_action(const std::string& mongo_action) {
        if (mongo_action == last_action) {
            return true;
        }
        
        auto it = action_mapping.find(mongo_action);
        if (it == action_mapping.end()) {
            std::cout << "[ROBOT] Unknown action: " << mongo_action << std::endl;
            return false;
        }
        
        bool success = send_http_command(it->second);
        if (success) {
            last_action = mongo_action;
        }
        
        return success;
    }
    
    const std::string& get_last_action() const {
        return last_action;
    }
};

class MongoDBTracker {
private:
    mongocxx::client client;
    mongocxx::database db;
    mongocxx::collection collection;

public:
    MongoDBTracker() : client{mongocxx::uri{}}, db{client["movement_tracker"]}, collection{db["movementracker"]} {
        std::cout << "[MONGO] MongoDB connection established" << std::endl;
        std::cout << "[MONGO] Target: movement_tracker.movementracker" << std::endl;
    }
    
    std::optional<bsoncxx::document::value> get_current_tracking() {
        try {
            auto opts = mongocxx::options::find{};
            opts.sort(document{} << "_id" << -1 << finalize);
            opts.limit(1);
            
            auto cursor = collection.find({}, opts);
            auto it = cursor.begin();
            
            if (it != cursor.end()) {
                return bsoncxx::document::value{*it};
            }
        } catch (const std::exception& e) {
            std::cout << "[MONGO] Failed to query data: " << e.what() << std::endl;
        }
        
        return {};
    }
};

class SimpleSync {
private:
    DarwinOpController robot;
    MongoDBTracker tracker;
    bool running;
    int last_total_actions;
    int sync_count;

public:
    SimpleSync(const std::string& robot_ip) 
        : robot(robot_ip), running(false), last_total_actions(0), sync_count(0) {
    }
    
    void run_sync_loop() {
        std::cout << "\nMongoDB -> Darwin-OP Sync Started!" << std::endl;
        std::cout << "Press Ctrl+C to stop.\n" << std::endl;
        
        running = true;
        int no_data_count = 0;
        
        while (running) {
            try {
                auto data = tracker.get_current_tracking();
                
                if (!data) {
                    no_data_count++;
                    if (no_data_count % 10 == 1) {
                        std::cout << "[SYNC] Waiting for data..." << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                
                no_data_count = 0;
                auto doc_view = data->view();
                
                int current_total = 0;
                std::string current_action = "idle";
                
                auto total_elem = doc_view["total_actions"];
                if (total_elem && total_elem.type() == bsoncxx::type::k_int32) {
                    current_total = total_elem.get_int32().value;
                }
                
                auto action_elem = doc_view["current_action"];
                if (action_elem && action_elem.type() == bsoncxx::type::k_string) {
                    current_action = std::string(action_elem.get_string().value);
                }
                
                if (current_total > last_total_actions) {
                    std::cout << "[SYNC] New action detected: " << current_action 
                              << " (total " << current_total << ")" << std::endl;
                    
                    bool success = robot.execute_action(current_action);
                    
                    if (success) {
                        last_total_actions = current_total;
                        sync_count++;
                        std::cout << "[SYNC] Sync #" << sync_count << " completed!" << std::endl;
                        
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    } else {
                        std::cout << "[SYNC] Command send failed, retrying..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                
            } catch (const std::exception& e) {
                std::cout << "[SYNC] Error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
        
        std::cout << "\nSync stopped!" << std::endl;
    }
};

int main() {
    std::cout << "MongoDB -> Darwin-OP Sync System (C++)" << std::endl;
    std::cout << std::string(50, '=') << std::endl;
    
    mongocxx::instance const inst{};
    
    std::string robot_ip;
    std::cout << "\nEnter Darwin-OP robot IP address (e.g., 192.168.1.100): ";
    std::cin >> robot_ip;
    std::cout << "Robot IP set to: " << robot_ip << std::endl;
    
    try {
        MongoDBTracker tracker;
        
        std::cout << "\nCommands:" << std::endl;
        std::cout << "  3 - Start Darwin-OP sync" << std::endl;
        std::cout << "  q - Quit" << std::endl;
        
        SimpleSync sync_manager(robot_ip);
        char choice;
        
        while (true) {
            std::cout << "\n> ";
            std::cin >> choice;
            
            if (choice == 'q' || choice == 'Q') {
                break;
            }
            else if (choice == '3') {
                sync_manager.run_sync_loop();
            }
            else {
                std::cout << "Unknown command." << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Program terminated." << std::endl;
    return 0;
}
