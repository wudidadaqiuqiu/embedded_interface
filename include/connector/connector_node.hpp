#pragma once
#include <atomic>
#include <thread>

#include "connector/connector.hpp"
#include "connector/pack_manager.hpp"

#define PRINT_FILE_AND_LINE() \
    std::cout << "File: " << __FILE__ << ", Line: " << __LINE__ << std::endl;

namespace connector {
template <ConnectorType CON_TYPE, typename MSGPackT>
class ConnectorSingleRecvNode {
   public:
    ConnectorSingleRecvNode(Connector<CON_TYPE>& con) : connector(con),
                                                        pack_manager() {
        is_end = false;
        // pub = nh.advertise<typename MSGPackT::MSGT>(topic_name, 10);
        thread = std::thread(&ConnectorSingleRecvNode<CON_TYPE, MSGPackT>::run, this);
    }
    ConnectorSingleRecvNode(const ConnectorSingleRecvNode&) = delete;

    ~ConnectorSingleRecvNode() {
        // 在这里设置成非阻塞没用
        is_end = true;
        if (thread.joinable()) {
            thread.join();
        }
    }

    void register_callback(std::function<void(const typename MSGPackT::MSGT&)> func) {
        pack_manager.register_callback(func);
    }

   private:
    Connector<CON_TYPE>& connector;
    PackManager<MSGPackT> pack_manager;
    std::thread thread;
    std::vector<uint8_t> buffer;
    uint32_t id;
    typename MSGPackT::MSGT msg;
    std::atomic<bool> is_end;

    void run() {
        while (!is_end) {
            try {
                connector.con_recv(buffer, id);
            } catch (const TimeoutException& e) {
                // std::cout << e.what() << std::endl;
                continue;
            } catch (const std::exception& e) {
                std::cout << "error: " << e.what() << std::endl;
                PRINT_FILE_AND_LINE();
                throw e;
            }
            // std::cout << "recv: " << id << std::endl;
            // auto start = std::chrono::high_resolution_clock::now();
            // std::cout << (short)(((short)buffer[2] << 8) | buffer[3]) << std::endl;
            MSGPackT::pack(msg, buffer, id);
            pack_manager.push_pack(msg);
            // pub.publish(msg);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> duration = end - start;
            // std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;
        }
    }

    // void reconnect
};

template <ConnectorType CON_TYPE, typename MSGPackT>
class ConnectorSendNode {
    using THIS = ConnectorSendNode<CON_TYPE, MSGPackT>;
    using CallbackType = void (THIS::*)(const typename MSGPackT::MSGT::ConstPtr&);  // 成员函数指针类型
   public:
    ConnectorSendNode(Connector<CON_TYPE>& con) : connector(con) {}

    void register_callback(std::function<void(const typename MSGPackT::MSGT::ConstPtr&)> callback) {
        callbacks_.register_callback(callback);
    }
    // void(T::*fp)(const boost::shared_ptr<M const>&)
    static CallbackType get_callback() {
        return &THIS::callback;
    }

   private:
    Connector<CON_TYPE>& connector;
    CallbacksContainer<typename MSGPackT::MSGT::ConstPtr> callbacks_;
    std::vector<uint8_t> buffer;
    uint32_t id_;

    void callback(const typename MSGPackT::MSGT::ConstPtr& msg) {
        MSGPackT::unpack(msg, buffer, id_);
        callbacks_.callback(msg);
        try {
            connector.con_send(buffer, id_);
        } catch (const std::exception& e) {
            std::cout << "error: " << e.what() << std::endl;
            return;
        }
    }
};

}  // namespace connector