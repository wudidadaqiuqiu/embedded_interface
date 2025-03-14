#pragma once
#include <atomic>
#include <thread>

#include "common/debug/log.hpp"
#include "connector/connector_def.hpp"
#include "connector/pack_manager.hpp"

#define PRINT_FILE_AND_LINE() \
    std::cout << "File: " << __FILE__ << ", Line: " << __LINE__ << std::endl;

namespace connector {
template <ConnectorType CON_TYPE, typename MSGPackT>
class ConnectorSingleRecvNode {
   public:
    using SharedPtr = std::shared_ptr<ConnectorSingleRecvNode<CON_TYPE, MSGPackT>>;
    ConnectorSingleRecvNode(Connector<CON_TYPE>& con) : connector(con),
                                                        pack_manager() {
        is_end = false;
        // std::cout << "ConnectorSingleRecvNode buffer size = " << buffer.size() << std::endl;
        buffer.resize(64);
        // pub = nh.advertise<typename MSGPackT::MSGT>(topic_name, 10);
        thread = std::thread(&ConnectorSingleRecvNode<CON_TYPE, MSGPackT>::run, this);
    }
    ConnectorSingleRecvNode(const ConnectorSingleRecvNode&) = delete;

    ~ConnectorSingleRecvNode() {
        // 在这里设置成非阻塞没用
        is_end = true;
        connector.con_close();
        if (thread.joinable()) {
            thread.join();
        }
    }

    void register_callback(std::function<void(const typename MSGPackT::MSGT&)> func) {
        pack_manager.register_callback(func);
    }

    void unregister() {
        pack_manager.unregister();
    }

    Connector<CON_TYPE>& get_connector() { return connector; }

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
                LOG_ERROR(1, "%s", e.what());
                continue;
            } catch (const std::exception& e) {
                LOG_ERROR(1, "error: %s, %s, %d", e.what(), __FILE__, __LINE__);
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
    void send(const typename MSGPackT::MSGT& msg) {
        MSGPackT::unpack(msg, buffer, id_);
        try {
            connector.con_send(buffer, id_);
        } catch (const std::exception& e) {
            LOG_ERROR(1, "error: %s", e.what());
            // std::cout << "error: " << e.what() << std::endl;
            return;
        }
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
            LOG_ERROR(1, "error: %s", e.what());
            // std::cout << "error: " << e.what() << std::endl;
            return;
        }
    }
};

}  // namespace connector