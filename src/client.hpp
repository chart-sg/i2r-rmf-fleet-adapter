#pragma once

#include <websocketpp/config/asio_client.hpp>

#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

#include "i2r_driver/i2r_driver.hpp"
#include "i2r_driver/mission_gen.hpp"
#include "i2r_driver/feedback_parser.hpp"

typedef websocketpp::client<websocketpp::config::asio_tls_client> client;

class connection_metadata {
public:
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;
    typedef std::shared_ptr<boost::asio::ssl::context> context_ptr;

    connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri)
      : m_id(id)
      , m_hdl(hdl)
      , m_status("Connecting")
      , m_uri(uri)
      , m_server("N/A")
    {}

    static context_ptr on_tls_init(websocketpp::connection_hdl);

    void on_open(client * c, websocketpp::connection_hdl hdl);

    void on_fail(client * c, websocketpp::connection_hdl hdl);
    
    void on_close(client * c, websocketpp::connection_hdl hdl);

    void on_message(websocketpp::connection_hdl, client::message_ptr msg);

    websocketpp::connection_hdl get_hdl() const {
        return m_hdl;
    }
    
    int get_id() const {
        return m_id;
    }
    
    std::string get_status() const {
        return m_status;
    }

    void record_sent_message(std::string message) {
        m_messages.push_back(">> " + message);
    }
    std::vector<std::string> m_messages;

    // Pass the fleet state pointer to connection_metadata to hold
    rmf_fleet_msgs::msg::FleetState::SharedPtr fs_ptr;
    void pass_fleet_state_ptr(const rmf_fleet_msgs::msg::FleetState::SharedPtr &_fs_ptr);

private:
    int m_id;
    websocketpp::connection_hdl m_hdl;
    std::string m_status;
    std::string m_uri;
    std::string m_server;
    std::string m_error_reason;
};

class websocket_endpoint {
public:

    websocket_endpoint () : m_next_id(0) {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

        m_endpoint.init_asio();
        m_endpoint.start_perpetual();

        m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
    
        mrccc_utils::mission_gen::identifyMe();
    }

    ~websocket_endpoint() {
        m_endpoint.stop_perpetual();
        
        for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
            if (it->second->get_status() != "Open") {
                // Only close open connections
                continue;
            }
            
            std::cout << "> Closing connection " << it->second->get_id() << std::endl;
            
            websocketpp::lib::error_code ec;
            m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
            if (ec) {
                std::cout << "> Error closing connection " << it->second->get_id() << ": "  
                          << ec.message() << std::endl;
            }
        }
        
        m_thread->join();
    }

    int connect(std::string const & uri);

    void close(int id, websocketpp::close::status::value code, std::string reason);
    void send(int id, std::string message);

    connection_metadata::ptr get_metadata(int id) const;

    // Pass the fleet state pointer to connection_metadata to hold
    void pass_fleet_state_ptr(const rmf_fleet_msgs::msg::FleetState::SharedPtr &_fs_ptr);

private:
    typedef std::map<int,connection_metadata::ptr> con_list;

    client m_endpoint;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

    con_list m_connection_list;
    int m_next_id;
};