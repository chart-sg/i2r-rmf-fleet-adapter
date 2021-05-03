/*
 * Copyright (c) 2014, Peter Thorson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the WebSocket++ Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// **NOTE:** This file is a snapshot of the WebSocket++ utility client tutorial.
// Additional related material can be found in the tutorials/utility_client
// directory of the WebSocket++ repository.

#include "client.hpp"

connection_metadata::context_ptr connection_metadata::on_tls_init(websocketpp::connection_hdl) {
    connection_metadata::context_ptr ctx = std::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::sslv23);

    try {
        ctx->set_options(boost::asio::ssl::context::default_workarounds |
                        boost::asio::ssl::context::no_sslv2 |
                        boost::asio::ssl::context::no_sslv3 |
                        boost::asio::ssl::context::single_dh_use);
    } catch (std::exception& e) {
        std::cout <<"Error in context pointer: "<< e.what() << std::endl;
    }
    return ctx;
}

void connection_metadata::connection_metadata::on_open(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Open";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
}

void connection_metadata::on_fail(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Failed";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
    m_error_reason = con->get_ec().message();
}

void connection_metadata::on_close(client * c, websocketpp::connection_hdl hdl) {
    m_status = "Closed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "close code: " << con->get_remote_close_code() << " (" 
        << websocketpp::close::status::get_string(con->get_remote_close_code()) 
        << "), close reason: " << con->get_remote_close_reason();
    m_error_reason = s.str();
}

void connection_metadata::on_message(websocketpp::connection_hdl, client::message_ptr msg) {
    if (msg->get_opcode() == websocketpp::frame::opcode::text) {
        m_messages.push_back("<< " + msg->get_payload());
    } else {
        m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload()));
    }
    const auto& fs = mrccc_utils::feedback_parser::RobotStateUpdate(std::string(msg->get_payload()));
    std::weak_ptr<Connection> c = std::make_shared<Connections>();
    
    // // Some standard checks to see if this is the right fleet
    // // if (fs->name!=fleet_name) return;

    // const auto connections = c.lock();
    // if (!connections)
    // return;

    // for (const auto& state : fs->robots)
    // {
    //     const auto insertion = connections->robots.insert({state.name, nullptr});
    //     const bool new_robot = insertion.second;
    //     if (new_robot)
    //     {
    //         // We have not seen this robot before, so let's add it to the fleet.
    //         connections->add_robot(fleet_name, state);
    //     }

    //     const auto& command = insertion.first->second;
    //     if (command)
    //     {
    //         // We are ready to command this robot, so let's update its state
    //         command->update_state(state);
    //     }
    // }
}

int websocket_endpoint::connect(std::string const & uri) {

    websocketpp::lib::error_code ec;

    m_endpoint.set_tls_init_handler(connection_metadata::on_tls_init);
    client::connection_ptr con = m_endpoint.get_connection(uri, ec);

    if (ec) {
        std::cout << "> Connect initialization error: " << ec.message() << std::endl;
        return -1;
    }

    int new_id = m_next_id++;
    connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri);
    m_connection_list[new_id] = metadata_ptr;

    con->set_open_handler(websocketpp::lib::bind(
        &connection_metadata::on_open,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1
    ));
    con->set_fail_handler(websocketpp::lib::bind(
        &connection_metadata::on_fail,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1
    ));
    con->set_close_handler(websocketpp::lib::bind(
        &connection_metadata::on_close,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1
    ));
    con->set_message_handler(websocketpp::lib::bind(
        &connection_metadata::on_message,
        metadata_ptr,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2
    ));

    m_endpoint.connect(con);

    return new_id;
}

void websocket_endpoint::close(int id, websocketpp::close::status::value code, std::string reason) {
    websocketpp::lib::error_code ec;
    
    con_list::iterator metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        std::cout << "> No connection found with id " << id << std::endl;
        return;
    }
    
    m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
    if (ec) {
        std::cout << "> Error initiating close: " << ec.message() << std::endl;
    }
}

void websocket_endpoint::send(int id, std::string message) {
    websocketpp::lib::error_code ec;
    
    con_list::iterator metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        std::cout << "> No connection found with id " << id << std::endl;
        return;
    }
    
    m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return;
    }
    
    metadata_it->second->record_sent_message(message);
}

connection_metadata::ptr websocket_endpoint::get_metadata(int id) const{
    websocket_endpoint::con_list::const_iterator metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        return connection_metadata::ptr();
    } else {
        return metadata_it->second;
    }
}

// int main() {
//     bool done = false;
//     std::string input;
//     websocket_endpoint endpoint;

//     while (!done) {
//         std::cout << "Enter Command: ";
//         std::getline(std::cin, input);

//         if (input == "quit") {
//             done = true;
//         } else if (input == "help") {
//             std::cout
//                 << "\nCommand List:\n"
//                 << "connect <ws uri>\n"
//                 << "send <connection id> <message>\n"
//                 << "close <connection id> [<close code:default=1000>] [<close reason>]\n"
//                 << "show <connection id>\n"
//                 << "help: Display this help text\n"
//                 << "quit: Exit the program\n"
//                 << std::endl;
//         } else if (input.substr(0,7) == "connect") {
//             int id = endpoint.connect(input.substr(8));
//             if (id != -1) {
//                 std::cout << "> Created connection with id " << id << std::endl;
//             }
//         } else if (input.substr(0,4) == "send") {
//             std::stringstream ss(input);
            
//             std::string cmd;
//             int id;
//             std::string message;
            
//             ss >> cmd >> id;
//             std::getline(ss,message);
            
//             endpoint.send(id, message);
//         } else if (input.substr(0,5) == "close") {
//             std::stringstream ss(input);
            
//             std::string cmd;
//             int id;
//             int close_code = websocketpp::close::status::normal;
//             std::string reason;
            
//             ss >> cmd >> id >> close_code;
//             std::getline(ss,reason);
            
//             endpoint.close(id, close_code, reason);
//         } else if (input.substr(0,4) == "show") {
//             int id = atoi(input.substr(5).c_str());

//             connection_metadata::ptr metadata = endpoint.get_metadata(id);
//             if (metadata) {
//                 // Removed the << operator overload
//                 // std::cout << *metadata << std::endl;
//             } else {
//                 std::cout << "> Unknown connection id " << id << std::endl;
//             }
//         } 
//         // else if (input.substr(0,5) == "dummy") {
//         //     int id = endpoint.connect("https://mrccc.chart.com.sg:5100");
//         //     if (id != -1) {
//         //         std::cout << "> Created connection with id " << id << std::endl;
//         //     }
//         //     endpoint.send();
//         // } 
//         else {
//             std::cout << "> Unrecognized Command" << std::endl;
//         }
//         connection_metadata::ptr metadata = endpoint.get_metadata(0);
//         for (const auto & msg : metadata->m_messages){
//             std::cout<<msg<<std::endl;
//             sleep(1);
//         } 
//     }

//     return 0;
// }