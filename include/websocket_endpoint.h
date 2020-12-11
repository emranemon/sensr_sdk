#pragma once

#include "websocketpp/client.hpp"
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/extensions/permessage_deflate/enabled.hpp"
#include <functional>

namespace sensr
{
    typedef websocketpp::client<websocketpp::config::asio_client> client;
    template <typename T>
    class connection_metadata
    {
    public:
        using MsgReceiver = std::function<void(const T& msg)>;

        connection_metadata(websocketpp::connection_hdl hdl, std::string uri, MsgReceiver receiver)
        : m_hdl(hdl), m_status("Connecting"), m_uri(uri), m_server("N/A"), msg_receiver_(receiver){}

        void on_open(client *c, websocketpp::connection_hdl hdl);
        void on_fail(client *c, websocketpp::connection_hdl hdl);
        void on_close(client *c, websocketpp::connection_hdl hdl);
        void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);
        inline std::string get_status() const { return m_status; }
        inline websocketpp::connection_hdl get_hdl() const { return m_hdl; }

    private:
        websocketpp::connection_hdl m_hdl;
        std::string m_status;
        std::string m_uri;
        std::string m_server;
        std::string m_error_reason;
        MsgReceiver msg_receiver_;
    };

    template <typename MsgType>
    class websocket_endpoint
    {
    public:
        using MsgReceiver = std::function<void(const MsgType& msg)>;
        websocket_endpoint();
        ~websocket_endpoint();

        int connect(std::string const &uri, MsgReceiver func) {
            websocketpp::lib::error_code ec;
            client::connection_ptr con = m_endpoint.get_connection(uri, ec);
            if (ec) {
                std::cout << "> Connect initialization error: " << ec.message() << std::endl;
                return -1;
            }
            m_connection.reset(new connection_metadata<MsgType>(con->get_handle(), uri, func));

            con->set_open_handler(websocketpp::lib::bind(
                &connection_metadata<MsgType>::on_open,
                m_connection.get(),
                &m_endpoint,
                websocketpp::lib::placeholders::_1));
            con->set_fail_handler(websocketpp::lib::bind(
                &connection_metadata<MsgType>::on_fail,
                m_connection.get(),
                &m_endpoint,
                websocketpp::lib::placeholders::_1));
            con->set_message_handler(websocketpp::lib::bind(
                &connection_metadata<MsgType>::on_message,
                m_connection.get(),
                websocketpp::lib::placeholders::_1,
                websocketpp::lib::placeholders::_2));

            m_endpoint.connect(con);
            return 0;
        };
        void close(websocketpp::close::status::value code);

    private:
        client m_endpoint;
        websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
        websocketpp::lib::unique_ptr<connection_metadata<MsgType>> m_connection;
    };

    template <typename T>
    void connection_metadata<T>::on_open(client *c, websocketpp::connection_hdl hdl)
    {
        m_status = "Open";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
    }
    template <typename T>
    void connection_metadata<T>::on_fail(client *c, websocketpp::connection_hdl hdl)
    {
        m_status = "Failed";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
        m_error_reason = con->get_ec().message();
    }

    template <typename T>
    void connection_metadata<T>::on_close(client *c, websocketpp::connection_hdl hdl)
    {
        m_status = "Closed";
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::stringstream s;
        s << "close code: " << con->get_remote_close_code() << " ("
          << websocketpp::close::status::get_string(con->get_remote_close_code())
          << "), close reason: " << con->get_remote_close_reason();
        m_error_reason = s.str();
    }

    template <typename T>
    void connection_metadata<T>::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
    {
        std::string message;
        if (msg->get_opcode() == websocketpp::frame::opcode::text)
        {
            message = msg->get_payload();
        }
        else
        {
            message = websocketpp::utility::to_hex(msg->get_payload());
        }
        if (msg_receiver_) 
        {
            T output;
            output.ParseFromString(message);        
            msg_receiver_(output);
        }
    }


    template <typename MsgType>
    websocket_endpoint<MsgType>::websocket_endpoint()
    {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

        m_endpoint.init_asio();
        m_endpoint.start_perpetual();

        m_thread.reset(new websocketpp::lib::thread(&client::run, &m_endpoint));
    }

    template <typename MsgType>
    websocket_endpoint<MsgType>::~websocket_endpoint() {
        m_endpoint.stop_perpetual();
        close(websocketpp::close::status::going_away);
        m_thread->join();
    }

    template <typename MsgType>
    void websocket_endpoint<MsgType>::close(websocketpp::close::status::value code) {
        websocketpp::lib::error_code ec;
        if (m_connection == nullptr) {
            std::cout << "> No connection found" << std::endl;
            return;
        }

        if (m_connection->get_status() != "Open") {
            // Only close open connections
            m_endpoint.close(m_connection->get_hdl(), code, "", ec);
            if (ec) {
                std::cout << "> Error closing connection : " << ec.message() << std::endl;
            }            
        }
        m_connection.reset();
    }
} // namespace sensr