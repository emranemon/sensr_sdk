#include "sensr.h"
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <typeinfo>
#include <algorithm>
#include <bitset>
#include <thread>
#include <chrono>
#include <fstream>
#include <mutex>
#if defined(__linux__)
#include <sys/time.h>
#endif
#include <google/protobuf/util/time_util.h>


class ConsoleUtil
{
public:
    enum Color : uint
    {
        Default = 0,
        Red = 1,
        Green = 2,
        Yellow = 3,
        Blue = 4
    };

    static void PrintLine(const std::string& message, Color color = Color::Default)
    {        
        switch (color)
        {
            case Red:
                std::cout << "\033[1;31m" << message << "\033[0m" << std::endl;
                break;
            case Green:
                std::cout << "\033[1;32m" << message << "\033[0m" << std::endl;
                break;
            case Yellow:
                std::cout << "\033[1;33m" << message << "\033[0m" << std::endl;
                break;
            case Blue:
                std::cout << "\033[1;34m" << message << "\033[0m" << std::endl;
                break;
            default:
                std::cout << message << std::endl;
                break;
        }
    }
};

class BinaryFile
{
public:
    BinaryFile() {}
    ~BinaryFile() { Close(); }

    bool Open(const std::string& file_name, std::ios::openmode mode)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!file_.is_open())
        {
            file_.open(file_name, mode | std::ios::binary);
        }
        return file_.is_open();
    }

    void Close()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (file_.is_open())
        {
            file_.close();
        }
    }

    bool Write(std::uint64_t position, const std::string& data)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!file_.is_open())
        {
            ConsoleUtil::PrintLine("File not open.", ConsoleUtil::Color::Red);
            return false;
        }
        if (position == std::numeric_limits<std::uint64_t>::max())
        {
            file_.seekp(0, std::ios::end); // Append
        }
        else
        {
            file_.seekp(position, std::ios::beg);
        }
        file_.write(data.c_str(), data.size());
        return file_.good();
    }

    std::string Read(std::uint64_t position, std::int32_t size)
    {
        std::uint64_t file_size = Size();
        if (position >= file_size)
        {
            ConsoleUtil::PrintLine("Position out of bounds.", ConsoleUtil::Color::Red);
            return "";
        }
        std::lock_guard<std::mutex> lock(mtx_);
        file_.seekg(position, std::ios::beg);
        if (size < 0 || position + static_cast<std::uint64_t>(size) > file_size)
        {
            size = static_cast<std::int32_t>(file_size - position);
        }
        std::string buffer(size, '\0');
        file_.read(&buffer[0], size);
        return buffer;
    }

    std::uint64_t Size()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!file_.is_open())
        {
            ConsoleUtil::PrintLine("File not open.", ConsoleUtil::Color::Red);
            return 0;
        }
        file_.seekg(0, std::ios::end);
        return static_cast<std::uint64_t>(file_.tellg());
    }

private:
    std::fstream file_;
    std::mutex mtx_;
};

class MessageUtil
{
public:
    static void PrintOutputMessage(const sensr_proto::OutputMessage& message)
    {
        // print stream message
        if (message.has_stream())
        {
            for(const auto& object : message.stream().objects())
            {
                std::stringstream ss;
                ss << "Obj ID : " << object.id() << ", Msg Timestamp: " 
                        << google::protobuf::util::TimeUtil::ToString(message.timestamp());
                ConsoleUtil::PrintLine(ss.str(), ConsoleUtil::Color::Green);
                ConsoleUtil::PrintLine("last_observed_timestamp: " + google::protobuf::util::TimeUtil::ToString(object.last_observed_timestamp()));
                //box size
                if(object.has_bbox())
                {
                    ss.clear();
                    ss.str("");
                    ss << "bbox Position: ["<< object.bbox().position().x() << "," << object.bbox().position().y() << "," 
                    << object.bbox().position().z() << "], ";
                    //box yaw
                    ss << "bbox yaw: " << object.bbox().yaw();
                    ConsoleUtil::PrintLine(ss.str());
                }
                //velocity
                if(object.has_velocity()) 
                {
                    ss.clear();
                    ss.str("");
                    ss << "velocity: [" << object.velocity().x() << "," << object.velocity().y() << "," 
                    << object.velocity().z() << "]";
                    ConsoleUtil::PrintLine(ss.str());
                }
                //tracking status
                ConsoleUtil::PrintLine("tracking status: " + TrackingStatus_Name(object.tracking_status()));
                //classification result
                ConsoleUtil::PrintLine("classification result: " + LabelType_Name(object.label()));
                // zone_ids
                if(object.zone_ids().size() > 0)
                {
                    ss.clear();
                    ss.str("");
                    ss << "zone_ids: [";
                    for(int i=0; i < object.zone_ids().size(); i++)
                    {
                        ss << object.zone_ids(i);
                        if (i < object.zone_ids().size() - 1)
                        {
                            ss << ", ";
                        }
                        else
                        {
                            ss << "]";
                        }
                    }
                    ConsoleUtil::PrintLine(ss.str());
                }
            }
        }
        else
        {
            ConsoleUtil::PrintLine("OutputMessage does not have StreamMessage.", ConsoleUtil::Color::Yellow);
        }

        // print event message
        if(message.has_event())
        {
            for(const auto& zone_event : message.event().zone())
            {
                std::stringstream ss;
                ss << "Zone ID : " << zone_event.id() << ", Msg Timestamp: " 
                        << google::protobuf::util::TimeUtil::ToString(message.timestamp());
                ConsoleUtil::PrintLine(ss.str(), ConsoleUtil::Color::Blue);
                // zone obj id
                ss.clear();
                ss.str("");
                ss << "Zone Obj ID : " << zone_event.object().id() << ", Zone Timestamp: " 
                        << google::protobuf::util::TimeUtil::ToString(zone_event.timestamp());
                ConsoleUtil::PrintLine(ss.str());
                // zone event type
                ConsoleUtil::PrintLine("zone event type: " + sensr_proto::ZoneEvent_Type_Name(zone_event.type()));
                // zone object position
                ss.clear();
                ss.str("");
                ss << "zone object position: [" << zone_event.object().position().x() << "," << zone_event.object().position().y() << "," 
                    << zone_event.object().position().z() << "]";
                ConsoleUtil::PrintLine(ss.str());
            }
        }
        else
        {
            ConsoleUtil::PrintLine("OutputMessage does not have EventMessage.", ConsoleUtil::Color::Yellow);
        }

        // print custom message
        // ....
    }
};

class MessageHandler
{
public:
    virtual ~MessageHandler() = default;

protected:
    static const std::int32_t DATA_SIZE_BITS = 32;
    std::string bin_file_name_;
    BinaryFile bin_file_;
    bool started_ = false;

    void Reset(const std::string& file_name)
    {
        bin_file_.Close();
        started_ = false;
        bin_file_name_ = file_name + ".bin";
    }

    bool OpenFile(std::ios_base::openmode mode)
    {
        started_ = bin_file_.Open(bin_file_name_, mode);
        return started_;
    }
};

class MessageRecorder : public MessageHandler
{
public:
    MessageRecorder() = default;

    bool Start(const std::string& file_name)
    {
        Reset(file_name);
        return OpenFile(std::ios_base::out);
    }

    void Stop()
    {
        Reset("");
    }

    void Record(const sensr_proto::OutputMessage& message)
    {
        if (!started_)
        {
            ConsoleUtil::PrintLine("Recorder has not started!", ConsoleUtil::Color::Yellow);
            return;
        }

        std::string data;
        if (message.SerializeToString(&data))
        {
            std::bitset<DATA_SIZE_BITS> data_size(data.size());
            bin_file_.Write(std::numeric_limits<std::uint64_t>::max(), data_size.to_string());
            bin_file_.Write(std::numeric_limits<std::uint64_t>::max(), data);
        }
        else
        {
            ConsoleUtil::PrintLine("sensr_proto::OutputMessage Serialization Failed.", ConsoleUtil::Color::Red);
        }
    }
};

class MessagePlayer : public MessageHandler
{
public:
    MessagePlayer() = default;

    bool Start(const std::string& file_name, const std::function<void(const sensr_proto::OutputMessage&)>& output_msg_delegate)
    {
        Reset(file_name);
        output_msg_delegate_ = output_msg_delegate;
        return OpenFile(std::ios_base::in);
    }

    void Stop()
    {
        Reset("");
        output_msg_delegate_ = nullptr;
    }

    void Play(int interval)
    {
        if (!started_ || !output_msg_delegate_)
        {
            ConsoleUtil::PrintLine("Player has not started!", ConsoleUtil::Color::Yellow);
            return;
        }
        std::uint64_t position = 0;
        std::uint64_t file_size = bin_file_.Size();
        while (position < file_size)
        {
            std::string data_size_bits = bin_file_.Read(position, DATA_SIZE_BITS);
            if (data_size_bits.empty())
            {
                break;
            }
            position += data_size_bits.size();
            std::bitset<DATA_SIZE_BITS> data_size(data_size_bits);
            std::string data = bin_file_.Read(position, static_cast<std::int32_t>(data_size.to_ulong()));
            if (data.empty())
            {
                break;
            }
            sensr_proto::OutputMessage message;
            if (message.ParseFromString(data))
            {
                output_msg_delegate_(message);
            }
            else
            {
                ConsoleUtil::PrintLine("sensr_proto::OutputMessage Deserialization Failed.", ConsoleUtil::Color::Red);
            }
            position += data.size();
            if (interval > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval));
            }
        }
        ConsoleUtil::PrintLine("Finished Reading File: " + bin_file_name_);
    }

private:
    std::function<void(const sensr_proto::OutputMessage&)> output_msg_delegate_;
};

class MessageRecorderOperator : public sensr::MessageListener
{
public:
    MessageRecorderOperator(sensr::Client* client, MessageRecorder* recorder) : MessageListener(ListeningType::kOutputMessage),
    client_(client), recorder_(recorder)
    {

    }

    void OnError(Error error, const std::string& reason) override
    {
        ConsoleUtil::PrintLine(reason, ConsoleUtil::Color::Red);
        if (error == sensr::MessageListener::Error::kOutputMessageConnection || 
        error == sensr::MessageListener::Error::kPointResultConnection ||
        error == sensr::MessageListener::Error::kOutputBufferOverflow)
        {
            client_->Reconnect();
        }
    }

    void OnGetOutputMessage(const sensr_proto::OutputMessage &message) override
    {
        recorder_->Record(message);
    }

private:
    sensr::Client* client_;
    MessageRecorder* recorder_;
};

int main(int argc, char *argv[])
{
    const char* file_name = "OutputMessage";
    const char* client_address = "";
    if (argc > 1)
    {
        client_address = argv[1];
    }

    std::shared_ptr<MessageHandler> handler;
    std::shared_ptr<sensr::MessageListener> listener;
    std::shared_ptr<sensr::Client> client;
    // example address, "wss://192.168.0.103:8085"
    std::string address = std::string(client_address);
    std::cout << "IP: [" << address << "]" << std::endl;

    // If address passed start recording
    if (address.size() > 0)
    {
        handler = std::make_shared<MessageRecorder>();
        client = std::make_shared<sensr::Client>(address);
        std::shared_ptr<MessageRecorder> recorder = std::dynamic_pointer_cast<MessageRecorder>(handler);
        if(!recorder || !recorder->Start(std::string(file_name)))
        {
            ConsoleUtil::PrintLine("StartRecorder Failed.", ConsoleUtil::Color::Red);
        }
        listener = std::make_shared<MessageRecorderOperator>(client.get(), recorder.get());
        if(!client->SubscribeMessageListener(listener))
        {
            ConsoleUtil::PrintLine("SubscribeMessageListener Failed.", ConsoleUtil::Color::Red);
        }
    }
    else
    {
        handler = std::make_shared<MessagePlayer>();
        std::shared_ptr<MessagePlayer> player = std::dynamic_pointer_cast<MessagePlayer>(handler);
        if(!player || !player->Start(std::string(file_name), MessageUtil::PrintOutputMessage))
        {
            ConsoleUtil::PrintLine("StartPlayer Failed.", ConsoleUtil::Color::Red);
        }
        // started playing recorded data : upto eof
        player->Play(1000); // print a message per sec for better readability
    }

    std::string s;
    std::getline(std::cin, s);

    while(s != "")
    { // if the person hits enter, s == "" and leave the loop
        std::cout << s << std::endl;
        std::getline(std::cin, s);
    }
    return 0;
}