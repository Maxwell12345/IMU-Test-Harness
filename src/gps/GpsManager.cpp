/******************************************************************************
 * File:             GpsManager.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 *
 ******************************************************************************/
#include "GpsManager.hpp"

void GpsManager::start() {
    this->m_serialPort.emplace(this->m_ioContext, this->m_comPort);

    this->m_serialPort.set_option(boost::asio::serial_port_base::baud_rate(this->m_baudRate));
    this->m_serialPort.set_option(boost::asio::serial_port_base::character_size(8));
    this->m_serialPort.set_option(
        boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    this->m_serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    this->m_serialPort.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    this->m_running.store(true);

    this->m_readThread = std::jthread([this](std::stop_token st) {
        this->readLoop(st);
        // TODO: set running to false
    });

    this->m_queueThread = std::jthread([this](std::stop_token st) {
    });
}

void GpsManager::stop() {
}

bool GpsManager::isRunning() const {
    return this->m_running.load();
}

GpsManagerStats GpsManager::getStats() const {
}

void GpsManager::readLoop(std::stop_token stopToken) {
    std::stop_callback onStop(stopToken, [this]() {
        boost::system::error_code ec;
        if (this->m_serialPort && this->m_serialPort->is_open()) {
            this->m_serialPort->cancel(ec);
        }
    });

    while (!stopToken.stop_requested()) {
        boost::system::error_code ec;
        boost::asio::read_until(*this->m_serialPort, this->m_readBuf, '\n', ec);
        if (ec)
            break;

        std::istream serialStream(&this->m_readBuf);
        std::string sentence;
        std::getline(stream, sentence);
        if (!sentence.empty() && sentence.back() == '\r') {
            sentence.pop_back(); {
                std::lock_guard<std::mutex> queueLock(this->m_queueMutex);
                this->m_sentenceQueue.push(std::move(sentence));
            }
            this->m_queueCv.notify_one();
        }
    }
}

void GpsManager::processQueue() {
    while (this->m_running.load()) {
        try {
            std::string serialSentence;
            {
                std::unique_lock<std::mutex> queueLock(this->m_queueMutex);
                this->m_queueCv.wait(queueLock, [this]() { return !this->m_sentenceQueue.empty(); });
                serialSentence = this->m_sentenceQueue.front();
                this->m_sentenceQueue.pop();
            }

            // call method from NMEAParserWrapper to process the sentence
            // TODO: take the NMEAParserWrapper class defined inside Crusader NMEASocketManager.hpp, and combine all the Crusader NMEAParserWrapper code into our NMEAParser
            //      We want the parser to combine the ProcessNMEABuffer methods together, and combine the ProcessRxCommand methods together
            //      From there what we need is to take the new data and create an updated GPSUpdate object that will be sent using this->m_GpsUpdateCallback

            // create a new GPS update using this most current information

            // send that update to everyone who needs it
            // this->m_gpsUpdateCallback(updatedGps);
        }
        catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    }
}

bool GpsManager::validateFix(const IMUUtils::GpsUpdate &update) const {
}

void GpsManager::publishUpdate(IMUUtils::GpsUpdate update) {
}
