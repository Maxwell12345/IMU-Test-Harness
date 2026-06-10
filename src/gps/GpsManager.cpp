/******************************************************************************
 * File:             GpsManager.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 *
 ******************************************************************************/
#include "GpsManager.hpp"
#include ""

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


            NMEAParserData::ERROR_E e = m_nmeaParser.ProcessNMEABuffer(serialSentence.c_str(), serialSentence.size());

            if (e != NMEAParserData::ERROR_E::ERROR_OK) {
                std::cout << "Not a NMEA payload!\n";
            }

            if (serialSentence.find("GGA") != std::string::npos) {
                NMEASentenceGGA gga = m_nmeaParser.GetGGA();


                this->m_gpsUpdateCallback(updatedGps);
            }
            else if (serialSentence.find("GSV") != std::string::npos) {
                NMEASentenceGSV gsv = m_nmeaParser.GetGSV();
            }
            else if (serialSentence.find("RMC") != std::string::npos) {
                NMEASentenceRMC rmc = m_nmeaParser.GetRMC();
            }
            else if (serialSentence.find("GSA") != std::string::npos) {
                NMEASentenceGSA gsa = m_nmeaParser.GetGSA();
            }
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
