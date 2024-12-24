#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <string>

class Socket {
 public:
  Socket(const std::string& host, unsigned short port)
      : io_context_(), socket_(io_context_) {
    try {
      boost::asio::ip::tcp::resolver resolver(io_context_);
      auto endpoints = resolver.resolve(host, std::to_string(port));
      boost::asio::connect(socket_, endpoints);
      std::cout << "Connected to " << host << ":" << port << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Connection failed: " << e.what() << std::endl;
      throw;
    }
  }

  ~Socket() { close(); }

  void send(const std::string& message) {
    try {
      boost::asio::write(socket_, boost::asio::buffer(message));
    } catch (const std::exception& e) {
      std::cerr << "Send failed: " << e.what() << std::endl;
    }
  }

  std::string recv() {
    try {
      boost::asio::streambuf buffer;
      boost::asio::read_until(socket_, buffer, "\n");
      std::istream input(&buffer);
      std::string response;
      std::getline(input, response);
      return response;
    } catch (const std::exception& e) {
      std::cerr << "Receive failed: " << e.what() << std::endl;
      return "";
    }
  }

  void recv_k_bytes(unsigned char* data, size_t k) {
    try {
      boost::asio::read(socket_, boost::asio::buffer(data, k));
    } catch (const std::exception& e) {
      std::cerr << "Receive failed: " << e.what() << std::endl;
    }
  }

  void close() {
    if (socket_.is_open()) {
      try {
        socket_.close();
      } catch (const std::exception& e) {
        std::cerr << "Close failed: " << e.what() << std::endl;
      }
    }
  }

 private:
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;
};

/*
 * Example usage

int main()
{
    try
    {
        Socket client("127.0.0.1", 8080);
        client.send("Hello, Server!\n");
        std::string response = client.recv();
        std::cout << "Server response: " << response << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

*/