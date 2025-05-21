#include "client.hpp"

RobotClient::RobotClient(const string& ip, int port)
  : serverIp(ip), serverPort(port), isRunning(true), socketError(false) {
    // Socket thread (making sure that the socket to the pi stays open and re-connects when needed)
    socketThread = thread(&RobotClient::runSocketConnectionLoop, this);
  }
RobotClient::~RobotClient() {
    isRunning = false;
    if (socketThread.joinable()) socketThread.join();
}

void RobotClient::runSocketConnectionLoop() {
    while (isRunning) {
        if (sockfd == -1 || socketError) {
            while (isRunning) {
                cerr << "Lost connection to the RPi, attempting reconnection." << endl;
                // Attempt to reconnect to the server
                sockfd = socket(AF_INET, SOCK_STREAM, 0);
                if (sockfd < 0) {
                    cerr << "Could not create socket with error '" << strerror(errno) << "', trying again." << endl;
                    continue;
                }

                sockaddr_in serv_addr{};
                serv_addr.sin_family = AF_INET;
                serv_addr.sin_port = htons(serverPort);

                int ok = inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr);
                if (ok != 1) {
                    cerr << "inet_pton failed for IP [" << serverIp << "] with error '" << strerror(errno) << "', trying reconnection again." << endl;
                    close(sockfd);
                    continue;
                }

                if (connect(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
                    cerr << "Connection to RPi failed with error '" << strerror(errno) << "', trying reconnection again." << endl;
                    close(sockfd);
                    continue;
                }
                cout << "Successfully reconnected, allowing messages to be sent." << endl;
                socketError = false;
                break;
            }
        }
    }
}

void RobotClient::sendToServer(const vector<uint8_t>& data) {
  // int sock = socket(AF_INET, SOCK_STREAM, 0);
  // if (sock < 0) {
  //   cerr << "Socket creation failed: " << strerror(errno) << endl;
  //   return;
  // }
  //
  // sockaddr_in serv_addr{};
  // serv_addr.sin_family = AF_INET;
  // serv_addr.sin_port = htons(serverPort);
  //
  // int ok = inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr);
  // if (ok != 1) {
  //   cerr << "inet_pton failed for IP [" << serverIp << "], return code = " << ok << endl;
  //   close(sock);
  //   return;
  // }
  //
  // cout << "Connecting to " << serverIp << ":" << serverPort << "...\n";
  //
  // if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
  //   cerr << "Connection to RPi failed: " << strerror(errno) << endl;
  //   close(sock);
  //   return;
  // }
  if (socketError) {
    return;
  }

  ssize_t sent = send(sockfd, data.data(), data.size(), 0);
  if (sent < 0) {
    cerr << "Send failed: " << strerror(errno) << endl;
    close(sockfd);
    socketError = true;
    return;
  }

  char buffer[1024] = {0};
  ssize_t received = read(sockfd, buffer, 1023);
  if (received < 0) {
    cerr << "Read failed: " << strerror(errno) << endl;
    close(sockfd);
    socketError = true;
  } else {
    buffer[received] = '\0';
    cout << "RPi Response: " << buffer << endl;
  }

  // close(sockfd);
}

void RobotClient::sendRawCommand(const vector<uint8_t>& command) {
  sendToServer(command);
}

void RobotClient::sendLocomotionCommand(float omega, float vx, float vy, float scalar) {
  auto encode16 = [](int16_t val) {
    return vector<uint8_t>{(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  };

  vector<uint8_t> args;
  auto append = [&](float val) {
    int16_t scaled = static_cast<int16_t>((int)(val * 100));
    auto bytes = encode16(scaled);
    args.insert(args.end(), bytes.begin(), bytes.end());
  };

  append(omega);
  append(vx);
  append(vy);
  auto scalar_bytes = encode16(scalar);
  args.insert(args.end(), scalar_bytes.begin(), scalar_bytes.end());

  sendCommandToESP(0x41, 1, args);  // 0x41 = Locomotion I2C ESP
}


void RobotClient::sendCustomMoveCommand(float lower, float upper, float left, float right) {
  vector<uint8_t> args;
  auto append16 = [&](int16_t val) {
  args.push_back((val >> 8) & 0xFF);
  args.push_back(val & 0xFF);
  };


  // append16(lower);
  // append16(upper);
  // append16(left);
  // append16(right);
  append16(lower);
  append16(upper * 10000.0 / 7.5);
  append16(left * 10000.0 / 7.5);
  append16(right * 10000.0 / 7.5);
  append16(12000);
  append16(4000);
  append16(4000);
  append16(4000);

  sendCommandToESP(0x40, 4, args);
}

void RobotClient::sendHomingCommand() {
  sendCommandToESP(0x40, 1, {});
}

void RobotClient::sendMoveFullCommand() {
  sendCommandToESP(0x40, 2, {});
}

void RobotClient::sendMovePartialCommand() {
  sendCommandToESP(0x40, 3, {});
}

void RobotClient::sendOpenLowerGrippers(bool close) {
  sendCommandToESP(0x42, 1, {static_cast<uint8_t>(close ? 1 : 0)});
}

void RobotClient::sendOpenUpperGrippers(bool close) {
  sendCommandToESP(0x42, 2, {static_cast<uint8_t>(close ? 1 : 0)});
}

void RobotClient::sendTwistOuterGrippers(bool close) {
  sendCommandToESP(0x42, 3, {static_cast<uint8_t>(close ? 1 : 0)});
}

void RobotClient::sendCommandToESP(uint8_t espID, uint8_t cmdID, const std::vector<uint8_t>& args) {
  vector<uint8_t> packet;
  packet.push_back(espID);
  packet.push_back(cmdID);
  packet.insert(packet.end(), args.begin(), args.end());
  packet.push_back(255); // optional end marker
  sendToServer(packet);
}

void RobotClient::sendNewESPMoveCommand(int16_t velX, int16_t velY, int16_t turn) {
  vector<uint8_t> args;

  auto append16 = [&](int16_t val) {
      args.push_back((val >> 8) & 0xFF);
      args.push_back(val & 0xFF);
  };

  append16(velX);
  append16(velY);
  append16(turn);

  sendCommandToESP(0x10, 1, args);
}
