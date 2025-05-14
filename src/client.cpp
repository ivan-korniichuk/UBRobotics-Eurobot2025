#include "client.hpp"

RobotClient::RobotClient(const string& ip, int port)
  : serverIp(ip), serverPort(port) {}
RobotClient::~RobotClient() {}

void RobotClient::sendToServer(const vector<uint8_t>& data) {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    cerr << "Socket creation failed: " << strerror(errno) << endl;
    return;
  }

  sockaddr_in serv_addr{};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(serverPort);

  int ok = inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr);
  if (ok != 1) {
    cerr << "inet_pton failed for IP [" << serverIp << "], return code = " << ok << endl;
    close(sock);
    return;
  }

  cout << "Connecting to " << serverIp << ":" << serverPort << "...\n";

  if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    cerr << "Connection to RPi failed: " << strerror(errno) << endl;
    close(sock);
    return;
  }

  ssize_t sent = send(sock, data.data(), data.size(), 0);
  if (sent < 0) {
    cerr << "Send failed: " << strerror(errno) << endl;
    close(sock);
    return;
  }

  char buffer[1024] = {0};
  ssize_t received = read(sock, buffer, 1023);
  if (received < 0) {
    cerr << "Read failed: " << strerror(errno) << endl;
  } else {
    buffer[received] = '\0';
    cout << "RPi Response: " << buffer << endl;
  }

  close(sock);
}

void RobotClient::sendRawCommand(const vector<uint8_t>& command) {
  sendToServer(command);
}

void RobotClient::sendCustomMoveCommand(int16_t lower, int16_t upper, int16_t left, int16_t right) {
vector<uint8_t> args;
auto append16 = [&](int16_t val) {
args.push_back((val >> 8) & 0xFF);
args.push_back(val & 0xFF);
};

append16(lower);
append16(upper);
append16(left);
append16(right);
append16(12000);
append16(3300);
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
  std::vector<uint8_t> packet;
  packet.push_back(espID);
  packet.push_back(cmdID);
  packet.insert(packet.end(), args.begin(), args.end());
  packet.push_back(255); // optional end marker
  sendToServer(packet);
}
