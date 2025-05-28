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

  // cout << "Connecting to " << serverIp << ":" << serverPort << "...\n";

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
    // cout << "RPi Response: " << buffer << endl;
  }

  close(sock);
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

void RobotClient::sendCommandToESP(uint8_t espID, uint8_t cmdID, const vector<uint8_t>& args) {
  lock_guard<mutex> lock(sendMutex);
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

void RobotClient::sendGrab(bool grab) {
  vector<uint8_t> args;

  if (grab) {
    sendCommandToESP(0x11, 1, args);
  } else {
    sendCommandToESP(0x11, 2, args);
  }
}

void RobotClient::sendSeek() {
  vector<uint8_t> args;

  sendCommandToESP(0x10, 2, args);
}

void RobotClient::registerWithRPi() {
  cout << "[Client] start registerWithRPi" << "\n";
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(serverPort);
  inet_pton(AF_INET, serverIp.c_str(), &addr.sin_addr);

  // connect(sock, (struct sockaddr*)&addr, sizeof(addr));
  if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    cerr << "[Client] Failed to connect to RPi: " << strerror(errno) << endl;
    close(sock);
    return;
  }
  send(sock, "REGISTER_FOR_PULLCORD", 23, 0);
  char ack[8] = {0};
  recv(sock, ack, 7, 0);
  cout << "[Client] Registered: " << ack << "\n";
  close(sock);
}

void RobotClient::waitForCordSignal(int listenPort) {
  inserted = false;
  pulled = false;

  int serverSock = socket(AF_INET, SOCK_STREAM, 0);
  if (serverSock < 0) {
    cerr << "[Client] Failed to create socket\n";
    return;
  }

  int opt = 1;
  setsockopt(serverSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(listenPort);

  if (::bind(serverSock, (sockaddr*)&addr, sizeof(addr)) < 0) {
    cerr << "[Client] Failed to bind to port " << listenPort << ": " << strerror(errno) << endl;
    close(serverSock);
    return;
  }

  listen(serverSock, 1);

  cout << "[Client] Waiting for cord signals...\n";

  while (!inserted || !pulled) {
    int clientSock = accept(serverSock, nullptr, nullptr);
    if (clientSock < 0) {
      cerr << "[Client] Failed to accept connection\n";
      continue;
    }

    char buf[32] = {0};
    recv(clientSock, buf, 31, 0);
    close(clientSock);

    string msg(buf);
    cout << "[Client] Received: " << msg << "\n";
    if (msg == "INSERTED") inserted = true;
    if (msg == "PULLED" && inserted) pulled = true;
  }

  cout << "[Client] Cord pull confirmed. Proceeding.\n";
  close(serverSock);
}

void RobotClient::sendSimasCommand(uint8_t targetID, uint8_t command, const vector<uint8_t>& payload) {
  vector<uint8_t> packet;

  packet.push_back(0x12);
  packet.push_back(command);
  packet.push_back(targetID);

  for (uint8_t b : payload) {
    packet.push_back(b);       // data[1] to data[7] (max 7 bytes)
  }

  while (packet.size() < 10) packet.push_back(0); // pad to 10 bytes

  sendToServer(packet);
}
