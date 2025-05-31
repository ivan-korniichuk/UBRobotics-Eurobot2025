#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <atomic>
#include <mutex>

using namespace std;

class RobotClient {
public:
    RobotClient(const string& ip, int port);
    ~RobotClient();

    void sendRawCommand(const vector<uint8_t>& command);
    
    // Example abstraction: move with speed and direction
    // void sendMoveCommand(uint8_t speed , int8_t direction);
    void sendHomingCommand();
    void sendTwistOuterGrippers(bool close);
    void sendOpenUpperGrippers(bool close);
    void sendOpenLowerGrippers(bool close);
    void sendMovePartialCommand();
    void sendMoveFullCommand();
    // lower upper left right grippers
    void sendCustomMoveCommand(float lower, float upper, float left, float right);
    void sendLocomotionCommand(float omega, float vx, float vy, float scalar);
    void sendNewESPMoveCommand(int16_t velX, int16_t velY, int16_t turn);
    void waitForCordSignal(int listenPort = 5050);
    void registerWithRPi();
    void sendSimasCommand(uint8_t targetID, uint8_t command, const vector<uint8_t>& payload);
    void sendGrab(bool grab);
    void sendSeek();
    
private:
    string serverIp;
    int serverPort;
    atomic<bool> inserted{false};
    atomic<bool> pulled{false};
    mutex sendMutex;

    void sendToServer(const vector<uint8_t>& data);
    void sendCommandToESP(uint8_t espID, uint8_t cmdID, const std::vector<uint8_t>& args);
};