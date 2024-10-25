#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cmath>

#include <hiwin_robot_client_library/commander.h>

#define PARA_LEN 249

namespace hrsdk
{

enum class SpaceOperationTypes
{
  Cartesian = 0,
  Joint,
  Tool,
};

enum class CommandType
{
  Get = 0,
  Set,
  MonitorSet,
};

enum class CommandId : uint16_t
{
  GetPermissions = 0x000A,
  EnableRobot = 0x0578,
  SetRobotMode = 0x058C,
  GetRobotMode = 0x058D,
  PtpJoint = 0x07D6,
  GetActualPosition = 0x0866,
  GetActualRPM = 0x0867,
  GetActualCurrent = 0x100A,
  GetHrssMode = 0x1036,
};

struct Commandformat
{
  uint16_t cmd_id;
  uint16_t param[249];
} __attribute__((__packed__));

struct Responseformat
{
  uint16_t cmd_id;
  uint16_t result;
  uint16_t data[248];
} __attribute__((__packed__));

Commander::Commander(const std::string& robot_ip) : robot_ip_(robot_ip)
{
}

Commander::~Commander()
{
}

bool Commander::connect()
{
  if (getState() == socket::SocketState::Connected)
  {
    std::cout << "Socket is already connected. Refusing to reconnect." << std::endl;
    return false;
  }

  if (!TCPClient::setup(robot_ip_, 1503, std::chrono::seconds(10)))
  {
    return false;
  }

  return true;
}

int Commander::getPermissions()
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::GetPermissions);
  w.param[0] = 0;
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  return 0;
}

int Commander::enableRobot()
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::EnableRobot);
  w.param[0] = 1;
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  return 0;
}

int Commander::getActualRPM(double (&velocities)[6])
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::GetActualRPM);
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  int32_t value;
  for (size_t i = 0; i < 6; i++)
  {
    memcpy(&value, ((data_r + 6) + (i * 4)), sizeof(int32_t));
    velocities[i] = value / 1000.0;
  }

  return 0;
}

int Commander::getActualCurrent(double (&efforts)[6])
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::GetActualCurrent);
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  int32_t value;
  for (size_t i = 0; i < 6; i++)
  {
    memcpy(&value, ((data_r + 6) + (i * 4)), sizeof(int32_t));
    efforts[i] = value / 1000.0;
  }

  return 0;
}

int Commander::getActualPosition(double (&positions)[6])
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::GetActualPosition);
  w.param[0] = static_cast<uint16_t>(SpaceOperationTypes::Joint);
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  int32_t value;
  for (size_t i = 0; i < 6; i++)
  {
    memcpy(&value, ((data_r + 6) + (i * 4)), sizeof(int32_t));
    positions[i] = (value / 1000.0) * (M_PI / 180);
  }

  return 0;
}

int Commander::ptpJoint(double (&positions)[6], double ratio)
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::PtpJoint);

  int32_t acceleration_time = 250000;
  memcpy(&w.param[0], &acceleration_time, sizeof(int32_t));

  int32_t ratio_integer = static_cast<int>(std::round(ratio * 1000));
  memcpy(&w.param[2], &ratio_integer, sizeof(int32_t));

  double pos_deg;
  int32_t deg_integer;
  for (size_t i = 0; i < 6; i++)
  {
    pos_deg = positions[i] * (180 / M_PI) * 1000.0;
    std::cout << pos_deg << " ";
    deg_integer = static_cast<int>(std::round(pos_deg));
    memcpy(&w.param[(i * 2) + 4], &deg_integer, sizeof(int32_t));
  }
  std::cout << std::endl;
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);
}

int Commander::getHrssMode()
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::GetHrssMode);
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);

  return 0;
}

int Commander::setRobotMode(RobotMode mode)
{
  size_t written;
  Commandformat w = {};
  const uint8_t* data_w = static_cast<const uint8_t*>(static_cast<void*>(&w));

  w.cmd_id = static_cast<uint16_t>(CommandId::SetRobotMode);
  w.param[0] = static_cast<uint16_t>(mode);
  TCPClient::write(data_w, sizeof(Commandformat), written);

  size_t read_chars;
  Responseformat r = {};
  uint8_t* data_r = static_cast<uint8_t*>(static_cast<void*>(&r));

  TCPClient::read(data_r, sizeof(Responseformat), read_chars);
  std::cout << r.cmd_id << std::endl;
  std::cout << r.data[0] << std::endl;

  return 0;
}

int Commander::getRobotMode(RobotMode& mode)
{
  return 0;
}

}  // namespace hrsdk
