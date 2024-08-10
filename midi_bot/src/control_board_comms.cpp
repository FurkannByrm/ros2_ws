#include "midi_bot/control_board_comms.hpp"
#include "midi_bot/wheel.hpp"

LibSerial::BaudRate ControlBoardComms::convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
    std::cout<<"Error! Baud rate " << baud_rate << " not supported! Default to 57600";  
    return LibSerial::BaudRate::BAUD_57600;
  }
}

void ControlBoardComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  timeout_ms_ = timeout_ms_;
  serial_conn_.Open(serial_device);
  serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

void ControlBoardComms::disconnect()
{
  serial_conn_.Close();
}

bool ControlBoardComms::connected() const
{
  return serial_conn_.IsOpen();
}

std::string ControlBoardComms::send_msg(const std::string &msg_to_send, bool print_output)
{
  serial_conn_.FlushIOBuffers();
  serial_conn_.Write(msg_to_send);
  std::string response = "";
  try
  {
    serial_conn_.ReadLine(response, '\n', timeout_ms_);
  }

  catch(const LibSerial::ReadTimeout&)
  {
    std::cout<<"the ReadyByte() call has timed out."<<std::endl;
  }

  if(print_output)
  {
    std::cout<<"sent: "<<msg_to_send<<"Recv: "<<response <<std::endl;
  }
  return response; 
}

void ControlBoardComms::send_empty_msg()
{
  std::string response = send_msg("\r", true); 
}
void ControlBoardComms::read_encoder_values(int &val_1, int &val_2)
{
  std::string response = send_msg("e\r", true);
  std::string delimeter = " ";
  size_t del_pos = response.find(delimeter);
  std::string token_1 = response.substr(0,del_pos);
  std::string token_2 = response.substr(del_pos + delimeter.length());

  val_1 = std::atoi(token_1.c_str());
  val_2 = std::atoi(token_2.c_str());
}

void ControlBoardComms::set_motor_values(int val_1, int val_2)
{
  std::stringstream ss;
  ss<<"m "<<val_1<< " "<<val_2<<"\r";
  send_msg(ss.str(),true);
}

void ControlBoardComms::set_pid_values(double k_p, double k_d, double k_i, double k_o)
{
  std::stringstream ss;
  ss<<"u "<<k_p<<":"<<k_d<<":"<<k_i<<":"<<k_o<<":"<<"\r";
  send_msg(ss.str(), true);
}

Wheel::Wheel(const std::string &Wheel_name, int counts_per_rev)
{
  setup(Wheel_name, counts_per_rev);
}

void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
}

double Wheel::calc_enc_angle()
{
  return enc * rads_per_count;
}