/* Driver node that communicates with an Otto Bock SensorHand Speed 
 * prosthetics device.
 * 
 * Institute for Robotics (ROBIN), JKU Linz
 * Alexander Reiter
 * January 2014
 * 
 * This node establishes a serial connection with an Otto Bock 
 * SensorHand Speed prosthetics device and allows to issue gripping 
 * commands and receive feedback.
 * 
 * The device offers multiple modes of operation:
 *  0: gripping with soft grip fastening
 *  1: hand stops after first grip
 *  2: maximum velocity, maximum force will be applied
 *  3,4: undocumented
 * 
 * PARAMETERS:
 *  port (str): name of serial device (default /dev/ttyUSB0)
 *  baud_rate (int): baud rate of serial interface (needs to be 19200 = default)
 * 
 * PUBLISHED TOPICS:
 *  hand_dump (hex_serial::hand_dump): topic on which feedback is published
 * 
 * SUBSCRIBED TOPICS:
 *  none
 * 
 * OFFERED SERVICES:
 *  hand_srv (hex_serial::hand_srv): offers access to device functionality
*/

#include <stdio.h>
#include <vector>
#include <math.h> 
#include <sstream>

#include <ros/ros.h>
#include <cereal_port/CerealPort.h>

#include "hex_serial/hand_cmd.h"
#include "hex_serial/hand_dump.h"

#define TIMEOUT 100

// structs and enums
enum Command_id {START_DUMP = 0, STOP_DUMP = 1, OPEN = 2, CLOSE = 3, STOP = 4, GET_SN = 5, RESET = 6, PRGM = 7};
typedef struct {
    Command_id id;
    double param;
    char* response; // data vector of expected response
} Command;

enum State {IDLE, READ_FIRST, READ_LEN, READ_DATA, READ_CS} state;

// function declarations
bool readByte(cereal::CerealPort* device, char* data);
bool sendCommand(cereal::CerealPort* device, Command* cmd);
bool srv_callback(hex_serial::hand_cmd::Request& request, hex_serial::hand_cmd::Response& response);
void publish_dump(ros::Publisher& dump_pub, char* data);


// vector of pending commands
std::vector<Command> pending_cmd;
bool dump_mode = false;

/* NOTE:
 * If a message that is being sent by the hand is interrupted by a 
 * command, the message will be re-sent completely before the response 
 * to the command will be sent.
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "hand_node");
  ros::NodeHandle n("~"); // local node handle for accessing local parameters

  cereal::CerealPort device;  // instance of CerealPort

  // init service for controlling the hand device
  ros::ServiceServer service = n.advertiseService("hand_srv", srv_callback);
  
  // init publisher for dump values
  ros::Publisher dump_pub = n.advertise<hex_serial::hand_dump>("hand_dump", 1000);
  
  std::string port;
  if (!n.hasParam("port")) {
    ROS_WARN("Parameter port not received, using default value /dev/ttyUSB0");
  }
  n.param<std::string>("port", port, "/dev/ttyUSB0");

  int baud_rate;
  if (!n.hasParam("baud_rate")) {
    ROS_WARN("Parameter baud_rate not received, using default value 19200");
  }
  n.param<int>("baud_rate", baud_rate, 19200);

  try { 
    device.open(port.c_str(), baud_rate); 
  }
  catch(cereal::Exception& e) {
      ROS_FATAL("Unable to open the serial port %s.", port.c_str());
      ROS_BREAK();
  }
  ROS_INFO("The serial port %s is opened at %d baud", port.c_str(), baud_rate);

  // prepare variables for reading data from the serial connection
  char reply; // received byte
  char* data; // data vector
  size_t data_pos = 0;  // position in data vector
  uint8_t data_len = 0; // length of expected data stream
  char cs = 0x00; // check sum
  
  // give ROS some time to sync
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
  state = IDLE;
  ROS_INFO("state IDLE");
  
  
  Command cmd;  // declare a Command instance for various purposes
  
  // flush serial connection buffer
  device.flush();
  // reset hand electronics
  cmd.param = 0.;
  cmd.id = RESET;
  //pending_cmd.push_back(cmd);
  sendCommand(&device, &cmd);
  device.flush();
  
  bool dump_msg = false;
  bool busy = false;
  bool waiting_for_dump;
  
  ros::Rate r(100);
  while(ros::ok()) {
    if(!pending_cmd.empty() && !busy && !waiting_for_dump) {  // commands pending
      device.flush();
      sendCommand(&device, &(pending_cmd[0]));
      state = READ_FIRST;
      busy = true;
    }
    switch(state) {
      case READ_FIRST: // check for dump symbol
        if(readByte(&device, &reply)) {
          if((uint8_t) reply == 0xff) { // dump symbol
            state = READ_LEN;
            ROS_INFO("state READ_LEN");
            dump_msg = true;
          } else {
            data_len = (uint8_t) reply;
            ROS_INFO("data_len is %u", data_len);
            if(!data) free(data);
            data = (char*) calloc(data_len, sizeof(char));
            data_pos = 0;
            cs = reply; // message length is counted in check sum
            state = READ_DATA;
            ROS_INFO("state READ_DATA (from READ_FIRST)");
            dump_msg = false;
          }
        }
        break;
      case READ_LEN:  // read size of message length
        if(readByte(&device, &reply)) {
          data_len = (uint8_t) reply;
          ROS_INFO("data_len is %u", data_len);
          if(!data) free(data);
          data = (char*) calloc(data_len, sizeof(char));
          data_pos = 0;
          cs = reply; // message length is counted in check sum
          state = READ_DATA;
          ROS_INFO("state READ_DATA (from READ_LEN)");
        }
        break;
      case READ_DATA: // read data
        if(readByte(&device, &reply)) { 
          if(data_pos < data_len - 1) { // check sum takes up last position in received frame and is counted in data_len
            data[data_pos] = reply;
            data_pos++;
            cs = cs + reply;
            if(data_pos == data_len - 1) {
              state = READ_CS;
              ROS_INFO("state READ_CS");
            }
          } 
        }
        break;
      case READ_CS: // read and check check sum
        if(readByte(&device, &reply)) { 
          if(reply != cs) {
            ROS_WARN("Check sum incorrect, received data will be discharded");
            data_len = 0;
            free(data);
            break;
          }
          if(data_len == 2 && data[0] == 0x73) {  // dump start command confirmed
            dump_mode = true;
            waiting_for_dump = true;
            state = READ_FIRST;
            ROS_INFO("state READ_FIRST");
          }
          if(data_len == 2 && data[0] == 0x7a) {  // dump stop command confirmed
            dump_mode = false;
            ROS_INFO("DUMP MODE OFF");
          }
          if(dump_msg && dump_mode) {
            publish_dump(dump_pub, data);
            cmd.id = STOP_DUMP;
            sendCommand(&device, &cmd);
            ros::Duration(0.05).sleep();
            device.flush();
            cmd.id = START_DUMP;
            pending_cmd.push_back(cmd);
            waiting_for_dump = false;
          } else {
            if(data_pos+1 != (uint8_t) pending_cmd[0].response[0]) {
              ROS_WARN("Response length of %u incorrect, expected %u", (uint8_t) pending_cmd[0].response[0], data_pos+1 );
            } else {
              for(size_t i = 0; i < data_pos; i++) {
                if(data[i] != pending_cmd[0].response[i+1]) {
                  ROS_WARN("Response data %02x incorrect, expected %02x", (uint8_t) data[i], (uint8_t) pending_cmd[0].response[i] );
                }
              }
            }
            pending_cmd.erase(pending_cmd.begin());
          }
          if(!dump_mode) {
            state = IDLE;
            ROS_INFO("state IDLE");
            // remove all START_DUMPs
            for(size_t i = 0; i < pending_cmd.size(); i++) {
              if(pending_cmd[i].id == START_DUMP) {
                pending_cmd.erase(pending_cmd.begin() + i);
                i--;
              }
            }
            
          }
          ROS_INFO("Received data is:");
          for(size_t i = 0; i < data_pos; i++) {
            ROS_INFO("%02x", (uint8_t) data[i]);
          }
          busy = false;
          
          free(data);
        }
        break;
      default:
        break;
    }
    
    ros::spinOnce();
    r.sleep();
  }
  free(data);
  return 0;
}

/* Function that reads a byte from the serial port device and stores it
 * in data if received correctly.
 * 
 * INPUT:   cereal::CerealPort &device: pointer to serial port device instance
 *          char* data: pointer to data
 * OUTPUT:  bool: true if transmission was successful, false otherwise
 */
bool readByte(cereal::CerealPort* device, char* data) {
  try {
    device->readBytes(data, 1, TIMEOUT); // timeout in ms
  } catch(cereal::TimeoutException& e) {
      ROS_ERROR("Timeout while reading from serial port.");
      return false;
  }
  ROS_INFO("Received %02x", (uint8_t) *data);
  return true;
}

/* Function that sends a pre-defined command over the serial  
 * interface device. The command may incorporate additional information 
 * which has to be provided in additional arguments (e.g. speed for 
 * opening or closing the hand).
 * 
 * INPUT:   cereal::CerealPort &device: pointer to serial port device instance
 *          Command* cmd: pointer to command
 * OUTPUT:  bool:   true if transmission successful, otherwise false
 */
bool sendCommand(cereal::CerealPort* device, Command* cmd) {
  uint8_t cmd_size = 0;  // length of command excluding length symbol, including check sum
  
  char cmd_buffer[5]; // longest command is size 6 (including length symbol and check sum)
  
  // longest reply is of length 8 (serial number; including length, excluding check sum)
  // first element will hold length of expected reply (excluding length symbol), comparable to first symbol of actual reply
  cmd->response = (char*) calloc(8, sizeof(char));
  
  switch(cmd->id) {
    case START_DUMP:
      cmd_size = 2;
      cmd_buffer[1] = 0x73;
      //state = READ_FIRST;
      //ROS_INFO("state READ_FIRST");
      
      cmd->response[0] = 2;
      cmd->response[1] = cmd_buffer[1];
      
      break;
    case STOP_DUMP:
      cmd_size = 2;
      cmd_buffer[1] = 0x7a;
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 2;
      cmd->response[1] = cmd_buffer[1];
      
      // dump mode will be turned off after stop command response was received
      break;
    case OPEN:
      cmd_size = 4;
      cmd_buffer[1] = 0x75;
      { // calculate speed
      cmd->param = std::min(std::max(fabs(cmd->param), 0.), 1.0);  // saturate v with 0..1
      cmd_buffer[2] = (char) (0x31 + cmd->param*(0x85 - 0x31));
      }
      cmd_buffer[3] = 0x00;
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 3;
      cmd->response[1] = cmd_buffer[2];
      cmd->response[2] = cmd_buffer[3];
      break;
    case CLOSE:
      cmd_size = 4;
      cmd_buffer[1] = 0x75;
      cmd_buffer[2] = 0x00;
      { // calculate speed
      cmd->param = std::min(std::max(fabs(cmd->param), 0.), 1.0);  // saturate v with 0..1
      cmd_buffer[3] = (char) (0x31 + cmd->param*(0x85 - 0x31));
      }
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 3;
      cmd->response[1] = cmd_buffer[2];
      cmd->response[2] = cmd_buffer[3];
      break;
    case STOP:
      cmd_size = 4;
      cmd_buffer[1] = 0x75;
      cmd_buffer[2] = 0x00;
      cmd_buffer[3] = 0x00;
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 3;
      cmd->response[1] = cmd_buffer[2];
      cmd->response[2] = cmd_buffer[3];
      break;
    case GET_SN:
      cmd_size = 2;
      cmd_buffer[1] = 0x76;
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      // this will be different on each hand
      cmd->response[0] = 8;
      cmd->response[1] = 0x31;
      cmd->response[2] = 0x31;
      cmd->response[3] = 0x30;
      cmd->response[4] = 0x2c;
      cmd->response[5] = 0x30;
      cmd->response[6] = 0x2e;
      cmd->response[7] = 0x35;
      break;
    case RESET:
      cmd_size = 2;
      cmd_buffer[1] = 0x6d;
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 2;
      cmd->response[1] = 0x6d;
      break;
    case PRGM:
      cmd_size = 3;
      cmd_buffer[1] = 0x61;
      cmd_buffer[2] = (char) (cmd->param);
      //state = READ_LEN;
      //ROS_INFO("state READ_LEN");
      
      cmd->response[0] = 3;
      cmd->response[1] = 0x61;
      cmd->response[2] = cmd_buffer[2];
      break;
    default:
      ;
  }
  cmd_buffer[0] = cmd_size;
  cmd_buffer[cmd_size] = 0;
  
  // calculate check sum
  for(uint8_t i = 0; i < cmd_size; i++) {
    cmd_buffer[cmd_size] = cmd_buffer[cmd_size] + cmd_buffer[i];
  }
  
  std::stringstream ss;
  ss << "Command";
  char substr[4];
  for(uint8_t i = 0; i <= cmd_size; i++) {
    snprintf(substr, 4, " %02x", cmd_buffer[i]);
    ss << substr;
  }
  if(device->write(cmd_buffer, cmd_size + 1) == cmd_size + 1) {
    ss << " sent successfully";
    ROS_INFO_STREAM(ss.str());
    return true;
  } else {
    ss << " NOT successful";
    ROS_INFO_STREAM(ss.str());
    return false;
  }
  
}

/* Service callback function that is used to receive operation requests 
 * for the hand device. If the communication with the device succeeded,
 * true is returned as the function return value as well as in the 
 * service response data structure.
 * 
 * INPUT: hex_serial::hand_cmd::Request&: pointer to request structure
 *        hex_serial::hand_cmd::Response&: pointer to response structure
 * OUTPUT: bool if communiction with the hand device was successful
 */
bool srv_callback(hex_serial::hand_cmd::Request& request, hex_serial::hand_cmd::Response& response) {
  ROS_INFO("Hand service called");
  
  Command cmd;
  cmd.param = request.param;
  cmd.id = (Command_id) request.cmd;
  pending_cmd.push_back(cmd);
  
  response.success = true;
  return true;
}

/* Function that reads data from the data received from the hand device
 * and converts it from binary to numeric values, stores it in a 
 * hex_serial::hand_dump structure and published the message by means of 
 * the provided publisher.
 * 
 * Conversion information in [brackets]:
 * float32 V_open, V_close # electrode voltages in V [0 .. 85 -> 0 .. 1.5 V]
 * float32 V_bat       # supply voltage in V [140 -> 4.9 V, 238 -> 8.4 V ]
 * uint8 state         # state number of the hand's internal state machine [no conversion]
 * float32 F_bracket   # bracket force in N [0 .. 110 -> 0 .. 120 N ]
 * float32 F_pad{1,2,3}# pad force in N [ no sure, maybe like brackets]
 * float32 I           # supply current in A [ 0 .. 141 -> 0 .. 1.5 A]
 * 
 * INPUT: ros::Publisher& dump_pub: publisher for sending dump data
 *        char* data: data array of size 9
 * OUTPUT:  none
 */
void publish_dump(ros::Publisher& dump_pub, char* data) {
  hex_serial::hand_dump msg;
  msg.V_open = 1.5/85 * (uint8_t) data[0];
  msg.V_close = 1.5/85 * (uint8_t) data[1];
  msg.V_bat = 8.4/238 * (uint8_t) data[2];
  msg.state = (uint8_t) data[3];
  msg.F_bracket = 120./110 * (uint8_t) data[4];
  msg.F_pad1 = 120./110 * (uint8_t) data[5];
  msg.F_pad2 = 120./110 * (uint8_t) data[6];
  msg.F_pad3 = 120./110 * (uint8_t) data[7];
  msg.I = 1.5/141 * (uint8_t) data[8];
  
  dump_pub.publish(msg);
}
