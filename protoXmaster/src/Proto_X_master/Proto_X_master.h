#ifndef PROTO_X_MASTER_H
#define PROTO_X_MASTER_H
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include "serial/c_file/serial.cc"

// #define DEBUG_test
#define DISPLAY_ERROR

enum PROTOCOL_ERROR_CODE
{
    NO_ERROR = 0x00,
    RESUILT_FAIL = 0x01,
    INSTRUCTION_ERROR = 0x02,
    CRC_ERROR = 0x03,
    DATA_RANGE_ERROR = 0x04,
    DATA_SECTION_ERROR = 0x05,
    LENGTH_ERROR = 0x08,
    ////// MASTER detect error from slave /////
    INSTRUCTION_SLAVE_ERROR = 0x14,
    CRC_SLAVE_ERROR = 0x15,
    TIMEOUT_ERROR = 0x20,
};

//! ProtoXmaster class for Unix to communication on serial com port to ProtoXslave.
/*!
    @author Kraiwit Trisaksri
    @date 13 Jan 2020

    \par Description of Protocol Packet
    
    <b>Instruction Packet</b> ( master send to slave )
     Header1 | Header2 | Length | Instruction | Param(eter)1 | ... | Param(eter)N | CRC_HighByte | CRC_LowByte 
     -------- | ------- | ------ | ----------- | ------------ | --- | ------------ | ------------ | -----------
      0xFF   |  0xFF   | Length | Instruction |    Param1    | ... |    ParamN    |     CRC_H    |    CRC_L   
    \arg Header1     =   0xFF
    \arg Header2     =   0xFF
    \arg Length      =   Bytes of \a Instruction(1) + \a Param(N) + \a CRC(2)
    \arg Instruction default list :<br> 
    &ensp; &emsp; \b 0x01 -> ping to slave.The ping protocol has not \a Param(eter) <br>
    &ensp; &emsp; \b 0x02 -> read data from slave. <br>
    &emsp; &emsp; &emsp; &emsp; &emsp; \a Param1 = starter address to read. <br>
    &emsp; &emsp; &emsp; &emsp; &emsp; \a Param2 = length of the data to read. <br>
    &ensp; &emsp; \b 0x03 -> write data to slave. <br>
    &emsp; &emsp; &emsp; &emsp; &emsp; \a Param1 = starter address to write. <br>
    &emsp; &emsp; &emsp; &emsp; &emsp; \a Param2 to \a ParamN = data to write (byte per byte). <br>

    <b>Status Packet</b> ( slave reply to master )
     Header1 | Header2 | Length | Instruction | Error | Begin Addr | Param(eter)1 | ... | Param(eter)N | CRC_HighByte | CRC_LowByte 
    -------- | ------- | ------ | ----------- | ----------------- | ------------ | --- | ------------ | ------------ | -----------
      0xFF   |  0xFF   | Length | Instruction |<b> Error code </b>| Begin Addr |   Param1    | ... |    ParamN    |     CRC_H    |    CRC_L   
    \arg Header1     =   0xFF
    \arg Header2     =   0xFF
    \arg Length      =   Bytes of \a Instruction(1) + \a Error(1) + \a BeginAddr(1) + \a Param(N) + \a CRC(2)
    \arg Instruction =   0x55 (Return Status Instruction)
    \arg Error_code  =   @b PROTOCOL_ERROR_CODE // user can modify //
    \arg Param(eter) :<br>
    &ensp; &emsp; @b read => the param is the data in the starter address to the last of length to read.<br>
    &ensp; &emsp; @b ping/write => the status protocol has not @a Param(eter).

    @b PROTOCOL_ERROR_CODE default code // user can modify //
    \arg 0x00 => NO_ERROR
    \arg 0x01 => RESUILT_FAIL
    \arg 0x02 => INSTRUCTION_ERROR
    \arg 0x03 => CRC_ERROR (crc from master error)
    \arg 0x04 => DATA_RANGE_ERROR
    \arg 0x05 => DATA_SECTION_ERROR
    \arg 0x08 => LENGTH_ERROR
    \arg 0x14 => INSTRUCTION_SLAVE_ERROR
    \arg 0x15 => CRC_SLAVE_ERROR
    \arg 0x20 => TIMEOUT_ERROR
*/

template <class Table>
class ProtoXmaster
{
public:
    /** @brief Construction ProtoXmaster Object
     * @param T         pointer to Obj \<Table\> that we use to be register table of the ProtoXslave 
     * */
    // ProtoXmaster(Table *T, std::string port, int baudrate = 9600);
    ProtoXmaster(Table *T);

    /** @brief set serial port of ProtoXmaster
     * @param port      name of serial port that want to communication with ProtoXslave Protocol -> example "/dev/ttyACM0" or "/dev/ttyUSB0"
     * @param baudrate  buadrate of serial default = 115200
     * */
    void setPort(std::string port, int baudrate = 115200);

    /** @brief begin connect to serial com port
     * @return 0 = Connection successful.
     * @return 1 = Connection fail.
     * */
    bool begin();

    /** @brief reconnecting to serial com port until connect successful
     * @param reconnect_time_us delay time before reconnect again (unit: microseconds) default = 5000000 microseconds
     * */
    void begin_until_connected(unsigned long reconnect_time_us = 5000000);

    /** @brief get auto return from protoXslave to protoXmaster table.
     * */
    void get_auto_return_toTable();

    /** @brief send ping protocol in serial communication port to slave
     * @return @b PROTOCOL_ERROR_CODE [if return 0 => NO_ERROR]
     * */
    uint8_t send_ping();

    /** @brief send ping protocol in serial communication port to slave
     * */
    void send_ping_no_return();

    /** @brief send protocol to read data from slave and save the data to the Table
     * @param start_address start address to read data
     * @param size size of data that want to read (unit: byte(s))
     * @return @b PROTOCOL_ERROR_CODE [if return 0 => NO_ERROR]
     * */
    uint8_t send_read_data_toTable(uint8_t start_address, uint8_t size);

    /** @brief send protocol to write data to slave Table
     * @param start_address start address to write data
     * @param data pointer to the data that want to write :::: example uint8_t data[4] = {0xE8, 0x03, 0x00, 0x00}
     * @param size size of data :::: example size = 4;
     * @return @b PROTOCOL_ERROR_CODE [if return 0 => NO_ERROR]
     * */
    uint8_t send_write_data(uint8_t start_address, uint8_t *data, uint8_t size_data);

    /** @brief send protocol with new instruction that slave know what the code of instruction
     * @param new_inst new instruction code <b>[except 0x01(ping), 0x02(read), 0x03(write)]</b>
     * @param param pointer to param in the protocol packet
     * @param size size of param
     * @return @b PROTOCOL_ERROR_CODE [if return 0 => NO_ERROR]
     * */
    uint8_t send_new_instruction(uint8_t new_inst, uint8_t *param, uint8_t size_param);

    /** @brief get the status packet that is returned by slave
     * @param dest destination to contain the status packet
     * @return size of present _statusPacket
     * */
    uint8_t get_statusPacket(uint8_t *dest); // for use in new callback

    /** @brief get the address of the input(X) on the Table
     * @param data pointer to the new struct
     * @return address of the @p data in the Table
     * */
    template <class X>
    uint8_t get_address(X *data);
    bool    _rx_ready;

private:
    /** @brief prepare the _instructionPacket and transmitting protocol to the serial port
     * */
    void transmit_InstructionPacket();

    /** @brief receiving protocol from serial port and check the CRC and contain the correct protocol to _statusPacket
     * */
    void receive_StatusPacket();
    void read_serial_to_buffer();

    /** @brief compute the CRC of the data
     * @param data_blk_ptr pointer to the data[] that want to compute the CRC
     * @param data_blk_size size of the data[]
     * @return CRC in uint16_t(2 bytes)
     * @note if the data have the 2 last bytes is correct CRC ==> function will return 0
     * */
    uint16_t update_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size);

    /** @brief display the error
     * @param e @b PROTOCOL_ERROR_CODE
     * @note need to define DISPLAY_ERR to enable this function
     * */
    void display_err(PROTOCOL_ERROR_CODE e);

    serial::Serial  *_ser;              /*!< pointer to Serial Interface*/
    std::string     _port;              /*!< serial com port directory :::: example _port = "/dev/ttyACM0"*/
    int             _baud;              /*!< baudrate of serial communication*/

    uint8_t *_starter_table;            /*!< pointer to address of the first byte of Table(register_table) object*/
    uint8_t _instructionPacket[256];    /*!< buffer to prepare InstructionPacket is sent by master to slave*/
    uint8_t _statusPacket[256];         /*!< contain StatusPacket is returned by slave to master*/
    uint8_t _size_table;                /*!< size of struct \<Table\>*/
};
#endif
