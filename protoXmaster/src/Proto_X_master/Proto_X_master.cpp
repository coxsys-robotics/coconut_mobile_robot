#include "Proto_X_master.h"

template <class Table>
ProtoXmaster<Table>::ProtoXmaster(Table *T)
{
    _starter_table = (uint8_t *)T;
    _size_table = sizeof(*T);
#ifdef DEBUG_test
    std::cout << "Table size: " << std::dec << 0 + _size_table << " Bytes" << std::endl;
#endif
    if (sizeof(*T) > 251)
    {
        std::cerr << "\033[0;31m"
                  << "[ERROR]size of Table must be <= 251 bytes."
                  << "\033[0m" << std::endl;
        exit(1);
    }
    _statusPacket[0] = 0xFF;
    _statusPacket[1] = 0xFF;
    _rx_ready = false;
}

template <class Table>
void ProtoXmaster<Table>::setPort(std::string port, int baudrate)
{
    _port = port;
    _baud = baudrate;
}

template <class Table>
bool ProtoXmaster<Table>::begin()
{
    try
    {
        std::cout << "Connecting... to port "
                  << "\033[0;33m"
                  << "\e[1m"
                  << "\"" << _port << "\""
                  << "\033[0m"
                  << "\e[0m"
                  << "\tbaudrate: "
                  << "\e[1m" << _baud << "\e[0m" << std::endl;
        this->_ser = new serial::Serial(_port, _baud, serial::Timeout(5, 1000, 0, 5, 0));
        if (_ser->isOpen())
        {
            std::cout << "-> "
                      << "\033[0;32m"
                      << "Connection successful."
                      << "\033[0m" << std::endl;
            _ser->flush();
            while (_ser->available())
            {
                _ser->read(1);
            }
            _rx_ready = true;
            return 0;
        }
    }
    catch (serial::IOException &err)
    {
        std::cerr << "\033[0;31m" << err.what() << std::endl;
        std::cout << "[ERROR] Connection fail, Check your port " << _port << "."
                  << "\033[0m" << std::endl;
    }
    return 1;
}
template <class Table>
void ProtoXmaster<Table>::begin_until_connected(unsigned long reconnect_time_us)
{
    float sec = (float)reconnect_time_us / 1000000.0f;
    clock_t timer;
    while (this->begin())
    {
        std::cout << "Reconnecting... in " << sec << " second(s)." << std::endl;
        timer = (clock() / CLOCKS_PER_SEC) * 1000000;
        while ((clock() / CLOCKS_PER_SEC) * 1000000 - timer <= reconnect_time_us)
            ;
    }
}

template <class Table>
void ProtoXmaster<Table>::get_auto_return_toTable()
{
    this->read_serial_to_buffer();
    //// save data to the table ////
    if (_statusPacket[4] != 0x00)
    {
        display_err((PROTOCOL_ERROR_CODE)_statusPacket[4]);
    }
    else
    {
        if (_statusPacket[1] == 0x00)
        {
            display_err(LENGTH_ERROR);
        }
        else
        {
            if ((_statusPacket[2] == 0x04) &&
                (_statusPacket[3] == 0x55) &&
                (_statusPacket[4] == 0x00) &&
                (_statusPacket[5] == 0xFE) &&
                (_statusPacket[6] == 0x56))
            //////// FF FF 04 55 00 FE 56 >>> slave send ping return
            {
#ifdef DEBUG_test
                std::cout << "\033[0;34m"
                          << ">> slave had send ping return : FF FF 04 55 00 FE 56"
                          << "\033[0m" << std::endl;
#endif // DEBUG_test
            }
            else
            {
                if (((_statusPacket[5] + _statusPacket[2]) - 4) < _size_table)
                {
                    uint8_t start_address = _statusPacket[5];
                    uint8_t statusP[256];
                    uint8_t *p;
                    p = _starter_table + start_address;
                    uint8_t N = (get_statusPacket(statusP)) - 8; // -2(Header) -1(Length) -1(Instruction) -1(Error) -1(Start_Addr) -2(CRC)
                    for (uint8_t i = 0; i < N; i++)
                        *(p + i) = *(statusP + 6 + i); // +2(Header) +1(Lenght) +1(Instruction) +1(Error) +1(Start_Addr)
                }
            }
        }
    }
}

template <class Table>
uint8_t ProtoXmaster<Table>::send_ping()
{
    _instructionPacket[0] = 0x03; // LENGTH
    _instructionPacket[1] = 0x01; // INSTRUCTION ping
    this->transmit_InstructionPacket();
    this->receive_StatusPacket();
    return _statusPacket[4]; // return ERROR_CODE
}

template <class Table>
void ProtoXmaster<Table>::send_ping_no_return()
{
    _instructionPacket[0] = 0x03; // LENGTH
    _instructionPacket[1] = 0x01; // INSTRUCTION ping
    this->transmit_InstructionPacket();
}

template <class Table>
uint8_t ProtoXmaster<Table>::send_read_data_toTable(uint8_t start_address, uint8_t size)
{
    _instructionPacket[0] = 0x05;
    _instructionPacket[1] = 0x02; // Read instruction
    _instructionPacket[2] = start_address;
    _instructionPacket[3] = size;
    this->transmit_InstructionPacket();
    this->receive_StatusPacket();
    if (_statusPacket[4] == 0x00)
    {
        uint8_t start_addr = _statusPacket[5];
        uint8_t statusP[256];
        uint8_t *p;
        p = _starter_table + start_addr;
        uint8_t N = (get_statusPacket(statusP)) - 8; // -2(Header) -1(Length) -1(Instruction) -1(Error) -1(Start_Addr) -2(CRC)
        for (uint8_t i = 0; i < N; i++)
            *(p + i) = *(statusP + 6 + i); // +2(Header) +1(Lenght) +1(Instruction) +1(Error) +1(Start_Addr)
    }
    return _statusPacket[4]; // return ERROR_CODE
}

template <class Table>
uint8_t ProtoXmaster<Table>::send_write_data(uint8_t start_address, uint8_t *data, uint8_t size_data)
{
    _instructionPacket[0] = size_data + 4; // +1(inst) +1(param1/address) +2(CRC)
    _instructionPacket[1] = 0x03;          // Write instruction
    _instructionPacket[2] = start_address;
    for (uint8_t i = 0; i < size_data; i++)
        _instructionPacket[i + 3] = *(data + i);
    this->transmit_InstructionPacket();
    return 0;
}

template <class Table>
uint8_t ProtoXmaster<Table>::send_new_instruction(uint8_t new_inst, uint8_t *param, uint8_t size_param)
{
    _instructionPacket[0] = size_param + 3; // + 1(Instruction) + 2(CRC)
    _instructionPacket[1] = new_inst;
    for (uint8_t i = 0; i < size_param; i++)
        _instructionPacket[i + 2] = *(param + i);
    this->transmit_InstructionPacket();
    this->receive_StatusPacket();
    return _statusPacket[4]; // return ERROR_CODE
}

template <class Table>
uint8_t ProtoXmaster<Table>::get_statusPacket(uint8_t *dest) // for use in new callback
{
    uint8_t N = _statusPacket[2] + 3; // +2(Header) +1(Length)
    memcpy(dest, &_statusPacket[0], N);
    return N;
}

template <class Table>
template <class X>
uint8_t ProtoXmaster<Table>::get_address(X *data)
{
    return (uint8_t *)data - _starter_table;
}

//#######################################################################################################//
//######################################### Private Function ############################################//
//#######################################################################################################//
template <class Table>
void ProtoXmaster<Table>::transmit_InstructionPacket()
{
    _ser->flushInput();
    std::vector<uint8_t> cmd;
    cmd.push_back(0xFF);
    cmd.push_back(0xFF);
    cmd.push_back(_instructionPacket[0]);
    cmd.push_back(_instructionPacket[1]);
    for (uint8_t i = 2; i < (_instructionPacket[0] - 1); i++)
        cmd.push_back(_instructionPacket[i]);

    uint16_t crc = this->update_crc(&cmd[2], cmd.size() - 2);
    cmd.push_back((crc >> 8) & 0xFF);
    cmd.push_back(crc & 0xFF);
    _ser->write(cmd);
#ifdef DEBUG_test
    std::cout << "\033[0;33m"
              << "transmit <<< "
              << "\033[0m" << std::uppercase;
    for (int i = 0; i < cmd.size(); ++i)
        std::cout << std::hex << (int)cmd[i] << " ";
    std::cout << "\n";
#endif
}

template <class Table>
void ProtoXmaster<Table>::receive_StatusPacket()
{
    this->read_serial_to_buffer();
#ifdef DISPLAY_ERROR
    display_err((PROTOCOL_ERROR_CODE)_statusPacket[4]);
#endif
}

template <class Table>
void ProtoXmaster<Table>::read_serial_to_buffer()
{
    try
    {
        uint8_t _temp_buffer[256];
        this->_rx_ready = false;
        if (_ser->waitReadable())
        {
            if (uint8_t(*_ser->read(1).c_str()) == 0xFF)
            {
                if (uint8_t(*_ser->read(1).c_str()) == 0xFF)
                {
                    _temp_buffer[0] = uint8_t(*_ser->read(1).c_str());
                    // read protocol contain to _temp_buffer[1] ... _temp_buffer[n] -> number _temp_buffer[0] bytes
                    uint8_t size = _ser->read(&_temp_buffer[1], _temp_buffer[0]) + 1; // +1 Length
#ifdef DEBUG_test
                    std::cout << "\033[0;33m"
                              << "receive >>> "
                              << "\033[0m"
                              << "FF FF ";
                    for (int i = 0; i < size; i++)
                        std::cout << std::hex << std::uppercase << (int)_temp_buffer[i] << " ";
                    std::cout << std::endl;
#endif
                    if ((_temp_buffer[0] >= 3) && (_temp_buffer[0] <= 250))
                    {
                        uint16_t crc = update_crc(_temp_buffer, size);
                        if (!crc)
                        {
                            if (_temp_buffer[1] == 0x55) // 0x55 = instruction return status from slave
                            {
                                if (_temp_buffer[2] == 0x00) // NO_ERROR
                                {
                                    memcpy(&_statusPacket[2], &_temp_buffer[0], size);
#ifdef DEBUG_test
                                    std::cout << "\033[0;32m"
                                              << "+ CRC is Correct. +"
                                              << "\033[0m" << std::endl;
#endif
                                }
                                else
                                {
                                    _statusPacket[4] = _temp_buffer[2]; // set for return
                                }
                            }
                            else // Instruction from slave error
                            {
                                _statusPacket[4] = (uint8_t)INSTRUCTION_SLAVE_ERROR; // set for return
                            }
                        }
                        else // CRC from statusPacket error
                        {
                            _statusPacket[4] = (uint8_t)CRC_SLAVE_ERROR; // set for return
                        }
                    }
                    else
                    {
                        _statusPacket[4] = (uint8_t)LENGTH_ERROR; // set for return
                        _ser->flushInput();                       // serial flush input
                        std::cout << "--------- Flush input -----------" << std::endl;
                    }
                }
            }
        }
        else // timeout
        {
            _statusPacket[4] = (uint8_t)TIMEOUT_ERROR; // set for return
        }
        this->_rx_ready = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[0;31m" << e.what() << "\033[0m" << std::endl;
        exit(EXIT_FAILURE);
    }
}

template <class Table>
uint16_t ProtoXmaster<Table>::update_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t crc_accum = 0;
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

template <class Table>
void ProtoXmaster<Table>::display_err(PROTOCOL_ERROR_CODE e)
{
    switch (e)
    {
    case NO_ERROR:
        break;
    case RESUILT_FAIL:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Result fail, slave cannot compute the protocol."
                  << "\033[0m" << std::endl;
        break;
    case INSTRUCTION_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Instruction from master error."
                  << "\033[0m" << std::endl;
        break;
    case CRC_ERROR:
        std::cerr << "\031[0;31m"
                  << "[ERROR]CRC from master is incorrect."
                  << "\033[0m" << std::endl;
        break;
    case DATA_RANGE_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Data range error.Cannot access the address out of the table."
                  << "\033[0m" << std::endl;
        break;
    case DATA_SECTION_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Data section error.More than 1 section in 1 protocol or unknown section."
                  << "\033[0m" << std::endl;
        break;
    case LENGTH_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Length error.Length in protocol is under(length = 0) or over."
                  << "\033[0m" << std::endl;
        break;
        //////////////////////////////////////////// Master detect error from slave //////////////////////////////////////////////////
    case INSTRUCTION_SLAVE_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Instruction from slave is incorrect."
                  << "\033[0m" << std::endl;
        break;
    case CRC_SLAVE_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]CRC from slave is incorrect."
                  << "\033[0m" << std::endl;
        break;
    case TIMEOUT_ERROR:
        std::cerr << "\033[0;31m"
                  << "[ERROR]Serial timeout, slave is not responding."
                  << "\033[0m" << std::endl;
        break;
    default:
        std::cerr << "\033[7;31m"
                  << "[ERROR]Unknown PROTOCOL_ERROR_CODE."
                  << "\033[0m" << std::endl;
        break;
    }
}