
#define RING_BUFFER_CAPACITY 65

#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <arpa/tftp.h>

#include <iostream>
#include <iomanip>
#include <string>
#include "../PktBuffer.h"

struct TAnalogPin {
    // Поля, к которым доступ осуществляется и из loop и из ISR
    volatile uint8_t  id;      // Номер физического пина
    volatile uint16_t curVal;  // Мгновенное значение пина
    volatile uint16_t maxVal;  // Максимальное значение пина
    volatile uint16_t minVal;  // Минимальное значение пина

    // Поля, к которым доступ осуществляется только из ISR
    uint16_t maxClcVal;        // Временное значение во время вычисления maxVal
    uint16_t minClcVal;        // Временное значение во время вычисления minVal

    TAnalogPin (uint8_t id):
        id(id),
        curVal(0x0000),
        maxVal(0x0000),
        minVal(0x0000),
        maxClcVal(0x0000),
        minClcVal(0xFFFF) {}
};

TAnalogPin aPin[8] = {0, 1, 2, 3, 4, 5, 6, 7};

const std::string PORT {"/dev/ttyUSB0"};
int fd; /* Файловый дескриптор для порта */
char buf[512];/*размер зависит от размера строки принимаемых данных*/
int outa=0; 
int iIn;
 
int open_port(void);
int main(void) {
    fd = open(PORT.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); /*'open_port()' - Открывает последовательный порт */
    if (fd == -1) {
        std::cout << "open_port: Unable to open " <<  PORT << std::endl;
    } else {
        struct termios options; /*структура для установки порта*/
        tcgetattr(fd, &options); /*читает пораметры порта*/
        cfsetispeed(&options, 19200); /*установка скорости порта*/
        cfsetospeed(&options, 19200); /*установка скорости порта*/
        options.c_cflag &= ~PARENB; /*выкл проверка четности*/
        options.c_cflag &= ~CSTOPB; /*выкл 2-х стобит, вкл 1 стопбит*/
        options.c_cflag &= ~CSIZE; /*выкл битовой маски*/
        options.c_cflag |= CS8; /*вкл 8бит*/
        tcsetattr(fd, TCSANOW, &options); /*сохронения параметров порта*/

        TPktBuffer pb{};

        do {
            iIn = read(fd, buf, RING_BUFFER_CAPACITY - 1); /*чтения приходящих данных из порта*/
            if (iIn >= 0) {
                for (int idx = 0; idx < iIn; idx++) {
                    if (!pb.Put(buf[idx])) {
                        std::cout << "End of ring buffer!!!" << std::endl;
                        return 2;
                    }
                }
            } else {
                std::cout << "Read error: " << ((errno == EACCESS) ? "EACCESS" : ((errno == EBADF) ? "EBADF" : "UNKNOWN")) << std::endl;
                return 1;
            }


            uint8_t idType;
            uint8_t idPin;
            uint16_t data;
            while (pb.GetPkt(idType, idPin, data) == true) {
                switch (idType) {
                    case 0x01:
                        aPin[idPin].curVal = data;
                        break;
                    case 0x02:
                        aPin[idPin].minVal = data;
                        break;
                    case 0x03:
                        aPin[idPin].maxVal = data;
                        break;
                }
            }
            if (iIn > 0) {
                for (int idx = 0; idx < 8; idx ++) {
                    std::cout << std::hex << std::setw(4) << idx
                        << ": curVal = " << std::setw(4) << aPin[idx].curVal
                        << "; minVal = " << std::setw(4) << aPin[idx].minVal
                        << "; maxVal = " << std::setw(4) << aPin[idx].maxVal
                        << std::endl;
                }
                for (int idx = 0; idx < 8; idx ++) {
                    if (aPin[idx].maxVal - aPin[idx].minVal > 0x0F) {
                        std::cout << "FALSE RESULT !!!!!!!" << std::endl;
                    }
                }
            }
        } while (true);
    }


return 0;
};
