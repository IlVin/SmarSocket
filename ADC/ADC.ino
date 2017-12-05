
#define PINLST_SZ 8            // Размер списка пинов
#define PKT_LEN 4              // Длина пакета
#define RING_BUFFER_CAPACITY ((3 * PKT_LEN * PINLST_SZ) + 1)

#include "RingIndex.h"
#include "PktBuffer.h"

#include <limits.h>
#include "Arduino.h"

// http://www.atmel.com/images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet_Complete.pdf
// http://codius.ru/articles/Arduino_%D1%83%D1%81%D0%BA%D0%BE%D1%80%D1%8F%D0%B5%D0%BC_%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D1%83_%D0%BF%D0%BB%D0%B0%D1%82%D1%8B_%D0%A7%D0%B0%D1%81%D1%82%D1%8C_2_%D0%90%D0%BD%D0%B0%D0%BB%D0%BE%D0%B3%D0%BE_%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9_%D0%BF%D1%80%D0%B5%D0%BE%D0%B1%D1%80%D0%B0%D0%B7%D0%BE%D0%B2%D0%B0%D1%82%D0%B5%D0%BB%D1%8C_%D0%90%D0%A6%D0%9F_%D0%B8_analogRead
// http://www.gaw.ru/html.cgi/txt/doc/micros/avr/arh128/12.htm
// https://geektimes.ru/post/263024/
// https://habrahabr.ru/post/321008/
// https://code.google.com/archive/p/arduino-timerone/downloads
// http://wiki.openmusiclabs.com/wiki/ArduinoFHT
// http://masteringarduino.blogspot.ru/2013/11/USART.html

//http://www.avrbeginners.net/architecture/adc/adc.html - Объясняется как переключать ADMUX


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
        maxVal(0xFFFF & 0b0111111111111111),
        minVal(0x0000),
        maxClcVal(0x0000),
        minClcVal(0xFFFF & 0b0111111111111111) {}
};

// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | Регистры | Биты                                                          |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// |          |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | ADCSRA   | ADEN  | ADSC  | ADATE | ADIF  | ADIE  | ADPS[2:0]             |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | ADMUX    |  REFS[1:0]    | ADLAR |   ×   |  MUX[3:0]                     |
// +----------+---------------+-------+-------+-------+-------+-------+-------+
// | ADCSRB   | ADTS[2:0]             |   ×   |   ×   |   ×   |   ×   |   ×   |
// +----------+---------------+-------+-------+-------+-------+-------+-------+
// | ADCL     | Результат преобразования. Младший разряд                      |
// +----------+                                                               +
// | ADCH     | Старшие или младшие - зависит от значения бита ADLAR          |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | SREG     |   I   |   T   |   H   |   S   |   V   |   N   |   Z   |   C   |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// ADEN  (ADC Enable, включение АЦП) — флаг, разрешающий использование АЦП, при сбросе флага во время преобразования процесс останавливается;
// ADSC  (ADC Start Conversion, запуск преобразования) — флаг установленный в 1 запускает процесс преобразования.
//       В режиме Free Running Mode (ADATE=1, ADTS[2:0]=000) этот флаг должен быть установлен в единицу один раз — для запуска первого преобразования, следующие происходят автоматически.
//       Для инициализации АЦП нужно установить ADSC во время или после разрешения АЦП (ADEN)
// ADATE (ADC Auto Trigger Enable) — выбор режима работы АЦП.
//       0 – разовое преобразование (Single Conversion Mode);
//       1 – включение автоматического преобразования по срабатыванию триггера (заданного сигнала).
//           Источник автоматического запуска задается битами ADTS[2:0] регистра ADCSRB.
// ADTS[2:0] Источник запуска преобразования ADC
//       [000] - Постоянное преобразование (Free Running mode)
//       [001] - Аналоговый компаратор
//       [010] - Внешний запрос на прерывание 0
//       [011] - Timer/Counter0 Compare Match
//       [100] - Timer/Counter0 Overflow
//       [101] - Timer/Counter1 Compare Match B
//       [110] - Timer/Counter1 Overflow
//       [111] - Timer/Counter1 Capture Event
// ADPS - ADC Prescaler Select
//      ADPS[2:0]: B011, B100, B101, B110, B111
//      K        :    8,   16,   32,   64,  128
// ADIF (ADC Interrupt Flag) — флаг прерывания от компаратора
// ADIE (ADC Interrupt Enable) — разрешение прерывания от компаратора
//
// REFS[1:0] (Reference Selection Bit) — биты определяют источник опорного напряжения. 
//      REFS[1:0]: B00,      B01,      B11
//      Source   : AREF(~В), AVCC(5В), Internal(2.56В)
// ADLAR (ADC Left Adjust Result) — бит отвечающий за порядок записи битов результата в регистры
//      ADLAR = [0] => ADCL = [7654321], ADCH = [      98]; 10 бит - для точного измерения
//      ADLAR = [1] => ADCL = [10     ], ADCH = [98765432];  8 бит - для отбрасывания младших бит
// MUX[3:0] (Multiplexer Selection Input) — биты выбора аналогового входа.
//      MUX[3:0]: B0000, B0001, B0010, B0011, B0100, B0101, B0110, B0111, B1000, B1110,      B1111
//      Source  : ADC0,  ADC1,  ADC2,  ADC3,  ADC4,  ADC5,  ADC6,  ADC7,  Temp,  VBG(1.1В),  GND(0В)
// SREG.C=1: флаг переноса (CARRY) - указывает на переполнение при выполнении арифметической или логической операции
// SREG.Z=1: нулевой флаг (ZERO) - устанавливается, когда результат арифметической/логической операции равен 0. Если результат не равен 0, то флаг сбрасывается.
// SREG.N=1: флаг отрицательного результат (NEGATIVE) - устанавливается, когда результат арифметической/логической операции меньше 0
// SREG.T  - флаг копирования (Transfer of Copy). Используется программистом по своему усмотрению
// SREG.I=1 - прерывания разрешены


inline void SetupADCSRA() {
    //        ADEN           - Подать питание на АЦП
    //        |ADSC          - Стартовать АЦП c инициализацией
    //        ||ADATE        - Оцифровка по срабатываению триггера ADTS[2:0]
    //        |||ADIF        - флаг прерывания от компаратора
    //        ||||ADIE       - разрешение прерывания от компаратора
    //        |||||ADPS[2:0] - Коэффициент делителя частоты АЦП K = 64
    //        ||||||||
    ADCSRA = B11001110;
}

inline void SetupADCSRB() {
    //        ADTS[2:0]      - Free Running mode
    //        |||xxxxx       - Резервные биты
    //        ||||||||
    ADCSRB = B00000000;
}

inline void SetupADMUX(uint8_t src = 0){
    //       REFS[1:0]        - Источником опорного напряжения является питание
    //       ||ADLAR          - Режим 10 бит
    //       |||x             - Резервный бит
    //       ||||MUX[3:0]     - Источник сигнала
    //       ||||||||
    ADMUX = B01000000 | (src & B00001111);
}

// Аналоговые пины
static TAnalogPin aPin[PINLST_SZ] = {
    0b0000, // A0
    0b0001, // A1
    0b0010, // A2,
    0b0011, // A3,
    0b0100, // A4,
    0b0101, // A5,
    0b0110, // A6,
    0b0111  // A7
};
static TRingIndex aPinIdx {PINLST_SZ};  // Индексатор аналоговых пинов
static uint16_t adcCnt = 0;

inline void StartAdc() {
    SetupADCSRB();
    SetupADMUX(aPin[aPinIdx.idx].id);          // Записываем в буфер текущий пин [n]
    SetupADCSRA();                             // АЦП начал оцифровывать пин [n]
}

/* **** Interrupt service routine **** */
// Биты MUXn и REFS1:0 в регистре ADMUX поддерживают одноступенчатую буферизацию через временный регистр.
// Поэтому изменения вступают в силу в безопасный момент -  в течение одного такта синхронизации АЦП перед оцифровкой сигнала.
// Если выполнено чтение ADCL, то доступ к этим регистрам для АЦП будет заблокирован, пока не будет считан регистр ADCH.
ISR(ADC_vect){
    aPinIdx.Fwd(1);
    SetupADMUX(aPin[aPinIdx.idx].id); // Кладем в буфер номер следующего пина
    SetupADCSRA();                    // АЦП начал оцифровывать пин [n]

    uint8_t lo = ADCL; // Автоматически блокируется доступ АЦП к регистрам ADCL и ADCH
    uint8_t hi = ADCH; // Автоматически разблокируется доступ АЦП к регистрам ADCL и ADCH
    uint8_t idx = ADMUX & 0x0F;

    uint16_t curVal = (hi << 8) | lo;
    uint16_t minVal = aPin[idx].minClcVal;
    uint16_t maxVal = aPin[idx].maxClcVal;
    if (curVal < minVal) {
        aPin[idx].minClcVal = curVal;
    } else if (curVal > maxVal) {
        aPin[idx].maxClcVal = curVal;
    }
    aPin[idx].curVal = curVal;

    if (adcCnt == 4000) {
        aPin[idx].minVal = aPin[idx].minClcVal;
        aPin[idx].maxVal = aPin[idx].maxClcVal;
        aPin[idx].minClcVal = curVal;
        aPin[idx].maxClcVal = curVal;
    } else if (adcCnt >= 4008) {
        adcCnt = 0;
    } else {
        adcCnt++;
    }
}

// Установка режима пинов
inline void SetupPins() {
    for (int8_t pinIdx = PINLST_SZ; pinIdx--;) {
        pinMode(aPin[pinIdx].id, INPUT);
    };
}

/* ******************************
 *        UART Interface        *
 ****************************** */
// UART интерфейс по своей природе асинхронный, работающий с единичными байтами
// Наша задача - организовать передачу телеметрии со SLAVE на MASTER и передачу команд с MASTER на SLAVE
// Предлагается следовать асинхронной природе UART интерфейса и организовать 2 независимых канала передачи данных
// 1) SLAVE -> MASTER - Передача телеметрии в режиме "нон-стоп"
// 2) MASTER -> SLAVE - Передача управляющих команд
// Предлагается не реализовывать диалоговый режим, усложняющий алгоритм взаимодействия, а просто слать пакеты данных

// --=== ПАКЕТ ===--
// Пакет - это определенным образом закодированная последовательность байтов вида:
// [Id][Data1] .. [DataK]
// uint8_t [Id]    - Идентификатор пакета
// uint8_t [Data*] - Байты данных
// Из определения пакета видно, что он состоит из нескольких байт данных.
// Предстоит передавать эту последовательность байт через UART и не понятно с какого байта прочитает пакет
// принимающая сторона. Для синхронизации принимающей и передающей сторон нужен механизм гарантированного
// определения где [Id], а где [Data] байты. Предлагается маркровать байты крайним левым битом:
// [Id      ][Data1   ] .. [DataK   ]
// [1*******][0*******] .. [0*******]
// * - любой бит
// Идея заимствована из стандарта  кодирования UTF-8.

// ---== ПЕРЕДАЧА ТЕЛЕМЕТРИИ от SLAVE к MASTER ==---
// Для передачи показаний АЦП используются пакеты вида [Id][DataHi][DataLo]
// Вся телеметрия представляет собой последовательность из N пакетов:
// [Id1][DataHi1][DataLo1] .. [IdN][DataHiN][DataLoN]
// [Id*] - идентификатор пакета
// [DataHi*][DataLo*] - старший и младший байты значения датчика.
//            Телеметрия передается как знаковое 14 битное число - int16_t
// int16_t VALUE = static_cast<int16_t>( (static_cast<uint16_t>([DataHi]) << 7) | static_cast<uint16_t>([DataLo]) )

// ---== ПЕРЕДАЧА КОМАНД от MASTER к SLAVE ==---
// Телеметрия переданная мастером является командами для SLAVE и имеет тот же формат

// ---== Формат Id ==---
// Id поделен на 2 части: старшую и младшую.
// Первая часть задает тип телеметрии/команды, а вторая номер ноги:
// [1][*][*][*][*][*][*][*]
// [   TYPE   ][    PIN   ]
// TYPE = [0x00 .. 0x07] - Диапазон типа отчета (3 бита)
// 0x00 - Тестовый вывод
// 0x01 - Мгновенное значение аналогового пина [*]
// 0x02 - Минимальное значение аналогового пина [*] за последний период
// 0x03 - Максимальное значение аналогового пина [*] за последний период
// 0x04 - Значение цифрового пина [*]
// 0x[4567]* - Зарезервировано
//        Значениями цифровых пинов могут быть - 0 или 1. Все, что не 0, то является 1.

TPktBuffer uartRxBuf; // Буфер приема
TPktBuffer uartTxBuf; // Буфер передачи

TRingIndex prepareIdx {PINLST_SZ};  // Индексатор аналоговых пинов
inline void PrepareTxData() {
    uint8_t idx = prepareIdx.idx;
    uartTxBuf.PutPkt(0x01, idx, aPin[idx].curVal);
    uartTxBuf.PutPkt(0x02, idx, aPin[idx].minVal);
    uartTxBuf.PutPkt(0x03, idx, aPin[idx].maxVal);
    prepareIdx.Fwd(1);
}

// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | Регистры | Биты                                                          |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// |          |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | UBRRnH   |   x   |   x   |   x   |   x   |          UBRRn[11:8]          |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | UBRRnL   |                          UBRRn[7:0]                           |
// +----------+---------------+-------+-------+-------+-------+-------+-------+
// UBRR[11:0] - USART Baud Rate Register
//    This is a 12-bit register which contains the USART baud rate.
//    The UBRRnH contains the four most significant bits,
//    and the UBRRnL contains the eight least significant bits of the USART baud rate.
//    Ongoing transmissions by the Transmitter and Receiver will be corrupted
//    if the baud rate is changed. Writing UBRRnL will trigger an immediate update
//    of the baud rate prescaler.

inline void SetupUcsrA() {
    //          RXC         -   завершение приёма
    //          |TXC        -   завершение передачи
    //          ||UDRE      -   отсутствие данных для отправки
    //          |||FE       -   ошибка кадра
    //          ||||DOR     -   ошибка переполнение буфера
    //          |||||PE     -   ошибка чётности
    //          ||||||U2X   -   Двойная скорость
    //          |||||||MPCM -   Многопроцессорный режим
    //          ||||||||
    UCSR0A =   B00000000;
}

inline void SetupUcsrB() {
    //          RXCIE       -   прерывание при приёме данных
    //          |TXCIE      -   прерывание при завершение передачи
    //          ||UDRIE     -   прерывание отсутствие данных для отправки
    //          |||RXEN     -   разрешение приёма
    //          ||||TXEN    -   разрешение передачи
    //          |||||UCSZ2  -   UCSZ0:2 размер кадра данных
    //          ||||||RXB8  -   9 бит принятых данных
    //          |||||||TXB8 -   9 бит переданных данных
    //          ||||||||
    UCSR0B  =  B10111000;   //  разрешен приём и передача по UART
}

inline void SetupUcsrC() {
    //          URSEL       -   режим: 0-асинхронный
    //          |UMSEL      -   режим: 0-асинхронный
    //          ||UPM1      -   UPM0:1 чётность
    //          |||UPM0     -   UPM0:1 чётность
    //          ||||USBS    -   Стоп биты: 0-1, 1-2
    //          |||||UCSZ1  -   UCSZ0:2 размер кадра данных
    //          ||||||UCSZ0 -   UCSZ0:2 размер кадра данных
    //          |||||||UCPOL-   в синхронном режиме - тактирование
    //          ||||||||
    UCSR0C =   B00000110;   //  8-битовая посылка
}

inline void SetupUbrr(uint16_t baud) {
    uint16_t baud_setting = (F_CPU / 8 / baud  - 1) / 2;
    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting;
}

int InitUsart(uint16_t baud) {
    SetupUcsrA();     // установка UCSRA регистра
    SetupUbrr(baud);  // Установка скорости соединения
    SetupUcsrC();     // установка UCSRC регистра
    SetupUcsrB();     // установка UCSRB регистра - автоматический старт
}

//Прерывание по отправке данных
ISR (USART_UDRE_vect) {
    uint8_t data;
    while (!uartTxBuf.Get(data)) {
        PrepareTxData(); // Если в буфере нет данных, добавим их туда
    }
    UDR0 = data;
}

////Прерывание по приему данных
ISR(USART_RX_vect) {
    uint8_t data = UDR0;
////    while (!uartRxBuf.Put(data)) {
////        // Если в буфере нет места новым данным, освобождаем место, выкидывая первый пакет,
////        // чтобы не нарушать формат пакетов
////        uint8_t idType;
////        uint8_t idPin;
////        uint16_t data;
////        uartRxBuf.GetPkt(idType, idPin, data);
////    }
}
/* *************************************** */

void setup() {
    pinMode(13, OUTPUT);
    SetupPins();
    StartAdc();
    InitUsart(19200);
}

void loop() {
//    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//    delay(1000);              // wait for a second
//    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
//    delay(1000);
}
