
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


#define PINLST_SZ 8                                                                // Размер списка пинов
struct TAnalogPin {
    // Поля, к которым доступ осуществляется и из loop и из ISR
    volatile uint8_t  id;      // Номер физического пина
    volatile uint16_t curVal;  // Мгновенное значение пина
    volatile uint16_t maxVal;  // Максимальное значение пина за время сэмпла
    volatile uint16_t minVal;  // Минимальное значение пина за время сэмпла

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

// Аналоговые пины
static TAnalogPin aPin[PINLST_SZ] = { A0, A1, A2, A3, A4, A5, A6, A7 };
TRingIndex aPinIdx {PINLST_SZ};  // Индексатор аналоговых пинов

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
    //        ADEN           = [1]   - Подать питание на АЦП
    //        |ADSC          = [1]   - Стартовать АЦП c инициализацией
    //        ||ADATE        = [1]   - Оцифровка по срабатываению триггера ADTS[2:0]
    //        |||ADIF        = [0]   - флаг прерывания от компаратора
    //        ||||ADIE       = [1]   - разрешение прерывания от компаратора
    //        |||||ADPS[2:0] = [110] - Коэффициент делителя частоты АЦП K = 64
    //        ||||||||
    ADCSRA = B11101110;
}

inline void SetupADCSRB() {
    //        ADTS[2:0] = [000]   - Free Running mode
    //        |||xxxxx  = [00000] - Резервные биты
    //        ||||||||
    ADCSRB = B00000000;
}

inline void SetupADMUX(uint8_t SRC = 0){
    //       REFS[1:0]    = [01]  - Источником опорного напряжения является питание
    //       ||ADLAR      = [0]   - Режим 10 бит
    //       |||x         = [0]   - Резервный бит
    //       ||||MUX[3:0] = [SRC] - Источник сигнала
    //       ||||||||
    ADMUX = B01000000 | (SRC & B00001111);
}

inline void StartAdc() {
    SetupADCSRB();
    SetupADMUX(aPin[aPinIdx.idx].id);          // Записываем в буфер текущий пин [n]
    sei();                                     // Прерывания нужно разрешить
    SetupADCSRA();                             // АЦП начал оцифровывать пин [n]
    SetupADMUX(aPin[aPinIdx.CalcFwd(1)].id);   // Записываем в буфер номер следующего пина [n+1],
                                               // который АЦП будет оцифровывать во время прерывания
}

/* **** Interrupt service routine **** */
// Биты MUXn и REFS1:0 в регистре ADMUX поддерживают одноступенчатую буферизацию через временный регистр.
// Поэтому изменения вступают в силу в безопасный момент -  в течение одного такта синхронизации АЦП перед оцифровкой сигнала.
// Если выполнено чтение ADCL, то доступ к этим регистрам для АЦП будет заблокирован, пока не будет считан регистр ADCH.
ISR(ADC_vect){
    uint8_t lo = ADCL; // Автоматически блокируется доступ АЦП к регистрам ADCL и ADCH
    uint8_t hi = ADCH; // Автоматически разблокируется доступ АЦП к регистрам ADCL и ADCH
    aPin[aPinIdx.idx].curVal = (hi << 8) | lo;
    // aPinIdx.idx указывает на предыдущий пин [n-1] - именно для него АЦП прислал результат
    // ADMUX указывает на пин [n], который в данный момент оцифровывает АЦП
    aPinIdx.Fwd(1); // Переходим на пин [n], для которого результат будет готов в следующем прерывании
    SetupADMUX(aPin[aPinIdx.CalcFwd(1)].id); // Кладем в буфер номер следующего пина
}

// WAVE SAMPLE
#define WAVE_SAMPLE_SZ 500
uint16_t curWS = 0;

// aPinIdx.tail - указывает на пин, с которым работает вычислитель Min/Max
TRingIndex aPinIdxTimer(PINLST_SZ);
ISR(TIMER1_COMPA_vect){
    if (curWS < WAVE_SAMPLE_SZ) { // Ищем максимальное/минимальное значения
        uint8_t pinIdx = aPinIdxTimer.idx;
        uint16_t val = aPin[pinIdx].curVal;
        if (val > aPin[pinIdx].maxClcVal)
            aPin[pinIdx].maxClcVal = val;
        if (val < aPin[pinIdx].minClcVal)
            aPin[pinIdx].minClcVal = val;
        curWS++;
    } else {
        uint8_t pinIdx = aPinIdxTimer.idx;
        aPin[pinIdx].maxVal = aPin[pinIdx].maxClcVal;
        aPin[pinIdx].maxClcVal = 0x0000;
        aPin[pinIdx].minVal = aPin[pinIdx].minClcVal;
        aPin[pinIdx].minClcVal = 0xFFFF;
        curWS = 0;
        aPinIdxTimer.Fwd(1);
    }
}

/* ************ TIMER ***************** */
// http://narodstream.ru/avr-urok-10-tajmery-schetchiki-preryvaniya/
//
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | Регистры | Биты                                                          |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// |          |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | TCNTn    | Количество тиков таймера                                      |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | OCRnA    | Младший байт значения останова счетчика                       |
// +----------+---------------+-------+-------+-------+-------+-------+-------+
// | OCRnB    | Старший байт значения останова счетчика                       |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | TCCRnA   | COMnA1| COMnA0| COMnB1| COMnB0| COMnC1| COMnC0| WGMn1 | WGMn0 |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | TCCRnB   | ICNCn | ICESn |   x   | WGMn3 | WGMn2 | CSn2  | CSn1  | CSn0  |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | TCCRnC   | FOCnA | FOCnB |   x   |   x   |   x   |   x   |   x   |   x   |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// | TIMSKn   |   x   |   x   | ICIEn |   x   | OCIEnC| OCIEnB| OCIEnA| TOIEn |
// +----------+-------+-------+-------+-------+-------+-------+-------+-------+
// TCNTn = TCNTnH[15:8] + TCNTnL[7:0] - Счетчик тиков таймера
// OCRnA = OCRnAH[15:8] + OCRnAL[7:0], OCRnB = OCRnBH[15:8] + OCRnBL[7:0] - Числа, с которыми сравнивается TCNTn
// TCCRnA - Timer/Counter Control Register A
//      COMnA1,COMnA0, COMnB1 и COMnB0 - контролируют поведение выводов OC1A и OC1B
//      FOC1A, FOC1B, WGM11 и WGM10 служат для задания работы ТС1 как широтно-импульсного модулятора.
// TCCRnB - Timer/Counter Control Register B
//      ICNCn - Input Capture Noise Canceler (for PWM)
//      ICESn - Input Capture Edge Select    (for PWM)
//      WGMn[3:2] - Waveform Generation Mode (for PWM)
//      CSn[2:0]  - Clock Select
//      +------+------+------+--------------------------------------------------------+
//      | CSn2 | CSn1 | CSn0 | Description                                            |
//      +------+------+------+--------------------------------------------------------+
//      |  0   |  0   |  0   | No clock source. (Timer/Counter stopped)               |
//      |  0   |  0   |  1   | CLK/1                                                  |
//      |  0   |  1   |  0   | CLK/8                                                  |
//      |  0   |  1   |  1   | CLK/64                                                 |
//      |  1   |  0   |  0   | CLK/256                                                |
//      |  1   |  0   |  1   | CLK/1024                                               |
//      |  1   |  1   |  0   | External clock source on Tn pin. Clock on falling edge |
//      |  1   |  1   |  1   | External clock source on Tn pin. Clock on rising edge  |
//      +------+------+------+--------------------------------------------------------+

// FOCnA  - Force Output Compare for Channel A
// TIMSKn - Timer/Counter Interrupt Mask Register
//      TICIEn - Timer/Countern, Input Capture Interrupt Enable
//      OCIEnC - Timer/Countern, Output Compare C Match Interrupt Enable
//      OCIEnB - Timer/Countern, Output Compare B Match Interrupt Enable
//      OCIEnA - Timer/Countern, Output Compare A Match Interrupt Enable
//      TOIEn  - Timer/Countern, Overflow Interrupt Enable


// https://github.com/radiolok/arduino_rms_count/blob/master/Urms_calc/Urms_calc.pde

#define WAVE_FREQ 40         // Частота исследуемой волны
#define CPU_FREQ 16000000    // Частота процессора

inline void StartTimer1() {
    //        COMnA1       = [0] - контролируют поведение выводов OCnA
    //        |COMnA0      = [0] - контролируют поведение выводов OCnA
    //        ||COMnB1     = [0] - контролируют поведение выводов OCnB
    //        |||COMnB0    = [0] - контролируют поведение выводов OCnB
    //        ||||COMnC1   = [0] - контролируют поведение выводов OCnC
    //        |||||COMnC0  = [0] - контролируют поведение выводов OCnC
    //        ||||||WGMn1  = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    //        |||||||WGMn0 = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    //        ||||||||
    TCCR1A = B00000000;

    //        x            = [0]
    //        |x           = [0]
    //        ||ICIE1      = [0] - Input Capture Interrupt Enable
    //        |||x         = [0]
    //        ||||OCIE1C   = [0] - Прерывание типа [TIMER1 COMPC] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1C(H и L))
    //        |||||OCIE1B  = [0] - Прерывание типа [TIMER1 COMPB] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1B(H и L))
    //        ||||||OCIE1A = [1] - Прерывание типа [TIMER1 COMPA] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1A(H и L))
    //        |||||||TOIE1 = [0] - процессор реагирует на сигнал переполнения ТС1 и вызывает прерывание.
    //        ||||||||
    TIMSK1 = B00000010;

    // IF OCRnA == TCNTn THEN INTERRUPT
    OCR1A = CPU_FREQ / (WAVE_SAMPLE_SZ * WAVE_FREQ);
    //OCR1A = 1600;

    //        ICNC1       = [0] - PWM
    //        |ICES1      = [0] - PWM
    //        ||x         = [0]
    //        |||WGM13    = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    //        ||||WGM12   = [1] - Режим СТС (сброс по совпадению) WGMn = [0100]
    //        |||||CS12   = [0] - Делитель (CLK/1) CSn = [001] [101]
    //        ||||||CS11  = [0] - Делитель (CLK/1) CSn = [001]
    //        |||||||CS10 = [1] - Делитель (CLK/1) CSn = [001]
    //        ||||||||
    TCCR1B = B00001001;
}

inline void StopTimer1() {
    TCCR1B = 0;
}

// Установка режима пинов
inline void SetupPins() {
    for (int8_t pinIdx = PINLST_SZ; --pinIdx;) {
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
// 0x0* - Тестовый вывод
// 0x1* - Мгновенное значение аналогового пина [*]
// 0x2* - Максимальное значение аналогового пина [*] за последний период
// 0x3* - Значение цифрового пина [*]
// 0x[4567]* - Зарезервировано
//        Значениями цифровых пинов могут быть - 0 или 1. Все, что не 0, то является 1.

volatile TPktBuffer uartRxBuf; // Буфер приема
volatile TPktBuffer uartTxBuf; // Буфер передачи

inline void PrepareTxData() {
//    uartTxBuf.PutPkt(0x80, 0x05, (static_cast<uint16_t>('O') << 8) | static_cast<uint16_t>('k')); // Тестовый пакет постит строку '…Ok'
    uartTxBuf.Put('@');
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
    StartTimer1();
    InitUsart(19200);
}

void loop() {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    //PrepareTxData();
    UDR0 = '\n';
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);
}
