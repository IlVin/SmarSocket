
#include <Wire.h>
#include <limits>

// http://codius.ru/articles/Arduino_%D1%83%D1%81%D0%BA%D0%BE%D1%80%D1%8F%D0%B5%D0%BC_%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D1%83_%D0%BF%D0%BB%D0%B0%D1%82%D1%8B_%D0%A7%D0%B0%D1%81%D1%82%D1%8C_2_%D0%90%D0%BD%D0%B0%D0%BB%D0%BE%D0%B3%D0%BE_%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9_%D0%BF%D1%80%D0%B5%D0%BE%D0%B1%D1%80%D0%B0%D0%B7%D0%BE%D0%B2%D0%B0%D1%82%D0%B5%D0%BB%D1%8C_%D0%90%D0%A6%D0%9F_%D0%B8_analogRead
// http://www.gaw.ru/html.cgi/txt/doc/micros/avr/arh128/12.htm
// https://geektimes.ru/post/263024/
// https://habrahabr.ru/post/321008/
// https://code.google.com/archive/p/arduino-timerone/downloads
// http://wiki.openmusiclabs.com/wiki/ArduinoFHT

template <typename T>
class TFifoIndex {
    T bufSize;
    T head;
    T tail;

    public:
    TFifo (size_t bufSize): bufSize(bufSize), head(0), tail(0) { }
    ~TFifo() {};

    bool IsEmpty() {
        return head == tail;
    }

    bool IsFull() {
        return head == ((tail + 1) % bufSize);
    }

    T CalcHeadFwd (T fwd = 1) {
        return (head + fwd) % bufSize;
    }

    void HeadFwd (T fwd = 1) {
        head = CalcHeadFwd(fwd);
    }

    T CalcTailFwd (size_t fwd = 0) {
        return (tail + fwd) % bufSize;
    }

    void TailFwd (T fwd = 1) {
        head = CalcTailFwd(fwd);
    }

    T size() {
        return (head + bufSize - tail) % bufSize;
    }
};


#define PINLST_SZ 8                                                         // Размер списка пинов
// К этим переменным доступ 
volatile uint8_t aPinList[PINBUF_SZ] = { A0, A1, A2, A3, A4, A5, A6, A7 }; // Список аналоговых пинов
volatile uint16_t aPinCurVal[PINBUF_SZ] = { 0 };                           // Мгновенные значения пинов
volatile static uint16_t aPinMax[PINBUF_SZ] = { 0 };                       // Максимальные значения пинов
volatile static uint16_t aPinMin[PINBUF_SZ] = { 0 };                       // Минимальные значения пинов


volatile TFifoIndex<uint8_t> aPinIdx {PIN_SZ};  // Индексатор аналоговых пинов: aPinIdx.head - ISR ADC; aPinIdx.tail - ISR Timer

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
    SetupADMUX(aPinList[PinIdx.head]);         // Записываем в буфер текущий пин [n]
    sei();                                     // Прерывания нужно разрешить
    SetupADCSRA();                             // АЦП начал оцифровывать пин [n]
    SetupADMUX(aPinList[aPinIdx.CalcHead(1)]); // Записываем в буфер номер следующего пина [n+1],
                                               // который АЦП будет оцифровывать во время прерывания
}

/* **** Interrupt service routine **** */
// Биты MUXn и REFS1:0 в регистре ADMUX поддерживают одноступенчатую буферизацию через временный регистр.
// Поэтому изменения вступают в силу в безопасный момент -  в течение одного такта синхронизации АЦП перед оцифровкой сигнала.
// Если выполнено чтение ADCL, то доступ к этим регистрам для АЦП будет заблокирован, пока не будет считан регистр ADCH.
ISR(ADC_vect){
    uint8_t lo = ADCL; // Автоматически блокируется доступ АЦП к регистрам ADCL и ADCH
    uint8_t hi = ADCH; // Автоматически разблокируется доступ АЦП к регистрам ADCL и ADCH
    aPinCurVal[aPinIdx.head] = (hi << 8) | lo;
    // aPinIdx.head указывает на предыдущий пин [n-1] - именно для него АЦП прислал результат
    // ADMUX указывает на пин [n], который в данный момент оцифровывает АЦП
    aPinIdx.HeadFwd(1); // Переходим на пин [n], для которого результат будет готов в следующем прерывании
    SetupADMUX(aPinList[aPinIdx.CalcHeadFwd(1)]); // Кладем в буфер номер следующего пина
}

// WAVE SAMPLE
#define V0 0x0200
#define WAVE_SAMPLE_SZ 500
volatile uint16_t curWS = 0;

// К этим переменым доступ только из прерывания, поэтому volatile не нужен
static uint16_t aPinClcMax[PINBUF_SZ] = { std::numeric_limits<int16_t>::min() }; // Буфер для вычисления aPinMax
static uint16_t aPinClcMin[PINBUF_SZ] = { std::numeric_limits<int16_t>::max() }; // Буфер для вычисления aPinMin

// aPinIdx.tail - указывает на пин, с которым работает вычислитель Min/Max
ISR(TIMER1_COMPA_vect){
    if (curWS < WAVE_SAMPLE_SZ) { // Ищем максимальное/минимальное значения
        uint8_t pinIdx = aPinIdx.tail;
        uint16_t val = aPinVal[pinIdx];
        if (val > aPinClcMax[pinIdx])
            aPinClcMax[pinIdx] = val;
        if (val < aPinClcMin[pinIdx])
            aPinClcMin[pinIdx] = val;
        curWS++;
    } else {
        uint8_t pinIdx = aPinIdx.tail;
        aPinMax[pinIdx] = aPinClcMax[pinIdx];
        aPinClcMax[pinIdx] = std::numeric_limits<int16_t>::min();
        aPinMin[pinIdx] = aPinClcMin[pinIdx];
        aPinClcMin[pinIdx] = std::numeric_limits<int16_t>::max();
        curWS = 0;
        aPinIdx.TailFwd(1);
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
    for (int8_t i = 0; i < pinListSize; ++i) {
        pinMode(pinList[i], INPUT);
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
// 0x[01234567]* - Запрещены
// 0x8* - Тестовый вывод
// 0x9* - Мгновенное значение аналогового пина [*]
// 0xA* - Максимальное значение аналогового пина [*] за последний период
// 0xB* - Значение цифрового пина [*]
// 0x[CDEF]* - Зарезервировано
//        Значениями цифровых пинов могут быть - 0 или 1. Все, что не 0, то является 1.

#define UART_BUF_SZ 16
volatile uint8_t uartBuf[UART_BUF_SZ];
volatile uint8_t uartHead = 0;
volatile uint8_t uartTail = 0;

int InitUart(void) {
    //  Установка скорости 9600
    UBRRH=0;    //  UBRR=f/(16*band)-1 f=8000000Гц band=9600,
    UBRRL=51;   //  нормальный асинхронный двунаправленный режим работы
    //          RXC         -   завершение приёма
    //          |TXC        -   завершение передачи
    //          ||UDRE      -   отсутствие данных для отправки
    //          |||FE       -   ошибка кадра
    //          ||||DOR     -   ошибка переполнение буфера
    //          |||||PE     -   ошибка чётности
    //          ||||||U2X   -   Двойная скорость
    //          |||||||MPCM -   Многопроцессорный режим
    //          ||||||||
    //          76543210
    UCSRA =   0b00000000;
    //          RXCIE       -   прерывание при приёме данных
    //          |TXCIE      -   прерывание при завершение передачи
    //          ||UDRIE     -   прерывание отсутствие данных для отправки
    //          |||RXEN     -   разрешение приёма
    //          ||||TXEN    -   разрешение передачи
    //          |||||UCSZ2  -   UCSZ0:2 размер кадра данных
    //          ||||||RXB8  -   9 бит принятых данных
    //          |||||||TXB8 -   9 бит переданных данных
    //          ||||||||
    //          76543210
    UCSRB =   0b00011000;   //  разрешен приём и передача по UART
    //          URSEL       -   всегда 1
    //          |UMSEL      -   режим:1-синхронный 0-асинхронный
    //          ||UPM1      -   UPM0:1 чётность
    //          |||UPM0     -   UPM0:1 чётность
    //          ||||USBS    -   топ биты: 0-1, 1-2
    //          |||||UCSZ1  -   UCSZ0:2 размер кадра данных
    //          ||||||UCSZ0 -   UCSZ0:2 размер кадра данных
    //          |||||||UCPOL-   в синхронном режиме - тактирование
    //          ||||||||
    //          76543210
    UCSRC =   0b10000110;   //  8-битовая посылка
}

void SendUart(unsigned char c) {
    //  Устанавливается, когда регистр свободен)
    if ( uartTail != uartHead && (UCSRA & (1 << UDRE)) != 0 ) {
        UDR = uartBuf[uartTail];
        uartTail = (uartTail + 1) % UART_BUF_SZ;
    }
}

unsigned char GetChUart(void)//    Получение байта
{
    while(!(UCSRA&(1< <RXC))) //  Устанавливается, когда регистр свободен
        {}
        return UDR;
}

//Прерывание отправки данных
ISR (USART_UDRE_vect) {
    buffer_index++;
    if (buffer_index==Nbyte) {
        UCSR0B &=~(1<<UDRIE0);
        ClearBuffUart();
    } else {
        UDR0 = buff_uart[buffer_index];
    }
}

//Прием данных
char USART_Receive( void ) {
    while ( !(UCSR0A & (1<<RXC0)) );
    return UDR0;
}

//Прерывание по приему данных
ISR(USART_RX_vect) {
    unsigned char b;
    b = UDR0;
    buff_uart[Nbyte] = b;
    Nbyte++;
    if (b == (uint8_t)0xFF) {
        USART_Transmit();
    }
    if (Nbyte == BUFFER_UART_SIZE) {
        USART_Transmit();
    }
}





void WireOnRequest() {
    for (byte Id = 0x00; Id < 0x03; ++Id) {
        Wire.write(Id);
        Wire.write('H');
        Wire.write('i');
    }
}

void WireOnReceive() {
    Serial.print("Wire OnReceive: ");
    while (0 < Wire.available()) { // loop through all but the last
        uint8_t c = Wire.read(); // receive byte as a character
        Serial.print(c);         // print the byte
        Serial.print(", ");
    }
    Serial.println("");
}

/* *************************************** */


void setup() {
    Serial.begin(9600);
    SetupPins();
    StartAdc();
    StartTimer1();
    Wire.begin(0x0A);
    Wire.onRequest(WireOnRequest);
    Wire.onReceive(WireOnReceive);
}

static uint32_t tcnt = 0;
static uint32_t prevTcnt = 0;

void loop() {
    Serial.print(tcnt-prevTcnt);
    prevTcnt = tcnt;
    Serial.print(",  ");
    Serial.print(OCR1A);
    Serial.print(": ");
    for (int i = 0; i < 33; i++) {
        Serial.print(waveSample[i]);
        Serial.print("; ");
    }
    Serial.println("");
    delay(1000);
}
