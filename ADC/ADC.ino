// http://codius.ru/articles/Arduino_%D1%83%D1%81%D0%BA%D0%BE%D1%80%D1%8F%D0%B5%D0%BC_%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D1%83_%D0%BF%D0%BB%D0%B0%D1%82%D1%8B_%D0%A7%D0%B0%D1%81%D1%82%D1%8C_2_%D0%90%D0%BD%D0%B0%D0%BB%D0%BE%D0%B3%D0%BE_%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9_%D0%BF%D1%80%D0%B5%D0%BE%D0%B1%D1%80%D0%B0%D0%B7%D0%BE%D0%B2%D0%B0%D1%82%D0%B5%D0%BB%D1%8C_%D0%90%D0%A6%D0%9F_%D0%B8_analogRead
// http://www.gaw.ru/html.cgi/txt/doc/micros/avr/arh128/12.htm
// https://geektimes.ru/post/263024/
// https://habrahabr.ru/post/321008/
// https://code.google.com/archive/p/arduino-timerone/downloads
// http://wiki.openmusiclabs.com/wiki/ArduinoFHT

// pinList - Циклический буфер с ID пинов, работающих в режиме АЦП
#define PINBUF_SZ 8
const uint8_t pinListSize = PINBUF_SZ;                                  // Размер буфера
volatile uint8_t pinList[PINBUF_SZ] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Буфер
volatile uint16_t pinValues[PINBUF_SZ] = {0};                           // Значения пинов
volatile uint16_t pinRms[PINBUF_SZ] = {0};                              // RMS для пинов
volatile uint8_t curPin = 0;                                            // Указатель на в данный момент оцифрованный пин

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


inline uint8_t pin(uint8_t pinNum) {
    return pinNum % pinListSize;
}

inline void SetupADCSRA() {
    // ADEN      = [1]   - Подать питание на АЦП
    // ADSC      = [1]   - Стартовать АЦП c инициализацией
    // ADATE     = [1]   - Оцифровка по срабатываению триггера ADTS[2:0]
    // ADIF      = [0]   - флаг прерывания от компаратора
    // ADIE      = [1]   - разрешение прерывания от компаратора
    // ADPS[2:0] = [110] - Коэффициент делителя частоты АЦП K = 64
    ADCSRA = B11101110;
}

inline void SetupADCSRB() {
    // ADTS[2:0] = [000]   - Free Running mode
    // xxxxx     = [00000] - Резервные биты
    ADCSRB = B00000000;
}

inline void SetupADMUX(uint8_t SRC = 0){
    // REFS[1:0] = [01]   - Источником опорного напряжения является питание
    // ADLAR     = [0]    - Режим 10 бит
    // x         = [0]    - Резервный бит
    // MUX[3:0]  = [SRC]  - Источник сигнала
    ADMUX = B01000000 | (SRC & B00001111);
}

inline void StartAdc() {
    SetupADCSRB();

    curPin = 0;
    SetupADMUX(curPin);
    uint8_t oldSREG = SREG;
    sei();                       // Прерывания нужно разрешить
    SetupADCSRA();               // Старт АЦП
    // АЦП начал оцифровывать пин curPin
    SetupADMUX(pin(curPin + 1)); // Записываем в буфер номер следующего пина,
                                 // который АЦП будет оцифровывать во время прерывания
    SREG = oldSREG;
}

/* **** Interrupt service routine **** */
// Биты MUXn и REFS1:0 в регистре ADMUX поддерживают одноступенчатую буферизацию через временный регистр.
// Поэтому изменения вступают в силу в безопасный момент -  в течение одного такта синхронизации АЦП перед оцифровкой сигнала.
// Если выполнено чтение ADCL, то доступ к этим регистрам для АЦП будет заблокирован, пока не будет считан регистр ADCH.
ISR(ADC_vect){
    uint8_t lo = ADCL; // Автоматически блокируется доступ АЦП к регистрам ADCL и ADCH
    uint8_t hi = ADCH; // Автоматически разблокируется доступ АЦП к регистрам ADCL и ADCH
    pinValues[curPin] = (hi << 8) | lo;
    // curPin указывает на предыдущий пин [n-1] - именно для него АЦП прислал результат
    // ADMUX указывает на пин [n], который в данный момент оцифровывает АЦП
    curPin = ADMUX & B00001111;    // В следующем прерывании будет готов результат для пина из ADMUX
    SetupADMUX(pin(curPin + 1));   // Кладем в буфер номер следующего пина
}

// WAVE SAMPLE
#define V0 0x0200
#define WAVE_SAMPLE_SZ 500
volatile uint8_t wavePin = 0; // Указывает номер пина из pinList, для которого строится сэмпл
volatile uint8_t waveSample[WAVE_SAMPLE_SZ] = {0};
volatile uint16_t curWS = 0;
volatile uint32_t tcnt = 0;
volatile uint32_t prevTcnt = 0;

ISR(TIMER1_COMPA_vect){
    if (curWS < WAVE_SAMPLE_SZ) {
        uint16_t val = pinValues[wavePin];       // Читаем мгновенное значение пина
        if (val < V0) {
            waveSample[curWS] = (uint8_t)(((V0 - val) >> 1) & 0x00FF);
        } else {
            waveSample[curWS] = (uint8_t)(((val - V0) >> 1) & 0x00FF);
        }
        curWS++;
        tcnt++;
    } else {
        // Останавливаем таймер
        StopTimer1();
        sei();
        pinRms[wavePin] = CalcRMS();
        wavePin = pin(wavePin + 1);
        curWS = 0;
        StartTimer1();
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
    // COMnA1 = [0] - контролируют поведение выводов OCnA
    // COMnA0 = [0] - контролируют поведение выводов OCnA
    // COMnB1 = [0] - контролируют поведение выводов OCnB
    // COMnB0 = [0] - контролируют поведение выводов OCnB
    // COMnC1 = [0] - контролируют поведение выводов OCnC
    // COMnC0 = [0] - контролируют поведение выводов OCnC
    // WGMn1  = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    // WGMn0  = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    TCCR1A = B00000000;

    // x      = [0]
    // x      = [0]
    // ICIE1  = [0] - Input Capture Interrupt Enable
    // x      = [0]
    // OCIE1C = [0] - Прерывание типа [TIMER1 COMPC] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1C(H и L))
    // OCIE1B = [0] - Прерывание типа [TIMER1 COMPB] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1B(H и L))
    // OCIE1A = [1] - Прерывание типа [TIMER1 COMPA] (разрешение прерывания TCNT1 счетчика по совпадению с OCR1A(H и L))
    // TOIE1  = [0] - процессор реагирует на сигнал переполнения ТС1 и вызывает прерывание.
    TIMSK1 = B00000010;

    // IF OCRnA == TCNTn THEN INTERRUPT
    OCR1A = CPU_FREQ / (WAVE_SAMPLE_SZ * WAVE_FREQ);
    //OCR1A = 1600;

    // ICNC1 = [0] - PWM
    // ICES1 = [0] - PWM
    //   x   = [0]
    // WGM13 = [0] - Режим СТС (сброс по совпадению) WGMn = [0100]
    // WGM12 = [1] - Режим СТС (сброс по совпадению) WGMn = [0100]
    // CS12  = [0] - Делитель (CLK/1) CSn = [001] [101]
    // CS11  = [0] - Делитель (CLK/1) CSn = [001]
    // CS10  = [1] - Делитель (CLK/1) CSn = [001]
    TCCR1B = B00001001;
}

inline void StopTimer1() {
      TCCR1B = 0;
}

uint16_t CalcRMS() {
    return 0;
}


// Установка режима пинов
inline void SetupPins() {
    for (int8_t i = 0; i < pinListSize; ++i) {
        pinMode(pinList[i], INPUT);
    };
}

void setup() {
  Serial.begin(9600);
  SetupPins();
  StartAdc();
  StartTimer1();
}

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
