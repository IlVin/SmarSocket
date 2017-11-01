
// pinList - Циклический буфер с ID пинов, работающих в режиме АЦП
#define PINBUF_SZ 8
volatile int8_t pinListSize = PINBUF_SZ;                               // Размер буфера
volatile int8_t pinList[PINBUF_SZ] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Буфер
volatile int8_t curPin = 0;                                            // Указатель на в данный момент оцифровываемый пин

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
// +----------+---------------+-------+-------+-------+-------+-------+-------+
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


inline void SetupADCSRA() {
    // ADEN      = [1]   - Подать питание на АЦП
    // ADSC      = [1]   - Стартовать АЦП c инициализацией
    // ADATE     = [1]   - Оцифровка по срабатываению триггера ADTS[2:0]
    // ADIF      = [0]   - флаг прерывания от компаратора
    // ADIE      = [0]   - разрешение прерывания от компаратора
    // ADPS[2:0] = [110] - Коэффициент делителя частоты АЦП K = 64
    ADCSRA = B11100110;
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
    SetupADMUX();
    SetupADCSRA(); // Старт АЦП
}

// Установка режима пинов
inline void SetupPins() {
    for (int8_t i = 0; i < pinListSize; ++i) {
        pinMode(pinList[i], INPUT);
    };
}


void setup() {
  SetupPins();
  StartAdc();
}

void loop() {
//  Serial.println(analogRead(pinIn));
//  delay(1000);
}
