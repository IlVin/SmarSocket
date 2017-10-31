
// Code from http://arduino.ru/forum/obshchii/gy-712-30a-udalennyi-kontrol-peremennogo-toka
// Code is ugly, but it works...

#define   PIN_FAZA1   A5 // Куда воткнут датчик тока на первой фазе
#define   PIN_FAZA2   A6 // Куда воткнут датчик тока на второй фазе
#define   PIN_FAZA3   A7 // Куда воткнут датчик тока на третьей фазе

//=======================Класс для энергоснабжения====================
class energo {
    public:
        energo(int,int);
        void start_sensor();
        void get_mAmper();
    private:
        int pin_faza;
        int numer_faza;
        int current_apmer=0;
        const unsigned long sampleTime = 100000UL;                  // образец 100мс для 50грц
        const unsigned long numSamples = 250UL;                     // какое количество значений берем чтобы получить точный результат, но и чтобы не загубить АЦП
        const unsigned long sampleInterval = sampleTime/numSamples; // интервал выборки, должен быть больше, чем  время преобразования АЦП
        int adc_zero1;                                              // Переменная автоматической калибровки
        unsigned long int periodSend_mysql=180000;                  // период через который отправляем данные на сервер.
        unsigned long int nextSend_mysql=30000;                     // следующее время отправки данных на сервер, сначала делаем задержку иначе получим первые показания фигню.
        unsigned long int mAmper_period=0;                          // кубышка куда складываем значение силы тока внутри периода измерений...
        int maxAmper_value=0;                                       // предыдущее значение измерения силы тока
        int countMonitoring=0;                                      // счетчик измерений
        int mysql_id_service=10;                                    // id сервиса в системе
        int mysql_actionGet_amper=36;                               // id action в системе
        int mysql_actionOff_amper;                                  // на фига объявил не знаю, наверное прозапас.
};

energo::energo(int _numer_faza,int _pin_faza){                      //конструктор пустой почти, вычисления внутри конструктора - риутал вызывающий потом зеленых человечков..
    numer_faza=_numer_faza;
    pin_faza=_pin_faza;
}

// тут калибруем датчик
void energo::start_sensor() {
    long VQ = 0;
    pinMode(pin_faza, INPUT);
    Serial.println(pin_faza);
    //читаем 5000 раз
    for (int i=0; i < 5000; i++) {
        VQ += analogRead(pin_faza);
        delay(1); //где то вычитал, на фига не знаю. но убирать нельзя!!!!!
    }

   VQ /= 5000;

   //Вывод в монитор на позырить - в продуктиве убрать!
   Serial.print(map(VQ, 0, 1023, 0, 5000)); Serial.println(" mV");
   adc_zero1 = int(VQ); //коэфициентик посчиатлся..
}

void energo::get_mAmper() {
    unsigned long currentAcc = 0;
    unsigned int count = 0;
    unsigned long prevMicros = micros()-sampleInterval;
    unsigned int mAmper=0;
    int rms_int=0;

    //Далше начинается магия сперто из модного учебника  и забыто, НЕ ЛЕЗТЬ, а то фиг вспомнишь!
    while (count &lt; numSamples) {
        if (micros()-prevMicros >= sampleInterval) {
            int adc_raw = analogRead(pin_faza)-adc_zero1;
            currentAcc += (unsigned long)(adc_raw * adc_raw);
            ++count;
            prevMicros += sampleInterval;
        }
    }
    float rms = sqrt((float)currentAcc/(float)numSamples) * (75.7576 / 1024.0); // Для татчика 30А,  другие номиналы вместо 75.7576 берем килькилятор и высчитываем

    //Магия закончилась - начинается бытовуха!

    rms_int=rms*1000; //переводим из дробных ампер в целые милиамперы
    mAmper_period+=rms_int; //складываем показания в одну кубышку
    if (rms_int&gt;maxAmper_value)maxAmper_value=rms_int; //вычисляем пиковый ток в периоде.

    countMonitoring++; //увеличиваем счетчик количества снятых показаний
//закоментировано для форума -  части переменных в этом коде нет.. Для примера
 /* if(nextSend_mysql&lt;millis()){ // если пришла пора отправлять данные на сервер - отправляем
       mAmper=mAmper_period/countMonitoring;
       command="";
       command=String(id_user_system_mysql)+":"+String(mysql_id_service)+":"+String(mysql_actionGet_amper)+":"+String(pin_faza-54)+":"+String(numer_faza)+":"+String(mAmper)+":"+String(maxAmper_value);
       Serial.println(command);
       Serial1.flush();
       Serial1.println(command);                                                       
       command="";
       nextSend_mysql=millis()+periodSend_mysql;
       countMonitoring=0; // начинаем заново высчитывать среднее и пиковое значение
       mAmper_period=0;
       maxAmper_value=0;
    }
*/
}

//Объявляем объекты
energo *faza1 = new energo(1,PIN_FAZA1);
energo *faza2 = new energo(2,PIN_FAZA2);
energo *faza3 = new energo(3,PIN_FAZA3);

void setup {
     Serial.begin(9600);
     Serial1.begin(9600);
     faza1->start_sensor(); //запускаем датчики тока...
     faza2->start_sensor(); //целых 15 секунд ждем, но искусство требует жертв :)
     faza3->start_sensor(); //
}

void loop{
       faza1->get_mAmper(); //проверяем, анализируем и отправляем... 
       faza2->get_mAmper();
       faza3->get_mAmper();
}

