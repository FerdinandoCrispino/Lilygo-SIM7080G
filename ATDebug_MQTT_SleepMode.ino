/**
 * @file      ATDebug.ino
 * @author    Ferdinando Crispino
 * @date      2025-06-26
 * @Placa     ESP32S3 DEv Module
 * @hardware  LilyGo SIM7080G
 */

#include <esp_task_wdt.h>

#include <Arduino.h>
//#include <Wire.h>

#include "FS.h"
#include "FFat.h"
#include "esp_vfs_fat.h"
#include "esp_err.h"

#include "utilities.h"
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
#define SDA_PIN 21 
#define SCL_PIN 47
TwoWire I2C_BUS(1);  // bus 0 já esta sendo utilizado pelo PMU

#define LigaVcc_PIN 9   // pino de comando que liga ou desliga da fonte para o step up

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>


#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

void getWakeupReason();

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
XPowersPMU  PMU;

#include <stdio.h>
#include "esp_log.h"
#include "driver/temperature_sensor.h"

#include <esp_adc_cal.h>
esp_adc_cal_characteristics_t adc_chars;
unsigned int adc_ini_fail = 0;
#include "time.h"
#include <string.h>
#include <cmath> // For math.sqrt
float desvio;

// sensor interno de temperatura do chip
temperature_sensor_handle_t temp_sensor = NULL;
temperature_sensor_config_t temp_sensor_config = {.range_min = -10,.range_max = 80, };

unsigned long previousMillisPar = millis();
volatile int flagDeletarArquivo = 0;            // volatile pq é compartilhada com o ISR
volatile int flagDeletaAllData = 0;
volatile int flagReadAllData = 0;
unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 500;        // milliseconds
const int pinDelArq = 16;                       // pino de interrupção para deletar arquivo temporário
const int pinNotGo2sleep = 17;                  // pino high nao vai ida para sleep!!  
const int pinDeletaAllData = 18;                // pino de interrupção para deletar o arquivo permanente
const int pinReadAllData = 8;                   // pino de interrupção para Leitura do arquivo permanente

RTC_DATA_ATTR int myvar = 0;                    // variavel no RTC para ser usado quando reboot

// **************************************Configuracao do sistema ***************************************************
//
#define WDT_TIMEOUT 300                   // Watchdog Tempo limite em segundos

// Set to wake up every 4 minutes to send data to platform
#define uS_TO_S_FACTOR 1000000ULL        /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  240        //14400=4horas   21600=6horas    /* Time ESP32 will go to sleep (in seconds) */

struct Config {
  int tempoAmostra = 10000;                     // tempo de espera apos acordar para fazer a leitura (millisegundos)
  int numLeituras = 100;                        // numero de amostras para uma leitura 
  String idSendor = "NV001";
  String codProj = "Proj. teste";
  String SIMconf = "kiteiot.vivo.com.br";
  // servidor MQTT
  //String NBiot_URL = "998dfb6da8844d948a9bbfdfe42b32d4.s1.eu.hivemq.cloud";
  //String NBiot_port = "8883";
  //String NBiot_username = "esp32";                //HiveMQ Cloud only allows secure TLS connections
  //String NBiot_password = "Yyy01020304@";
  
  String NBiot_URL = "crystalmq.bevywise.com";      //ferdinando.crispino@gmail.com - #Mqqt#01
  String NBiot_port = "1883";
  //String NBiot_username = "teste_não_conectar";  
  String NBiot_username = "EeObn5ov3lmm7b1zZo";
  String NBiot_password = "QJeEn6EawcJ8gSoQCF";
  String NBiot_client = "teste1";              // nome do cliente na plataforma MQTT - IOT
  String NBiot_topic = "sensor_01";            // topico para envio de dados na plataforma MQTT - IOT
  const char* nomeArq = "/dados.csv";          // nome do arquivo de dados temporario para caso de falha de comunicaçao com o MQTT
  const char* nomeArqAllData = "/AllData.csv"; // nome do arquivo de dados permanente (todas as medições)
  int num_sensor = 1;                          // numero de sensores existentes (1 a 4) 
  bool GPS_active = true;                      // ativa ou desativa a leitura do GPS ao iniciar 
};
struct Config cfg;

// *****************************************************************************************************************


// estrutura de dados de saida do ensaio
struct estruturaDadosEnsaio {    
  String dt          = "";   
  String press_M1[4] = {"0"};
  String per_bateria = "0";
  String vol_bateria = "0"; 
  String chip_temp   = "0";  
  String chip_RSSI   = "0";
  String lat         = "0";
  String lon         = "0";
}; 
struct estruturaDadosEnsaio result;

void esp_set_time() {
  modem.sendAT("+cntp"); 
  modem.waitResponse(3000UL);    
  delay(2000);
  String data_sim;  
  modem.sendAT("+CCLK?");
  //modem.waitResponse(5000);
  modem.waitResponse(30000UL,data_sim);
  //Buscar linha com +CCLK
  int idx = data_sim.indexOf("+CCLK:");
  Serial.print("Busca indexOf +CCLK:");
  Serial.println(data_sim);
  Serial.println(idx);
  if (idx >= 0) {
    int start = data_sim.indexOf('"', idx);
    int end = data_sim.indexOf('"', start + 1);
    String datetime = data_sim.substring(start + 1, end);  // Ex: "24/06/20,20:14:35+00"
    Serial.print("Data extraída: ");
    Serial.println(datetime);
    if (datetime.substring(0,3) != "80") {
      set_rtc_time(datetime);     
    } else {
       Serial.println("Data invalida, RTC não será atualizado.");
    }  
  }
}

String esp_get_time() {
    struct tm timeinfo;
    getLocalTime(&timeinfo);      
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M", &timeinfo);
    Serial.println("getLocalTime:");
    Serial.println(buffer);
    return buffer;
}

void set_rtc_time(String datetime) {
    // Parse string
    struct tm tm;
    int tz = 0;
    memset(&tm, 0, sizeof(tm));

    sscanf(datetime.c_str(), "%2d/%2d/%2d,%2d:%2d:%2d%3d",
           &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
           &tm.tm_hour, &tm.tm_min, &tm.tm_sec, &tz);

    tm.tm_year += 2000 - 1900; // Ajuste do ano
    tm.tm_mon -= 1;            // Janeiro = 0
    time_t t = mktime(&tm);
    if (t == -1) {
      Serial.println("Erro ao converter hora.");
      return;
    }
    struct timeval now = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&now, NULL);
    //Serial.print("now:");
    //Serial.println(t);
    //Serial.println(tm.tm_year);
    //Serial.println(tm.tm_mon);
    //Serial.println(tm.tm_mday);
    //Serial.println(tm.tm_hour);
    //Serial.println(tm.tm_min);
    //Serial.println(tm.tm_sec);
    //Serial.println(tz);    
    Serial.println(esp_get_time());    
}

void leArquivo(fs::FS &fs, const char * path){
    //Lê arquivo 
  if (FFat.exists(path)) {
    File fileread = FFat.open(path, FILE_READ);
    while (fileread.available()) {
      String line = fileread.readStringUntil('\n');
      Serial.println(line);
    }
    fileread.close();
  }else {
    Serial.print ("Arquivo inexistente! ");
    Serial.println(path);
  }
}

void listarArquivosFFat() {
  Serial.println("Arquivos na FFat:");
  File root = FFat.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Falha ao abrir diretório raiz da FFat.");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    Serial.printf("  %s\t%u bytes\n", file.name(), file.size());
    file = root.openNextFile();
  }
}

void deleteArqDados(fs::FS &fs, const char * path){
  if (fs.exists(path)) {    
    if (fs.remove(path)) {
      Serial.println("Arquivo removido com sucesso.");
    } else {      
      Serial.println("Erro ao remover arquivo:");
    }
  } else {
    Serial.println("Arquivo não existe.");
  }
}

// interrupção de hardware  --------------------
void IRAM_ATTR ISR_delArq() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceDelay) {
    flagDeletarArquivo = 1;
    Serial.println("Interrupção deleta arquivo temporario!"); 
    lastInterruptTime = currentTime; 
  }
}

// interrupção de hardware  --------------------
void IRAM_ATTR ISR_delArqAllData() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceDelay) {
    flagDeletaAllData = 1;
    Serial.println("Interrupção delete arquivo AllData!"); 
    lastInterruptTime = currentTime; 
  }
}

// interrupção de hardware  --------------------
void IRAM_ATTR ISR_readArqAllData() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceDelay) {
    flagReadAllData = 1;
    Serial.println("Interrupção Leitura do arquivo AllData!"); 
    lastInterruptTime = currentTime; 
  }
}

    
bool isConnect()
{
    modem.sendAT("+SMSTATE?");
    if (modem.waitResponse("+SMSTATE: ")) {
        String res =  modem.stream.readStringUntil('\r');
        // se retornar erro pode ser problema na comunicação e não que o sistema está desconectado
        if (res == "ERROR"){
          return false;
          modem.sendAT("+SMDISC");
          modem.waitResponse(3000);
        }
        return res.toInt();
    }
    return false;
}

bool sendpayload(const estruturaDadosEnsaio& data2send)
{    
    String payload;    
    //char buffer_payload[70];    
    //float valSensor = (float)(random(100, 1000))/100;    
    //char len[10];
    //dtostrf(valSensor, 5, 3, len);     
    
    payload = "{\"DT\":"+ String("\"") + data2send.dt+ String("\"") +
              ",\"NV\":" + data2send.press_M1[0] + 
              ",\"T\":" + data2send.chip_temp + 
              ",\"BAT\":" + data2send.per_bateria + 
              ",\"Vbat\":" + data2send.vol_bateria + 
              ",\"RSSI\":" + data2send.chip_RSSI +
              ",\"lat\":" + data2send.lat +
              ",\"lon\":" + data2send.lon +
              " }";

    // Message length, range: 0-1024
    String pubStr =  "+SMPUB=\"sensor_01\"," + String(payload.length()) + ",1,1";
    Serial.println(pubStr);       
    //delay(10);
    Serial.println(payload);
    
    modem.sendAT(pubStr);   
    if (modem.waitResponse(">") == 1) {
        modem.stream.write(payload.c_str(), payload.length());
        //Serial.print("\nTry publish payload: ");
        //Serial.println(payload);

        if (modem.waitResponse(3000)) {
            Serial.println("Send Packet success!");
            return true;
        } else {
            Serial.println("Send Packet failed!");
        }
    }
    return false;
}


bool saveArq(const char* filename, const estruturaDadosEnsaio& data) {
  //Grava arquivo
  File file = FFat.open(filename, "a");
  if (!file) {
    Serial.print("Failed to open file for writing: ");
    Serial.println(filename);
    return false;
  } else {
    Serial.print("Open file for writing: ");
    Serial.println(filename);
  }
  //Serial.println(data.dt);
  //Serial.println(data.press_M1[0]);
  //Serial.println(data.chip_temp);
  //Serial.println(data.per_bateria);
  // Write the struct to the file
  file.println(data.dt+","+data.press_M1[0]+","+data.chip_temp+","+data.per_bateria+","+data.vol_bateria+","+data.chip_RSSI+","+data.lat+","+data.lon);
  file.close();
  return true;
} 

String split(String strData){
  String delimiter = ",";
  int startIndex = 0;
  int endIndex;
  String item; 
  while ((endIndex = strData.indexOf(delimiter, startIndex)) != -1) {
    item = strData.substring(startIndex, endIndex);
    Serial.println(item);
    startIndex = endIndex + 1;
  }
  return item;
}

bool readArq2send(const char* filename) {
  bool flagApagarArquivo = true;
  if (FFat.exists(filename)) {
    File file = FFat.open(filename, FILE_READ);
    if (!file) {
      Serial.print("Failed to open file for reading: ");
      Serial.println(filename);
      return false;
    } 
    
    char datahora[25];   // "2025/06/22 18:15:32"
    float pres;          // 3.4545      
    float temp;          // -1.0
    int bat;
    int vbat;
    char rssi[6];;
    float lat;
    float lon;
    estruturaDadosEnsaio data;  
    
    while (file.available()) {
      String line = file.readStringUntil('\n');
      //Serial.println("Envinado dados do arquivo!");
      //Serial.println(line);     
     
      sscanf(line.c_str(), "%[^,],%f,%.1f,%d,%d,%[^,],%f,%f",datahora, &pres, &temp, &bat, &vbat, &rssi, &lat, &lon);
      data.dt = String(datahora);
      data.press_M1[0] = String(pres);
      data.chip_temp = String(temp);
      data.per_bateria = String(bat);      
      data.vol_bateria = String(vbat);      
      data.chip_RSSI = String(rssi);  
      data.lat = String(lat);  
      data.lon = String(lon);      
      //Enviando dados
      if (!sendpayload(data)){
        flagApagarArquivo = false;
      }
      //reset Watchdog 
      esp_task_wdt_reset();         
      file.println(data.dt +","+ data.press_M1[0] +","+ data.chip_temp +","+ data.per_bateria+","+ data.vol_bateria+","+ data.chip_RSSI+","+data.lat+","+data.lon);
      delay(1000);
    }
    file.close();
    
    // todos os dados enviados
    // apagar arquivo!!!
    if (flagApagarArquivo){
      deleteArqDados(FFat, cfg.nomeArq);   
    }   
  }
  return true;    
}

void setup()
{   
    pinMode(LigaVcc_PIN, OUTPUT);     //config pino de liga e desliga do 5v
    digitalWrite(LigaVcc_PIN, LOW);
    
    // Config sensor de temperatura interno
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
  
  
    Serial.begin(115200);
    delay(500);
 
    // Initialize SPIFFS
    if (!FFat.begin(true)) {
      Serial.println("FFat Mount Failed");
      return;
    }
    //else {
    //  deleteArqDados(FFat, cfg.nomeArq);     
    //}

    // Desliga watchdog
    esp_task_wdt_deinit(); //wdt is enabled by default, so we need to 'deinit' it first

    
    /*
    // Get all information of your FFat ===========================================
    unsigned int totalBytes = FFat.totalBytes();
    unsigned int usedBytes = FFat.usedBytes();
    unsigned int freeBytes  = FFat.freeBytes();
 
    Serial.println("File system info."); 
    Serial.print("Total space:      ");
    Serial.print(totalBytes);
    Serial.println("byte");
 
    Serial.print("Total space used: ");
    Serial.print(usedBytes);
    Serial.println("byte");
 
    Serial.print("Total space free: ");
    Serial.print(freeBytes);
    Serial.println("byte"); 
    Serial.println();
    */
    
    
    delay(1000);
    getWakeupReason();
    
    /*********************************
     *  step 1 : Initialize power chip,
     *  turn on modem and gps antenna power channel
    ***********************************/
    Serial.print("I2C PMU: ");
    Serial.println(AXP2101_SLAVE_ADDRESS);
    Serial.println(I2C_SDA);
    Serial.println(I2C_SCL);
    
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("Failed to initialize power.....");
        assert(0);
    }

    // Set the led light on to indicate that the board has been turned on
    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);


    // Turn off other unused power domains
    PMU.disableDC2();
    PMU.disableDC4();
    PMU.disableDC5();
    PMU.disableALDO1();

    PMU.disableALDO2();
    PMU.disableALDO3();
    PMU.disableALDO4();
    PMU.disableBLDO2();
    PMU.disableCPUSLDO();
    PMU.disableDLDO1();
    PMU.disableDLDO2();

    // ESP32S3 power supply cannot be turned off
    // PMU.disableDC1();

    // If it is a power cycle, turn off the modem power. Then restart it
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED ) {
        PMU.disableDC3();
        // Wait a minute
        delay(200);
    }

    //! Do not turn off BLDO1, which controls the 3.3V power supply for level conversion.
    //! If it is turned off, it will not be able to communicate with the modem normally
    PMU.setBLDO1Voltage(3300);    // Set the power supply for level conversion to 3300mV
    PMU.enableBLDO1();

    // Set the working voltage of the modem, please do not modify the parameters
    PMU.setDC3Voltage(3000);    // SIM7080 Modem main power channel 2700~ 3400V
    PMU.enableDC3();

    // Modem GPS Power channel
    PMU.setBLDO2Voltage(3300);
    PMU.enableBLDO2();      // The antenna power must be turned on to use the GPS function


    // DC5 Imax=2A
    PMU.setDC5Voltage(3300);
    PMU.enableDC5();      // sensor de pressão 3v3 para step up 9V
    
    // Enable internal ADC detection
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();
    //PMU.enableTemperatureMeasure();
    
    // TS Pin detection must be disable, otherwise it cannot be charged
    PMU.disableTSPinMeasure();
    
    /*********************************
     * step 2 : start modem
    ***********************************/

    Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);

    pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
    pinMode(BOARD_MODEM_DTR_PIN, OUTPUT);
    pinMode(BOARD_MODEM_RI_PIN, INPUT);

    /**
     *  When the user sets "AT+CSCLK=1", pull up the DTR pin,
     *  the module will automatically enter the sleep mode.
     *  At this time, the serial port function cannot communicate normally.
     *  Pulling DTR low in this mode can wake up the module
     */
    digitalWrite(BOARD_MODEM_DTR_PIN, LOW);

    
    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.print(".");
        if (retry++ > 10) {
            // Pull down PWRKEY for more than 1 second according to manual requirements
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
            retry = 0;
            Serial.println(F("***********************************************************"));
            Serial.println(F(" Failed to connect to the modem! Check the baud and try again."));
            Serial.println(F("***********************************************************\n"));
        }
    }
    Serial.println("Modem started!");

    /*
    First we configure the wake up source
    We set our ESP32 to wake up every 60 seconds
    */
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    Serial.print("Hora do esp:");  Serial.println(esp_get_time());

    
    modem.sendAT("+CMEE=2");
    modem.waitResponse();

    modem.sendAT("+CNMP=2");
    modem.waitResponse();

    modem.sendAT("+CMNB=3");
    modem.waitResponse();

  
    delay(2000);
    int8_t resCops;
    String strresCops;
    modem.sendAT("+COPS?");
    resCops = modem.waitResponse(30000);
    strresCops =  modem.stream.readStringUntil('\r');
    Serial.println(strresCops);
    
        
    modem.sendAT("+CNCFG=0,1,\"kiteiot.vivo.com.br\"");
    modem.waitResponse();
    
    modem.sendAT("+CNBS=1");
    modem.waitResponse();

    // Nova forma: usar a struct de configuração
    const esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,                 // em milissegundos
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Monitora ambos os núcleos
      .trigger_panic = true                             // Reinicia se travar
    };      
    // Inicializa com a struct
    esp_task_wdt_init(&wdt_config);      
    // Adiciona a tarefa principal (loop)
    esp_task_wdt_add(NULL);
    

    if (cfg.GPS_active) {
        // Config GPS  ==============================================================================
        // When configuring GNSS, you need to stop GPS first
        modem.disableGPS();
        delay(500);
        // GNSS Work Mode Set GPS+BEIDOU
        modem.sendAT("+CGNSMOD=1,0,1,0,0");
        modem.waitResponse();
         
        //modem.sendAT("+SGNSCMD=1,0");
        //modem.waitResponse(30000);
        
        // Turn off GNSS.
        //modem.sendAT("+SGNSCMD=0");
        //modem.waitResponse();
        //delay(500);
        
        // Disconnect the network. GPS and network cannot be enabled at the same time
        modem.gprsDisconnect();
        
        // GPS function needs to be enabled for the first use
        if (modem.enableGPS() == false) {
            Serial.print("Modem enable gps function failed!!");        
        }
        
        String result_gps;
        int cnt = 0;
        while (1) {
          if (cnt >= 50){
            break;
          }
          
          esp_task_wdt_reset();           //reset Watchdog 
            
          //Upload targeting data every 10 seconds
          modem.sendAT("+CGNSINF");
          modem.waitResponse(10000L, "+CGNSINF:");
          String result_gps = modem.stream.readStringUntil('\n');
          result_gps.trim();
          int commaIndex1 = result_gps.indexOf(',');                    //GNSS run status
          int commaIndex2 = result_gps.indexOf(',', commaIndex1 + 1);   // Fix Status
          int commaIndex3 = result_gps.indexOf(',', commaIndex2 + 1);   // UTC time
          int commaIndex4 = result_gps.indexOf(',', commaIndex3 + 1);   //Latitude
          int commaIndex5 = result_gps.indexOf(',', commaIndex4 + 1);   //Longitude
          int commaIndex6 = result_gps.indexOf(',', commaIndex5 + 1);   //MSL Altitude
          
          if (result_gps.substring(0, commaIndex1) == "1") {
            result.lat = result_gps.substring(commaIndex3 + 1, commaIndex4);
            result.lon = result_gps.substring(commaIndex4 + 1, commaIndex5);  
            Serial.print("Latitude: "); Serial.println(result.lat);
            Serial.print("Longitude: "); Serial.println(result.lon);
            //Serial.print("count: "); Serial.println(cnt);
            if (result_gps.substring(commaIndex1 + 1, commaIndex2) == "1") {
              break;
            }
          } 
          delay(2000);
          cnt = cnt+1;
        }
        // When you need to access the network after positioning, you need to disable GPS
        modem.disableGPS();
        // ======================================================================================
    }
    
    modem.sendAT("+CNACT=0,1");
    modem.waitResponse(30000);    
    delay(1000);
    modem.sendAT("+CNACT?");
    modem.waitResponse();
    
    delay(1000);
    modem.sendAT("+CGACT=1,1");
    modem.waitResponse();
    
    delay(1000);    
    int8_t ret_cntp;
    modem.sendAT("+cntp=200.160.7.186,-12,0,2");    //a.st1.ntp.br
    ret_cntp = modem.waitResponse(3000UL);
    //Serial.println(ret_cntp);
    if (ret_cntp != 1) {
      delay (2000);
      modem.sendAT("+cntp=200.160.7.186,-12,0,2");
      modem.waitResponse(3000UL);
    }
    
    delay(2000);
    esp_set_time();
    
    delay(2000);
    modem.sendAT("+CSQ"); 
    modem.waitResponse(10000UL,"+CSQ: ");
    String modem_rssi = modem.stream.readStringUntil('\n');
    String item;
    item = modem_rssi.substring(0, modem_rssi.indexOf(",", 0));    
    result.chip_RSSI = item;
    Serial.print("Potencia do sinal:");  Serial.println(result.chip_RSSI);
        
    delay(2000);
    modem.sendAT("+CGPADDR=1");
    modem.waitResponse();

    delay(2000);
    // If it is already connected, disconnect it first
    //modem.sendAT("+SMDISC");
    //modem.waitResponse();
    modem.sendAT("+SMCONF=\"URL\",\"" + cfg.NBiot_URL + "\",1883");
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"KEEPTIME\",90");
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"CLEANSS\",0");     //<cleanss> Session clean in. 0 persent session - 1 new session
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"QOS\",1");        //<qos> Send packet QOS level. 0 At most once 1 At lease once 2 Only once
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"RETAIN\",1");    //Retain identification. 0 Message will not be saved or removed or replaced 1 Message and its <qos> will be saved
    modem.waitResponse();
    
    modem.sendAT("+SMCONF=\"CLIENTID\",\"" + cfg.NBiot_client + "\"");
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"USERNAME\",\"" + cfg.NBiot_username + "\"");
    modem.waitResponse();
    modem.sendAT("+SMCONF=\"PASSWORD\",\"" + cfg.NBiot_password + "\"");
    modem.waitResponse();
    
       
    
    delay(1000);
    int8_t ret;
    int8_t count_conn = 0;
    modem.sendAT("+SMCONN");
    ret = modem.waitResponse(30000);
    do {
        if (ret != 1) {
            Serial.println(ret);
            Serial.println("Connect failed, retry connect ..."); delay(500);               
            
            modem.sendAT("+CNACT=0,1");
            modem.waitResponse(3000);
            //reset Watchdog 
            esp_task_wdt_reset(); 
            
            modem.sendAT("+CNACT?");            
            modem.waitResponse(3000);
            //reset Watchdog 
            esp_task_wdt_reset(); 
            
            modem.sendAT("+COPS=4,2,\"724010\"");
            modem.waitResponse(3000);
            //reset Watchdog 
            esp_task_wdt_reset(); 
            
            modem.sendAT("+COPS?");
            modem.waitResponse(3000);  
            //reset Watchdog 
            esp_task_wdt_reset(); 
            
            modem.sendAT("+SMDISC");
            modem.waitResponse();
    
            modem.sendAT("+SMCONF?");
            modem.waitResponse(3000);
            //reset Watchdog 
            esp_task_wdt_reset(); 
            
            modem.sendAT("+SMCONN");
            modem.waitResponse(3000);                              
            count_conn++;
            if (count_conn >= 3) {
              Serial.println("Connect failed");
              break;
            }
        } else {
            Serial.println("MQTT Client connected!");      
        }
    } while (ret != 1);
 
    uint8_t low_warn_per = PMU.getLowBatWarnThreshold();
    Serial.printf("Default low battery warning threshold is %d percentage\n", low_warn_per);
    uint8_t low_shutdown_per = PMU.getLowBatShutdownThreshold();
    Serial.printf("Default low battery shutdown threshold is %d percentage\n", low_shutdown_per);
    
    // setup ADC 
    Serial.println("setup ADC....");
    I2C_BUS.begin(SDA_PIN, SCL_PIN, 100000);
    if (!ads.begin(0x48, &I2C_BUS)) {
      Serial.println("Failed to initialize ADS.");
      adc_ini_fail = 1;      
    } else {
      adc_ini_fail = 0;
      ads.setGain(GAIN_ONE);                             // +/- 4.096V 1 bit = 2mV
      Serial.println("setup ADC Ok.");
    }

    pinMode(pinNotGo2sleep, INPUT_PULLDOWN);           // pino de controle - impede de ir para o sleep mode     

    pinMode(pinReadAllData, INPUT_PULLDOWN); 
    attachInterrupt(pinReadAllData, ISR_readArqAllData, RISING); //pino para interrupção para Leitura do arquivo de todas mas medições
    
    pinMode(pinDeletaAllData, INPUT_PULLDOWN); 
    attachInterrupt(pinDeletaAllData, ISR_delArqAllData, RISING); //pino para interrupção para deletar arquivo com todas as medições
    
    pinMode(pinDelArq, INPUT_PULLDOWN);                   //pino para interrupção para deletar arquivo de dados temporario
    attachInterrupt(pinDelArq, ISR_delArq, RISING); 

    previousMillisPar = millis();  
     
    //scannerI2C();    
}


float calculate_std_dev(int16_t data[], float mean, int num_values) {
    if (num_values <= 1) {
        return 0; // Standard deviation is undefined for 1 or fewer values
    }
    
    float sum_sq_diff = 0;

    for (int i = 0; i < num_values; i++) {
        float diff = data[i] - mean;
        sum_sq_diff += diff * diff; // Square the difference
    }

    // For sample standard deviation, divide by (num_values - 1)
    float variance = sum_sq_diff / (num_values - 1);
    return sqrt(variance);
}


void readSensor()
{   
    // Ligar a fonte de tensão do lilyGO
    digitalWrite(LigaVcc_PIN, HIGH);
    delay(600); // Atraso para estabilizar 
    
    //float vref = adc_chars.vref; // Obtain the device ADC reference voltage
    //Serial.print(F("vref is : ")); Serial.println(vref);

    long soma = 0;  
    long media = 0;
    int16_t  leitura;
    float voltage;
    int16_t  all_read[cfg.numLeituras];

    // verifica as tensões do sistema
    Serial.printf("Tensão DC5:%u", PMU.getDC5Voltage()); Serial.println("mV");      
    Serial.print("getBattVoltage:"); Serial.print(PMU.getBattVoltage()); Serial.println("mV");
    Serial.print("getVbusVoltage:"); Serial.print(PMU.getVbusVoltage()); Serial.println("mV");
    Serial.print("getSystemVoltage:"); Serial.print(PMU.getSystemVoltage()); Serial.println("mV");
    
    // ----------------------------------------------------------
    // Timestamp 
    //String dt;  
    //modem.sendAT("+CCLK?");
    //modem.waitResponse(2000UL,dt);
    //dt = modem.stream.readString();
    //printf("Data: %s \n", dt);    
    //result.dt = dt.substring(10,30);
    //result.dt = dt.substring(10,18);
    //result.hora = dt.substring(19,27);

    esp_set_time();
    result.dt = esp_get_time();
    /*
    if (result.dt.substring(0, 4).toInt() < 2000) {     // ano < 2000
      Serial.printf("esp_get_time fail! Retrail. %s \n",result.dt.substring(0, 4) );
      esp_set_time();
      result.dt = esp_get_time();      
    }
    */
    
    if (cfg.num_sensor > 4) {
       cfg.num_sensor = 1;
    }
    if (adc_ini_fail == 0) { 
      for (int sensor = 0; sensor < cfg.num_sensor; sensor++) {
        soma = 0;
        media = 0;
        for (int i = 0; i < cfg.numLeituras; i++) {
          leitura = ads.readADC_SingleEnded(sensor);       
          all_read[i] = leitura;   
          Serial.printf("ADC leitura = %d\n",leitura);            
          // voltage = (leitura * 0.125)/1000;
          // voltage = (leitura / 32768)* 4096;
          soma += leitura;     
          //reset Watchdog 
          esp_task_wdt_reset();
          delay(50); // Pequeno atraso para estabilizar leituras
        }  
        media = soma / cfg.numLeituras;
        desvio = calculate_std_dev(all_read, media, cfg.numLeituras); 
        
        //Serial.printf("ADC media = %d\n",media);
        //Serial.printf("ADC cfg = %d\n",cfg.numLeituras);    
        result.press_M1[sensor] = String(media);
        
        Serial.printf("ADC analog value = %s\n",result.press_M1[sensor]);    
        voltage = ads.computeVolts(leitura); 
        Serial.printf("ADC Analog Voltage = %.2f\n",voltage);       
        Serial.printf("ADC Desvio = %.2f\n",desvio);           
      }
    } else {
      Serial.print("ADC Fail!!!");
    }
    
    // DesLigar a fonte de tensão do LilyGO que alimenta o step up 
    digitalWrite(LigaVcc_PIN, LOW);
    
    // ----------------------------------------------------------
    // leitura do sensor de temeratura interna
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));    
    float temperature = 0;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &temperature));
    result.chip_temp = String(temperature);
    Serial.printf("Internal Temperature: %.2f °C\n", temperature);
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));


    // ----------------------------------------------------------
    // Battery information
    int bat_percent = PMU.getBatteryPercent();
    int bat_voltage = PMU.getBattVoltage();
    result.per_bateria = String(bat_percent);
    result.vol_bateria = String(bat_voltage);
    Serial.printf("Porcentagem da bateria: %.d %%\n", bat_percent);
    Serial.printf("Tensão da bateria: %.d % mV\n", bat_voltage);
    //modem.sendAT("+CBC");
    //modem.waitResponse();

    /*
    // modem sinal -----------------------------------------------
    modem.sendAT("+CSQ"); 
    modem.waitResponse(10000UL,"+CSQ: ");
    String modem_rssi = modem.stream.readStringUntil('\n');
    modem_rssi.trim();
    if (modem_rssi != "") {
      result.chip_RSSI = modem_rssi;
    }
    //Serial.print("Potencia do sinal:");  Serial.println(result.chip_RSSI);
    */
}  

void getWakeupReason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        //!< In case of deep sleep, reset was not caused by exit from deep sleep
        Serial.println("In case of deep sleep, reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_ALL:
        //!< Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source
        Serial.println("Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source");
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        //!< Wakeup caused by external signal using RTC_IO
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        //!< Wakeup caused by external signal using RTC_CNTL
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        //!< Wakeup caused by timer
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        //!< Wakeup caused by touchpad
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        //!< Wakeup caused by ULP program
        Serial.println("Wakeup caused by ULP program");
        break;
    case  ESP_SLEEP_WAKEUP_GPIO:
        //!< Wakeup caused by GPIO (light sleep only)
        Serial.println("Wakeup caused by GPIO (light sleep only)");
        break;
    case ESP_SLEEP_WAKEUP_UART:
        //!< Wakeup caused by UART (light sleep only)
        Serial.println("Wakeup caused by UART (light sleep only)");
        break;
    case ESP_SLEEP_WAKEUP_WIFI:
        //!< Wakeup caused by WIFI (light sleep only)
        Serial.println("Wakeup caused by WIFI (light sleep only)");
        break;
    case ESP_SLEEP_WAKEUP_COCPU:
        //!< Wakeup caused by COCPU int
        Serial.println("Wakeup caused by COCPU int");
        break;
    case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG:
        //!< Wakeup caused by COCPU crash
        Serial.println("Wakeup caused by COCPU crash");
        break;
    case  ESP_SLEEP_WAKEUP_BT:
        //!< Wakeup caused by BT (light sleep only)
        Serial.println("Wakeup caused by BT (light sleep only)");
        break;
    default :
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;

    }
}

void scannerI2C()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
        
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}




void loop()
{   
    //reset Watchdog 
    esp_task_wdt_reset();    

    // deleta arquivo de dados temporarios
    if (flagDeletarArquivo == 1){      
      flagDeletarArquivo = 0;
      deleteArqDados(FFat, cfg.nomeArq);      
    } 
    if (flagDeletaAllData == 1){      
      flagDeletaAllData = 0;
      deleteArqDados(FFat, cfg.nomeArqAllData);      
    } 
    
    if (flagReadAllData == 1) {
      flagReadAllData = 0;
      leArquivo(FFat, cfg.nomeArqAllData);   
    }
    
    if (millis() - previousMillisPar >= cfg.tempoAmostra)  {
        //Serial.println("scannerI2C....");
        //scannerI2C(); 
        
        listarArquivosFFat();
                                
        readSensor();                
        previousMillisPar = millis(); 
         
               
        // salva arquivo permanente com todas as leituras realizada
        saveArq(cfg.nomeArqAllData, result);

        /*
        int count = 0;
        while (!isConnect()) {
            //reset Watchdog 
            esp_task_wdt_reset();    
            Serial.println("MQTT Client disconnect!"); delay(1000); 
            modem.sendAT("+CFUN?");
            modem.waitResponse(3000);
                        
            modem.sendAT("+SMCONF");
            modem.waitResponse(3000);
            modem.sendAT("+SMCONN");
            modem.waitResponse(3000);          
            count++;
            if (count == 3) {
              Serial.println("salvar dados se a conexão falhar");
              saveArq(cfg.nomeArq, result);
              break;
            }
        }   
        */
        
        // leitura do arquivo temporario
        leArquivo(FFat, cfg.nomeArq); 
        
        //if (isConnect()) {
        readArq2send(cfg.nomeArq);
        Serial.println("Enviando dados...");
        if (!sendpayload(result)) { 
          Serial.println("Salvando dados - Falha de envio!");
          saveArq(cfg.nomeArq, result);            
        }
        //}
          
        if (digitalRead(pinNotGo2sleep) == HIGH) {
          Serial.println("Will NOT sleep!!!");
        } else {   
          
          Serial.println("Going to sleep now");
          Serial.flush();
          
          // You can turn off the modem or PMU power supply voltage
          // It can also keep the SIM7080G powered
          modem.poweroff();
          PMU.disableDC3();
          PMU.disableBLDO2();  
          PMU.disableDC5();
          
          esp_deep_sleep_start();
          Serial.println("This will never be printed");
       }   
    }
  
    while (SerialAT.available()) {        
        Serial.write(SerialAT.read());
    }    
    while (Serial.available()) {  
        SerialAT.write(Serial.read());        
    }
    delay(1);
}
