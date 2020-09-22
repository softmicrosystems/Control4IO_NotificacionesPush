
//version probada el 220920

//https://www.luisllamas.es/4-consejos-para-programar-codigo-mas-limpio-en-arduino/
//https://pubsubclient.knolleary.net/api
//https://github.com/knolleary/pubsubclient/issues/86

//solicitada p//si cuando da tension ya esta registrado en la red se enciende un led fijo (en ewelink ocupa el canal 1)
//mientras espera codigo, titila 3 veces con un separacion entre los destellos
//cuando aprendi-> da 5 beeps cortos
//Cuando no tiene ningun usuario debe oscila led estado LENTO
// Cuando tiene pore lo menos un usario, da piquiutos cada 5 segundos

//Cuando no tiene datos de red es la oscilacion de siempre
//Cuando no tiene internet -> es oscilacion rapida
//Cuando pulsa de5 svaegundos boton (Programacion) borra a default la red
//Cuando pulsa 10 segundos borra a default las programaciones

//Asegurae en mi plaqueta el led indicador con las polaridades, despuews adaptar a gonner

//A0:20:A6:12:58:15defaux
//131ab0sde
//3bd01dcac
//modulos actualesactmobu
//3bd1c1c
//43569rintentan
//125815
//3bcf3an
//1310b1

//https://programarfacil.com/esp8266/domotica-sonoff-wifi-espurna/https://programarfacil.com/esp8266/domotica-sonoff-wifi-espurna/
//https://www.itead.cc/sonoff-rf.html
//agregado de tx
//"modelo/idubicacion/IDModulo/TipoModulo/CanalRX/NivelSalida")
//y determina un 0 y 1 para apagar o encender salida
//como puede haber varias salida para un mismo receptor, se agrega salida de receptor (4 canales)
// la determinacion de como llegar a ese ubicacion se programa en el telefono mediante los graficos de tab/grupo/
//1- Ingresar tipos de modelo (automatico por el tipo de app android) -> Gon_PMG_AD
//2- Ubicacion geografica del modulo (casa o depto_ calle_piso_depto) -> Francia xxxx
//3- Id.Modulo    -> 125815
//4- Id.Receptor  -> 1,2 hasta 16
//5- Tipo Modulo  ->  0= Modulo alarma/PGM
//                    1= Receptor 2 o 4 canales
//                    2= salida PWM/LUZ

//5- CanalRx      -> caso Tipo=0 solo es PGM1 0 o 1
//                -> caso tipo=1 será 1,2  o 1,2,3,4 segun cual es la salida del receptor selecciondo
//                -> caso tipo=2 Cual de la lamparas sera controlado en intensidad (1 a 4)
//
//6- Nivel Salida -> 0 o 1  NOTA= si es una salida PWM (control luz el valor)S

//https://iotbyhvm.ooo/arduino-pubsubclient-arduino-client-for-mqtt/
// fcm.googleapis.com/fcm/send
//https://www.grc.com/fingerprints.htm
//https://github.com/francibm97/UM3750
//https://drive.google.com/drive/folders/0BzqRbfG5oG5XcXRTZzB2WmpzM3c

//https://techtutorialsx.com/2017/01/21/esp8266-watchdog-functions/
//https://github.com/FirebaseExtended/firebase-arduino/pull/401/commits/3530f7de36740a85dffe6f85333fbc91cbfacbfc
//https://dzone.com/articles/iot-push-notifications-arduino
//http://www.bujarra.com/poniendo-tasmota-en-un-sonoff-con-una-raspberry-pi/

//https://github.com/arendst/Sonoff-Tasmota/wiki/Commands

//http://www.bujarra.com/usando-un-lector-de-huellas-dactilares-en-raspberry-pi/#more-22425

//https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/

//https://firebase.google.com/docs/reference/fcm/rest/v1/projects.messages#androidnotification

//35956 3081245999    Motorola
// 5060141652    Lg viejo
// 3090066175    Lg nuevo?r
// 4098062429    quantumtesta

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#else
#include <WiFi.h>          //https://github.com/esp8266/Arduino
#endif

//needed for librarys
//#include <DNSServer.h>

#if defined(ESP8266)
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#else
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager
#endif

#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <PubSubClient.h>

//______________________________________________________
#define FIREBASE_HOST "controlalarmasgonner.firebaseio.com"
const int timeZone = -3; // Central European Time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//________________________________
/*
  //ENTRADAS DE BORNERA  BORNERA NODEMCU
  int PinArmDesarm = 2;        //D4
  int PinInSir    = 14;       //D5

  int PinLedVerde = 15; //D8
  int PinInPanico = 13;    //D7

  // SALIDAS DE BORNERAs lee
  //int PinOut1Pgm = 0;   //D3
  //int PinOut2Pgm = 4;   //D2
  int PinOutRemoto = 5; //D1
  int PinLedVerde = 16; //D0
*/
//________________________________
//ENTRADAS DE BORNERA  BORNERA NODEMCU     PINOUT WRCOM2
int PinEntrada1 = 2;    //D4
int PinEntrada2 = 14;   //D5
int PinEntrada3 = 12;   //D6
int PinEntrada4 = 13;   //D7

// SALIDAS DE BORNERAs lee
int PinOut1Pgm = 0;     //D3
int PinOut2Pgm = 4;     //D2
int PinOut3Pgm = 5;     //D1
int PinOut4Pgm = 16;    //D0

int PinLedVerde = 15;   //D8

int PinInAux    = 12;

//_________________
//Posiciones eeprom de parametros a usar
const int E2_IDUsua1 = 00;
const int E2_IDUsua2 = 10;
const int E2_IDUsua3 = 20;
const int E2_IDUsua4 = 30;
//_____
const int E2_NombreUsua1 = 80;
const int E2_NombreUsua2 = 100;
const int E2_NombreUsua3 = 120;
const int E2_NombreUsua4 = 140;
//________
const int E2_UbicacionMod = 240;
const int E2_RutIn1Usua = 270;     //guarda en un byte con posicion de cada usuario habilitadio para enviar entrada1!
const int E2_RutIn2Usua = 271;
const int E2_RutIn3Usua = 272;
const int E2_RutIn4Usua = 273;

const int E2_RutOut1Usua = 274;     //guarda en un byte con posicion de cada usuario habilitadio para enviar salida1!
const int E2_RutOut2Usua = 275;
const int E2_RutOut3Usua = 276;
const int E2_RutOut4Usua = 277;

const int E2TimPGM1 = 278;
const int E2TimPGM2 = 279;
const int E2TimPGM3 = 280;
const int E2TimPGM4 = 281;

const int E2ModoPGM1 = 282;
const int E2ModoPGM2 = 283;
const int E2ModoPGM3 = 284;
const int E2ModoPGM4 = 285;

const int E2_UBICA_IN1 = 290;
const int E2_UBICA_IN2 = 310;
const int E2_UBICA_IN3 = 330;
const int E2_UBICA_IN4 = 350;

const int E2_UBICA_OUT1 = 370;
const int E2_UBICA_OUT2 = 390;
const int E2_UBICA_OUT3 = 410;
const int E2_UBICA_OUT4 = 430;

const int E2_RUTEO_IN1 = 450;
const int E2_RUTEO_IN2 = 451;
const int E2_RUTEO_IN3 = 452;
const int E2_RUTEO_IN4 = 453;

const int E2_RUTEO_OUT1 = 454;
const int E2_RUTEO_OUT2 = 455;
const int E2_RUTEO_OUT3 = 456;
const int E2_RUTEO_OUT4 = 457;

const int E2_TIPO_LAZO_IN1 = 458;
const int E2_TIPO_LAZO_IN2 = 459;
const int E2_TIPO_LAZO_IN3 = 460;
const int E2_TIPO_LAZO_IN4 = 461;

const int E2_VINC_IN1 = 462;
const int E2_VINC_IN2 = 463;
const int E2_VINC_IN3 = 464;
const int E2_VINC_IN4 = 465;

const int E2_VINC_OUT1 = 466;
const int E2_VINC_OUT2 = 467;
const int E2_VINC_OUT3 = 468;
const int E2_VINC_OUT4 = 469;

const int E2_TIM_CONF_IN1 = 470;
const int E2_TIM_CONF_IN2 = 471;
const int E2_TIM_CONF_IN3 = 472;
const int E2_TIM_CONF_IN4 = 473;
const int InicE2 = 474;


String clientId;

String ValUbicacionMod;
String UsuarioGenerador;
String Titulo;
String Descripcion;
String TituloEvento = "Evento en " + ValUbicacionMod;
String TextoNovedad = Descripcion;
int count;
int n = 0;
String temporal;
String MensajeEnviar;
String GeneradorPublicacion;
String HoraFechaActual;
String string_variable;
String ValorBuscado;
String IdModuloDestinatario;
String ImeiAutorizado;
String DirSubscribe;

//byte DebeRepetirEvento;
byte posicion;
byte ValModoOutAD;
byte ValTimOutAD;
byte HayInternet;
byte HayDeteccionAux;
byte HayUsuario1, HayUsuario2, HayUsuario3, HayUsuario4;
byte EstaContandoTiempoResetRed;
byte EstaContandoTiempoDefault;
byte SyncLedAmarillo;
byte cntFaltaInternet;
//byte NoHayConexionMQTT;
byte ExisteUsuario;
String NombreUsuaEmisor;
byte mask;

byte DebeContarTiempoDefault;
byte YaContol5SegRstSSID;
byte YaContoTiempoDefault;
byte TotalLeds;
byte HayNPEnviada;

//_____________
byte EstaEnNotificacion;
byte StIn1, StIn2, StIn3, StIn4;
byte ValRutIn1, ValRutIn2, ValRutIn3, ValRutIn4;
byte ValRutOut1, ValRutOut2, ValRutOut3, ValRutOut4;
byte ValRutOut1Usua, ValRutOut2Usua, ValRutOut3Usua, ValRutOut4Usua;
byte ValHabUsuaIN1, ValRutIn2Usua, ValRutIn3Usua, ValRutIn4Usua;
String ValIdReporte1, ValIdReporte2, ValIdReporte3, ValIdReporte4;
String ValUbicacionID1, ValUbicacionID2, ValUbicacionID3, ValUbicacionID4;
String ValUbicaIN1, ValUbicaIN2, ValUbicaIN3, ValUbicaIN4;
String ValUbicaOUT1, ValUbicaOUT2, ValUbicaOUT3, ValUbicaOUT4;

byte ValVincIn1Out1, ValVincIn1Out2, ValVincIn1Out3, ValVincIn1Out4;
byte ValVincIn2Out1, ValVincIn2Out2, ValVincIn2Out3, ValVincIn2Out4;
byte ValVincIn3Out1, ValVincIn3Out2, ValVincIn3Out3, ValVincIn3Out4;
byte ValVincIn4Out1, ValVincIn4Out2, ValVincIn4Out3, ValVincIn4Out4;

byte ValLazoIn1, ValLazoIn2, ValLazoIn3, ValLazoIn4;
byte ValVincIn1, ValVincIn2, ValVincIn3, ValVincIn4;

byte ValTConfIn1, ValTConfIn2, ValTConfIn3, ValTConfIn4;
byte ValTimPGM1, ValTimPGM2, ValTimPGM3, ValTimPGM4;
byte ValModoPGM1, ValModoPGM2, ValModoPGM3, ValModoPGM4;
byte ValModoCuentaPGM1, ValModoCuentaPGM2, ValModoCuentaPGM3, ValModoCuentaPGM4;
byte ValRutId1, ValRutId2, ValRutId3, ValRutId4, ValRutId5, ValRutId6, ValRutId7, ValRutId8;



byte Usuarios;
int tempo;
byte MuestraCuentaMinutos;
byte CoincideDiaSemanaActual;
byte EsAccionLocal;
byte CntTimDefault;


byte DebeDesactivarSalida1, DebeDesactivarSalida2, DebeDesactivarSalida3, DebeDesactivarSalida4;

byte DesactivaPorTiempoPGM1, DesactivaPorTiempoPGM2, DesactivaPorTiempoPGM3, DesactivaPorTiempoPGM4;

byte AvisarCambioIDUsuarios;
byte TimTimerIn1, TimTimerIn2, TimTimerIn3, TimTimerIn4;

byte EsModoShare;
byte YaInicializoEEprom;
byte Respuesta;

byte EstaArmado;
byte StSir;
byte posSalida;

byte DebeResponderAOrigen;
byte DebeEnviarNotificacionesSegunRuteo;

int CntTimOut1, CntTimOut2, CntTimOut3, CntTimOut4;

byte LedTest;
byte HayDeteccionPanico;
byte DebeOscErrStatus;
byte HayAvisoFaltaInternet;
bool Start = false;

byte StPGM1, StPGM2, StPGM3, StPGM4;
byte YaDetectoIn1, YaDetectoIn2, YaDetectoIn3, YaDetectoIn4;

//________________________________________
String UsuarioEmisor;
String valCompara;
String DiaFiltrado;
String OrigenTel;
String ValTim1, ValTim2, ValTim3, ValTim4, ValTim5, ValTim6, ValTim7, ValTim8;
String RamHHTim1, RamMMTim1, RamSelDias1, RamModoAct1;
String RamHHTim2, RamMMTim2, RamSelDias2, RamModoAct2;
String RamHHTim3, RamMMTim3, RamSelDias3, RamModoAct3;
String RamHHTim4, RamMMTim4, RamSelDias4, RamModoAct4;
String RamHHTim5, RamMMTim5, RamSelDias5, RamModoAct5;
String RamHHTim6, RamMMTim6, RamSelDias6, RamModoAct6;
String RamHHTim7, RamMMTim7, RamSelDias7, RamModoAct7;
String RamHHTim8, RamMMTim8, RamSelDias8, RamModoAct8;

String RamDiaActual, RamMesActual, RamAnoActual, RamHoraActual, RamMinuteActual, RamDiaSemanaActual;

int a;
int b;
int c;
int d;
unsigned long timeLastCheck = 0;
unsigned long intervalCheck = 4000;

//_____
//String UbicacionIdUsuarioProgramador;
String DirPublicacion;

String IDOrigenEnvio;
String stringOne;
String DatoRecibido;
String resultado1;
String IdImeiUsuarioNuevo;
String NombreUsuarioNuevo;
String TxtUbicaModulo;

String DireccionOrigen;
String ValClaveAdmin;
String ValClaveUsua;
String ModuloTest;
String ComandoActualizaValores;
String RamValClvAdmin;


//const int mqttPort = 1883;
//const char* mqttServer = "167.71.119.247";
//const char* mqttUser = "tecnosoftmicro";
//const char* mqttPass = "TecnoMicro54Mqtt";


const int mqttPort = 1883;
const char* mqttServer = "ioticos.org";
const char* mqttUser = "MuCaxo96Gx7MK7k";
const char* mqttPass = "jp2i3ued0J355KN";
const char* root_topic_subscribe = "GN8rYIiMtH9Tt89";
const char* root_topic_publish = "GN8rYIiMtH9Tt89/output";


//___________________________________________
WiFiClient espClient;
PubSubClient MQTTClient(espClient);
String IdOrigen;

int value;
int pos, pos1;
//_____
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 2500;

unsigned long startMillis1;
unsigned long currentMillis1;
const unsigned long period1 = 60000;
//___________________
int CntContMs;
int CntSegundos;
int CntMinutos;

//___
int CntSegundosPGM1, CntSegundosPGM2, CntSegundosPGM3, CntSegundosPGM4;
int CntMinutosPGM1, CntMinutosPGM2, CntMinutosPGM3, CntMinutosPGM4;
int CntHorasPGM1, CntHorasPGM2, CntHorasPGM3, CntHorasPGM4;

byte DebeContarSegPGM1, DebeContarSegPGM2, DebeContarSegPGM3, DebeContarSegPGM4;
byte DebeContarMinPGM1, DebeContarMinPGM2, DebeContarMinPGM3, DebeContarMinPGM4;
byte DebeContarHrPGM1, DebeContarHrPGM2, DebeContarHrPGM3, DebeContarHrPGM4;

String resultado;
String variable;
String recibido;
String NumSerialESP;


int YaDetectoEntradaSirena = 0;
int YaEnvioCambioPGM1 = 0;
byte EnvioDetArmado;
String url;
//String Origen= "" + IdOrigen + "";

String IdDestino;
byte estadetectandoNoConnect;

WiFiUDP Udp;
unsigned int localPort = 8888; // local port to listen for UDP packets
Ticker tickerSSID;
Ticker blinker;

int address;
int posicionWr;
int DatoE2;
String CmdBasico = "&from=" + IdOrigen + "&to=" + IdDestino + "&data=";
String CmdAccion;
String TopicoDestino;

//_____________________
void setup() {
  Serial.begin(115200);
  pinMode(PinOut1Pgm, OUTPUT);
  pinMode(PinOut2Pgm, OUTPUT);
  pinMode(PinOut3Pgm, OUTPUT);
  pinMode(PinOut4Pgm, OUTPUT);
  pinMode(PinLedVerde, OUTPUT);
  digitalWrite(PinOut1Pgm, LOW);
  digitalWrite(PinOut2Pgm, LOW);
  digitalWrite(PinOut3Pgm, LOW);
  digitalWrite(PinOut4Pgm, LOW);
  digitalWrite(PinLedVerde, LOW);

  pinMode(PinEntrada1, INPUT_PULLUP);
  pinMode(PinEntrada2, INPUT_PULLUP);
  pinMode(PinEntrada3, INPUT_PULLUP);
  pinMode(PinEntrada4, INPUT_PULLUP);

  //___________________
  //Local intialization. Once its business is done, there is no need to keep it around
  //start tickerSSID with 0.5 because we start in AP mode and try to connect
  tickerSSID.attach(0.6, tick);
  blinker.attach(0.1, Timers);
  WiFiManager wifiManager;
  wifiManager.setTimeout(120);
  wifiManager.setBreakAfterConfig(true);
  //wifiManager.resetSettings();
  NumSerialESP = String(ESP.getChipId(), HEX);
  if (NumSerialESP.length() < 6 ) {
    NumSerialESP = "0" + NumSerialESP;
  }

  Serial.println(NumSerialESP);
  String NumSerialESP1 = String(ESP.getChipId(), DEC);
  Serial.println(NumSerialESP1);

  wifiManager.autoConnect();
  Serial.println("__________________");
  if (!wifiManager.autoConnect())  {
    Serial.println("Fallo conexion, debera resetear para ver si conecta");
    tickerSSID.detach();
    delay(3000);
    for (int i = 0; i < 5; i++) {
      digitalWrite(PinLedVerde, LOW); //led verde
      delay(50);
      digitalWrite(PinLedVerde, HIGH);
      delay(50);
    }
    Serial.println("Va a resetear por falta de red!");
    ESP.reset();
    delay(2000);
  }

  Serial.println("__________________");
  //if you get here you have connected to the WiFi
  Serial.println("Conectado a la red :)");
  tickerSSID.detach();
  //keep LED on
  digitalWrite(PinLedVerde, LOW);
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());

  EEPROM.begin(2048);
  // ESP.wdtDisable();   ???????????????????????????????????
  ESP.wdtEnable(5000);

  //________
  address = InicE2;
  RdE2();
  YaInicializoEEprom = Respuesta;

  if (YaInicializoEEprom != 51)  {
    //BorraE2();
    CargaDefault();
    DatoE2 = 51;
    address = InicE2;
    WrE2();
    //cleansession=false
    //Serial.println("Grabacion completa");
  }

  digitalWrite(PinLedVerde, HIGH);
  timeClient.begin();
  timeClient.setTimeOffset(-10800);

  //if (digitalRead(PinArmDesarm) == 0) {
  //  Serial.println("La entrada esta en 0 armado -> debe ver estado previo al reset");
  //address = E2_ST_ARMED;
  //RdE2();
  //if (Respuesta == 1) {
  //  EnvioDetArmado = 1; //SI ARRANCA ARMADO ES PORQUE YA ESTABA ARMADO
  //} else {
  //  Serial.println("La entrada esta en 1 -> desarmado, No hace nada");
  //}
  //}

  //ESP.wdtDisable();
  //ESP.wdtEnable(5000);
  GetExternalIP();
  delay(3000);
  /*
    mySwitch = RCSwitch();
    mySwitch.enableTransmit(16);
    mySwitch.setPulseLength(1);
    mySwitch.setProtocol(6);
    mySwitch.setRepeatTransmit(15);
  */
  MQTTClient.setServer(mqttServer, mqttPort);
  MQTTClient.setCallback(callback);

  while (!MQTTClient.connected()) {
    if (MQTTClient.connect("MQTT4IO", mqttUser, mqttPass)) {
      Serial.println("SE HA CONECTADO EL MQTT");
      Serial.println("______________________");
      Serial.println(" ");

      DirSubscribe = "GN8rYIiMtH9Tt89/";
      DirSubscribe = DirSubscribe + String(NumSerialESP).c_str();
      Serial.println("Subscribe a : " + DirSubscribe);
      Serial.println("_____________________");

      temporal = "SOFTMICRO4IO";
      MQTTClient.subscribe("GN8rYIiMtH9Tt89/SOFTMICRO4IO");   //String(temporal).c_str());   //subscripcion universal!
      Serial.println("Acaba de subscribirse a SOFTMICRO4IO!");
      LeeDatosMemoria();
      Serial.println("_____________________");
      delay(3000);

    } else {
      Serial.print("failed with state ");
      Serial.print(MQTTClient.state());
      //NoHayConexionMQTT = 1;
      return;
    }
  }
}

time_t prevDisplay = 0; // when the digital clock was displayed

//________________________________________
void CargaDefault() {
  Serial.println("Grabando default de eeprom");

  CargaDefaultUbicacion();
  LeeUbicacionModulo();

  Carga_Default_ID_Usuario1();
  BorraE2promNombreUsuario1();

  Carga_Default_ID_Usuario2();
  BorraE2promNombreUsuario2();

  Carga_Default_ID_Usuario3();
  BorraE2promNombreUsuario3();

  Carga_Default_ID_Usuario4();
  BorraE2promNombreUsuario4();

  Carga_Default_Ubica_IN1();
  Carga_Default_Ubica_IN2();
  Carga_Default_Ubica_IN3();
  Carga_Default_Ubica_IN4();

  Carga_Default_Ubica_OUT1();
  Carga_Default_Ubica_OUT2();
  Carga_Default_Ubica_OUT3();
  Carga_Default_Ubica_OUT4();

  DatoE2 = 2;
  address = E2ModoPGM1;
  WrE2();
  address = E2ModoPGM2;
  WrE2();
  address = E2ModoPGM3;
  WrE2();
  address = E2ModoPGM4;
  WrE2();

  DatoE2 = 1;
  address = E2TimPGM1;
  WrE2();
  address = E2TimPGM2;
  WrE2();
  address = E2TimPGM3;
  WrE2();
  address = E2TimPGM4;
  WrE2();

  Carga_Default_Ruteo_IN();
  Carga_Default_Ruteo_OUT();
  Carga_Default_LAZO_IN();
  Carga_Default_VINC_IN();

  Carga_Default_TCONF_IN();

  DatoE2 = 01;
  address = E2_RutIn1Usua;    //indica en posicion de bit para el byte que usuarios estan haniliotados para detecciones en entrada 1
  WrE2();
  address = E2_RutIn2Usua;    //Idem para entrada 2
  WrE2();
  address = E2_RutIn3Usua;    //Idem para entrada 3
  WrE2();
  address = E2_RutIn4Usua;    //Idem para entrada 4
  WrE2();

  address = E2_RutOut1Usua;   //Idem para salida 1
  WrE2();
  address = E2_RutOut2Usua;   //Idem para salida 2
  WrE2();
  address = E2_RutOut3Usua;   //Idem para salida 3
  WrE2();
  address = E2_RutOut4Usua;   //Idem para salida 4
  WrE2();
}

void Carga_Default_LAZO_IN() {
  DatoE2 = 0;
  address = E2_TIPO_LAZO_IN1;
  WrE2();
  address = E2_TIPO_LAZO_IN2;
  WrE2();
  address = E2_TIPO_LAZO_IN3;
  WrE2();
  address = E2_TIPO_LAZO_IN4;
  WrE2();
}

void Carga_Default_Ubica_IN1() {
  address = E2_UBICA_IN1;
  //string_variable = "Ubicación entrada1";
  string_variable = "entrada 1         ";
  GrabaPaqueteE2();
  LeeUbicacionIN1();
}

void Carga_Default_Ubica_IN2() {
  address = E2_UBICA_IN2;
  string_variable = "entrada 2         ";
  GrabaPaqueteE2();
  LeeUbicacionIN2();
}

void Carga_Default_Ubica_IN3() {
  address = E2_UBICA_IN3;
  string_variable = "entrada 3         ";
  GrabaPaqueteE2();
  LeeUbicacionIN3();
}

void Carga_Default_Ubica_IN4() {
  address = E2_UBICA_IN4;
  string_variable = "entrada 4         ";
  GrabaPaqueteE2();
  LeeUbicacionIN4();
}

void Carga_Default_Ubica_OUT1() {
  address = E2_UBICA_OUT1;
  //string_variable = "Ubicación salida 1  ";
  string_variable = "salida 1          ";
  GrabaPaqueteE2();
  LeeUbicacionOUT1();
}

void Carga_Default_Ubica_OUT2() {
  address = E2_UBICA_OUT2;
  string_variable = "salida 2          ";
  GrabaPaqueteE2();
  LeeUbicacionOUT2();
}

void Carga_Default_Ubica_OUT3() {
  address = E2_UBICA_OUT3;
  string_variable = "salida 3          ";
  GrabaPaqueteE2();
  LeeUbicacionOUT3();
}

void Carga_Default_Ubica_OUT4() {
  address = E2_UBICA_OUT4;
  string_variable = "salida 4          ";;
  GrabaPaqueteE2();
  LeeUbicacionOUT4();
}

void CargaDefaultUbicacion() {
  address = E2_UbicacionMod;
  string_variable = "ubicacion de modulo ";
  GrabaPaqueteE2();
}

void Carga_Default_VINC_IN() {
  DatoE2 = 0;
  address = E2_VINC_IN1;
  WrE2();
  address = E2_VINC_IN2;
  WrE2();
  address = E2_VINC_IN3;
  WrE2();
  address = E2_VINC_IN4;
  WrE2();
}

void Carga_Default_VINC_OUT() {
  DatoE2 = 0;
  address = E2_VINC_OUT1;
  WrE2();
  address = E2_VINC_OUT2;
  WrE2();
  address = E2_VINC_OUT3;
  WrE2();
  address = E2_VINC_OUT4;
  WrE2();
}

void Carga_Default_TCONF_IN() {
  DatoE2 = 0;
  address = E2_TIM_CONF_IN1;
  WrE2();
  address = E2_TIM_CONF_IN2;
  WrE2();
  address = E2_TIM_CONF_IN3;
  WrE2();
  address = E2_TIM_CONF_IN4;
  WrE2();
}

void Carga_Default_Ruteo_IN() {
  DatoE2 = 3;
  address = E2_RUTEO_IN1;
  WrE2();
  address = E2_RUTEO_IN2;
  WrE2();
  address = E2_RUTEO_IN3;
  WrE2();
  address = E2_RUTEO_IN4;
  WrE2();
}

void Carga_Default_Ruteo_OUT() {
  DatoE2 = 3;
  address = E2_RUTEO_OUT1;
  WrE2();
  address = E2_RUTEO_OUT2;
  WrE2();
  address = E2_RUTEO_OUT3;
  WrE2();
  address = E2_RUTEO_OUT4;
  WrE2();
}

void LeeUbicacionOUT1() {
  address = E2_UBICA_OUT1;
  ValUbicaOUT1 = LeeComunUbicacionZonas();
  ValUbicaOUT1.trim();
  Serial.println("Ubicacion Salida Z1 = " + ValUbicaOUT1);
}

void LeeUbicacionOUT2() {
  address = E2_UBICA_OUT2;
  ValUbicaOUT2 = LeeComunUbicacionZonas();
  ValUbicaOUT2.trim();
  Serial.println("Ubicacion Salida Z2 = " + ValUbicaOUT2);
}

void LeeUbicacionOUT3() {
  address = E2_UBICA_OUT3;
  ValUbicaOUT3 = LeeComunUbicacionZonas();
  ValUbicaOUT3.trim();
  Serial.println("Ubicacion Salida Z3 = " + ValUbicaOUT3);
}

void LeeUbicacionOUT4() {
  address = E2_UBICA_OUT4;
  ValUbicaOUT4 = LeeComunUbicacionZonas();
  ValUbicaOUT4.trim();
  Serial.println("Ubicacion Salida Z4 = " + ValUbicaOUT4);
}

String LeeComunUbicacionZonas() {
  String ValIdUsuario = "";
  for (int i = address; i < address + 18; i++) {
    ValIdUsuario = ValIdUsuario + char(EEPROM.read(i));
  }
  return ValIdUsuario;
}

void LeeUbicacionIN1() {
  address = E2_UBICA_IN1;
  ValUbicaIN1 = LeeComunUbicacionZonas();
  ValUbicaIN1.trim();
  Serial.println("Ubicacion entrada Z1 = " + ValUbicaIN1);
}

void LeeUbicacionIN2() {
  address = E2_UBICA_IN2;
  ValUbicaIN2 = LeeComunUbicacionZonas();
  ValUbicaIN2.trim();
  Serial.println("Ubicacion entrada Z2 = " + ValUbicaIN2);
}

void LeeUbicacionIN3() {
  address = E2_UBICA_IN3;
  ValUbicaIN3 = LeeComunUbicacionZonas();
  ValUbicaIN3.trim();
  Serial.println("Ubicacion entrada Z3 = " + ValUbicaIN3);
}

void LeeUbicacionIN4() {
  address = E2_UBICA_IN4;
  ValUbicaIN4 = LeeComunUbicacionZonas();
  ValUbicaIN4.trim();
  Serial.println("Ubicacion entrada Z4 = " + ValUbicaIN4);
}

void tick() {
  //toggle state
  int state = digitalRead(PinLedVerde); // get the current state of GPIO1 pin
  digitalWrite(PinLedVerde, !state);    // set pin to the opposite state
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  tickerSSID.attach(0.4, tick);
}


void loop() {
  ChequeaEnvioPaquete();
  AnalizaStringRecibido();
  CuentaMinutosReloj();

  currentMillis1 = millis(); //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis1 - startMillis1 >= period1) {
    startMillis1 = currentMillis1;
    GetExternalIP();
  }
  currentMillis = millis(); //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  {
    startMillis = currentMillis;
  }
  LeeEntrada1();
  LeeEntrada2();
  LeeEntrada3();
  LeeEntrada4();
  if (!MQTTClient.connected()) {
    reconnect();
  }
  MQTTClient.loop();
}


//___________________________
void OscilaLedStatus() {
  if (DebeOscErrStatus == 1)  {
    if (LedTest == 1)    {
      digitalWrite(PinLedVerde, LOW); //apaga led
      LedTest = 0;
    } else {
      digitalWrite(PinLedVerde, HIGH); //Activa leg
      LedTest = 1;
    }
  }
}

void Timers() {
  OscilaLedStatus();
  CntContMs = CntContMs + 1;
  if (CntContMs >= 10) {
    CntContMs = 0;
    CntSegundos++;      //contado de segundos
    //Serial.println(CntSegundos);
    //Serial.print(".");
    if (CntSegundos == 60)    {
      CntSegundos = 0;
      MuestraCuentaMinutos = 1;
    }

    ControlaTimSegPGM1();
    ControlaTimSegPGM2();
    ControlaTimSegPGM3();
    ControlaTimSegPGM4();

    ControlaTiempoDefSSID();

    ControlaTimMinPGM1();
    ControlaTimMinPGM2();
    ControlaTimMinPGM3();
    ControlaTimMinPGM4();

    ControlaTimHorasPGM1();
    ControlaTimHorasPGM2();
    ControlaTimHorasPGM3();
    ControlaTimHorasPGM4();
  }
}

//______________
// decodifica q10 posiciones del usuario y 20 de la ubicacion
void GrabaE2_IDUsuaProgramado() {
  Serial.println("Telefono a guardar : " + resultado);
  int j = 0;
  for (int i = address; i < address + resultado.length(); i++)  {
    EEPROM.write(i, resultado[j]); //Write one by one with starting address of 0x0F
    j = j + 1;
  }
  EEPROM.commit(); //Store data to EEPROM
}

void GrabaE2_UbicacionUsuarios() {
  // resultado = recibido.substring(pos + 16, pos + 36);
  Serial.println("Identificacion del usuario : " + resultado);
  int j = 0;
  for (int i = address; i < address + resultado.length(); i++) {
    EEPROM.write(i, resultado[j]); //Write one by one with starting address of 0x0F
    j = j + 1;
  }
  EEPROM.commit(); //Store data to EEPROM
}

void RdE2() {
  byte value;
  // read a byte from the current address of the EEPROM
  value = EEPROM.read(address);
  Respuesta = value;
  //Serial.println(Respuesta);
  //Serial.println("____________");
}

void WrE2() {
  //EEPROM Write
  EEPROM.write(address, DatoE2);
  EEPROM.commit();
  //Serial.println( "Grabo dato en eeprom");
  //Serial.println(DatoE2);
}

void BorraE2() {
  //Serial.println("Borrando E2prom");
  String resultado;
  EEPROM.begin(512);
  String stringOne = "Borrando posicion= ";
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < 1500; i++)  {
    EEPROM.write(i, 0);
    resultado = stringOne + i;
    Serial.println(resultado);
    delay(20);
  }
  EEPROM.commit();
  //Serial.println("Fin de BORRADO");
}

void GrabaPaqueteE2() {
  int j = 0;
  for (int i = address; i < address + string_variable.length(); i++) {
    EEPROM.write(i, string_variable[j]); //Write one by one with starting address of 0x0F
    j++;                                 //=j+1;
  }
  EEPROM.commit();
}

void Carga_Default_ID_Usuario1() {
  address = E2_IDUsua1;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario1();
}

void Carga_Default_ID_Usuario2() {
  address = E2_IDUsua2;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario2();
}

void Carga_Default_ID_Usuario3() {
  address = E2_IDUsua3;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario3();
}

void Carga_Default_ID_Usuario4() {
  address = E2_IDUsua4;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario4();
}

void LeeUbicacionModulo() {
  // lee datos de ubicacion
  address = E2_UbicacionMod;
  ValUbicacionMod = "";
  for (int i = address; i < address + 20; i++)
  {
    ValUbicacionMod = ValUbicacionMod + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  Serial.println("Ubicacion del modulo= " + ValUbicacionMod);
}

void LeeDatosMemoria() {
  LeeIDUsuario1();
  LeeIDUsuario2();
  LeeIDUsuario3();
  LeeIDUsuario4();
  LeeUbicacionModulo();

  LeeUbicaUsuario1();
  LeeUbicaUsuario2();
  LeeUbicaUsuario3();
  LeeUbicaUsuario4();

  LeeUbicacionIN1();
  LeeUbicacionIN2();
  LeeUbicacionIN3();
  LeeUbicacionIN4();
  LeeUbicacionOUT1();
  LeeUbicacionOUT2();
  LeeUbicacionOUT3();
  LeeUbicacionOUT4();

  //LeeValorClaveAdmin();
  //LeeValorClaveUsuario();
  LeeValoresRuteo();

  LeeRuteoIn1();
  LeeRuteoIn2();
  LeeRuteoIn3();
  LeeRuteoIn4();

  LeeRuteoOut1();
  LeeRuteoOut2();
  LeeRuteoOut3();
  LeeRuteoOut4();

  LeeLazoIn1();
  LeeLazoIn2();
  LeeLazoIn3();
  LeeLazoIn4();

  LeeVincEnt1();
  LeeVincEnt2();
  LeeVincEnt3();
  LeeVincEnt4();

  LeeTConfIn1();
  LeeTConfIn2();
  LeeTConfIn3();
  LeeTConfIn4();

  LeeModoUsoPGM1();
  LeeModoUsoPGM2();
  LeeModoUsoPGM3();
  LeeModoUsoPGM4();

  LeeTiempoPGM1();
  LeeTiempoPGM2();
  LeeTiempoPGM3();
  LeeTiempoPGM4();
}

void LeeVincEnt1() {
  address = E2_VINC_IN1;
  RdE2();
  ValVincIn1 = Respuesta;
  Serial.println("Vinculo IN 1: " + String(ValVincIn1));
  Usuarios = ValVincIn1;
  mask = 1;
  if (Usuarios & mask) {
    Serial.println("SALIDA 1 acciona con ENTRADA1");
    ValVincIn1Out1 = 1;
  } else {
    ValVincIn1Out1 = 0;
  }
  mask = 2;
  if (Usuarios & mask) {
    Serial.println("SALIDA 2 acciona con ENTRADA1");
    ValVincIn1Out2 = 1;
  } else {
    //Serial.println("SALIDA 2 no acciona con ENTRADA1");
    ValVincIn1Out2 = 0;
  }
  mask = 0x4;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA1");
    ValVincIn1Out3 = 1;
  } else {
    //Serial.println("SALIDA 3 no acciona con ENTRADA1");
    ValVincIn1Out3 = 0;
  }
  mask = 0x8;
  if (Usuarios & mask) {
    Serial.println("SALIDA 4 acciona con ENTRADA 1");
    ValVincIn1Out4 = 1;
  } else {
    //Serial.println("SALIDA 4 no acciona con ENTRADA 1");
    ValVincIn1Out4 = 0;
  }
}

void LeeVincEnt2() {
  address = E2_VINC_IN2;
  RdE2();
  ValVincIn2 = Respuesta;
  Serial.println("Vinculo IN 2: " + String(ValVincIn2));
  Usuarios = ValVincIn2;
  mask = 1;
  if (Usuarios & mask) {
    Serial.println("SALIDA 1 acciona con ENTRADA 2");
    ValVincIn2Out1 = 1;
  } else {
    //Serial.println("SALIDA 1 no acciona con ENTRADA 2");
    ValVincIn2Out1 = 0;
  }
  mask = 2;
  if (Usuarios & mask) {
    Serial.println("SALIDA 2 acciona con ENTRADA 2");
    ValVincIn2Out2 = 1;
  } else {
    //Serial.println("SALIDA 2 no acciona con ENTRADA 2");
    ValVincIn2Out2 = 0;
  }
  mask = 0x4;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA 2");
    ValVincIn2Out3 = 1;
  } else {
    //Serial.println("SALIDA 3 no acciona con ENTRADA 2");
    ValVincIn2Out3 = 0;
  }
  mask = 0x8;
  if (Usuarios & mask) {
    Serial.println("SALIDA 4 acciona con ENTRADA 2");
    ValVincIn2Out4 = 1;
  } else {
    //Serial.println("SALIDA 4 no acciona con ENTRADA 2");
    ValVincIn2Out4 = 0;
  }
}

void LeeVincEnt3() {
  address = E2_VINC_IN3;
  RdE2();
  ValVincIn3 = Respuesta;
  Serial.println("Vinculo IN 3: " + String(ValVincIn3));

  Usuarios = ValVincIn3;
  mask = 1;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA 3");
    ValVincIn3Out1 = 1;
  } else {
    //Serial.println("SALIDA 1 no acciona con ENTRADA 3");
    ValVincIn3Out1 = 0;
  }
  mask = 2;
  if (Usuarios & mask) {
    Serial.println("SALIDA 2 acciona con ENTRADA 3");
    ValVincIn3Out2 = 1;
  } else {
    //Serial.println("SALIDA 2 no acciona con ENTRADA 3");
    ValVincIn3Out2 = 0;
  }
  mask = 0x4;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA 3");
    ValVincIn3Out3 = 1;
  } else {
    //Serial.println("SALIDA 3 no acciona con ENTRADA 3");
    ValVincIn3Out3 = 0;
  }
  mask = 0x8;
  if (Usuarios & mask) {
    Serial.println("SALIDA 4 acciona con ENTRADA 3");
    ValVincIn3Out4 = 1;
  } else {
    // Serial.println("SALIDA 4 no acciona con ENTRADA 3");
    ValVincIn3Out4 = 0;
  }
}

void LeeVincEnt4() {
  address = E2_VINC_IN4;
  RdE2();
  ValVincIn4 = Respuesta;
  Serial.println("Vinculo IN 4: " + String(ValVincIn4));
  Usuarios = ValVincIn4;

  mask = 1;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA 4");
    ValVincIn4Out1 = 1;
  } else {
    //Serial.println("SALIDA 1 no acciona con ENTRADA 4");
    ValVincIn4Out1 = 0;
  }

  mask = 2;
  if (Usuarios & mask) {
    Serial.println("SALIDA 2 acciona con ENTRADA 4");
    ValVincIn4Out2 = 1;
  } else {
    //Serial.println("SALIDA 2 no acciona con ENTRADA 4");
    ValVincIn4Out2 = 0;
  }

  mask = 0x4;
  if (Usuarios & mask) {
    Serial.println("SALIDA 3 acciona con ENTRADA 4");
    ValVincIn4Out3 = 1;
  } else {
    //Serial.println("SALIDA 3 no acciona con ENTRADA 4");
    ValVincIn4Out3 = 0;
  }

  mask = 0x8;
  if (Usuarios & mask) {
    Serial.println("SALIDA 4 acciona con ENTRADA 4");
    ValVincIn4Out4 = 1;
  } else {
    //Serial.println("SALIDA 4 no acciona con ENTRADA 4");
    ValVincIn4Out4 = 0;
  }
}

void LeeLazoIn1() {
  address = E2_TIPO_LAZO_IN1;
  RdE2();
  ValLazoIn1 = Respuesta;
  Serial.println("Tipo lazo IN1: " + String(ValLazoIn1));
}

void LeeLazoIn2() {
  address = E2_TIPO_LAZO_IN2;
  RdE2();
  ValLazoIn2 = Respuesta;
  Serial.println("Tipo lazo IN2: " + String(ValLazoIn2));
}

void LeeLazoIn3() {
  address = E2_TIPO_LAZO_IN3;
  RdE2();
  ValLazoIn3 = Respuesta;
  Serial.println("Tipo lazo IN3: " +  String(ValLazoIn3));
}

void LeeLazoIn4() {
  address = E2_TIPO_LAZO_IN4;
  RdE2();
  ValLazoIn4 = Respuesta;
  Serial.println("Tipo lazo IN4: " +  String(ValLazoIn4));
}

void LeeRuteoIn1() {
  address = E2_RUTEO_IN1;
  RdE2();
  ValRutIn1 = Respuesta;
  Serial.println("Ruteo IN1: " + String(ValRutIn1));
}

void LeeRuteoIn2() {
  address = E2_RUTEO_IN2;
  RdE2();
  ValRutIn2 = Respuesta;
  Serial.println("Ruteo IN2: " + String(ValRutIn2));
}

void LeeRuteoIn3() {
  address = E2_RUTEO_IN3;
  RdE2();
  ValRutIn3 = Respuesta;
  Serial.println("Ruteo IN3: " + String(ValRutIn3));
}

void LeeRuteoIn4() {
  address = E2_RUTEO_IN4;
  RdE2();
  ValRutIn4 = Respuesta;
  Serial.println("Ruteo IN4: " + String(ValRutIn4));
}


void LeeRuteoOut1() {
  address = E2_RUTEO_OUT1;
  RdE2();
  ValRutOut1 = Respuesta;
  Serial.println("Ruteo Out 1: " + String(ValRutOut1));
}

void LeeRuteoOut2() {
  address = E2_RUTEO_OUT2;
  RdE2();
  ValRutOut2 = Respuesta;
  Serial.println("Ruteo Out 2: " + String(ValRutOut2));
}

void LeeRuteoOut3() {
  address = E2_RUTEO_OUT3;
  RdE2();
  ValRutOut3 = Respuesta;
  Serial.println("Ruteo Out 3: " + String(ValRutOut3));
}

void LeeRuteoOut4() {
  address = E2_RUTEO_OUT4;
  RdE2();
  ValRutOut4 = Respuesta;
  Serial.println("Ruteo Out 4: " + String(ValRutOut4));
}


void LeeTConfIn1() {
  address = E2_TIM_CONF_IN1;
  RdE2();
  ValTConfIn1 = Respuesta;
  Serial.println("Tiempo conf.IN1: " + String(ValTConfIn1));
}

void LeeTConfIn2() {
  address = E2_TIM_CONF_IN2;
  RdE2();
  ValTConfIn2 = Respuesta;
  Serial.println("Tiempo conf.IN2: " + String(ValTConfIn2));
}

void LeeTConfIn3() {
  address = E2_TIM_CONF_IN3;
  RdE2();
  ValTConfIn3 = Respuesta;
  Serial.println("Tiempo conf.IN3: " + String(ValTConfIn3));
}

void LeeTConfIn4() {
  address = E2_TIM_CONF_IN4;
  RdE2();
  ValTConfIn4 = Respuesta;
  Serial.println("Tiempo conf.IN4: " + String(ValTConfIn4));
}

//_______________________________________________________________________
void ChequeaEnvioPaquete() {
  HayInternet = 1;
  if (HayInternet == 1)  {
    if (DebeResponderAOrigen == 1) {
      DebeResponderAOrigen = 0;
      EnviaComandoADestino();
      DatoRecibido = "";

      //___________
    } else if (DebeDesactivarSalida1 == 1) {
      DebeDesactivarSalida1 = 0;
      CortaSalidaPGM1();

      //___________
    } else if (DebeDesactivarSalida2 == 1) {
      DebeDesactivarSalida2 = 0;
      CortaSalidaPGM2();

      //___________
    } else if (DebeDesactivarSalida3 == 1) {
      DebeDesactivarSalida3 = 0;
      CortaSalidaPGM3();

      //___________
    } else if (DebeDesactivarSalida4 == 1) {
      DebeDesactivarSalida4 = 0;
      CortaSalidaPGM4();


      //___________
    } else if (DebeEnviarNotificacionesSegunRuteo == 1) {
      DebeEnviarNotificacionesSegunRuteo = 0;
      Selecciona_RespuestaComun_Notificacion();
      EnviaNotificacionPush(NumSerialESP);
    }

  } else {
    Serial.println ("No envio comando por falta de internet");
  }
}

//______
void LeeModoUsoPGM1() {
  address = E2ModoPGM1;
  ValModoPGM1 = EEPROM.read(address);
  //ValModoPGM1 = value;
  Serial.println("Modo PGM1= " + String(ValModoPGM1));
}

void LeeModoUsoPGM2() {
  address = E2ModoPGM2;
  ValModoPGM2 = EEPROM.read(address);
  Serial.println("Modo PGM2= " + String(ValModoPGM2));
}

void LeeModoUsoPGM3() {
  address = E2ModoPGM3;
  ValModoPGM3 = EEPROM.read(address);
  Serial.println("Modo PGM3= " + String(ValModoPGM3));
}

void LeeModoUsoPGM4() {
  address = E2ModoPGM4;
  ValModoPGM4 = EEPROM.read(address);
  Serial.println("Modo PGM4= " + String(ValModoPGM4));
}


void LeeTiempoPGM1() {
  address = E2TimPGM1;
  ValTimPGM1 = EEPROM.read(address);
  Serial.println("Tiempo PGM1= " + String(ValTimPGM1));
}

void LeeTiempoPGM2() {
  address = E2TimPGM2;
  ValTimPGM2 = EEPROM.read(address);
  Serial.println("Tiempo PGM2= " + String(ValTimPGM2));
}

void LeeTiempoPGM3() {
  address = E2TimPGM3;
  ValTimPGM3 = EEPROM.read(address);
  Serial.println("Tiempo PGM3= " + String(ValTimPGM3));
}

void LeeTiempoPGM4() {
  address = E2TimPGM4;
  ValTimPGM4 = EEPROM.read(address);
  Serial.println("Tiempo PGM4= " + String(ValTimPGM4));
}


//_______________________________________
void AnalizaStringRecibido() {
  if (HayInternet == 1)  {
    if (DatoRecibido != "")  {
      DatoRecibido.replace('** DONATIONWARE **', ' ');
      Serial.println("Se va a procesar este mensaje -> " + DatoRecibido);
      variable = DatoRecibido;
      DatoRecibido = "";
      TotalLeds = 2;
      GeneraBeepsLed();

      //1- Determina datos a procesar y quien lo envió!(IDOrigenEnvio)
      //String ValorBuscado;
      ValorBuscado = "&to=";
      pos = variable.indexOf(ValorBuscado);
      //Serial.println(" busqueda de id origen");
      if (pos >= 0) {
        IdModuloDestinatario = variable.substring(pos + 4, pos + 10);
      }
      //Serial.println("Busqueda de valor 'data'...");

      ValorBuscado = "data";
      DebeOscErrStatus = 0;
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        recibido = variable.substring(pos + 5, pos + 17);
        ValorBuscado = "from";
        pos = variable.indexOf(ValorBuscado);
        if (pos >= 0) {
          IdDestino = variable.substring(pos + 5, pos + 15);
          UsuarioEmisor = IdDestino;
          //Serial.println("Enviado desde : " + String(IdDestino));                //Recupera valor IMEI del emisor del comando
          //Serial.println("Moulo destinatario : " + OrigenTopicoEnvio);
          Serial.println("________________");
        }
      }

      //_________________________________________________________
      //if (IdModuloDestinatario == NumSerialESP || EsModoShare == 1)  {
      //  EsModoShare = 0;
      ValorBuscado = "solicitadatos";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        // Analiza si el numero de id del modulo es el que corresponde al modulo
        //ValUbicacionMod.trim();
        //Serial.println("Ubicacion de modulo = " + ValUbicacionMod);
        //if (ValUbicacionMod == "ubicacion de modulo") {
        //Caso MODULO NUEVO

        //BuscaCoincideIdUsua();
        //if (ExisteUsuario == 1) {
        //  Serial.println("TELEFONO EXISTENTE- MODULO NUEVO ");
        //  //caso TELEFONO NUEVO - MODULO NUEVO
        //  //Si no tiene datos de ubicacion de modulo y tampoco el numero de usuario recibido, le envia sus datos como default al usuario
        //} else {
        //  Serial.println("TELEFONO NUEVO- MODULO NUEVO ");
        //  //caso TELEFONO EXISTENTE - MODULO NUEVO
        //  //Si no tiene datos de ubicacion pero si dispone del ID del usuaio,va a recibir del usuario la actualizacion de datoa desde el telefojo (UPLOAD1).
        //}
        //} else {
        //  //Caso MODULO EXISTENTE"
        //  BuscaCoincideIdUsua();
        //  if (ExisteUsuario == 1) {
        //    Serial.println("TELEFONO EXISTENTE- MODULO EXISTENTE ");
        //    //caso TELEFONO EXISTENTE - MODULO EXISTENTE
        //    //si ya tiene datos DE UBICACION y del usuario, le envia los datos propios como actualizacion de modulo a usuario
        //  } else {
        //    Serial.println("TELEFONO NUEVO  MODULO EXISTENTE ");
        //    //TELEFONO NUEVO - MODULO EXISTENTE
        //    //Si tiene datos de ubicacion pero no coincide el ID de usuaio, le envia a este sus datos para que se actualice el usuario desde los datos de telefdono.
        //  }
        //}
        EnviaDatosModuloAUsuario();
        //}
      }

      //________
      ValorBuscado = "estado";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        Serial.println("ESTADO ACTUAL");
        ArmaEstadoInOutActual();
        CmdAccion = resultado;
        DebeResponderAOrigen = 1;
        Serial.println(CmdAccion);
      }

      //____________________
      ValorBuscado = "RUTOUT";
      DecodificaRuteoOutPGM();

      //___________________
      ValorBuscado = "RUTIN";
      DecodificaRuteoInPGM();

      //__________
      ValorBuscado = "UBICACIONES";
      DecodificaUbicaciones();

      //_____________
      ValorBuscado = "TIPOLAZO";
      DecodeTipoLazo();

      //_____________
      ValorBuscado = "VIN1";
      DecodificaVinculos();

      //_________
      ValorBuscado = "RUTTELIN";
      DecoRuteoEntradasxUsuario();

      //________________
      ValorBuscado = "RUTTELOUT";
      DecoRuteoSalidasxUsuario();

      //________
      ValorBuscado = "upload1";
      DecodificaDatosTelefono();

      //______
      ValorBuscado = "newmod";
      DecoNombreUbicaImeiUsuario();

      //________
      ValorBuscado = "TSAL1";
      ProcesaTiempoModoPGM();

      //___________________
      pos = variable.indexOf("borrausuario");
      ProcesaBorradoUsuario();

      //___________________
      pos = variable.indexOf("borramod");
      ProcesaBorraModulo();

      //_______________________________________________________________________________________________
      pos = variable.indexOf("on_out1");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println(NombreUsuaEmisor);
        ActivaSalida1();
      }

      //________
      pos = variable.indexOf("off_out1");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        CortaSalidaPGM1();
      }

      //________
      pos = variable.indexOf("on_out2");
      if (pos >= 0)  {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 11);
        Serial.println("ANALIZAR AQUI!");
        Serial.println(NombreUsuaEmisor);
        ActivaSalida2();
      }

      //________
      pos = variable.indexOf("off_out2");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println(NombreUsuaEmisor);
        CortaSalidaPGM2();
      }

      //_______________________________________________________________________________________________
      pos = variable.indexOf("on_out3");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println(NombreUsuaEmisor);
        ActivaSalida3();
      }

      //________
      pos = variable.indexOf("off_out3");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println(NombreUsuaEmisor);
        CortaSalidaPGM3();
      }

      pos = variable.indexOf("on_out4");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println(NombreUsuaEmisor);
        ActivaSalida4();
      }

      pos = variable.indexOf("off_out4");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        CortaSalidaPGM4();
      }

      pos = variable.indexOf("TEL1");
      DecodificaIdNombreUsua1();

      pos = variable.indexOf("TEL2 ");
      DecodificaIdNombreUsua2();

      pos = variable.indexOf("TEL3 ");
      DecodificaIdNombreUsua3();

      pos = variable.indexOf("TEL4 ");
      DecodificaIdNombreUsua4();
    }
  }
}



//_______
void LeeIDUsuario1() {
  address = E2_IDUsua1;
  ValIdReporte1 = LeeComunIDUsuario();
  //Serial.println("Valor IdUsuario 1  actual : " + );
  pos = ValIdReporte1.indexOf("?");
  if (pos >= 0)  {
    HayUsuario1 = 0;
  } else  {
    HayUsuario1 = 1;
  }
  Serial.println("Valor IdUsuario 1 : " + ValIdReporte1);
  Serial.println("___________");
}

//_____
void LeeIDUsuario2() {
  address = E2_IDUsua2;
  ValIdReporte2 = LeeComunIDUsuario();
  pos = ValIdReporte2.indexOf("?");
  if (pos >= 0)  {
    HayUsuario2 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario2 = 1;
  }
  Serial.println("Valor IdUsuario 2  actual : " + ValIdReporte2);
}

//_____
void LeeIDUsuario3() {
  address = E2_IDUsua3;
  ValIdReporte3 = LeeComunIDUsuario();
  pos = ValIdReporte3.indexOf("?");
  if (pos >= 0)  {
    HayUsuario3 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario3 = 1;
  }
  Serial.println("Valor IdUsuario 3  actual : " + ValIdReporte3);
}

//_____
void LeeIDUsuario4() {
  address = E2_IDUsua4;
  ValIdReporte4 = LeeComunIDUsuario();
  pos = ValIdReporte4.indexOf("?");
  if (pos >= 0)  {
    HayUsuario4 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario4 = 1;
  }
  Serial.println("Valor IdUsuario 4  actual : " + ValIdReporte4);
}

//_______________________________________________________
String LeeComunIDUsuario() {
  String ValIdUsuario = "";
  for (int i = address; i < address + 10; i++) {
    ValIdUsuario = ValIdUsuario + char(EEPROM.read(i));
  }
  ValIdUsuario = ValIdUsuario.substring(0, 10);
  return ValIdUsuario;
}

//_________________________________________________________________________________________
void LeeUbicaUsuario1() {
  address = E2_NombreUsua1;
  ValUbicacionID1 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.1 = " + ValUbicacionID1);
}

//_______
void LeeUbicaUsuario2() {
  address = E2_NombreUsua2;
  ValUbicacionID2 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.2 = " + ValUbicacionID2);
}
//_______
void LeeUbicaUsuario3() {
  address = E2_NombreUsua3;
  ValUbicacionID3 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.3 = " + ValUbicacionID3);
}
//_______
void LeeUbicaUsuario4() {
  address = E2_NombreUsua4;
  ValUbicacionID4 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.4 = " + ValUbicacionID4);
}

//________________________________
String LeeComunUbicaUsua() {
  String ValUbicaUsua = "";
  for (int i = address; i < address + 20; i++)
  {
    ValUbicaUsua = ValUbicaUsua + char(EEPROM.read(i));
  }
  ValUbicaUsua = ValUbicaUsua.substring(0, 20);
  return ValUbicaUsua;
}

//_______
void BorraE2promNombreUsuario1() {
  address = E2_NombreUsua1;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario2() {
  address = E2_NombreUsua2;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario3() {
  address = E2_NombreUsua3;
  borraBloqueE2prom();
}
//_______
void BorraE2promNombreUsuario4() {
  address = E2_NombreUsua4;
  borraBloqueE2prom();
}

//_____________
void borraBloqueE2prom() {
  resultado1 = string_variable;
  string_variable = "?                   ";
  GrabaPaqueteE2();
  string_variable = resultado1;
}

void CortaSalidaPGM1() {
  DebeContarMinPGM1 = 0;
  DebeContarHrPGM1 = 0;
  DebeContarSegPGM1 = 0;

  DesactivaPorTiempoPGM1 = 0;
  Serial.println("DESACTIVA PGM1");
  digitalWrite(PinOut1Pgm, LOW);
  StPGM1 = 0;
  CntTimOut1 = 0;
  if (DesactivaPorTiempoPGM1 == 1)  {
    Serial.println("El corte de la salida 1 fue por tiempo");
  } else {
    Serial.println("El corte de la salida 1 fue por accion de usuario");
  }
  delay(1000);
  CmdAccion = "Desactivacion salida 1";
  //DebeEnviarNotificacionesSegunRuteo = 1;
  EnviaNotificacionPush(NumSerialESP);
}


//___________________
void CortaSalidaPGM2() {
  //EnviaNotificacionPush(NumSerialESP);
  digitalWrite(PinOut2Pgm, LOW);
  DebeContarSegPGM2 = 0; //ya no cuenta en segundos
  DebeContarMinPGM2 = 0;
  DebeContarHrPGM2 = 0;
  StPGM2 = 0;
  CntTimOut2 = 0;               //Anula tiempo de pgm 2
  if (DesactivaPorTiempoPGM2 == 1)  {
    Serial.println("El corte de la salida 21 fue por tiempo");
  } else {
    Serial.println("El corte de la salida 2 fue por accion de usuario");
  }
  delay(1000);
  CmdAccion = "Desactivacion salida 2";
  EnviaNotificacionPush(NumSerialESP);
}

void CortaSalidaPGM3() {
  DesactivaPorTiempoPGM3 = 0;
  digitalWrite(PinOut3Pgm, LOW);
  DebeContarSegPGM3 = 0;
  DebeContarMinPGM3 = 0;
  DebeContarHrPGM3 = 0;
  StPGM3 = 0;
  CntTimOut3 = 0;         //Anula tiempo de pgm 3
  if (DesactivaPorTiempoPGM3 == 1)  {
    Serial.println("El corte de la salida 3 fue por tiempo");
  } else {
    Serial.println("El corte de la salida 3 fue por accion de usuario");
  }
  delay(1000);
  CmdAccion = "Desactivacion salida 3";
  EnviaNotificacionPush(NumSerialESP);
}


void CortaSalidaPGM4() {
  DesactivaPorTiempoPGM4 = 0;
  Serial.println("DESACTIVA PGM4");
  digitalWrite(PinOut4Pgm, LOW);
  DebeContarSegPGM4 = 0; //ya no cuenta en segundos
  DebeContarMinPGM4 = 0;
  DebeContarHrPGM4 = 0;
  StPGM4 = 0;
  CntTimOut4 = 0;         //Anula tiempo de pgm 1
  if (DesactivaPorTiempoPGM4 == 1)  {
    Serial.println("El corte de la salida 4 fue por tiempo");
  } else {
    Serial.println("El corte de la salida 4 fue por accion de usuario");
  }
  delay(1000);
  CmdAccion = "Desactivacion salida 4";
  EnviaNotificacionPush(NumSerialESP);
}


void GetExternalIP() {
  return;

  WiFiClient client1;
  if (!client1.connect("api.ipify.org", 80)) {

    if (HayAvisoFaltaInternet == 0) {
      Serial.println("Falla conexion internet externa - sin ip publica");
      cntFaltaInternet++;
      reconnect();
      if (cntFaltaInternet > 3) {
        Serial.println("Confirmado falta de internet");
        cntFaltaInternet = 0;
        DebeOscErrStatus = 1; //debe oscilar por falta de internet
        HayInternet = 0;
        HayAvisoFaltaInternet = 1;
      }
    }
  } else {
    if (HayAvisoFaltaInternet == 1) {
      Serial.print("Hubo aviso falta de internet");
    }
    int timeout = millis() + 5000;
    client1.print("GET / ? format = json HTTP / 1.1\r\nHost : api.ipify.org\r\n\r\n");
    while (client1.available() == 0) {
      if (timeout - millis() < 0) {
        //Serial.println(" >>> Client Timeout !");
        client1.stop();
        return;
      }
    }
    int size;
    while ((size = client1.available()) > 0) {
      uint8_t *msg = (uint8_t *)malloc(size);
      size = client1.read(msg, size);
      free(msg);
      if (HayInternet == 0) {
        HayInternet = 1;
        DebeOscErrStatus = 0; //debe oscilar por falta de internet
      }
      cntFaltaInternet = 0;
      DebeOscErrStatus = 0;
      if (HayAvisoFaltaInternet == 1) {
        HayAvisoFaltaInternet = 01;
        digitalWrite(PinLedVerde, LOW);
        Serial.println("Hay IP publica");
      }
    }
  }
}


void EnviaDatosModuloAUsuario() {
  ValUbicacionID2.trim();
  ValUbicacionID3.trim();
  ValUbicacionID4.trim();
  Serial.println(IdDestino);
  LeeValoresRuteo();
  ValUbicacionMod.trim();
  LeeUbicacionIN1();
  String CmdAccion = "datosmodulo " + NumSerialESP + "," + ValUbicacionMod + ","
                     + ValIdReporte1 + "," + ValIdReporte2 + "," + ValIdReporte3 + "," + ValIdReporte4 + ","
                     + ValUbicacionID1 + "," + ValUbicacionID2 + "," + ValUbicacionID3 + "," + ValUbicacionID4 + ","
                     + ValUbicaIN1 + "," + ValUbicaIN2 + ","  + ValUbicaIN3 + "," + ValUbicaIN4 + ","
                     + ValUbicaOUT1 + "," + ValUbicaOUT2 + "," + ValUbicaOUT3 + ","  + ValUbicaOUT4 + ","
                     + ValRutIn1 + "," + ValRutIn2 + "," + ValRutIn3 + "," + ValRutIn4 + ","
                     + ValRutOut1 + "," + ValRutOut2 + "," + ValRutOut3 + "," + ValRutOut4 + ","
                     + ValLazoIn1 + "," + ValLazoIn2 + ","  + ValLazoIn3 + "," + ValLazoIn4 + ","
                     + ValVincIn1 + "," + ValVincIn2 + ","  + ValVincIn3 + "," + ValVincIn4 + ","
                     + ValTConfIn1 + "," + ValTConfIn2 + ","  + ValTConfIn3 + "," + ValTConfIn4 + ","
                     + ValTimPGM1 + ","  + ValTimPGM2 + ","  + ValTimPGM3 + "," + ValTimPGM4 + ","
                     + ValModoPGM1 + "," + ValModoPGM2 + "," + ValModoPGM3 + "," + ValModoPGM4 +  ","
                     + ValHabUsuaIN1 + "," + ValRutIn2Usua + "," +  ValRutIn3Usua + "," + ValRutIn4Usua + ","
                     + ValRutOut1Usua + "," + ValRutOut2Usua + "," + ValRutOut3Usua + "," + ValRutOut4Usua;

  Serial.println("Total caracteres a enviar");
  Serial.println(CmdAccion.length());
  Serial.println(CmdAccion);

  CmdBasico = "&from=" + NumSerialESP + "&to=" + IdDestino + "&data=";
  CmdBasico = CmdBasico + CmdAccion;
  MensajeEnviar = CmdBasico;

  String DirPublicacion = "GN8rYIiMtH9Tt89/output/";
  DirPublicacion = DirPublicacion + String(IdDestino).c_str();
  Serial.println(DirPublicacion);

  MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str(), true);
  Serial.println("Publica al usuario : " + DirPublicacion);     //IdDestino);

  //bot.sendMessage(CHAT_ID, "Bot started up", "");

  if (MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str()) == true) {
    //if (MQTTClient.publish(String(IdDestino).c_str(), String(MensajeEnviar).c_str()) == true) {
    Serial.println("Envio OK");
    Serial.println("________________________");
  } else {
    Serial.println("Error sending message");
  }
  variable = "";
  DatoRecibido = "";
}

void EnviaComandoADestino() {
  if (HayInternet == 1) {
    Serial.println("Id Destino : " + IdDestino);
    String DirPublicacion = "GN8rYIiMtH9Tt89/output/";
    DirPublicacion = DirPublicacion + String(IdDestino).c_str();
    Serial.println("Direccion de publicacion : " + DirPublicacion);

    CmdBasico = "&from=" + NumSerialESP + "&to=" + IdDestino + "&data=";
    CmdBasico = CmdBasico + CmdAccion;
    MensajeEnviar = "message" + CmdBasico;
    Serial.println("Texto enviar : " + MensajeEnviar);
    MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str(), true);
    Serial.println("Publica al usuario : " + DirPublicacion);     //IdDestino);

    if (MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str()) == true) {
      Serial.println("Envio OK");
      Serial.println("________________________");
    } else {
      Serial.println("Error sending message");
    }

  } else {
    Serial.println("No puede enviar comando porque no hay internet");
  }
  Serial.println("________________");
}

void Selecciona_RespuestaComun_Notificacion() {
  if (HayInternet == 1)  {
    LeeArmaTimeDate();
    if (EsAccionLocal == 1) {
      EsAccionLocal = 0;
      UsuarioGenerador = "Detección interna en el sistema";
    } else {
      UsuarioGenerador = UsuarioEmisor + "," + NombreUsuaEmisor;     //IMPORTANTE AQUI DEBE AGREGAR NOMBRE DE USUARIO QUE ENVIO EL PEDIDO DE ACTIVACION
    }
  }
}

void BuscaCoincideIdUsua() {
  //Determina di el destino corresponde a un usuario habilitado
  ExisteUsuario = 0;
  if (IdDestino == ValIdReporte1) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte2) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte3) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte4) {
    ExisteUsuario = 1;
  }
}

void reconnect() {
  //if (EstaEnNotificacion == 0 ) {
  while (!MQTTClient.connected()) {
    //Serial.print("Intentando conexión Mqtt...");
    // Creamos un cliente ID
    //String clientId = "mqttx_9351f744";
    String clientId = "SOFTMICRO4IO";
    clientId += String(random(0xffff), HEX);
    if (MQTTClient.connect(clientId.c_str(), mqttUser, mqttPass)) {
      Serial.println("Conectado!");

      Serial.println(DirSubscribe);
      if (MQTTClient.subscribe(String(NumSerialESP).c_str())) {
        Serial.println("Suscripcion ok");

        MQTTClient.subscribe("GN8rYIiMtH9Tt89/SOFTMICRO4IO");
        Serial.println("Acaba de subscribirse a SOFTMICRO4IO!");

      } else {
        Serial.println("fallo Suscripción");
      }
    } else {
      Serial.print("falló :( con error -> ");
      Serial.print(MQTTClient.state());
      Serial.println(" Intentamos de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void LeeRutIn1Usua() {
  ValHabUsuaIN1 = EEPROM.read(E2_RutIn1Usua);
  stringOne = "Ruteo IN1 x usuario =";
  Serial.println(stringOne + ValHabUsuaIN1);
}

void LeeRutIn2Usua() {
  ValRutIn2Usua = EEPROM.read(E2_RutIn2Usua);
  stringOne = "Ruteo IN2 x usuario =";
  Serial.println(stringOne + ValRutIn2Usua);
}

void LeeRutIn3Usua() {
  ValRutIn3Usua = EEPROM.read(E2_RutIn3Usua);     //E2_Ruteo_ID3);
  stringOne = "Ruteo IN3 x usuario =";
  Serial.println(stringOne + ValRutIn3Usua);
}

void LeeRutIn4Usua() {
  ValRutIn4Usua = EEPROM.read(E2_RutIn4Usua);
  stringOne = "Ruteo IN4 x usuario =";
  Serial.println(stringOne + ValRutIn4Usua);
}

void LeeRutOut1Usua() {
  address = E2_RutOut1Usua;
  ValRutOut1Usua = EEPROM.read(address);
  stringOne = "Ruteo OUT1 x usuario =";
  Serial.println(stringOne +  ValRutOut1Usua);
}

void LeeRutOut2Usua() {
  address = E2_RutOut2Usua;
  ValRutOut2Usua = EEPROM.read(address);
  stringOne = "Ruteo OUT2 x usuario =";
  Serial.println(stringOne + ValRutOut2Usua);
}

void LeeRutOut3Usua() {
  address = E2_RutOut3Usua;
  ValRutOut3Usua = EEPROM.read(address);
  stringOne = "Ruteo OUT3 x usuario =";
  Serial.println(stringOne + ValRutOut3Usua);
}

void LeeRutOut4Usua() {
  address = E2_RutOut4Usua;
  ValRutOut4Usua = EEPROM.read(address);
  stringOne = "Ruteo OUT4 x usuario = ";
  Serial.println(stringOne + ValRutOut4Usua);
}

void DecodeBitsxByte() {
  mask = 1;
  if (Usuarios & mask) { // if bitwise AND resolves to true
    Serial.println("Hay usuario 1 para salida seleccionada");
  } else {
    Serial.println("No hay usuario 1 para salida seleccionada");
  }

  mask = 2;
  if (Usuarios & mask) { // if bitwise AND resolves to true
    Serial.println("Hay usuario 2 para salida");
  }
  mask = 0x4;
  if (Usuarios & mask) { // if bitwise AND resolves to true
    Serial.println("Hay usuario 3 para salida");
  }
  mask = 0x8;
  if (Usuarios & mask) { // if bitwise AND resolves to true
    Serial.println("Hay usuario 4 para salida");
  }
  Serial.println("*****************");
}

void LeeValoresRuteo() {
  LeeRutIn1Usua();
  LeeRutIn2Usua();
  LeeRutIn3Usua();
  LeeRutIn4Usua();
  LeeRutOut1Usua();
  LeeRutOut2Usua();
  LeeRutOut3Usua();
  LeeRutOut4Usua();
}


//_____________________________________
void LeeEntrada1() {
  if (digitalRead(PinEntrada1) == 0) {
    if (ValLazoIn1 == 0) {
      if (YaDetectoIn1 == 0) {
        LeeTConfIn1();
        TimTimerIn1 = ValTConfIn1;
        delay(100);
        if (digitalRead(PinEntrada1) == 0) {
          YaDetectoIn1 = 1;
          if (ValRutIn1 != 0) {
            if (ValRutIn1 == 3 || ValRutIn1 == 1) {
              Serial.println("DETECCION DE ENTRADA 1 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN1(); CmdAccion = "Entrada 1 activada";
              Serial.println("Genera NP zona 1 abierta");
              //DebeEnviarNotificacionesSegunRuteo = 1;
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn1 == 1) {
        delay(100);
        if (digitalRead(PinEntrada1) == 0) {
          YaDetectoIn1 = 0;
          TimTimerIn1 = 0;     //Termnina con el tiempo de conteo de confirmacion
          StIn1 = 1;
          if (ValRutIn1 != 0) {
            if (ValRutIn1 == 3 || ValRutIn1 == 2) {
              Serial.println("DETECCION DE ENTRADA 1 VALIDA SEGUN RUTEO POR POSITIVO");
              CmdAccion = "Entrada 1 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }

  } else {
    if (ValLazoIn1 == 0) {
      if (YaDetectoIn1 == 1) {
        delay(100);
        if (digitalRead(PinEntrada1) == 1) {
          YaDetectoIn1 = 0;
          TimTimerIn1 = 0;     //Termnina con el tiempo de conteo de confirmacion
          if (ValRutIn1 != 0) {
            if (ValRutIn1 == 3 || ValRutIn1 == 2) {
              Serial.println("DETECCION NORMALIZACION ENTRADA 1 SEGUN RUTEO POR MASA");
              StIn1 = 0;
              CmdAccion = "Entrada 1 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn1 == 0) {
        delay(100);
        if (digitalRead(PinEntrada1) == 1) {
          YaDetectoIn1 = 1;
          if (ValRutIn1 != 0) {
            if (ValRutIn1 == 3 || ValRutIn1 == 1) {
              Serial.println("DETECCION DE ENTRADA 1 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN1();
              CmdAccion = "Entrada 1 activada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }
  }
}

void GeneraAvisoNotificacionEntrada() {
  IDOrigenEnvio = "dispositivo";
  UsuarioGenerador = "Desde su alarma";
  //caca3
  //DebeEnviarNotificacionesSegunRuteo = 1;
  EnviaNotificacionPush(NumSerialESP);
}

void ControlaActivaVinculosIN1() {
  if (ValVincIn1Out1 == 1) {
    ActivaSalida1();
    Serial.println("ACTIVA SALIDA 1 POR VINCULO");
  }
  if (ValVincIn1Out2 == 1) {
    ActivaSalida2();
    Serial.println("ACTIVA SALIDA 2 POR VINCULO");
  }
  if (ValVincIn1Out3 == 1) {
    ActivaSalida3();
    Serial.println("ACTIVA SALIDA 3 POR VINCULO");
  }
  if (ValVincIn1Out4 == 1) {
    ActivaSalida4();
    Serial.println("ACTIVA SALIDA 4 POR VINCULO");
  }
}

void ControlaActivaVinculosIN2() {
  if (ValVincIn2Out1 == 1) {
    ActivaSalida1();
    Serial.println("ACTIVA SALIDA 1 POR VINCULO");
  }
  if (ValVincIn2Out2 == 1) {
    ActivaSalida2();
    Serial.println("ACTIVA SALIDA 2 POR VINCULO");
  }
  if (ValVincIn2Out3 == 1) {
    ActivaSalida3();
    Serial.println("ACTIVA SALIDA 3 POR VINCULO");
  }
  if (ValVincIn2Out4 == 1) {
    ActivaSalida4();
    Serial.println("ACTIVA SALIDA 4 POR VINCULO");
  }
}

void ControlaActivaVinculosIN3() {
  if (ValVincIn3Out1 == 1) {
    ActivaSalida1();
    Serial.println("ACTIVA SALIDA 1 POR VINCULO");
  }
  if (ValVincIn3Out2 == 1) {
    ActivaSalida2();
    Serial.println("ACTIVA SALIDA 2 POR VINCULO");
  }
  if (ValVincIn3Out3 == 1) {
    ActivaSalida3();
    Serial.println("ACTIVA SALIDA 3 POR VINCULO");
  }
  if (ValVincIn3Out4 == 1) {
    ActivaSalida4();
    Serial.println("ACTIVA SALIDA 4 POR VINCULO");
  }
}

void ControlaActivaVinculosIN4() {
  if (ValVincIn4Out1 == 1) {
    ActivaSalida1();
    Serial.println("ACTIVA SALIDA 1 POR VINCULO");
  }
  if (ValVincIn4Out2 == 1) {
    ActivaSalida2();
    Serial.println("ACTIVA SALIDA 2 POR VINCULO");
  }
  if (ValVincIn4Out3 == 1) {
    ActivaSalida3();
    Serial.println("ACTIVA SALIDA 3 POR VINCULO");
  }
  if (ValVincIn4Out4 == 1) {
    ActivaSalida4();
    Serial.println("ACTIVA SALIDA 4 POR VINCULO");
  }
}

//_____________________________________
void LeeEntrada2() {
  if (digitalRead(PinEntrada2) == 0) {
    if (ValLazoIn2 == 0) {
      if (YaDetectoIn2 == 0) {
        LeeTConfIn2();
        TimTimerIn2 = ValTConfIn2;
        delay(100);
        if (digitalRead(PinEntrada2) == 0) {
          YaDetectoIn2 = 1;
          if (ValRutIn2 != 0) {
            if (ValRutIn2 == 3 || ValRutIn2 == 1) {
              Serial.println("DETECCION DE ENTRADA 2 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN2();
              CmdAccion = "Entrada 2 activada";
              Serial.println("Genera NP zona 2 abierta");
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn2 == 1) {
        delay(100);
        if (digitalRead(PinEntrada2) == 0) {
          YaDetectoIn2 = 0;
          TimTimerIn2 = 0;     //Termnina con el tiempo de conteo de confirmacion
          StIn2 = 1;
          if (ValRutIn2 != 0) {
            if (ValRutIn2 == 3 || ValRutIn2 == 2) {
              Serial.println("DETECCION DE ENTRADA 2 VALIDA SEGUN RUTEO POR POSITIVO");
              CmdAccion = "Entrada 2 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }

  } else {
    if (ValLazoIn2 == 0) {
      if (YaDetectoIn2 == 1) {
        delay(100);
        if (digitalRead(PinEntrada2) == 1) {
          YaDetectoIn2 = 0;
          TimTimerIn2 = 0;     //Termnina con el tiempo de conteo de confirmacion
          if (ValRutIn2 != 0) {
            if (ValRutIn2 == 3 || ValRutIn2 == 2) {
              Serial.println("DETECCION DE ENTRADA 2 VALIDA SEGUN RUTEO POR MASA");
              StIn2 = 0;
              CmdAccion = "Entrada 2 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn2 == 0) {
        delay(100);
        if (digitalRead(PinEntrada2) == 1) {
          YaDetectoIn2 = 1;
          if (ValRutIn2 != 0) {
            if (ValRutIn2 == 3 || ValRutIn2 == 1) {
              Serial.println("DETECCION DE ENTRADA 2 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN2();
              CmdAccion = "Entrada 2 activada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }
  }
}

//_____________________________________
void LeeEntrada3() {
  //BorraSSID_E2();
  //return;
  if (digitalRead(PinEntrada3) == 0) {
    if (ValLazoIn3 == 0) {
      if (YaDetectoIn3 == 0) {
        LeeTConfIn3();
        TimTimerIn3 = ValTConfIn3;
        delay(100);
        if (digitalRead(PinEntrada3) == 0) {
          YaDetectoIn3 = 1;
          StIn3 = 1;
          if (ValRutIn3 != 0) {
            if (ValRutIn3 == 3 || ValRutIn3 == 1) {
              Serial.println("DETECCION DE ENTRADA 3 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN3();
              CmdAccion = "Entrada 3 activada";
              Serial.println("Genera NP zona 3 abierta");
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn3 == 1) {
        delay(100);
        if (digitalRead(PinEntrada3) == 0) {
          YaDetectoIn3 = 0;
          TimTimerIn3 = 0;     //Termnina con el tiempo de conteo de confirmacion
          StIn3 = 0;
          if (ValRutIn3 != 0) {
            if (ValRutIn3 == 3 || ValRutIn3 == 2) {
              Serial.println("NORMQALIZACION ENTRADA 3 RUTEO POR POSITIVO");
              CmdAccion = "Entrada 3 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }

  } else {
    if (ValLazoIn3 == 0) {
      if (YaDetectoIn3 == 1) {
        delay(100);
        if (digitalRead(PinEntrada3) == 1) {
          YaDetectoIn3 = 0;
          StIn3 = 0;
          TimTimerIn3 = 0;     //Termnina con el tiempo de conteo de confirmacion
          if (ValRutIn3 != 0) {
            if (ValRutIn3 == 3 || ValRutIn3 == 2) {
              Serial.println("DETECCION DE ENTRADA 3 VALIDA SEGUN RUTEO POR MASA");
              CmdAccion = "Entrada 3 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn3 == 0) {
        delay(100);
        if (digitalRead(PinEntrada3) == 1) {
          YaDetectoIn3 = 1;
          StIn3 = 1;
          if (ValRutIn3 != 0) {
            if (ValRutIn3 == 3 || ValRutIn3 == 1) {
              Serial.println("DETECCION DE ENTRADA 3 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN3();
              CmdAccion = "Entrada 3 activada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }
  }
}

//_____________________________________
void LeeEntrada4() {
  if (digitalRead(PinEntrada4) == 0) {
    if (ValLazoIn4 == 0) {
      if (YaDetectoIn4 == 0) {
        LeeTConfIn4();
        TimTimerIn4 = ValTConfIn4;
        delay(100);
        if (digitalRead(PinEntrada4) == 0) {
          YaDetectoIn4 = 1;
          StIn4 = 1;
          if (ValRutIn4 != 0) {
            if (ValRutIn4 == 3 || ValRutIn4 == 1) {
              Serial.println("DETECCION DE ENTRADA 4 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN4();
              CmdAccion = "Entrada 4 activada";
              Serial.println("Genera NP zona 4 abierta");
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn4 == 1) {
        delay(100);
        if (digitalRead(PinEntrada4) == 0) {
          YaDetectoIn4 = 0;
          StIn4 = 0;
          if (ValRutIn4 != 0) {
            if (ValRutIn4 == 3 || ValRutIn4 == 2) {
              Serial.println("DETECCION DE ENTRADA 4 VALIDA SEGUN RUTEO POR POSITIVO");
              CmdAccion = "Entrada 4 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }

  } else {
    //para modo NORMAL A MASA -> EVENTO  abierto (NA) o 1
    if (ValLazoIn4 == 0) {
      if (YaDetectoIn4 == 1) {
        delay(100);
        if (digitalRead(PinEntrada4) == 1) {
          YaDetectoIn4 = 0;
          StIn4 = 0;
          if (ValRutIn4 != 0) {
            if (ValRutIn4 == 3 || ValRutIn4 == 2) {
              Serial.println("DETECCION DE ENTRADA 4 VALIDA SEGUN RUTEO POR MASA");
              CmdAccion = "Entrada 4 normalizada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    } else {
      if (YaDetectoIn4 == 0) {
        delay(100);
        if (digitalRead(PinEntrada4) == 1) {
          YaDetectoIn4 = 1;
          StIn4 = 1;
          if (ValRutIn4 != 0) {
            if (ValRutIn4 == 3 || ValRutIn4 == 1) {
              Serial.println("DETECCION DE ENTRADA 4 VALIDA SEGUN RUTEO POR MASA");
              ControlaActivaVinculosIN4();
              CmdAccion = "Entrada 4 activada";
              GeneraAvisoNotificacionEntrada();
            }
          }
        }
      }
    }
  }
}


//________________
void ArmaEstadoInOutActual() {
  resultado = "ESTADO ACTUAL ";
  /*
    LeeLazoIn1();
    LeeLazoIn2();
    LeeLazoIn3();
    LeeLazoIn4();

    Serial.println("###############");
    Serial.println(ValLazoIn1);
    Serial.println(ValLazoIn2);
    Serial.println(ValLazoIn3);
    Serial.println(ValLazoIn4);
    Serial.println("****************");
    Serial.println(digitalRead(PinEntrada1));
    Serial.println(digitalRead(PinEntrada2));
    Serial.println(digitalRead(PinEntrada3));
    Serial.println(digitalRead(PinEntrada4));
    Serial.println("****************");
  */
  if (ValLazoIn1 == 0) {
    //Serial.println("Es OPCION deteccion evento por MASA");
    if (digitalRead(PinEntrada1) == 0)  {         // Detecta entra a masa (ojo que la SIRENA puede ser una MASA o un ALTO
      Serial.println("ENTRADA A MASA ");
      resultado = resultado + "IN1_1,";
      Serial.println("Entrada1 abierta(activa)");
    } else {
      Serial.println("Entrada1 normal");
      resultado = resultado + "IN1_0,";
    }

  } else  {
    //Serial.println("ENTRADA A ABIERTA");
    if (digitalRead(PinEntrada1) == 0)  {
      Serial.println("Entrada1 normal");
      resultado = resultado + "IN1_0,";
      //Serial.println("##################");

    } else {
      resultado = resultado + "IN1_1,";
      Serial.println("Entrada1 abierta(activa)");
    }
  }
  //Serial.println("##################");

  //_____________
  if (ValLazoIn2 == 0) {
    if (digitalRead(PinEntrada2) == 0)  {         // Detecta entra a masa (ojo que la SIRENA puede ser una MASA o un ALTO
      resultado = resultado + "IN2_1,";
      Serial.println("Entrada2 abierta(activa)");
    } else {
      Serial.println("Entrada2 normal");
      resultado = resultado + "IN2_0,";
    }

  } else  {
    if (digitalRead(PinEntrada2) == 0)  {
      Serial.println("Entrada2 normal");
      resultado = resultado + "IN2_0,";
    } else {
      resultado = resultado + "IN2_1,";
      Serial.println("Entrada2 abierta(activa)");
    }
  }
  //Serial.println("##################");

  if (ValLazoIn3 == 0) {
    if (digitalRead(PinEntrada2) == 0)  {         // OJO ES PINENTRADA3
      resultado = resultado + "IN3_1,";
      Serial.println("Entrada3 abierta(activa)");
    } else {
      Serial.println("Entrada3 normal");
      resultado = resultado + "IN3_0,";
    }
  } else  {
    if (digitalRead(PinEntrada2) == 0)  {
      Serial.println("Entrada3 normal");
      resultado = resultado + "IN3_0,";
    } else {
      resultado = resultado + "IN3_1,";
      Serial.println("Entrada 3 abierta(activa)");
    }
  }
  //Serial.println("##################");

  //________________
  if (ValLazoIn4 == 0) {
    if (digitalRead(PinEntrada4) == 0)  {         // Detecta entra a masa (
      resultado = resultado + "IN4_1,";
      Serial.println("Entrada4 abierta(activa)");
    } else {
      Serial.println("Entrada4 normal");
      resultado = resultado + "IN4_0,";
    }
  } else  {
    if (digitalRead(PinEntrada4) == 0)  {
      Serial.println("Entrada3 normal");
      resultado = resultado + "IN4_0,";
    } else {
      resultado = resultado + "IN4_1,";
      Serial.println("Entrada 4 abierta(activa)");
    }
  }
  //Serial.println("##################");

  //________
  if (digitalRead(PinOut1Pgm) == 0) {
    StPGM1 = 0;
    resultado = resultado + "PGM1_0,";
  } else {
    StPGM1 = 1;
    resultado = resultado + "PGM1_1,";
  }

  //_______________
  if (digitalRead(PinOut2Pgm) == 0)  {
    StPGM2 = 0;
    resultado = resultado + "PGM2_0,";
  } else {
    StPGM2 = 1;
    //Serial.println("Leyo salida 2 determinando que esta en 1");       //
    resultado = resultado + "PGM2_1,";
  }

  //________
  if (digitalRead(PinOut3Pgm) == 0) {
    StPGM3 = 0;
    resultado = resultado + "PGM3_0,";
  } else {
    StPGM3 = 1;
    resultado = resultado + "PGM3_1,";
  }

  //_______________
  if (digitalRead(PinOut4Pgm) == 0)  {
    StPGM4 = 0;
    resultado = resultado + "PGM4_0,";
  } else {
    StPGM4 = 1;
    //Serial.println("Leyo salida 2 determinando que esta en 1");       //
    resultado = resultado + "PGM4_1,";
  }
}

void ActivaSalida1() {
  Serial.println("*********************");
  Serial.println("ACTIVA PGM1");
  digitalWrite(PinOut1Pgm, HIGH);
  StPGM1 = 1;
  LeeModoUsoPGM1();
  LeeTiempoPGM1();
  CntTimOut1 = ValTimPGM1;
  if (ValModoPGM1 == 0) {
    Serial.println("Es modo toggle");

  } else if (ValModoPGM1 == 1) {
    Serial.println("Es cuenta PGM1 en segundos");
    DebeContarSegPGM1 = 1;          //habilita cuenta en SEGUNDOS para PGM 1

  } else if (ValModoPGM1 == 2) {
    Serial.println("Es cuenta PGM1 en minutos");
    CntSegundosPGM1 = 0;            // habilita cuenta en MINUTOS para PGM 1
    DebeContarMinPGM1 = 1;

  } else if (ValModoPGM1 == 3) {
    Serial.println("Es cuenta en horas");
    DebeContarHrPGM1 = 1;    // habilita cuenta en HORAS para PGM 1
    CntSegundosPGM1 = 0;
    CntMinutosPGM1 = 0;
    Serial.println("Cuenta tiempo PGM1 en HORAS");
  }
  delay(1000);
  CmdAccion = "Activacion salida 1";
  Selecciona_RespuestaComun_Notificacion();
  EnviaNotificacionPush(NumSerialESP);
  //DebeEnviarNotificacionesSegunRuteo = 1;
}


void ActivaSalida2() {
  Serial.println("*********************");
  Serial.println("ACTIVA PGM2");
  digitalWrite(PinOut2Pgm, HIGH);
  StPGM2 = 1;
  LeeModoUsoPGM2();
  LeeTiempoPGM2();
  LeeTiempoPGM2();
  // EstaEnviandoNotificacion=1;
  CntTimOut2 = ValTimPGM2;
  if (ValModoPGM2 == 0) {
    Serial.println("Es modo toggle");

  } else if (ValModoPGM2 == 1) {
    Serial.println("Es cuenta PGM2 en segundos");
    DebeContarSegPGM2 = 1;          //habilita cuenta en SEGUNDOS para PGM 2
    //DebeContarSegPGM2 = 0;

  } else if (ValModoPGM2 == 2) {
    Serial.println("Es cuenta PGM2 en minutos");
    DebeContarMinPGM2 = 1;            // habilita cuenta en MINUTOS para PGM 2

  } else if (ValModoPGM2 == 3) {
    Serial.println("Es cuenta en horas");
    DebeContarHrPGM2 = 1;    // habilita cuenta en HORAS para PGM 2
    Serial.println("Cuenta tiempo PGM2 en HORAS");
  }
  delay(1000);
  CmdAccion = "Activacion salida 2";
  Selecciona_RespuestaComun_Notificacion();
  EnviaNotificacionPush(NumSerialESP);
  //DebeEnviarNotificacionesSegunRuteo = 1;
}

void ActivaSalida3() {
  Serial.println("*********************");
  Serial.println("ACTIVA PGM3");
  digitalWrite(PinOut3Pgm, HIGH);
  StPGM3 = 1;
  LeeModoUsoPGM3();
  LeeTiempoPGM3();
  CntTimOut3 = ValTimPGM3;
  if (ValModoPGM3 == 0) {
    Serial.println("Es modo toggle");

  } else if (ValModoPGM3 == 1) {
    Serial.println("Es cuenta PGM3  en segundos");
    DebeContarSegPGM3 = 1;          //habilita cuenta en SEGUNDOS para PGM 3

  } else if (ValModoPGM3 == 2) {
    Serial.println("Es cuenta PGM3  en minutos");
    DebeContarMinPGM3 = 1;            // habilita cuenta en MINUTOS para PGM 3
    DebeContarSegPGM3 = 0;

  } else if (ValModoPGM3 == 3) {
    Serial.println("Es cuenta PGM3  en horas");
    DebeContarHrPGM3 = 1;    // habilita cuenta en HORAS para PGM 3
    DebeContarSegPGM3 = 0;    //Byte de usuarios hab. por salida 3
    DebeContarMinPGM3 = 0;
    Serial.println("Cuenta tiempo en HORAS");
  }
  delay(1000);
  CmdAccion = "Activacion salida 3";
  Selecciona_RespuestaComun_Notificacion();
  EnviaNotificacionPush(NumSerialESP);
  //DebeEnviarNotificacionesSegunRuteo = 1;
}

void  ActivaSalida4() {
  Serial.println("*********************");
  Serial.println("ACTIVA PGM4");
  digitalWrite(PinOut4Pgm, HIGH);
  StPGM4 = 1;
  LeeModoUsoPGM4();
  LeeTiempoPGM4();
  CntTimOut4 = ValTimPGM4;
  if (ValModoPGM4 == 0) {
    Serial.println("Es modo toggle");

  } else if (ValModoPGM4 == 1) {
    Serial.println("Es cuenta PGM4 en segundos");
    DebeContarSegPGM4 = 1;          //habilita cuenta en SEGUNDOS para PGM 1

  } else if (ValModoPGM4 == 2) {
    Serial.println("Es cuenta PGM4 en minutos");
    DebeContarMinPGM4 = 1;            // habilita cuenta en MINUTOS para PGM 1
    CntSegundosPGM4 = 0;
    DebeContarSegPGM4 = 0;

  } else if (ValModoPGM4 == 3) {
    Serial.println("Es cuenta en horas");
    DebeContarHrPGM4 = 1;    // habilita cuenta en HORAS para PGM 1
    CntSegundosPGM4 = 0;    //Byte de usuarios hab. por salida 1
    CntMinutosPGM4 = 0;
    Serial.println("Cuenta tiempo PGM4 en HORAS");
    DebeContarSegPGM4 = 0;
  }
  delay(1000);
  CmdAccion = "Activacion salida 4";
  Selecciona_RespuestaComun_Notificacion();
  EnviaNotificacionPush(NumSerialESP);
  //DebeEnviarNotificacionesSegunRuteo = 1;
}


void DecodificaDatosTelefono() {
  pos1 = variable.indexOf(ValorBuscado);
  if (pos1 >= 0)  {
    Serial.println("BUFFER A ANALIZAR : " + String(variable));
    Serial.println("________________________________________");
    int TotalRecibido = variable.length();
    Serial.println(TotalRecibido);

    String BufferLimpio;
    BufferLimpio = variable.substring(pos1, pos1 + TotalRecibido);
    Serial.println(BufferLimpio);
    Serial.println("__");

    pos1 = 8;

    string_variable = BufferLimpio.substring(pos1, pos1 + 10);
    Serial.println("ID usua 1 : " +  string_variable);
    address = E2_IDUsua1;
    GrabaPaqueteE2();
    LeeIDUsuario1();

    //___________
    string_variable = BufferLimpio.substring(pos1 + 11, pos1 + 21);
    //string_resultado.trim();
    Serial.println("ID usua 2 : " +  string_variable);
    address = E2_IDUsua2;
    GrabaPaqueteE2();
    LeeIDUsuario2();

    //__________
    string_variable = BufferLimpio.substring(pos1 + 22, pos1 + 32);
    Serial.println("ID usua 3 : " +  string_variable);
    address = E2_IDUsua3;
    GrabaPaqueteE2();
    LeeIDUsuario3();

    //___________
    string_variable = BufferLimpio.substring(pos1 + 33, pos1 + 43);
    //string_resultado.trim();
    Serial.println("ID usua 4 : " +  string_variable);
    address = E2_IDUsua4;
    GrabaPaqueteE2();
    LeeIDUsuario4();

    //__________________________________________________________________________________________________
    string_variable = BufferLimpio.substring(pos1 + 44, pos1 + 64);
    // Serial.println("ubicacion usua 1 : " +  string_variable);
    BorraE2promNombreUsuario1();
    address = E2_NombreUsua1;
    GrabaPaqueteE2();
    LeeUbicaUsuario1();

    //___________
    string_variable = BufferLimpio.substring(pos1 + 65, pos1 + 85);
    //Serial.println("ubicacion usua 2 : " +  string_variable);
    BorraE2promNombreUsuario2();
    address = E2_NombreUsua2;
    GrabaPaqueteE2();
    LeeUbicaUsuario2();

    //___________
    string_variable = BufferLimpio.substring(pos1 + 86, pos1 + 106);
    //Serial.println("ubicacion usua 3 : " +  string_variable);
    BorraE2promNombreUsuario3();
    address = E2_NombreUsua3;
    GrabaPaqueteE2();
    LeeUbicaUsuario3();

    //___________
    string_variable = BufferLimpio.substring(pos1 + 107, pos1 + 127);
    //Serial.println("ubicacion usua 4 : " +  string_variable);
    BorraE2promNombreUsuario4();
    address = E2_NombreUsua4;
    GrabaPaqueteE2();
    LeeUbicaUsuario4();
    //___________________________
    string_variable = variable.substring(pos + 168, pos + 188);
    //Serial.println("Ubicacion modulo: " + string_variable);
    address = E2_UbicacionMod;
    GrabaPaqueteE2();
    LeeUbicacionModulo();

    //_________
    resultado = variable.substring(pos + 189, pos + 191);
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN1;
    WrE2();
    Serial.println("Tiempo CONF.IN1 : " + String(DatoE2));
    //_________

    resultado = variable.substring(pos + 192, pos + 194);     //tiempo confirmacion ent2
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN2;
    WrE2();
    Serial.println("Tiempo CONF.IN 2 : " + String(DatoE2));

    resultado = variable.substring(pos + 195, pos + 197);     //tiempo confirmacion ent3
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN3;
    WrE2();
    Serial.println("Tiempo CONF.IN 3 : " + String(DatoE2));

    resultado = variable.substring(pos + 198, pos + 200);     //tiempo confirmacion ent4
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN4;
    WrE2();
    Serial.println("Tiempo CONF.IN 4 : " + String(DatoE2));

    //taller              ,01,02,03,04,01,02,03,04,1,3,3,2,3,3,3,3,3,3,3,3,0,0,0,0,01,01,01,01,

    //______________________________________________________
    resultado = variable.substring(pos + 201, pos + 203);
    //Serial.println(resultado);
    address = E2TimPGM1;
    DatoE2 = resultado.toInt();
    Serial.println("Tiempo de PGM 1 : " + String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 204, pos + 206);
    //Serial.println(resultado);
    address = E2TimPGM2;
    DatoE2 = resultado.toInt();
    Serial.println("Tiempo de PGM 2 : " +  String(DatoE2));
    WrE2();

    //____________
    resultado = variable.substring(pos + 207, pos + 209);
    //Serial.println(resultado);
    address = E2TimPGM3;
    DatoE2 = resultado.toInt();
    Serial.println("Tiempo de PGM 3 : " + String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 210, pos + 212);
    //Serial.println(resultado);
    address = E2TimPGM4;
    DatoE2 = resultado.toInt();
    Serial.println("Tiempo de PGM 4 : " +  String(DatoE2));
    WrE2();

    //__________________________
    resultado = variable.substring(pos + 213, pos + 214);   //(pos + 105, pos + 106);             //Lee tiempo PGM1
    //Serial.println(resultado);
    address = E2ModoPGM1;
    DatoE2 = resultado.toInt();
    Serial.println("Modo PGM 1 : " +  String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 215, pos + 216);
    //Serial.println(resultado);
    address = E2ModoPGM2;
    DatoE2 = resultado.toInt();
    Serial.println("Modo PGM 2 : " +  String(DatoE2));
    WrE2();
    //_____________
    resultado = variable.substring(pos + 217, pos + 218);
    //Serial.println(resultado);
    address = E2ModoPGM3;
    DatoE2 = resultado.toInt();
    Serial.println("Modo PGM 3 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 219, pos + 220);
    //Serial.println(resultado);
    address = E2ModoPGM4;
    DatoE2 = resultado.toInt();
    Serial.println("Modo PGM 4 : " +  String(DatoE2));
    WrE2();

    //__________________________
    resultado = variable.substring(pos + 221, pos + 222);
    //Serial.println(resultado);
    address = E2_RUTEO_OUT1;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO OUT1 : " +  String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 223, pos + 224);
    //Serial.println(resultado);
    address = E2_RUTEO_OUT2;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO OUT2 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 225, pos + 226);
    //Serial.println(resultado);
    address = E2_RUTEO_OUT3;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO OUT 3 : " +  String(DatoE2));
    WrE2();
    //_____________
    resultado = variable.substring(pos + 227, pos + 228);
    //Serial.println(resultado);
    address = E2_RUTEO_OUT4;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO OUT 4 : " +  String(DatoE2));
    WrE2();

    //__________________________
    resultado = variable.substring(pos + 229, pos + 230);
    //Serial.println(resultado);
    address = E2_RUTEO_IN1;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO IN1 : " +  String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 231, pos + 232);
    //Serial.println(resultado);
    address = E2_RUTEO_IN2;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO IN 2 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 233, pos + 234);
    //Serial.println(resultado);
    address = E2_RUTEO_IN3;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO IN3 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 235, pos + 236);
    //Serial.println(resultado);
    address = E2_RUTEO_IN4;
    DatoE2 = resultado.toInt();
    Serial.println("Modo RUTEO IN4 : " +  String(DatoE2));
    WrE2();

    //__________________________
    resultado = variable.substring(pos + 237, pos + 238);
    //Serial.println(resultado);
    address = E2_TIPO_LAZO_IN1;
    DatoE2 = resultado.toInt();
    Serial.println("Modo TIPO LAZO Z1 : " +  String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 239, pos + 240);
    //Serial.println(resultado);
    address = E2_TIPO_LAZO_IN2;
    DatoE2 = resultado.toInt();
    Serial.println("Modo TIPO LAZO Z2 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 241, pos + 242);
    //Serial.println(resultado);
    address = E2_TIPO_LAZO_IN3;
    DatoE2 = resultado.toInt();
    Serial.println("Modo TIPO LAZO Z3 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 243, pos + 244);
    //Serial.println(resultado);
    address = E2_TIPO_LAZO_IN4;
    DatoE2 = resultado.toInt();
    Serial.println("Modo TIPO LAZO Z4 : " +  String(DatoE2));
    WrE2();

    //__________________________
    resultado = variable.substring(pos + 245, pos + 247);
    //Serial.println(resultado);
    address = E2_VINC_IN1;
    DatoE2 = resultado.toInt();
    Serial.println("Modo VINCULO IO 1 : " +  String(DatoE2));
    WrE2();

    //_____
    resultado = variable.substring(pos + 248 + pos + 250);
    //Serial.println(resultado);
    address = E2_VINC_IN2;
    DatoE2 = resultado.toInt();
    Serial.println("Modo VINCULO IO 2 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 251, pos + 253);
    //Serial.println(resultado);
    address = E2_VINC_IN3;
    DatoE2 = resultado.toInt();
    Serial.println("Modo VINCULO IO 3 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 254, pos + 256);
    //Serial.println(resultado);
    address = E2_VINC_IN4;
    DatoE2 = resultado.toInt();
    Serial.println("Modo VINCULO IO 4 : " +  String(DatoE2));
    WrE2();

    //_____________
    resultado = variable.substring(pos + 257, pos + 275);
    address = E2_UBICA_IN1;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionIN1();

    //_____________
    resultado = variable.substring(pos + 276, pos + 294);
    address = E2_UBICA_IN2;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionIN2();

    //_____________
    resultado = variable.substring(pos + 295, pos + 313);
    address = E2_UBICA_IN3;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionIN3();

    //_____________
    resultado = variable.substring(pos + 314, pos + 332);
    address = E2_UBICA_IN4;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionIN4();

    //____________
    resultado = variable.substring(pos + 333, pos + 351);
    address = E2_UBICA_OUT1;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionOUT1();

    //_____________
    resultado = variable.substring(pos + 352, pos + 370);
    address = E2_UBICA_OUT2;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionOUT2();

    //_____________
    resultado = variable.substring(pos + 371, pos + 389);
    address = E2_UBICA_OUT3;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionOUT3();

    //_____________
    resultado = variable.substring(pos + 390, pos + 408);
    address = E2_UBICA_OUT4;
    string_variable = resultado;
    GrabaPaqueteE2();
    LeeUbicacionOUT4();

    CmdAccion = "OK UPLOAD2";
    DebeResponderAOrigen = 1;
  }
}

void DecoNombreUbicaImeiUsuario() {
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    if (variable.substring(pos + 34, pos + 35) != ",") {
      Serial.println("Esta desfasado la recepcion y no debe guardar!");
      return;
    }
    Serial.println("_____________________");
    resultado = variable.substring(pos + 7, pos + 13);            //Id de modulo
    Serial.println("Id modulo : " + resultado);

    string_variable = variable.substring(pos + 14, pos + 34);     // texto ubicacion modulo
    Serial.println("Ubicacion modulo : " + string_variable);
    TxtUbicaModulo = string_variable;

    string_variable = variable.substring(pos + 35, pos + 55);     //Texto NOMBRE usuario a crear
    Serial.println("Nombre usuario : " + string_variable);
    NombreUsuarioNuevo = string_variable;

    //___________________-
    string_variable = variable.substring(pos + 56, pos + 66);   //  IMEI de nuevo usuario
    IdImeiUsuarioNuevo = string_variable;
    Serial.println("El imei que sera su publicador= " + IdImeiUsuarioNuevo);

    //______________
    string_variable = variable.substring(pos + 67, pos + 68);     //ubicacion donde guardar
    DatoE2 = string_variable.toInt();
    Serial.println("_______________________________");


    //ahora guarda en meorioa segunposicion que le hay dicho
    //cambio 21/4/20
    //si el imei ya estaba en la memoria-> fue creado por otro por lo tanto no cambia los datos aqui!

    byte ImeiExistente = 0;
    if (IdImeiUsuarioNuevo == ValIdReporte1 ) {
      ImeiExistente = 1;

    } else if (IdImeiUsuarioNuevo == ValIdReporte2 ) {
      ImeiExistente = 1;

    } else if (IdImeiUsuarioNuevo == ValIdReporte3 ) {
      ImeiExistente = 1;

    } else if (IdImeiUsuarioNuevo == ValIdReporte4 ) {
      ImeiExistente = 1;
    }
    //if (ImeiExistente == 0) {
    //  Serial.println("No existe ese usuario previamente, puede guardar sus datos");
    address = E2_UbicacionMod;
    string_variable = TxtUbicaModulo;
    GrabaPaqueteE2();
    LeeUbicacionModulo();

    //determina posicion libre para asignar nuevo usuario
    if (DatoE2 == 1) {
      BorraE2promNombreUsuario1();
      string_variable = NombreUsuarioNuevo;
      address = E2_NombreUsua1;    //caca
      GrabaPaqueteE2();

      LeeUbicaUsuario1();
      string_variable = IdImeiUsuarioNuevo;
      address = E2_IDUsua1;
      GrabaPaqueteE2();
      LeeIDUsuario1();


    } else if (DatoE2 == 2) {
      BorraE2promNombreUsuario2();
      string_variable = NombreUsuarioNuevo;
      address = E2_NombreUsua2;
      GrabaPaqueteE2();
      LeeUbicaUsuario2();
      string_variable = IdImeiUsuarioNuevo;
      address = E2_IDUsua2;
      GrabaPaqueteE2();
      LeeIDUsuario2();

    } else if (DatoE2 == 3) {
      BorraE2promNombreUsuario3();
      string_variable = NombreUsuarioNuevo;
      address = E2_NombreUsua3;
      GrabaPaqueteE2();
      LeeUbicaUsuario3();
      string_variable = IdImeiUsuarioNuevo;
      address = E2_IDUsua3;
      GrabaPaqueteE2();
      LeeIDUsuario3();

    } else if (DatoE2 == 4) {
      BorraE2promNombreUsuario4();
      string_variable = NombreUsuarioNuevo;
      address = E2_NombreUsua4;
      GrabaPaqueteE2();
      LeeUbicaUsuario4();
      string_variable = IdImeiUsuarioNuevo;
      address = E2_IDUsua4;
      GrabaPaqueteE2();
      LeeIDUsuario4();
    }

    EnviaDatosModuloAUsuario();
    //CmdAccion = "newmod " + NumSerialESP + "," + ValUbicacionMod + "," + NombreUsuarioNuevo + "," + IdImeiUsuarioNuevo + "," + String(DatoE2);
    variable = "";
    DatoRecibido = "";
  }
}

void ProcesaTiempoModoPGM() {
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    //Serial.println(variable);
    Serial.println("__________________________");
    //Serial.println(pos);

    resultado = variable.substring(pos + 6, pos + 7);     //1
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2ModoPGM1;
    WrE2();

    resultado = variable.substring(pos + 7, pos + 9);   //2
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2TimPGM1;
    WrE2();
    LeeModoUsoPGM1();
    LeeTiempoPGM1();

    //_____________
    pos = variable.indexOf("TSAL2");
    resultado = variable.substring(pos + 6, pos + 7);   //2
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2ModoPGM2;
    WrE2();
    resultado = variable.substring(pos + 7, pos + 9);  //3
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2TimPGM2;
    WrE2();
    LeeModoUsoPGM2();
    LeeTiempoPGM2();

    //_____________
    pos = variable.indexOf("TSAL3");
    resultado = variable.substring(pos + 6, pos + 7);   //,
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2ModoPGM3;
    WrE2();

    resultado = variable.substring(pos + 7, pos + 9);  //T
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2TimPGM3;
    WrE2();
    LeeModoUsoPGM3();
    LeeTiempoPGM3();

    //_________________
    pos = variable.indexOf("TSAL4");
    resultado = variable.substring(pos + 6, pos + 7);   //,
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    DatoE2 = resultado.toInt();
    address = E2ModoPGM4;
    WrE2();

    resultado = variable.substring(pos + 7, pos + 9);
    //Serial.println(resultado);
    DatoE2 = resultado.toInt();
    //Serial.println(String(DatoE2));
    address = E2TimPGM4;
    WrE2();
    LeeModoUsoPGM4();
    LeeTiempoPGM4();
    Serial.println("************************");
    CmdAccion = "modpgm1";
    DebeResponderAOrigen = 1;
  }
}

void ControlaTiempoDefSSID() {
  if (DebeContarTiempoDefault != 0)  {
    CntTimDefault++;      // = CntTimDefault + 1; //
    if (CntTimDefault < 6) {
      Serial.println("Cuenta tiempo para borrar SSID primero:");
    } else {
      Serial.println("Ahora cuenta para generar DEFAULT!");
    }
    Serial.println(CntTimDefault);
    //para hacer togle en cada acceso
    if (YaContol5SegRstSSID == 0) {
      int state1 = digitalRead(PinLedVerde); // get the current state of GPIO1 pin
      digitalWrite(PinLedVerde, !state1);
      if (CntTimDefault == 5) {
        //Luego de los primeros 5 segundo si soltar-> marca que Ya TENGO IUUNB MINIMO de 5 seg. y si suelto ahora
        //ya anulare el nombre de la RED ACTUAL!
        //Pero si sigo pulsado voy en busqueda del borrado general de la EEPTROM DEL MODULO (default)
        YaContol5SegRstSSID = 1;
        digitalWrite(PinLedVerde, HIGH);    //Deja fijo el led
        //CACA
        //DEBERIA ENCENDER UN LED FIJO POR 3 SEGUNDOS PARA DETERMINAR SI CONTINUA LA CUENTA RUMBO AL DEFAULT
      }

    } else if (CntTimDefault > 9) {
      Serial.println("Ya conto mas de 10 segundos-> va a general DEFAULT!");
      YaContoTiempoDefault = 1;  //YA gewnera DEFAULT!
      YaContol5SegRstSSID = 0;        //Ya no borra SSID de la red!
      //DebeContarTiempoDefault=0;
      DebeOscErrStatus = 1;
    }
  }
}


//________________________
void ControlaTimSegPGM1() {
  if (DebeContarSegPGM1 != 0)    {
    //si debe contar segundos en PGM1 ->
    if (CntTimOut1 != 0)      {
      CntTimOut1--;
      //stringOne = "Tiempo actual cuenta OUT 1 en seg= ";
      //Serial.println(stringOne + CntTimOut1);
      if (CntTimOut1 == 0) {
        DebeDesactivarSalida1 = 1;
        Serial.println("Debe desactivar salida 1 por segundos");
        DesactivaPorTiempoPGM1 = 1;
        DebeContarSegPGM1 = 0;
      }
    }
  }
}

void ControlaTimSegPGM2() {
  if (DebeContarSegPGM2 != 0)    {
    if (CntTimOut2 != 0)      {
      CntTimOut2--;
      //stringOne = "Tiempo actual cuenta OUT 2 en seg= ";
      //Serial.println(stringOne + CntTimOut2);
      if (CntTimOut2 == 0) {
        DebeDesactivarSalida2 = 1;
        DesactivaPorTiempoPGM2 = 1;
        DebeContarSegPGM2 = 0;
        Serial.println("Debe desactivar salida 2 por segundos");
      }
    }
  }
}

void ControlaTimSegPGM3() {
  if (DebeContarSegPGM3 != 0) {
    if (CntTimOut3 != 0) {
      CntTimOut3--;
      //stringOne = "Tiempo actual cuenta OUT 3 en seg= ";
      //Serial.println(stringOne + CntTimOut3);
      if (CntTimOut3 == 0) {
        DebeDesactivarSalida3 = 1;
        DesactivaPorTiempoPGM3 = 1;
        Serial.println("Debe desactivar salida 3 por segundos");
        DebeContarSegPGM3 = 0;
      }
    }
  }
}

//___
void ControlaTimSegPGM4() {
  if (DebeContarSegPGM4 != 0) {
    if (CntTimOut4 != 0) {
      CntTimOut4--;
      //stringOne = "Tiempo actual cuenta OUT 4 en seg= ";
      //Serial.println(stringOne + CntTimOut4);
      if (CntTimOut4 == 0) {
        DebeDesactivarSalida4 = 1;
        DesactivaPorTiempoPGM4 = 1;
        DebeContarSegPGM4 = 0;
        Serial.println("Debe desactivar salida 4 por segundos");
      }
    }
  }
}

void ControlaTimMinPGM1() {
  if (DebeContarMinPGM1 != 0) {
    //si debe contar MINUTOS en PGM1 ->
    //Serial.println("Cuenta en minutos");
    if (CntSegundosPGM1 == 60) {
      CntSegundosPGM1 = 0;
      if (CntTimOut1 != 0) {
        // si esta contando aun tiempo pgm1 en minutos...
        CntTimOut1--;     // = CntTimOut1 - 1;
        stringOne = "Tiempo actual cuenta minutos OUT 1= ";
        Serial.println(stringOne + CntTimOut1);
        if (CntTimOut1 == 0) {
          //Serial.println("Fin tiempo de activacion out 1");
          //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
          DebeDesactivarSalida1 = 1;
          DesactivaPorTiempoPGM1 = 1;
          Serial.println("Debe desactivar salida 1");
          DebeContarMinPGM1 = 0;
          CntSegundosPGM1 = 0;
        }
      }
    }
    CntSegundosPGM1++;
    //stringOne = "Cuenta segundos en contador de minutos PGM1= ";
    //Serial.println(stringOne + CntSegundosPGM1);
  }
}

void ControlaTimMinPGM2() {
  if (DebeContarMinPGM2 != 0) {
    //si debe contar MINUTOS en PGM2 ->
    if (CntSegundosPGM2 == 60) {
      CntSegundosPGM2 = 0;
      if (CntTimOut2 != 0) {
        // si esta contando aun tiempo pgm1 en minutos...
        CntTimOut2--;     // = CntTimOut1 - 1;
        stringOne = "Tiempo actual cuenta minutos OUT 2= ";
        Serial.println(stringOne + CntTimOut2);
        if (CntTimOut2 == 0) {
          DebeDesactivarSalida2 = 1;
          DesactivaPorTiempoPGM2 = 1;
          Serial.println("Debe desactivar salida 2");
          DebeContarMinPGM2 = 0;
          CntSegundosPGM2 = 0;
        }
      }
    }
    CntSegundosPGM2++;
    //stringOne = "Cuenta segundos en contador de minutos PGM2= ";
    //Serial.println(stringOne + CntSegundosPGM2);
  }
}

void ControlaTimMinPGM3() {
  if (DebeContarMinPGM3 != 0) {
    if (CntSegundosPGM3 == 60) {
      CntSegundosPGM3 = 0;
      if (CntTimOut3 != 0) {
        // si esta contando aun tiempo pgm3 en minutos...
        CntTimOut3--;
        stringOne = "Tiempo actual cuenta minutos OUT 3= ";
        Serial.println(stringOne + CntTimOut3);
        if (CntTimOut3 == 0) {
          DebeDesactivarSalida3 = 1;
          DesactivaPorTiempoPGM3 = 1;
          Serial.println("Debe desactivar salida 3");
          DebeContarMinPGM3 = 0;
          CntSegundosPGM3 = 0;
        }
      }
    }
    CntSegundosPGM3++;
    //stringOne = "Cuenta segundos en contador de minutos PGM3= ";
    //Serial.println(stringOne + CntSegundosPGM3);
  }
}

void ControlaTimMinPGM4() {
  if (DebeContarMinPGM4 != 0) {
    if (CntSegundosPGM4 == 60) {
      CntSegundosPGM4 = 0;
      if (CntTimOut4 != 0) {
        // si esta contando aun tiempo pgm4 en minutos...
        CntTimOut4--;
        stringOne = "Tiempo actual cuenta minutos OUT 4= ";
        Serial.println(stringOne + CntTimOut4);
        if (CntTimOut4 == 0) {
          DebeDesactivarSalida4 = 1;
          DesactivaPorTiempoPGM4 = 1;
          Serial.println("Debe desactivar salida 4");
          DebeContarMinPGM4 = 0;
          CntSegundosPGM4 = 0;
        }
      }
    }
    CntSegundosPGM4++;
    //stringOne = "Cuenta segundos en contador de minutos PGM4= ";
    //Serial.println(stringOne + CntSegundosPGM4);
  }
}

//_______________
void ControlaTimHorasPGM1() {
  if (DebeContarHrPGM1 != 0)    {
    if (CntMinutosPGM1 == 60)      {
      CntMinutosPGM1 = 0;
      //Serial.println("Conto 1 hora de PGM 1");
      //si esta contando horas en PGM1->
      CntTimOut1--;     // = CntTimOut1 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
      stringOne = "Tiempo actual cuenta horas OUT 1= ";
      Serial.println(stringOne + CntTimOut1);
      if (CntTimOut1 == 0)
      {
        //Serial.println("Fin tiempo de activacion out 1 en horas");
        //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
        DebeDesactivarSalida1 = 1;
        DesactivaPorTiempoPGM1 = 1;
        //Serial.println("Debe desactivar salida 1 POR HORAS");
        DebeContarHrPGM1 = 0;
        CntMinutosPGM1 = 0;
        CntSegundosPGM1 = 0;
      }
      //conto 1 hora
    }
    //Serial.println("Contador de segundos en modo PGM1 HORAS:");
    CntSegundosPGM1++;
    //stringOne = "Cuenta segundos en contador de HORAS PGM1= ";
    //Serial.println(stringOne + CntSegundosPGM1);

    if (CntSegundosPGM1 == 60) {
      CntSegundosPGM1 = 0;
      CntMinutosPGM1++;         //=CntMinutosPGM1+1;    //Incrementa contadOr minUtos para PGM 1
      //Serial.println("Incrementa contador de minutos de PGM1 en modo HORAS");
      //Serial.println(CntMinutosPGM1);
    }
  }
}
//caca3

//____
void ControlaTimHorasPGM2() {
  if (DebeContarHrPGM2 != 0) {
    //si debe contar MINUTOS en PGM2 ->
    //Si hay cuenta autorizada de PGM 1 en horas
    //Serial.println("Esta cointando tiempo PGM1 en HORAS");
    if (CntMinutosPGM2 == 60) {
      CntMinutosPGM2 = 0;
      //Serial.println("Conto 1 hora de PGM 1");
      //si esta contando horas en PGM1->
      CntTimOut2--;      // = CntTimOut2 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
      stringOne = "Tiempo actual cuenta horas OUT 2= ";
      Serial.println(stringOne + CntTimOut2);

      if (CntTimOut2 == 0) {
        //Serial.println("Fin tiempo de activacion out 2en horas");
        //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
        DebeDesactivarSalida2 = 1;
        DesactivaPorTiempoPGM2 = 1;
        //Serial.println("Debe desactivar salida 2 POR HORAS");
        DebeContarHrPGM2 = 0;
        CntMinutosPGM2 = 0;
        CntSegundosPGM2 = 0;
      }
      //conto 1 hora
    }
    //Serial.println("Contador de segundos en modo PGM1 HORAS:");
    CntSegundosPGM2++;
    //stringOne = "Cuenta segundos en contador de HORAS PGM2= ";
    //Serial.println(stringOne + CntSegundosPGM1);
    if (CntSegundosPGM2 == 60) {
      CntSegundosPGM2 = 0;
      CntMinutosPGM2++;
      //Serial.println("Incrementa contador de minutos de PGM2 en modo HORAS");
      //Serial.println(CntMinutosPGM2);
    }
  }
}

//_______________
void ControlaTimHorasPGM3() {
  if (DebeContarHrPGM3 != 0)    {
    if (CntMinutosPGM3 == 60)      {
      CntMinutosPGM3 = 0;
      //Serial.println("Conto 1 hora de PGM 3");
      //si esta contando horas en PGM1->
      CntTimOut3--;     // = CntTimOut1 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
      stringOne = "Tiempo actual cuenta horas OUT 3= ";
      Serial.println(stringOne + CntTimOut3);
      if (CntTimOut3 == 0) {
        //Serial.println("Fin tiempo de activacion out 1 en horas");
        //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
        DebeDesactivarSalida3 = 1;
        DesactivaPorTiempoPGM3 = 1;
        //Serial.println("Debe desactivar salida 3 POR HORAS");
        DebeContarHrPGM3 = 0;
        CntMinutosPGM3 = 0;
        CntSegundosPGM3 = 0;
      }
      //conto 1 hora
    }
    //Serial.println("Contador de segundos en modo PGM1 HORAS:");
    CntSegundosPGM3++;
    //stringOne = "Cuenta segundos en contador de HORAS PGM3= ";
    //Serial.println(stringOne + CntSegundosPGM3);

    if (CntSegundosPGM3 == 60) {
      CntSegundosPGM3 = 0;
      CntMinutosPGM3++;
      //Serial.println("Incrementa contador de minutos de PGM3 en modo HORAS");
      //Serial.println(CntMinutosPGM3);
    }
  }
}

void ControlaTimHorasPGM4() {
  if (DebeContarHrPGM4 != 0) {
    //Serial.println("Esta contando tiempo PGM1 en HORAS");
    if (CntMinutosPGM4 == 60) {
      CntMinutosPGM4 = 0;
      //Serial.println("Conto 1 hora de PGM 1");
      CntTimOut4--;      // = CntTimOut2 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
      //stringOne = "Tiempo actual cuenta horas OUT 2= ";
      //Serial.println(stringOne + CntTimOut2);
      if (CntTimOut4 == 0) {
        //Serial.println("Fin tiempo de activacion out 2en horas");
        //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
        DebeDesactivarSalida4 = 1;
        DesactivaPorTiempoPGM4 = 1;
        DebeContarHrPGM4 = 0;
        CntMinutosPGM4 = 0;
        CntSegundosPGM4 = 0;
        //Serial.println("Debe desactivar salida 2 POR HORAS");
      }
      //conto 1 hora
    }
    //Serial.println("Contador de segundos en modo PGM1 HORAS:");
    CntSegundosPGM4++;
    //stringOne = "Cuenta segundos en contador de HORAS PGM4= ";
    //Serial.println(stringOne + CntSegundosPGM4);
    if (CntSegundosPGM4 == 60) {
      CntSegundosPGM4 = 0;
      CntMinutosPGM4++;
      //Serial.println("Incrementa contador de minutos de PGM2 en modo HORAS");
      //Serial.println(CntMinutosPGM4);
    }
  }
}

void DecodificaRuteoOutPGM() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    Serial.println("Texto recibido " + variable);
    resultado = variable.substring(pos + 7, pos + 8);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_OUT1;
    WrE2();
    LeeRuteoOut1();

    resultado = variable.substring(pos + 8, pos + 9);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_OUT2;
    WrE2();
    LeeRuteoOut2();
    //__________

    resultado = variable.substring(pos + 9, pos + 10);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_OUT3;
    WrE2();
    LeeRuteoOut3();
    //__________

    resultado = variable.substring(pos + 10, pos + 11);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_OUT4;
    WrE2();
    LeeRuteoOut4();
    CmdAccion = "RUTOUT_OK";
    DebeResponderAOrigen = 1;
  }
}

void DecodificaRuteoInPGM() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    Serial.println("Texto recibido " + variable);
    resultado = variable.substring(pos + 6, pos + 7);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_IN1;
    WrE2();
    LeeRuteoIn1();

    //__________
    resultado = variable.substring(pos + 7, pos + 8);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_IN2;
    WrE2();
    LeeRuteoIn2();

    //__________
    resultado = variable.substring(pos + 8, pos + 9);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_IN3;
    WrE2();
    LeeRuteoIn3();

    //__________
    resultado = variable.substring(pos + 9, pos + 10);
    DatoE2 = resultado.toInt();
    //Serial.println(DatoE2);
    address = E2_RUTEO_IN4;
    WrE2();
    LeeRuteoIn4();

    CmdAccion = "RUTIN_OK";
    DebeResponderAOrigen = 1;
  }
}

void DecodificaUbicaciones() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    Serial.println("Texto recibido " + variable);
    resultado = variable.substring(pos + 12, pos + 30);
    //Serial.println(resultado);
    address = E2_UBICA_IN1;
    string_variable = "                  ";
    GrabaPaqueteE2();
    //caca8
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionIN1();

    resultado = variable.substring(pos + 31, pos + 49);
    //Serial.println(resultado);
    address = E2_UBICA_IN2;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionIN2();

    resultado = variable.substring(pos + 50, pos + 68);
    //Serial.println(resultado);
    address = E2_UBICA_IN3;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionIN3();

    resultado = variable.substring(pos + 69, pos + 87);
    //Serial.println(resultado);
    address = E2_UBICA_IN4;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionIN4();

    // salidas
    resultado = variable.substring(pos + 88, pos + 106);
    //Serial.println(resultado);
    address = E2_UBICA_OUT1;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionOUT1();

    resultado = variable.substring(pos + 107, pos + 125);
    //Serial.println(resultado);
    address = E2_UBICA_OUT2;
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionOUT2();

    resultado = variable.substring(pos + 126, pos + 144);
    //Serial.println(resultado);
    address = E2_UBICA_OUT3;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionOUT3();

    resultado = variable.substring(pos + 145, pos + 163);
    //Serial.println(resultado);
    address = E2_UBICA_OUT4;
    string_variable = "                  ";
    GrabaPaqueteE2();
    GrabaE2_UbicacionUsuarios();
    LeeUbicacionOUT4();

    CmdAccion = "UBICA_OK";
    DebeResponderAOrigen = 1;
  }
}

void DecodeTipoLazo() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    resultado = variable.substring(pos + 9, pos + 16);

    Usuarios = resultado.toInt();
    Serial.println("Recibido");
    Serial.println(Usuarios);
    Serial.println(resultado);

    resultado = variable.substring(pos + 9, pos + 10);
    Usuarios = resultado.toInt();

    if (Usuarios == 1 ) { // if bitwise AND resolves to true
      Serial.println("indica que es zona entrada 4 con contacto NA con MASA como NORMAL y ALARMA por ABIERTA");
      DatoE2 = 1;
    } else {
      Serial.println("indica que es zona entrada 4 con contacto NA con POSITIVO como NORMAL y alarma a masa");
      DatoE2 = 0;
    }
    address = E2_TIPO_LAZO_IN1;
    WrE2();
    LeeLazoIn1();

    //____________
    resultado = variable.substring(pos + 11, pos + 12);
    Serial.println(resultado);
    Usuarios = resultado.toInt();
    if (Usuarios == 1) {
      Serial.println("indica que es zona entrada 3 con contacto NA con MASA como NORMAL y ALARMA por ABIERTA");
      DatoE2 = 1;
    } else {
      Serial.println("indica que es zona entrada 3 con contacto NA con positivo con NORMAL y alarma a masa");
      DatoE2 = 0;
    }
    address = E2_TIPO_LAZO_IN2;
    WrE2();
    LeeLazoIn2();
    //___________

    resultado = variable.substring(pos + 13, pos + 14);
    Serial.println(resultado);
    Usuarios = resultado.toInt();
    if (Usuarios == 1) {
      Serial.println("indica que es zona entrada 2 con contacto NA con MASA como NORMAL y ALARMA por ABIERTA");
      DatoE2 = 1;
    } else {
      Serial.println("indica que es zona entrada 2 con contacto NA con positivo con NORMAL y alarma a masa");
      DatoE2 = 0;
    }
    address = E2_TIPO_LAZO_IN3;
    WrE2();
    LeeLazoIn3();

    //_______
    resultado = variable.substring(pos + 15, pos + 16);
    Serial.println(resultado);
    Usuarios = resultado.toInt();
    if (Usuarios == 1) {
      Serial.println("indica que es zona entrada 1 con contacto NA con MASA como NORMAL y ALARMA por ABIERTA");
      DatoE2 = 1;
    } else {
      Serial.println("indica que es zona entrada 1 con contacto NA con positivo con NORMAL y alarma a masa");
      DatoE2 = 0;
    }
    address = E2_TIPO_LAZO_IN4;
    WrE2();
    LeeLazoIn4();

    //________________
    ValorBuscado = "TENT1";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 6, pos + 8);

    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN1;
    WrE2();
    //Serial.println(DatoE2);
    LeeTConfIn1();
    //_______
    ValorBuscado = "TENT2";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 6, pos + 8);
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN2;
    WrE2();
    //Serial.println(DatoE2);
    LeeTConfIn2();
    //_______
    ValorBuscado = "TENT3";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 6, pos + 8);
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN3;
    WrE2();
    //Serial.println(DatoE2);
    LeeTConfIn3();
    //____
    ValorBuscado = "TENT4";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 6, pos + 8);
    DatoE2 = resultado.toInt();
    address = E2_TIM_CONF_IN4;
    WrE2();
    //Serial.println(DatoE2);
    LeeTConfIn4();
    //TIPOLAZO 1111,TENT1 X,TENT2 X,TENT3 X,TENR4 X
    CmdAccion = "TIPOLAZ";
    DebeResponderAOrigen = 1;
  }
}

void DecodificaVinculos() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    resultado = variable.substring(pos + 5, pos + 7);
    DatoE2 = resultado.toInt();
    address = E2_VINC_IN1;      //para entrada 1 unico byte
    WrE2();
    //Serial.println("Vinculo UNICO BYTE : " + String(DatoE2));
    LeeVincEnt1();

    ValorBuscado = "VIN2";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 5, pos + 7);
    DatoE2 = resultado.toInt();
    address = E2_VINC_IN2;      //para entrada 2 unico byte
    WrE2();
    //Serial.println("Vinculo UNICO BYTE : " + String(DatoE2));
    LeeVincEnt2();

    ValorBuscado = "VIN3";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 5, pos + 7);
    DatoE2 = resultado.toInt();
    address = E2_VINC_IN3;      //para entrada 3 unico byte
    WrE2();
    //Serial.println("Vinculo UNICO BYTE : " + String(DatoE2));
    LeeVincEnt3();

    ValorBuscado = "VIN4";
    pos = variable.indexOf(ValorBuscado);
    resultado = variable.substring(pos + 5, pos + 7);
    DatoE2 = resultado.toInt();
    address = E2_VINC_IN4;      //para entrada 2 unico byte
    WrE2();
    //Serial.println("Vinculo UNICO BYTE : " + String(DatoE2));
    LeeVincEnt4();
    CmdAccion = "VINCULOS_OK";
    DebeResponderAOrigen = 1;
  }
}

void ProcesaBorradoUsuario() {
  if (pos >= 0) {
    Serial.println ("Detecto BORRAR USUARIO!");
    Serial.println(variable);
    GeneradorPublicacion = IDOrigenEnvio;
    resultado = variable.substring(pos + 13, pos + 14);
    posicion = resultado.toInt();
    if (posicion == 1) {
      CmdAccion = "borrausua 1";
      EnviaComandoADestino();
      Serial.println("Va a borra los datos de usuario y nombre 1");
      Carga_Default_ID_Usuario1();     //Limpia todos los datos del usuario 1 en memoria
      BorraE2promNombreUsuario1();

    } else if (posicion == 2) {
      CmdAccion = "borrausua 2";
      EnviaComandoADestino();
      Serial.println("Va a borra los datos de usuario y nombre 2");
      Carga_Default_ID_Usuario2();
      BorraE2promNombreUsuario2();
    } else if (posicion == 3) {
      //Serial.println("debe enviar comando BORRA USUARIO 3");
      CmdAccion = "borrausua 3";
      EnviaComandoADestino();
      Serial.println("Va a borra los datos de usuario y nombre 3");
      Carga_Default_ID_Usuario3();
      BorraE2promNombreUsuario3();

    } else if (posicion == 4) {
      //Serial.println("debe enviar comando BORRA USUARIO 4");
      CmdAccion = "borrausua 4";
      Serial.println("Va a borra los datos de usuario y nombre 4");
      EnviaComandoADestino();
      Carga_Default_ID_Usuario4();
      BorraE2promNombreUsuario4();


    } else {
      Serial.println("NO ENTIENDIO EL NUMERO!!");
    }
    LeeDatosMemoria();
  }
}

void ProcesaBorraModulo() {
  if (pos >= 0)      {
    Serial.println("Detecto BORRAR PERFIL!");
    Serial.println(variable);
    resultado = variable.substring(pos + 9, pos + 15);
    //resultado.replace(',', ' ');
    //resultado.replace('"', ' ');
    //resultado.replace('}', ' ');

    Serial.println(resultado);
    CargaDefault();
    CmdAccion = "borramod ";
    CmdAccion = CmdAccion + resultado;
    Serial.println(CmdAccion);
    DebeResponderAOrigen = 1;
    LeeDatosMemoria();
  }
}

void DecoRuteoSalidasxUsuario() {
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    resultado = variable.substring(pos + 10, pos + 11);
    resultado.toInt();
    posSalida = resultado.toInt();

    resultado1 = variable.substring(pos + 12, pos + 15);
    resultado1 = resultado1.toInt();

    switch (posSalida) {
      case 1:
        DatoE2 = resultado1.toInt();
        address = E2_RutOut1Usua;
        WrE2();
        Serial.println("Byte de usuarios hab. por salida 1 es : " + String(DatoE2));
        LeeRutOut1Usua();     //lee valor guardado
        Usuarios = ValRutOut1Usua;
        DecodeBitsxByte();
        break;

      case 2:
        Serial.println("_____________");
        Serial.println("Es ruteo salida 2 segun usuarios");
        address = E2_RutOut2Usua;
        WrE2();
        LeeRutOut2Usua();     //lee valor guardado
        Usuarios = ValRutOut2Usua;
        DecodeBitsxByte();
        break;

      case 3:
        Serial.println("_____________");
        Serial.println("Es ruteo salida 3 para usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutOut3Usua;
        WrE2();
        LeeRutOut3Usua();     //lee valor guardado
        Usuarios = ValRutOut3Usua;
        DecodeBitsxByte();
        break;

      case 4:
        Serial.println("_____________");
        Serial.println("Es ruteo salida 4 para usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutOut4Usua;
        WrE2();
        LeeRutOut4Usua();     //lee valor guardado
        Usuarios = ValRutOut4Usua;
        DecodeBitsxByte();
        break;
    }
    CmdAccion = "RUTUSUAOUT_OK";
    DebeResponderAOrigen = 1;
  }
}
void DecoRuteoEntradasxUsuario() {
  //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
  pos = variable.indexOf(ValorBuscado);
  if (pos >= 0) {
    resultado = variable.substring(pos + 9, pos + 10);
    resultado.toInt();
    posSalida = resultado.toInt();
    resultado1 = variable.substring(pos + 11, pos + 14);

    Serial.println(resultado);
    Serial.println(resultado1);

    switch (posSalida) {
      case 1:
        Serial.println("_____________");
        Serial.println("Es ruteo ENTRADA 1 s/ usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutIn1Usua;
        WrE2();
        LeeRutIn1Usua();     //lee valor guardado
        Usuarios = ValHabUsuaIN1;
        DecodeBitsxByte();
        break;

      case 2:
        Serial.println("_____________");
        Serial.println("Es ruteo ENTRADA 2 segun usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutIn2Usua;
        WrE2();
        LeeRutIn2Usua();     //lee valor guardado
        Usuarios = ValRutIn2Usua;
        DecodeBitsxByte();
        break;

      case 3:
        Serial.println("_____________");
        Serial.println("Es ruteo ENTRADA 3 para usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutIn3Usua;
        WrE2();
        LeeRutIn3Usua();     //lee valor guardado
        Usuarios = ValRutIn3Usua;
        DecodeBitsxByte();
        break;

      case 4:
        Serial.println("_____________");
        Serial.println("Es ruteo ENTRADA 4 para usuarios");
        DatoE2 = resultado1.toInt();
        address = E2_RutIn4Usua;
        WrE2();
        LeeRutIn4Usua();     //lee valor guardado
        Usuarios = ValRutIn4Usua;
        DecodeBitsxByte();
        break;
    }
    CmdAccion = "RUTUSUAIN_OK";
    DebeResponderAOrigen = 1;
  }
}

//____________
void BorraSSID_E2() {
  if (digitalRead(PinInAux) == 0)  {
    if (HayDeteccionAux == 0) {
      HayDeteccionAux = 1;
      DebeContarTiempoDefault = 1;
      CntTimDefault = 0;        //empieza a contar tiempo para borrar red o default!
      YaContol5SegRstSSID = 0;      //Inicializa flag de resultado final
      YaContoTiempoDefault = 0;
      Serial.println("Debe empezar a contar 1 segundo y alternar led ON / OFF");
    }

  } else {
    // no esta pulsado boton de programacion
    if (HayDeteccionAux == 1) {
      HayDeteccionAux = 0;         //ya no detecta mas
      DebeOscErrStatus = 0;     //anula oscilacion por las dudas que se haya activado x tiempo

      if (DebeContarTiempoDefault == 1) {
        Serial.println("Detecta que solto pulsador AUXILIAR!");
        // ANALIZA CUANTOS SEGUNDOS ESTUVO DETECTANDO PARA DETERMINAR QUE ACCION REALIZAR
        DebeContarTiempoDefault = 0;    //Termina con la cuenta de ACCION de pulsador
        if (YaContol5SegRstSSID == 1 && YaContoTiempoDefault == 0 ) {
          Serial.println("Ya contando mas de 5 segundos -> ");
          YaContol5SegRstSSID == 0; //ACEPTA QUE CONTO MAS DE 5 SEG PERO MENOS DE 10 SEGUNDOS
          //se genera borrado de red por pulsacion constante pero desdpues de dos reset
          Serial.println("Pide nuevos datos de red!");
          WiFiManager wifiManager;
          wifiManager.resetSettings();

          digitalWrite(PinLedVerde, LOW);
          CntTimDefault = 0;
          YaContoTiempoDefault = 0;
          YaContol5SegRstSSID = 0;
          TotalLeds = 4;
          GeneraBeepsLed();
          ESP.reset();       // Reactivar!

        } else if (YaContoTiempoDefault == 1) {
          //aun no conto 10 segundos y siu suelta solo
          YaContoTiempoDefault == 0; //Acepta que conto mas de 6 pulsos
          Serial.println("CARGA DEFAULT!");
          CargaDefault();
          //queda encendido led amarillo hasta que hay algun usuario !!!!!!!!!!!!!!!!!!!!
          CntTimDefault = 0;
          YaContoTiempoDefault = 0;
          YaContol5SegRstSSID = 0;
          TotalLeds = 6;
          GeneraBeepsLed();
        }
        CntTimDefault = 0;
        //YaContoTiempoDefault = 0;
        YaContol5SegRstSSID = 0;
      }
    }
  }
}

void DecodificaIdNombreUsua1() {
  if (pos >= 0)  {
    //para el caso que reciba un texto de actualizacion de  usuario 1 + su identificacion
    Serial.println(variable);
    Serial.println("Hay prog.DE ID REPORTE 1!");
    IdOrigen = NumSerialESP;
    recibido = variable;
    address = E2_IDUsua1;
    resultado = recibido.substring(pos + 5, pos + 15);
    GrabaE2_IDUsuaProgramado();
    BorraE2promNombreUsuario1();
    address = E2_NombreUsua1;

    //_____________
    pos = variable.indexOf("UBICAID1");
    resultado = recibido.substring(pos + 9, pos + 29);
    GrabaE2_UbicacionUsuarios();
    LeeIDUsuario1();
    LeeUbicaUsuario1();
    CmdAccion = "accion-OKtel 1," + IdOrigen + "," + ValIdReporte1 + "," + ValUbicacionID1 + ",";
    //Serial.println(ComandoActualizaValores);
    DebeResponderAOrigen = 1;
  }
}

void DecodificaIdNombreUsua2() {
  if (pos >= 0) {
    Serial.println("Hay prog.DE ID REPORTE 2!");
    address = E2_IDUsua2;
    IdOrigen = NumSerialESP;
    recibido = variable;
    resultado = recibido.substring(pos + 5, pos + 15);
    GrabaE2_IDUsuaProgramado();
    BorraE2promNombreUsuario2();

    pos = variable.indexOf("UBICAID2");
    resultado = recibido.substring(pos + 9, pos + 29);
    GrabaE2_UbicacionUsuarios();
    LeeIDUsuario2();
    LeeUbicaUsuario2();
    ComandoActualizaValores = "accion-OKtel 2," + IdOrigen + "," + ValIdReporte2 + "," + ValUbicacionID2 + ",";
    //Serial.println(ComandoActualizaValores);
    //Serial.println("Actualiza a todos los usuario los datos del usuario 2");
    CmdAccion = ComandoActualizaValores;
    DebeResponderAOrigen = 1;
  }
}

void DecodificaIdNombreUsua3() {
  if (pos >= 0) {
    Serial.println("Hay prog.DE ID REPORTE 3!");
    IdOrigen = NumSerialESP;
    recibido = variable;
    address = E2_IDUsua3;
    resultado = recibido.substring(pos + 5, pos + 15);
    GrabaE2_IDUsuaProgramado();
    BorraE2promNombreUsuario3();
    address = E2_NombreUsua3;
    pos = variable.indexOf("UBICAID3");
    resultado = recibido.substring(pos + 9, pos + 29);
    GrabaE2_UbicacionUsuarios();
    LeeIDUsuario3();
    LeeUbicaUsuario3();
    ComandoActualizaValores = "accion-OKtel 3, " + IdOrigen + "," + ValIdReporte3 + "," + ValUbicacionID3 + ",";
    //Serial.println(ComandoActualizaValores);
    //Serial.println("Actualiza a todos los usuario los datos del usuario 3");
    CmdAccion = ComandoActualizaValores;
    DebeResponderAOrigen = 1;
  }
}


void DecodificaIdNombreUsua4() {
  if (pos >= 0) {
    Serial.print(variable);
    Serial.println("Hay prog.DE ID REPORTE 4!");
    address = E2_IDUsua4;
    IdOrigen = NumSerialESP;
    recibido = variable;
    resultado = recibido.substring(pos + 5, pos + 15);
    GrabaE2_IDUsuaProgramado();
    BorraE2promNombreUsuario4();
    address = E2_NombreUsua4;
    pos = variable.indexOf("UBICAID4");
    resultado = recibido.substring(pos + 9, pos + 29);
    GrabaE2_UbicacionUsuarios();
    LeeIDUsuario4();
    LeeUbicaUsuario4();
    ComandoActualizaValores = "accion-OKtel 4," + IdOrigen + "," + ValIdReporte4 + "," + ValUbicacionID4 + ",";
    Serial.println(ComandoActualizaValores);
    //Serial.println("Actualiza a todos los usuario los datos del usuario 4");
    CmdAccion = ComandoActualizaValores;
    DebeResponderAOrigen = 1;
  }
}

void CuentaMinutosReloj() {
  if (MuestraCuentaMinutos == 1)  {
    MuestraCuentaMinutos = 0;
    LeeArmaTimeDate();
  }
}

void GeneraBeepsLed() {
  for (int i = 0; i < TotalLeds; i++) {
    digitalWrite(PinLedVerde, HIGH);    //LOW);
    delay(90);
    digitalWrite(PinLedVerde, LOW);    //HIGH);
    delay(90);
  }
}


//_________________
//Recibe un texto MMQTT
void callback(char *topic, byte * payload, unsigned int length) {
  String incoming = "";
  for (int i = 0; i < length; i++)  {
    incoming += (char)payload[i];
  }
  incoming.trim();
  DatoRecibido = incoming;
  String str_topic(topic);

  Serial.println(topic);
  Serial.println(NumSerialESP);
  Serial.println(DatoRecibido);
  if (String(topic) == "SOFTMICRO4IO" || String(topic) == DirSubscribe) {
    ValorBuscado = "&to=";
    pos = DatoRecibido.indexOf(ValorBuscado);
    if (pos >= 0) {
      resultado = DatoRecibido.substring(pos + 4, pos + 10);
      Serial.println("modulo buscado");
      Serial.println(resultado);
    }
    //genera 2 leds de recepcion

    //Serial.println(NumSerialESP);
    Serial.println(incoming);
    ValorBuscado = "SHARE";
    pos = incoming.indexOf(ValorBuscado);
    if (pos >= 0) {
      //ES SHARE
      EsModoShare = 1;
    } else {
      EsModoShare = 0;
    }
    if (resultado == NumSerialESP) {
      DatoRecibido = incoming;
      //Serial.println("Se va a procesar este mensaje -> " + DatoRecibido);

    } else {
      if (EsModoShare == 1) {
        DatoRecibido = incoming;
        Serial.println("Se va a procesar mensaje por SHARE -> " + DatoRecibido);
      } else {
        Serial.println("No es modulo solicitado");
        DatoRecibido = "";
      }
    }
  }
}


void LeeArmaTimeDate() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  // Serial.print("Epoch Time: ");
  //Serial.println(epochTime);

  String formattedTime = timeClient.getFormattedTime();
  //Serial.print("Formatted Time: ");
  //Serial.println(formattedTime);

  int currentHour = timeClient.getHours();
  //Serial.print("Hour: ");
  //Serial.println(currentHour);

  int currentMinute = timeClient.getMinutes();
  //Serial.print("Minutes: ");
  //Serial.println(currentMinute);

  int currentSecond = timeClient.getSeconds();
  //Serial.print("Seconds: ");
  //Serial.println(currentSecond);

  //String weekDay = weekDays[timeClient.getDay()];
  //Serial.print("Week Day: ");
  //Serial.println(weekDay);

  //Get a time structure
  struct tm *ptm = gmtime ((time_t *)&epochTime);

  int monthDay = ptm->tm_mday;
  //Serial.print("Month day: ");
  //Serial.println(monthDay);

  int currentMonth = ptm->tm_mon + 1;
  //Serial.print("Month: ");
  //Serial.println(currentMonth);

  //String currentMonthName = months[currentMonth - 1];
  //Serial.print("Month name: ");
  //Serial.println(currentMonthName);

  int currentYear = ptm->tm_year + 1900;
  //Serial.print("Year: ");
  //Serial.println(currentYear);

  //Print complete date:
  String currentDate = String(monthDay) + "/" + String(currentMonth)  + "/" + String(currentYear);
  //Serial.print("Current date: ");
  //Serial.println(currentDate);
  //Serial.println("");
  HoraFechaActual = formattedTime + " - " + currentDate;
  Serial.println(HoraFechaActual);
}
//____________________________________________________________________________________________________________________
void EnviaNotificacionPush(String pTopic) {
  HayUsuario1 = 1;
  if (HayUsuario1 == 0 && HayUsuario2 == 0 && HayUsuario3 == 0 && HayUsuario4 == 0) {
    Serial.println("No va a enviar notificaciones push por no tener usuarios");
  } else {
    Serial.println("[HttpPost]" + pTopic);
    String host  = "fcm.googleapis.com";
    String url   = "/fcm/send";

    // String key   = "AAAAsUTebDk:APA91bG3ZZnWBraBR96d6Q-EELU89jt-rTXyOgjbwqhvYlKn18q4wK-e-cEWTFza4y2lxJXO86ArM_2BeZvYC9eW9Kb1NWI3xRbt_fXBOXRVJVBqWmLIMJwlGoNZGBEMJiVtQUXW0b_Z";
    Serial.println (ValUbicacionMod);
    Serial.println (UsuarioGenerador);
    //Serial.println (GeneradorPublicacion);

    //para Control4io
    //String key="AAAA6KJxKrU:APA91bFXoiefQvr_fbXGQ0Zl-t_VT8pTAKgCpxb7lJLcb61u6UqqO9RYYEvS0TmKnoDXbl0GhDAJFEf_iBidwbpYFdviLYTkD5_yh8eh61FssySOJK0edxGeNb283O0v8uErjO-Ky0bp";
    //para ControlPersonal
    String key = "AAAAtA3ovNE:APA91bH8SKLfgIzslq_AGpT6NP-oJocxxp71LHv0-85y87gl0wpSd2iVfe6Nq5IrcnCXYB4cXTHR8kOiRfc_oCpT937pRK58rCG_KHMmI_gKlaPasJG8UhMoSyIuZYgz7yGhNsVIJGSX";

    //UsuarioGenerador = "Alejandro";
    //GeneradorPublicacion = "6537836612";


    String data  = "";

    //-------- JSON CONSTRUCTION --------------------------------------
    StaticJsonDocument<512> doc;
    JsonObject root = doc.to<JsonObject>();
    root["to"]       = "/topics/" + pTopic;
    root["priority"] = 10;

    String strBody = "";
    strBody = strBody + CmdAccion + "\r\n";
    strBody = strBody + ValUbicacionMod + "\r\n";
    strBody = strBody + HoraFechaActual + "\r\n";
    strBody = strBody + UsuarioGenerador  + "\r\n";
    strBody = strBody + NumSerialESP  + "\r\n";
    strBody = strBody + GeneradorPublicacion  + "\r\n";

    //caca3

    JsonObject notif = root.createNestedObject("data");
    notif["title"]   = "Evento en " + ValUbicacionMod;
    notif["body"]    = strBody;
    serializeJson(root, data);
    Serial.print(data);

    //-----------------------------------------------------------------
    WiFiClientSecure client2;
    client2.setInsecure();
    if (client2.connect(host, 443))  {

      Serial.println("");
      Serial.println("Intenta enviar a cliente2");

      client2.println("POST " + url + " HTTP/1.1");
      client2.println("Host: " + (String)host);
      client2.println("User-Agent: ESP8266/1.0");
      client2.println("Connection: close");
      client2.println("Content-Type: application/json");
      client2.println("Authorization: key=" + key);
      client2.print("Content-Length: ");
      client2.println(data.length());
      client2.println();
      client2.println(data);

      //Serial.println("1.5 segundos de espera de envio ade NP");
      Serial.println(" ");
      delay(1000);

      /*
        String response = client2.readString();
        //Serial.println(response);
        int bodypos = response.indexOf("\r\n\r\n") + 4;
        Serial.println(response.substring(bodypos));        //Genera un 0
        //muestra el id de respuesta del envio
        //{"message_id":151923567798637701}
        //muestra un 0
        //hay 4 avances de lineas automatico
        ¨*/
      HayNPEnviada = 1;
      Serial.println("_________________");
    } else {
      HayNPEnviada = 0;
    }
  }
}
