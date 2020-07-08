
//////////////////////////////////////////////////////////////////////////////////////////
///////                 TFG    Julian Arambarri  Garcia    Julio 2020
/////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "jquery.h"

#include <Ticker.h>
#include <Wire.h> //libreria I2C
#include <math.h>
ESP8266WebServer server(80);

//Ticker visu;
#define TERMINAL_A1 0
#define TERMINAL_A2 1
#define TERMINAL_B1 2
#define TERMINAL_B2 3
#define TERMINAL_C1 4
#define TERMINAL_C2 5

///////////////////////////DAC////////////////////////////7
#define direcDAC 0b1100000 //direccion DAC es programable 96 en decimal
#define comando 0b010      //c2 c1 c0
#define funcion 0b00       //w1 w0
#define trama1DAC (comando << 5) | (funcion << 3)
#define vref 0b1 // seleccion de voltaje de referencia, 1-> 2.048; 0->Vdd
#define pd 0     // esto se puede quitar impedancia de salida
#define gain 0   // ganancia x1, se se pone a 1-> x2
#define trama2DAC (vref << 7) | (pd << 5)

////////////////////////POT//////////////////////////////////////
#define direcPOT 0b0101100

////////////////////DAC////////////////////////////
byte dacSelect[4] = {0b00, 0b01, 0b10, 0b11}; //seleccion de dac
int valor = 0;                                //pq no funciona con un char char ya que son 12 bits
byte iDACselect = 0;

////////////////////POT///////////////////////////

byte channelPOT[4] = {0b00, 0b01, 0b10, 0b11};
byte iPOTselect = 0;
byte datPOTresis = 0;

/////////////////////////////////////////////////
////////////////////////////////////////////////     FUNCIONES
void DACset(byte iDAC, int valorDAC);
float POTvalor(int POTres);
float POTresist(int D);
void DACset(byte iDAC, int valorDAC);
void POTset(byte iPOT, byte datPOT);
float mideTerminal(byte canal);
float measureI(byte canal1, byte canal2, float POTresist);

///////////////detectar
void detectar_terminal();
void detectar_diodo();
void detectar_NPNbjt();
void detectar_PNPbjt();
void detectar_MOSFET();
void detectar_NJFET();
void detectar_PJFET();

void terminals();

void flip();
void ver_datos();

//////////graficas
void graficas();
void diode_graph();
void nmos_graph();
void pmos_graph();
void nbjt_graph();
void pbjt_graph();
void njfet_graph();
void pjfet_graph();
void diode_zener_graph();

//////// internet
void init_internet();

////////server
void init_server();
void ActualizaGrafico();
void HandleRoot();
void HandleNotFound();
void IniciaProceso();
void resetbuffer();

////////////////////ADC/////////////////////////////la medida tarda un timepo de 103us  (106-3)
int analogInPin = A0; // ESP8266 Analog Pin ADC0 = A0

//////////////////////////////////////////////////
float VA1 = 0;
float VA2 = 0;
float VB1 = 0;
float VB2 = 0;
float VC1 = 0;
float VC2 = 0;
float Ibase = 0.0;
float Idrenador = 0.0;
float Iemisor = 0.0;
float Vce = 0;
float Vrc = 0.0;
float average = 0;
int Vg = 0;
float Vd = 0;
float Vds = 0;
float Vgset = 0;
float Vgs = 0;
int media = 0; //media de los valores finales leidos promediar 10
float VceMedia = 0.0;
float IbaseMedia = 0.0;
float IcolectorMedia = 0.0;
float IbSet = 0;
float V = 0;
float Vgate = 0;
float VgsMedia = 0;
float VdsMedia = 0;
float IdrenadorMedia = 0;
float Vdif1;
float Vdif2;
float Vdif3;
float Icolector;

float filtro = 0.09;
float Vdssal = 0;
float Vgssal = 0;
float Idrenadorsal = 0;
long countok = 0;

int Vcomp1 = 0;
int Vcomp2 = 0;
int Vcomp3 = 0;

/////detectar
int i = 0;
int Vb = 2000;
float Ia = 0;
float Ib = 0;
float Ic = 0;

///////////////////////////// asignación de transitor y de terminales
boolean NBJT = false;
boolean PBJT = false;
boolean NMOS = false;
boolean PMOS = false;
boolean NJFET = false;
boolean PJFET = false;
boolean DIODE = false;
boolean DIODE_ZENER = false;

//asignación de terminales
//bipolares
int select_colector;
int colector1;
int colector2;
int select_base;
int base1;
int base2;
int select_emisor;
int emisor1;
int emisor2;
//mosfet_jfet
int select_drain;
int drain1;
int drain2;
int select_gate;
int gate1;
int gate2;
int select_source;
int source1;
int source2;
//diodo

int anodo1 = 0;
int anodo2 = 0;
int select_anodo = 0;
int catodo1 = 0;
int catodo2 = 0;
int select_catodo = 0;

int ndetecciones = 0;
//BJT
int base = 0;
int colector = 0;
int emisor = 0;
//MOS y JFET
int gate = 0;
int drain = 0;
int source = 0;
//diodo
int anodo = 0;
int catodo = 0;

/////////////////////////// graficas
//general
int Vbas = 0;
//mostrar graph
boolean flag = false;
boolean charge = true;
int chargeI = 0;
boolean endGraph = false;
String carga = "";
String carga2 = "";
int numero = 0;
String label = "";
String label_value = "";

//mosfet
float Vgs_set = 0;

//jfet
int Vsource_min = 0;

////////////ver datos
float Icolectorsal;
float Vcesal;
float Ibasesal;
float valorx[400];
float valory[400];

float x = 0;
float y = 0;
float z = 0;
float x_sal = 0;
float y_sal = 0;
float z_sal = 0;
String estado[2] = {"MIDIENDO", "READY"};
bool ready = true;

//grafica
String imagen = "";
float x_min = 0;
float x_max = 0;
float y_min = 0;
float y_max = 0;
String position = "";
String deteccion = "";
String gain_trans = "";

boolean start = false;
/// ganancia hfe y transconductancia

float hfe; ///  ganancia y tranconductancia
float gm;
float vgs_anterior;
float id_anterior;
float vgs_final;
float id_final;

float ib_final;
float ic_final;


//////////////////////////////////////////////////////////////////////////////////////////
///////                 Setup  inicializamos
/////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  delay(10);

  init_internet();
  init_server();

  Wire.begin(2, 14); //Wire.begin(int sda, int scl)450KHz

  // flip the pin every 0.3s

  //
  //visu.attach(0.1, server.handleClient() );
  pinMode(5, OUTPUT);  //muxA
  pinMode(4, OUTPUT);  //muxB
  pinMode(12, OUTPUT); //muxc
  pinMode(16, OUTPUT); //leds
  //pinMode(15, INPUT);  //button
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                 loop  lazo basico
/////////////////////////////////////////////////////////////////////////////////////////

void loop()
{

  digitalWrite(16, HIGH);
  if (digitalRead(15) == HIGH)
  {

    while (digitalRead(15) == HIGH)
    {
      server.handleClient();
      delay(100);
    }
    ready = false;
    server.handleClient();

    digitalWrite(16, LOW);
    detectar_terminal();
    graficas();
    ready = true;
    start = false;
  }

  if (start)
  {

    ready = false;
    server.handleClient();

    digitalWrite(16, LOW);
    detectar_terminal();
    graficas();
    ready = true;
    start = false;
  }
  server.handleClient();
}

//////////////////////////////////////////////////////////////////////////////////////////
///////               Init internet  inicializamos internet
/////////////////////////////////////////////////////////////////////////////////////////
void init_internet()
{

  WiFi.mode(WIFI_STA);
  WiFi.begin("vodafone0DC8", "HAMVF92X8V2FY8");
  Serial.println("Connecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  // Mostrar mensaje de exito y dirección IP asignada
  Serial.println();
  Serial.print("Conectado a:\t");
  Serial.println(WiFi.SSID());
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////               Init server  inicializamos server
/////////////////////////////////////////////////////////////////////////////////////////

void init_server()
{

  server.on("/", HandleRoot);
  server.on("/U", ActualizaGrafico);
  server.on("/BOTON", IniciaProceso);
  server.on("/JQ.js", []() {
    server.send(200, "text/javascript", JQ, sizeof(JQ) - 1);
  });
  // Ruteo para URI desconocida
  server.onNotFound(HandleNotFound);
  // Iniciar servidor

  server.begin();
  Serial.println("HTTP server started");

  return;
}
//////////////////////////////////////////////////////////////////////////////////////////
///////               Handle Root
/////////////////////////////////////////////////////////////////////////////////////////
void HandleRoot()
{
  Serial.println("staaaaaaar1");
  String cuerpo = "<!DOCTYPE html><html>";
  cuerpo += "<head><meta charset='UTF-8'>";
  cuerpo += "</head>";
  cuerpo += "<body onload=\"update();setInterval('update()',1000);\">";
  cuerpo += "<h1 style='background-color:black;width:100%; text-align: center; color:white;'>CARACTERIZACIÓN DE TRANSISTORES Y DIODOS</h1><br/>";
  cuerpo += "<h3 style='text-align: center;'>He detectado un:  <div style='display:inline-block' id='frase1'></div><br/><div style='display:inline-block' id='frase2'></div></h3>";
  cuerpo += "<div style='text-align:center'>";
  cuerpo += "<div id='imagenes' style='height:150px;'>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-sF0u8XxSrTo/Xu3wj3D-_rI/AAAAAAAAADU/KL6KoGwQeRwqKB42ZyE6y-MF8BMtCA04ACK4BGAsYHg/s232/diodo.png' id='img1'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-iYOpNYfqt7U/Xu3wsqRR-UI/AAAAAAAAADg/O0QkGmY1VmAycX28PrP1RbUTXrKdlnGugCK4BGAsYHg/s389/DIODO2.png' id='img2'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-sfy6Vabf-Xw/Xu3xAf411uI/AAAAAAAAAEM/tW2b-YtaeJsTNz5Wxizjgf6tO4m8MmgfgCK4BGAsYHg/N-MOS.png' id='img3'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-ZYFeMBGNsOs/Xu3xH8fTVHI/AAAAAAAAAEg/0KAHOVlNTGIKQxlATFNMiHLZSyOXPxKDgCK4BGAsYHg/P-MOS.png' id='img4'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-HCpfOVwHYRM/Xu3wTwsgtAI/AAAAAAAAAC4/cxCj9JH726EtShZ-IM_98UnzzyDb9xkvwCK4BGAsYHg/s150/BJT-NPN%2B%25281%2529.png' id='img5'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-p9dVy_BRvs0/Xu3weBZ-rbI/AAAAAAAAADE/gUg2c3qUE_QTq5HFEDJopXLbDfbYih6ZACK4BGAsYHg/s150/BJT-PNP%2B%25281%2529.png' id='img6'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-IvWLiGACRXk/Xu3wzp1tPCI/AAAAAAAAADw/K6__g4loSloIC1fAf49PEx9Q0elBqw_FgCK4BGAsYHg/JFET-N.png' id='img7'/>";
  cuerpo += "<img style='display:none;' src='https://1.bp.blogspot.com/-ZacqvrrTumM/Xu3w6WE0o9I/AAAAAAAAAD8/khyqY7JteNkjPKOJHaBgRikQ05W2yFpVACK4BGAsYHg/JFET-P.png' id='img8'/>";
  cuerpo += "</div>";
  cuerpo += "<div style='padding:10px; font-size:35px;display:inline-block;' id='estado'></div>&nbsp;&nbsp;&nbsp;  ";
  cuerpo += "<button style=' font-size:40px; display:inline-block;' onclick='Leedatos();'>Leer datos</button></h2>";
  cuerpo += "</div>";
  cuerpo += "<div  style=\" margin: auto; width:745px;\"><div style='text-align:left;'>Intensidad (mA)</div>";
  cuerpo += "<div  style=\" margin: auto; width:700px;height:500px;\" id=\"flotcontainer\"></div>";
  cuerpo += "<div  style=\" margin: auto; width:700px;\"><div style='text-align:center;'>Voltaje (V)</div>";
  cuerpo += "<script src='/JQ.js'>;</script>";
  cuerpo += "<script>";
  cuerpo += "function Leedatos(){var xhttp;xhttp=new XMLHttpRequest();//creo peticion xhttp\n\
  xhttp.open('GET', '/BOTON', true);//genero http rewquest\n\
		xhttp.send();//envio http request\n\
    }";
  cuerpo += "function update(){\n\
	//console.log(\"update\");\n\
	var xhttp;xhttp=new XMLHttpRequest();//creo peticion xhttp\n\
	xhttp.onreadystatechange=function(){//evento cuando recibo la respuesta del servidor\n\
		if (xhttp.readyState == 4 && xhttp.status == 200)//si la peticion es correcta sigo\n\
		{\n\
			console.log(xhttp.responseText);//lo que recibo es un string, y lo imprimo por la consola del navegador para debuggear\n\
			var json = JSON.parse(xhttp.responseText);//convierto el sting a json\n\
            for(i in json.frases){\n\
        if(document.getElementById(i)){\n\
        if(i=='estado'){if(json.frases[i]=='READY'){document.getElementById(i).style.backgroundColor='green';}else{document.getElementById(i).style.backgroundColor='blue';}}\n\
        document.getElementById(i).innerHTML=json.frases[i]\n\
        }\n\
      }\n\
      if(json.graficos){\n\
      var nombreimagenes=['img1','img2','img3','img4','img5','img6','img7','img8'];\n\
      for(i in nombreimagenes){\n\
      if(json.imagen==nombreimagenes[i]){document.getElementById(nombreimagenes[i]).style.display='inline-block';}\n\
      else{document.getElementById(nombreimagenes[i]).style.display='none';}\n\
      }\n\
      console.log(json);//madar el json por la consola\n\
        console.log('actualizo grafica');\n\
        plot.getOptions().xaxes[0].min=json.opciones.xaxis.min;\n\
        plot.getOptions().xaxes[0].max=json.opciones.xaxis.max;\n\
        plot.getOptions().yaxes[0].min=json.opciones.yaxis.min;\n\
        plot.getOptions().yaxes[0].max=json.opciones.yaxis.max;\n\
        plot.getOptions().legend.position=json.opciones.legend.position;\n\
        plot.setData(json.graficos);//metes los nuevos datos\n\
        plot.resize();\n\
        plot.setupGrid(plot);\n\
        plot.draw(plot);// refrescar los cambio\n\
      }\n\
		};\n\
	};\n\
		xhttp.open('GET', '/U', true);//genero http rewquest\n\
		xhttp.send();}//envio http request";
  cuerpo += "</script>";
  cuerpo += "<script>";
  cuerpo += "var placeholder = $(\"#flotcontainer\"); //quedate con este id para luego trabajar\n\
  	    var plot;\n\
	      var plotop={xaxis: {min: 0, max:10},yaxis: {min: 0, max: 10},legend:{position:'se'}};";
  cuerpo += "$(document).ready(function(){	\n\
    var d2=[[1,1],[10,10]];\n\
    //d2=[{label:\"Foo\"},{label:\"PePe\"},{label:\"juan\"},{label:\"mikel\"}];\n\
    plot=$.plot(placeholder,d2,plotop);\n\
		});\n";
  cuerpo += "</script>";
  cuerpo += "</body>";
  cuerpo += "</html>";
  ;
  server.send(200, "text/html", cuerpo);
}

/*
    d2=[{label:\"Foo\",data:[[0, 0],[3, 3],[5, 5]]},{label:\"PePe\",data:[[1,9],[3, 3],[9, 2]]}];\n\

var plot = $.plot(placeholder, data, options) 
placeholde es un objeto jquery sobre el que se va a plotear anchura altura
data [ [x1, y1], [x2, y2], ... ]  a null value for lines is interpreted as a line segment end, i.e. the points before and after the null value are not connected.
label para leyendas
{
    label: "y = 3",
    data: [[0, 3], [10, 3]]
}
[ { label: "Foo", data: [ [10, 1], [17, -14], [30, 5] ] },
  { label: "Bar", data: [ [11, 13], [19, 11], [30, -7] ] }
]

// A null signifies separate line segments

		var d3 = [[0, 12], [7, 12], null, [7, 2.5], [12, 2.5]];

*/

/*


      for (i in document.getElementById('imagenes').children){//recorrer todas las imagenes\n\
        if(json.imagen==document.getElementById('imagenes').children[i].id){//si la imagen coincide con la que mando\n\
        document.getElementById('imagenes').children[i].style.display='inline-block';\n\
        }\n\
        else if(document.getElementById('imagenes').children[i].id=='undefined'){}\n\
        else{\n\
        document.getElementById('imagenes').children[i].style.display='none';\n\
        }\n\


        */

//////////////////////////////////////////////////////////////////////////////////////////
///////            Actualiza grafico  manda los valores a la pagina web
/////////////////////////////////////////////////////////////////////////////////////////

void ActualizaGrafico()
{
  Serial.println("staaaaaaar2");

  if (flag == true)
  {

    if (numero == 1)
    {
      // carga += "{\"imagen\": \"img1\",\"opciones\": {\"xaxis\": {\"min\": 0, \"max\":10},\"yaxis\": {\"min\": 0, \"max\": 7},\"legend\":{\"position\":\"se\"}},\"frases\": {\"frase1\":\"hola como estas\", \"frase2\":\"la ganancia es\",\"estado\":\"" + estado[ready] + "\"},\"graficos\":[";

      // si es la primera grafica que muestro
      carga += "{\"imagen\": \"";
      carga += imagen;
      carga += "\",\"opciones\": {\"xaxis\": {\"min\": ";
      //minX
      carga += x_min;
      carga += ", \"max\":";
      //maxX
      carga += x_max;
      carga += "},\"yaxis\": {\"min\": ";
      //minY
      carga += y_min;
      carga += ", \"max\": ";
      //maxY
      carga += y_max;
      carga += "},\"legend\":{\"position\":\"";
      //posición labels
      carga += position;
      carga += "\"}},\"frases\": {\"frase1\":\"";
      carga += deteccion;
      carga += "\", \"frase2\":\"";
      carga += gain_trans;
      carga += "\",\"estado\":\"" + estado[ready] + "\"},\"graficos\":[";
      //Serial.print("carga 1: ");
      //Serial.println(carga);
    }
    else
    {

      carga.remove(carga.length() - 2); // para que elimine la ultima "]]"
      carga += ',';
      Serial.println(numero);
    }
    carga += "{\"label\": ";
    //  carga += "\"pepe\"";
    carga += "\"";
    carga += label;
    carga += label_value;
    carga += "\"";

    carga += ", \"data\": [";

    for (uint16_t i = 0; i < countok; i++) // que es esto de uint16_t
    {

      carga += '[';
      carga += valorx[i];
      carga += ',';
      carga += valory[i];
      carga += "],";
      yield();
    }

    carga.remove(carga.length() - 1);
    carga += "]}]}";
    resetbuffer();
    Serial.println("#############################");
    Serial.println(numero);
    Serial.println("#############################");

    server.send(200, "application/json", carga);
    //Serial.println(carga);
    delay(200);
    flag = false;
  }
  else
  {
    // Serial.println("flga falso");
    // carga += "[[]]";
    Serial.print("carga vacia");
    //Serial.println(carga);

    server.send(200, "application/json", "{\"frases\": {\"estado\":\"" + estado[ready] + "\",\"frase2\": \""+gain_trans+"\"},\"imagen\": \"img1\"}");
    delay(200);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                      Inicia proceso  de medida
/////////////////////////////////////////////////////////////////////////////////////////
void IniciaProceso()
{
  if (ready)
  {
    start = true;
    server.send(200, "text/plain", "ok");
  }
  else
  {
    server.send(200, "text/plain", "nok");
  }
}

void resetbuffer()

{

  for (uint16_t i = 0; i < 292; i++)

  {
    valorx[i] = 0;

    valory[i] = 0;
  }
  countok = 0;
}

void HandleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                      Graficas
/////////////////////////////////////////////////////////////////////////////////////////
void graficas()
{

  if (ndetecciones == 1)
  { // compruebo que he detectado que tipo de transitor y que no lo he confundido entre varios tipos

    if (DIODE)
      diode_graph();
    if (DIODE_ZENER)
      diode_zener_graph();
    if (NMOS)
      nmos_graph();
    if (PMOS)
      pmos_graph();
    if (NBJT)
      nbjt_graph();
    if (PBJT)
      pbjt_graph();
    if (NJFET)
      njfet_graph();
    if (PJFET)
      pjfet_graph();

    //poner a false y los puntos a 0
    ndetecciones = 0;
    NBJT = false;
    PBJT = false;
    NMOS = false;
    PMOS = false;
    DIODE = false;
    DIODE_ZENER = false;
    NJFET = false;
    PJFET = false;
  }
  else
  {
    ndetecciones = 0;
    Serial.println("no es posible detectar de que transistor se trata");
  }
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                      Terminales
/////////////////////////////////////////////////////////////////////////////////////////

void terminals()
{
  if (DIODE | DIODE_ZENER)
  {

    if (anodo == 1)
    {
      anodo1 = 0; //terminal A1
      anodo2 = 1; // terminal A2
      select_anodo = 0;
    }
    if (anodo == 2)
    {
      anodo1 = 2; //terminal B1
      anodo2 = 3; //terminal B2
      select_anodo = 1;
    }
    if (anodo == 3)
    {
      anodo1 = 4; //terminal C1
      anodo2 = 5; //terminal C2
      select_anodo = 2;
    }
    if (catodo == 1)
    {
      catodo1 = 0; //terminal A1
      catodo2 = 1; // terminal A2
      select_catodo = 0;
    }
    if (catodo == 2)
    {
      catodo1 = 2; //terminal B1
      catodo2 = 3; //terminal B2
      select_catodo = 1;
    }
    if (catodo == 3)
    {
      catodo1 = 4; //terminal C1
      catodo2 = 5; //terminal C2
      select_catodo = 2;
    }
  }

  if (NMOS || PMOS || NJFET || PJFET)
  {

    if (drain == 1)
    {
      drain1 = 0; //terminal A1
      drain2 = 1; // terminal A2
      select_drain = 0;
    }
    if (drain == 2)
    {
      drain1 = 2; //terminal B1
      drain2 = 3; //terminal B2
      select_drain = 1;
    }
    if (drain == 3)
    {
      drain1 = 4; //terminal C1
      drain2 = 5; //terminal C2
      select_drain = 2;
    }
    if (gate == 1)
    {
      gate1 = 0;
      gate2 = 1;
      select_gate = 0;
    }
    if (gate == 2)
    {
      gate1 = 2;
      gate2 = 3;
      select_gate = 1;
    }
    if (gate == 3)
    {
      gate1 = 4;
      gate2 = 5;
      select_gate = 2;
    }
    server.handleClient();
    if (source == 1)
    {
      source1 = 0;
      source2 = 1;
      select_source = 0;
    }
    if (source == 2)
    {
      source1 = 2;
      source2 = 3;
      select_source = 1;
    }
    if (source == 3)
    {
      source1 = 4;
      source2 = 5;
      select_source = 2;
    }
  }

  if (NBJT || PBJT)
  {
    if (colector == 1)
    {
      colector1 = 0; //terminal A1
      colector2 = 1; // terminal A2
      select_colector = 0;
    }
    if (colector == 2)
    {
      colector1 = 2; //terminal B1
      colector2 = 3; //terminal B2
      select_colector = 1;
    }
    if (colector == 3)
    {
      colector1 = 4; //terminal C1
      colector2 = 5; //terminal C2
      select_colector = 2;
    }
    if (base == 1)
    {
      base1 = 0;
      base2 = 1;
      select_base = 0;
    }
    if (base == 2)
    {
      base1 = 2;
      base2 = 3;
      select_base = 1;
    }
    if (base == 3)
    {
      base1 = 4;
      base2 = 5;
      select_base = 2;
    }
    server.handleClient();
    if (emisor == 1)
    {
      emisor1 = 0;
      emisor2 = 1;
      select_emisor = 0;
    }
    if (emisor == 2)
    {
      emisor1 = 2;
      emisor2 = 3;
      select_emisor = 1;
    }
    if (emisor == 3)
    {
      emisor1 = 4;
      emisor2 = 5;
      select_emisor = 2;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    diodo zener
/////////////////////////////////////////////////////////////////////////////////////////

void diode_zener_graph()
{
  imagen = "img2";
  x_min = -5.5;
  x_max = 1;
  y_min = -3;
  y_max = 3.5;
  position = "ne";
  gain_trans = ""; 

  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.05;
  label = "Diodo Zener";

  terminals();

  Serial.print(select_anodo);
  Serial.print(" ");
  Serial.print(" ");
  Serial.println(select_catodo);

  Serial.print(anodo1);
  Serial.print(" ");
  Serial.print(anodo2);
  Serial.print(" ");
  Serial.print(catodo1);
  Serial.print(" ");
  Serial.println(catodo2);

  POTset(select_anodo, 255);  //946 medidos anodo
  POTset(select_catodo, 255); //948 medidos catodo

  DACset(select_anodo, 0);  //anodo
  DACset(select_catodo, 0); // catodo

  charge = true;
  countok = 0;
  ++numero;
  server.handleClient();

  for (V = 4000; V > 0; V -= 23) // aplico una tensión negativa al diodo 900 para los diodos normales y 3700 para los zener
  {
    DACset(select_catodo, V);
    //catodo
    x = mideTerminal(anodo2) - mideTerminal(catodo2); // Vd tensión anodo catodo
    y = measureI(anodo1, anodo2, 946);                //Idiodo corriente del diodo entre la resistencia que esta el catodo

    //server.handleClient();

    ver_datos();
    server.handleClient();

    //meter los handel client
  }
  for (V = 0; V < 3000; V += 23) //aplico una tensión positva al diodo 1300 , 1000 para los schottky
  {
    DACset(select_anodo, V); //anodo

    x = mideTerminal(anodo2) - mideTerminal(catodo2); // tensión anodo catodo
    y = measureI(anodo1, anodo2, 946);                //corriente del diodo entre la resistencia que esta el catodo

    ver_datos();
    //server.handleClient();
    yield();
  }

  label_value = "";

  flag = true;

  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }

  Serial.println("");
  Serial.println("/////////////////////////////////////////////////////////// ");
  Serial.println("");
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    diodo normal
/////////////////////////////////////////////////////////////////////////////////////////

void diode_graph()
{
  imagen = "img1";
  x_min = -2.4;
  x_max = 1;
  y_min = -0.5;
  y_max = 4;
  position = "ne";
  gain_trans = ""; 

  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.05;
  label = "Diodo";

  terminals();
  server.handleClient();

  Serial.print(select_anodo);
  Serial.print(" ");
  Serial.print(" ");
  Serial.println(select_catodo);

  Serial.print(anodo1);
  Serial.print(" ");
  Serial.print(anodo2);
  Serial.print(" ");
  Serial.print(catodo1);
  Serial.print(" ");
  Serial.println(catodo2);

  POTset(select_anodo, 255);  //946 medidos anodo
  POTset(select_catodo, 255); //948 medidos catodo

  DACset(select_anodo, 0);  //anodo
  DACset(select_catodo, 0); // catodo

  charge = true;
  countok = 0;
  ++numero;

  for (V = 900; V > 0; V -= 14) // aplico una tensión negativa al diodo 900 para los diodos normales y 3700 para los zener
  {
    DACset(select_catodo, V);
    //catodo
    x = mideTerminal(anodo2) - mideTerminal(catodo2); // Vd tensión anodo catodo
    y = measureI(anodo1, anodo2, 946);                //Idiodo corriente del diodo entre la resistencia que esta el catodo

    //server.handleClient();

    ver_datos();
    server.handleClient();

    //meter los handel client
  }
  for (V = 0; V < 3300; V += 14) //aplico una tensión positva al diodo 1300 , 1000 para los schottky
  {
    DACset(select_anodo, V); //anodo

    x = mideTerminal(anodo2) - mideTerminal(catodo2); // tensión anodo catodo
    y = measureI(anodo1, anodo2, 946);                //corriente del diodo entre la resistencia que esta el catodo

    ver_datos();
    //server.handleClient();
    yield();
  }

  label_value = "";

  flag = true;

  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }

  Serial.println("");
  Serial.println("/////////////////////////////////////////////////////////// ");
  Serial.println("");
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Pjfet graph
/////////////////////////////////////////////////////////////////////////////////////////

void pjfet_graph()
{
  imagen = "img8";
  x_min = -8;
  x_max = 0;
  y_min = -3;
  y_max = 0;
  position = "se";
  
  server.handleClient();

  Vsource_min = 4000;
  Vg = 4000;
  filtro = 0.08;
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  label = "Vgs: ";

  terminals();

  POTset(select_drain, 255);  //946 medidos
  POTset(select_gate, 255);   //948 medidos
  POTset(select_source, 255); //948 medidos

  Vsource_min = 4000;
  Vg = 4000;
  filtro = 0.03;

  for (Vgs_set = 0.6; Vgs_set >=0; Vgs_set -= 0.1)
  {
    vgs_anterior = z_sal;
    id_anterior = y_sal;

    DACset(select_drain, 4000);  //drain
    DACset(select_gate, 4000);   //gate
    DACset(select_source, 4000); //source

    charge = true;
    countok = 0;
    ++numero;

    for (V = 4000; V >= 0; V -= 20)
    {
      DACset(select_drain, V);
      if (countok == 0)
      {

        Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente

        while (Vgs > Vgs_set + 0.01 || Vgs < Vgs_set - 0.01)
        { //la primera vez fijo con Vs para obtener la Vgs que quiero ya que Vg=0

          if (Vgs > Vgs_set + 0.01)
            Vsource_min += 5;
          if (Vgs < Vgs_set - 0.01)
            Vsource_min -= 3;

          DACset(select_source, Vsource_min); //fuente
          V = Vsource_min;
          DACset(select_colector, V); // drenador
          yield();

          Vgs = mideTerminal(gate2) - mideTerminal(source2); //vgs
          yield();
        }
      }
      else
      {
        Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente
        while (Vgs > Vgs_set + 0.05 || Vgs < Vgs_set - 0.05)
        {

          yield();
          if (Vgs > Vgs_set + 0.05)
            Vg -= 3;
          if (Vgs < Vgs_set - 0.05)
            Vg += 5;
          DACset(select_gate, Vg);                           //aplico tension a la puerta
          Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente
          yield();
        }
      }

      yield();

      x = mideTerminal(drain2) - mideTerminal(source2); //calculo Vce
      y = measureI(drain1, drain2, 946);
      z = mideTerminal(gate2) - mideTerminal(source2);

      ver_datos();
      server.handleClient();
      yield();
    }
    label_value = Vgs_set;
    label_value += "V";

    flag = true;
gain_trans = "la transconductancia es: ?";
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }

    Serial.println("");
    Serial.println("/////////////////////////////////////////////////////////// ");
    Serial.println("");
  }

  vgs_final = z_sal;
  id_final = y_sal;

  gm = (fabs(id_final - id_anterior) / fabs(vgs_final - vgs_anterior));
  Serial.print("gm: ");
  Serial.println(gm);
  gain_trans = "la transconductancia es: "+(String)gm; 
  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Njfet graph
/////////////////////////////////////////////////////////////////////////////////////////

void njfet_graph()
{
  
  imagen = "img7";
  x_min = 0;
  x_max = 8;
  y_min = 0;
  y_max = 4;
  position = "ne";
  

  Vsource_min = 0;
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.08;
  label = "Vgs: ";

  terminals();

  POTset(select_gate, 255);   //946 medidos gate
  POTset(select_drain, 255);  //948 medidos drain
  POTset(select_source, 255); //948 medidos source

  for (Vgs_set = 0; Vgs_set > (-2.1); Vgs_set -= 0.4)
  {

    vgs_anterior = z_sal;
    id_anterior = y_sal;

    DACset(select_gate, 0);   //gate
    DACset(select_drain, 0);  //drain
    DACset(select_source, 0); //source

    charge = true;
    countok = 0;
    ++numero;

    for (V = 0; V < 4095; V += 14) // fijo una Vmin porque la tensión del drenador no puede ser mas bajo que el de la fuente
    {
      DACset(select_drain, V); //drain
      if (countok == 0)
      {
        Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente
        while (Vgs > Vgs_set + 0.05 || Vgs < Vgs_set - 0.05)
        { //la primera vez fijo con Vs para obtener la Vgs que quiero ya que Vg=0

          if (Vgs > Vgs_set + 0.05)
            Vsource_min += 5;
          if (Vgs < Vgs_set - 0.05)
            Vsource_min -= 3;

          DACset(select_source, Vsource_min); //fuente
          V = Vsource_min;
          DACset(select_drain, V); // drenador    igualo el valor del drenador para no tener Vds negativos }
          yield();

          Vgs = mideTerminal(gate2) - mideTerminal(source2); //vgs
        }
      }
      else /// el resto de veces como Vdrain aumenta Vs auementa y por tanto Vgs aumenta, voy manteniendo cte Vgs aumentado Vg
      {
        Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente
        while (Vgs >= Vgs_set + 0.05 || Vgs <= Vgs_set - 0.05)
        {
          yield();
          if (Vgs > Vgs_set + 0.05)
            Vg -= 3;
          if (Vgs < Vgs_set - 0.05)
            Vg += 5;
          DACset(select_gate, Vg);                           //aplico tension a la puerta
          Vgs = mideTerminal(gate2) - mideTerminal(source2); //Vgs entre la gate y la fuente
        }
      }

      x = mideTerminal(drain2) - mideTerminal(source2); //calculo Vds Vdrain-Vsource
      y = measureI(drain1, drain2, 946);                // caludo Idrenador
      z = mideTerminal(gate2) - mideTerminal(source2);  //calculo Vgs Vgate-Vsource

      ver_datos();
      server.handleClient();
      yield();
    }

    label_value = Vgs_set;
    label_value += "V";

    flag = true;
gain_trans = "la transconductancia es: ?";
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }

    Serial.println("");
    Serial.println("/////////////////////////////////////////////////////////// ");
    Serial.println("");
  }

  vgs_final = z_sal;
  id_final = y_sal;

  gm = (fabs(id_final - id_anterior) / fabs(vgs_final - vgs_anterior));
  Serial.print("gm: ");
  Serial.println(gm);
gain_trans = "la transconductancia es: "+(String)gm; 
  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Pmos graph
/////////////////////////////////////////////////////////////////////////////////////////

void pmos_graph()
{
  imagen = "img4";
  x_min = -9.5;
  x_max = 0;
  y_min = -7;
  y_max = 0;
  position = "se";
  

  terminals();

  POTset(select_drain, 255);  //948 medidos drain
  POTset(select_gate, 255);   //948 medidos gate
  POTset(select_source, 255); //948 medidos source

  Vg = 1366;  // valor de comienzo de Vgate para que vaya mas rápido
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.1;
  label = "Vgs: ";

  for (Vgs_set = -2.25; Vgs_set >= -2.55; Vgs_set -= 0.05)
  {

    vgs_anterior = z_sal;
    id_anterior = y_sal;

    DACset(select_drain, 0);     ///  el drenador a cero
    DACset(select_gate, 0);      // gate a 0
    DACset(select_source, 4000); // drenador a maxima tensión

    charge = true;
    countok = 0;
    ++numero;

    for (Vd = 4000; Vd >= 0; Vd -= 20)
    {

      DACset(select_drain, Vd); /// voy disminuyendo la tension del drenador para que auemente Vds

      Vgs = mideTerminal(gate2) - mideTerminal(source2); //calculo Vgs

      while (Vgs >= Vgs_set + 0.01 || Vgs <= Vgs_set - 0.01)
      {

        if (Vgs > (Vgs_set + 0.01))
          Vg -= 5;
        if (Vgs < (Vgs_set - 0.01))
          Vg += 3;

        DACset(select_gate, Vg);                           // vario la tension de la puerta para obetener la Vgs deseada y mantenerla
        Vgs = mideTerminal(gate2) - mideTerminal(source2); //calculo Vgs
        yield();
      }

      x = (mideTerminal(drain2) - mideTerminal(source2)); // Vds drain - Vsource
      y = measureI(drain1, drain2, 948);                  ////entre los dos terminales de la resistencia del drenador
      z = mideTerminal(gate2) - mideTerminal(source2);    //Vgs Vgate-Vsource

      ver_datos();
      server.handleClient();
      yield();
    }

    label_value = Vgs_set;
    label_value += "V";

    flag = true;
gain_trans = "la transconductancia es: ?"; 
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }

    Serial.println("");
    Serial.println("/////////////////////////////////////////////////////////// ");
    Serial.println("");
  }

  vgs_final = z_sal;
  id_final = y_sal;

  gm = (fabs(id_final - id_anterior) / fabs(vgs_final - vgs_anterior));
  Serial.print("gm: ");
  Serial.println(gm);
gain_trans = "la transconductancia es: "+(String)gm; 
  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Nmos graph
/////////////////////////////////////////////////////////////////////////////////////////

void nmos_graph()
{

  imagen = "img3";
  x_min = 0;
  x_max = 8;
  y_min = 0;
  y_max = 5;
  position = "ne";
  
  terminals();
  Serial.print(select_drain);
  Serial.print(" ");
  Serial.print(select_gate);
  Serial.print(" ");
  Serial.println(select_source);

  POTset(select_drain, 255);  //10 medidos drain
  POTset(select_gate, 255);   //100k medidos gate
  POTset(select_source, 255); //948 medidos source

  Vg = 1366;  // valor de comienzo de Vgate para que vaya mas rápido
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.03;
  label = "Vgs: ";

  ///////////////////////////////////////////7debug

  for (Vgs_set = 1.95; Vgs_set <= 2.15; Vgs_set += 0.05)
  {

    vgs_anterior = z_sal;
    id_anterior = y_sal;

    DACset(select_drain, 0);  //drain
    DACset(select_gate, 0);   //gate
    DACset(select_source, 0); //source

    charge = true;
    countok = 0;
    ++numero;

    for (Vd = 0; Vd <= 4000; Vd += 18)
    {

      DACset(select_drain, Vd); //drain

      Vgs = mideTerminal(gate2) - mideTerminal(source2); //calculo Vgs

      while (Vgs >= Vgs_set + 0.01 || Vgs <= Vgs_set - 0.01)
      {

        yield();
        if (Vgs > (Vgs_set + 0.01))
          Vg -= 3;
        if (Vgs < (Vgs_set - 0.01))
          Vg += 5;
        DACset(select_gate, Vg);
        Vgs = mideTerminal(gate2) - mideTerminal(source2); //calculo Vgs
      }

      x = mideTerminal(drain2) - mideTerminal(source2); // Vds drenador-fuente
      y = measureI(drain1, drain2, 948);                ////Idrenador entre los dos terminales de la resistencia del drenador
      z = mideTerminal(gate2) - mideTerminal(source2);  // Vgs Vgate -Vsource

      ver_datos();
      server.handleClient();
      yield();
    }

    label_value = Vgs_set;
    label_value += "V";

    flag = true;
gain_trans = "la transconductancia es: ?"; 
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }

    Serial.println("");
    Serial.println("/////////////////////////////////////////////////////////// ");
    Serial.println("");
  }

  vgs_final = z_sal;
  id_final = y_sal;

  gm = (fabs(id_final - id_anterior) / fabs(vgs_final - vgs_anterior));
  Serial.print("gm: ");
  Serial.println(gm);
gain_trans = "la transconductancia es: "+(String)gm; 
  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Pbjt graph
/////////////////////////////////////////////////////////////////////////////////////////

void pbjt_graph()
{
  imagen = "img6";
  x_min = -9;
  x_max = 0;
  y_min = -7;
  y_max = 0;
  position = "se";
  
  //borrar esto
  Serial.println("he llegado");
  Serial.println(colector);
  Serial.println(base);
  Serial.println(emisor);

  terminals();

  Vb = 1366;  // valor de comienzo de Vbase para que vaya mas rápido
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.09;
  label = "Ibase: ";

  POTset(select_colector, 255); //946 medidos colector
  POTset(select_base, 127);     //100k medidos base
  POTset(select_emisor, 255);   //948 medidos emisor

  for (IbSet = -2.0; IbSet > -23.0; IbSet -= 4.0) //IbSet = -2.0; IbSet > -23.0; IbSet -= 4.0
  {

    DACset(select_colector, 0);  //colector
    DACset(select_base, Vb);     //3,3vbase
    DACset(select_emisor, 4095); // emisor

    ++numero;
    charge = true;
    countok = 0;

    for (V = 4095; V >= 0; V -= 18)
    {

      DACset(select_colector, V); // colector

      Ibase = measureI(base1, base2, 100); //corriente de la base

      while (Ibase >= IbSet + 0.1 || Ibase <= IbSet - 0.1)
      {

        yield();

        if (Ibase > IbSet + 0.1)
          Vb -= 10;
        if (Ibase < IbSet - 0.1)
          Vb += 7;

        DACset(select_base, Vb); // base
        server.handleClient();
        Ibase = measureI(base1, base2, 100);
      }

      x = (mideTerminal(colector2) - mideTerminal(emisor2)); // calculo Vce Vcolector - Vemisor
      y = measureI(colector1, colector2, 946);               // calculo la corriente de drenador
      z = measureI(base1, base2, 100);                       // calculo la corriente de base

      ver_datos();
      server.handleClient();
      yield();
    }
    label_value = IbSet;
    label_value += "µA";
    flag = true;
    gain_trans = "la ganancia es: ?"; 
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }
    Serial.println("/////////////////////////////////////////////////////////// ");
  }
  ib_final = z_sal;
  ic_final = y_sal;

  hfe = (fabs(ic_final) / fabs(ib_final));
  // ib_final  son ua  multiplico por 1000
  hfe = hfe * 1000;
  Serial.print("hfe: ");
  Serial.println(hfe);
gain_trans = "la ganancia es: "+(String)hfe; 
  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  analisis    Nbjt graph
/////////////////////////////////////////////////////////////////////////////////////////

void nbjt_graph()
{
  imagen = "img5";
  x_min = 0;
  x_max = 9;
  y_min = 0;
  y_max = 3.5;
  position = "ne";
  
  //borrar esto
  Serial.println("he llegado");
  Serial.println(colector);
  Serial.println(base);
  Serial.println(emisor);

  terminals();

  Vb = 1366;  // valor de comienzo de Vbase para que vaya mas rápido
  carga = ""; // vacio la cadena donde voy a almacenar los datos de la grafica
  numero = 0; // el contador de graficas a 0
  filtro = 0.09;
  label = "Ibase: ";

  POTset(select_colector, 255); //946 medidos R colector
  POTset(select_base, 137);     //100k medidos R base
  POTset(select_emisor, 255);   //948 medidos R emisor

  for (IbSet = 2.0; IbSet < 9.0; IbSet += 1.0)
  {

    DACset(select_colector, 0); //tesión colector
    DACset(select_base, Vb);    //3,3vbase tensión base
    DACset(select_emisor, 0);   //tensión emisor

    ++numero; //saber el numero de la grafiaca
    charge = true;
    countok = 0;

    for (V = 0; V < 4000; V += 20)
    {
      DACset(select_colector, V); //voy aumentando la tensión del colector

      Ibase = measureI(base1, base2, 100); //corriente de base
      while (Ibase >= IbSet + 0.1 || Ibase <= IbSet - 0.1)
      {
        yield();

        if (Ibase > IbSet + 0.1)
          Vbas -= 10;
        if (Ibase < IbSet - 0.1)
          Vbas += 7;

        DACset(select_base, Vbas); //cambio tension de base
        Ibase = measureI(base1, base2, 100);
        server.handleClient();
      }

      x = mideTerminal(colector2) - mideTerminal(emisor2); //calculo Vce Vcolector-Vemiros
      y = measureI(colector1, colector2, 946);             //intensidad de colector
      z = measureI(base1, base2, 100);                     //calculo tension de base

      ver_datos();
      server.handleClient();
      yield();
    }
    label_value = IbSet;
    label_value += "µA";

    flag = true;
gain_trans = "la ganancia es: ?"; 
    while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
    {
      server.handleClient();
      Serial.print("/");
    }
    Serial.println("/////////////////////////////////////////////////////////// ");
  }

  ib_final = z_sal;
  ic_final = y_sal;

  hfe = (fabs(ic_final) / fabs(ib_final));
  // ib_final  son ua  multiplico por 1000
  hfe = hfe * 1000;
  Serial.print("hfe: ");
  Serial.println(hfe);
  gain_trans = "la ganancia es: "+(String)hfe; 

  /*
  while (flag == true) // hay que meterlo dentro de un while porque sino paraleliza el programa, mientas actualiza el web server sigue tomando datos.
  {
    server.handleClient();
    Serial.print("/");
  }
*/
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                   ver datos   filtramos  y almacenamos
/////////////////////////////////////////////////////////////////////////////////////////

void ver_datos()
{
  if (charge == true)
  { ///cargo el filtro para los primeros valores

    x_sal = x;
    y_sal = y;
    z_sal = z;

    charge = false;
    return;
  }

  ////aplicamos filtro

  x_sal = x_sal + (filtro * (x - x_sal));
  y_sal = y_sal + (filtro * (y - y_sal));
  z_sal = z_sal + (filtro * (z - z_sal));

  valorx[countok] = x_sal;

  valory[countok] = y_sal;
  Serial.print(valorx[countok]);

  Serial.print("  ");

  Serial.print(valory[countok]);
  Serial.print("  ");

  Serial.print(z_sal);
  Serial.print("  ");

  Serial.print(countok);
  Serial.print("  ");

  Serial.print(start);
  Serial.print("  ");

  Serial.println(numero);

  ++countok;

  yield();
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                  Detectamos terminales
/////////////////////////////////////////////////////////////////////////////////////////
void detectar_terminal()
{

  detectar_diodo();
  detectar_MOSFET();
  detectar_NPNbjt();
  detectar_PNPbjt();
  detectar_NJFET();
  detectar_PJFET();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                    Detectar diodo
/////////////////////////////////////////////////////////////////////////////////////////
void detectar_diodo()
{

  POTset(0, 255);
  POTset(1, 255);
  POTset(2, 255);

  ///////////////////////////////////////////// terminal A anodo

  DACset(0, 3000);
  DACset(1, 0);
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  DACset(0, 0);
  DACset(1, 0);
  DACset(2, 3000);

  Vcomp1 = int(VB2 * 10);
  Vcomp2 = int(VC2 * 10);

  if (Vcomp1 == 0)
  {

    Vdif1 = VA2 - VC2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    { ///esto se puede cumplir tb en los BJT PNP A podria ser el Colector y C la base
      DACset(0, 0);
      DACset(1, 3000); //// si es un BJT y aplicara a B al supuesto emisor tension la supuesta base C no deberia estas a A
      DACset(2, 0);

      VA2 = mideTerminal(TERMINAL_A2);
      VC2 = mideTerminal(TERMINAL_C2);

      Vcomp1 = int(VA2 * 10); ///si es un dido ni A ni B deberian verse afectados
      Vcomp2 = int(VC2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      { //// Si se cumple esto me aseguro de que no es un

        DACset(0, 3000);
        DACset(1, 3000); ///aplico voltaje al terminal B para comprobar que no se trata de un NMOS
        DACset(2, 0);    /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VA2 = mideTerminal(TERMINAL_A2);
        VC2 = mideTerminal(TERMINAL_C2);
        Vdif1 = VA2 - VC2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 0);
          DACset(1, 0); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 3500);

          VA2 = mideTerminal(TERMINAL_A2);
          Vcomp1 = int(VA2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 1 ANODO y terminal 3 CATODO");
            deteccion = "Diodo: terminal 1 ANODO y terminal 3 CATODO";
            DIODE = true;
            anodo = 1;
            catodo = 3;
            ++ndetecciones;
          }
          else
          {

            Serial.println("Diodo Zener: terminal 1 ANODO y terminal 3 CATODO");
            deteccion = "Diodo Zener: terminal 1 ANODO y terminal 3 CATODO";
            DIODE_ZENER = true;
            anodo = 1;
            catodo = 3;
            ++ndetecciones;
          }
        }
      }
    }
  }
  else if (Vcomp2 == 0)
  {

    Vdif1 = VA2 - VB2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    {

      DACset(0, 0);
      DACset(1, 0); //// si es un BJT y aplicara a B al supuesto emisor tension la supuesta base C no deberia estas a A
      DACset(2, 3000);

      VA2 = mideTerminal(TERMINAL_A2);
      VB2 = mideTerminal(TERMINAL_B2);

      Vcomp1 = int(VA2 * 10);
      Vcomp2 = int(VB2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      {

        DACset(0, 3000);
        DACset(1, 0);    ///aplico voltaje al terminal C para comprobar que no se trata de un NMOS
        DACset(2, 3000); /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VA2 = mideTerminal(TERMINAL_A2);
        VB2 = mideTerminal(TERMINAL_B2);
        Vdif1 = VA2 - VB2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 0);
          DACset(1, 3500); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 0);

          VA2 = mideTerminal(TERMINAL_A2);
          Vcomp1 = int(VA2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 1 ANODO y terminal 2 CATODO");
            deteccion = "Diodo: terminal 1 ANODO y terminal 2 CATODO";
            DIODE = true;
            anodo = 1;
            catodo = 2;
            ++ndetecciones;
          }
          else
          {

            Serial.println("Diodo Zenner: terminal 1 ANODO y terminal 2 CATODO");
            deteccion = "Diodo Zenner: terminal 1 ANODO y terminal 2 CATODO";
            DIODE_ZENER = true;
            anodo = 1;
            catodo = 2;
            ++ndetecciones;
          }
        }
      }
    }
  }

  ///////////////////////////////////////////// terminal B anodo

  DACset(0, 0);
  DACset(1, 3000);
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vcomp1 = int(VA2 * 10);
  Vcomp2 = int(VC2 * 10);

  if (Vcomp1 == 0)
  {

    Vdif1 = VB2 - VC2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    {

      DACset(0, 3000);
      DACset(1, 0);
      DACset(2, 0);

      VB2 = mideTerminal(TERMINAL_B2);
      VC2 = mideTerminal(TERMINAL_C2);

      Vcomp1 = int(VB2 * 10);
      Vcomp2 = int(VC2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      {

        DACset(0, 3000);
        DACset(1, 3000); ///aplico voltaje al terminal A para comprobar que no se trata de un NMOS
        DACset(2, 0);    /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VB2 = mideTerminal(TERMINAL_B2);
        VC2 = mideTerminal(TERMINAL_C2);
        Vdif1 = VB2 - VC2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 0);
          DACset(1, 0); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 3500);

          VB2 = mideTerminal(TERMINAL_B2);
          Vcomp1 = int(VB2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 2 ANODO y terminal 3 CATODO");
            deteccion = "Diodo: terminal 2 ANODO y terminal 3 CATODO";
            DIODE = true;
            anodo = 2;
            catodo = 3;
            ++ndetecciones;
          }
          else
          {
            Serial.println("Diodo Zener: terminal 2 ANODO y terminal 3 CATODO");
            deteccion = "Diodo Zener: terminal 2 ANODO y terminal 3 CATODO";
            DIODE_ZENER = true;
            anodo = 2;
            catodo = 3;
            ++ndetecciones;
          }
        }
      }
    }
  }
  else if (Vcomp2 == 0)
  {

    Vdif1 = VB2 - VA2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    {
      DACset(0, 0);
      DACset(1, 0);
      DACset(2, 3000);

      VA2 = mideTerminal(TERMINAL_A2);
      VB2 = mideTerminal(TERMINAL_B2);

      Vcomp1 = int(VA2 * 10);
      Vcomp2 = int(VB2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      {

        DACset(0, 0);
        DACset(1, 3000); ///aplico voltaje al terminal C para comprobar que no se trata de un NMOS
        DACset(2, 3000); /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VA2 = mideTerminal(TERMINAL_A2);
        VB2 = mideTerminal(TERMINAL_B2);
        Vdif1 = VB2 - VA2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 3500);
          DACset(1, 0); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 0);

          VB2 = mideTerminal(TERMINAL_B2);
          Vcomp1 = int(VB2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 2 ANODO y terminal 1 CATODO");
            deteccion = "Diodo: terminal 2 ANODO y terminal 1 CATODO";
            DIODE = true;
            anodo = 2;
            catodo = 1;
            ++ndetecciones;
          }
          else
          {
            Serial.println("Diodo Zener: terminal 2 ANODO y terminal 1 CATODO");
            deteccion = "Diodo Zener: terminal 2 ANODO y terminal 1 CATODO";
            DIODE_ZENER = true;
            anodo = 2;
            catodo = 1;
            ++ndetecciones;
          }
        }
      }
    }
  }

  ///////////////////////////////////////////// terminal C anodo

  DACset(0, 0);
  DACset(1, 0);
  DACset(2, 3000);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vcomp1 = int(VA2 * 10);
  Vcomp2 = int(VB2 * 10);

  if (Vcomp1 == 0)
  {

    Vdif1 = VC2 - VB2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    {

      DACset(0, 3000);
      DACset(1, 0);
      DACset(2, 0);

      VB2 = mideTerminal(TERMINAL_B2);
      VC2 = mideTerminal(TERMINAL_C2);

      Vcomp1 = int(VB2 * 10);
      Vcomp2 = int(VC2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      {

        DACset(0, 3000);
        DACset(1, 0);    ///aplico voltaje al terminal A para comprobar que no se trata de un NMOS
        DACset(2, 3000); /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VC2 = mideTerminal(TERMINAL_C2);
        VB2 = mideTerminal(TERMINAL_B2);
        Vdif1 = VC2 - VB2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 0);
          DACset(1, 3500); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 0);

          VC2 = mideTerminal(TERMINAL_C2);
          Vcomp1 = int(VC2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 3 ANODO y terminal 2 CATODO");
            deteccion = "Diodo: terminal 3 ANODO y terminal 2 CATODO";

            DIODE = true;
            anodo = 3;
            catodo = 2;
            ++ndetecciones;
          }
          else
          {
            Serial.println("Diodo Zener: terminal 3 ANODO y terminal 2 CATODO");
            deteccion = "Diodo Zener: terminal 3 ANODO y terminal 2 CATODO";

            DIODE_ZENER = true;
            anodo = 3;
            catodo = 2;
            ++ndetecciones;
          }
        }
      }
    }
  }
  else if (Vcomp2 == 0)
  {

    Vdif1 = VC2 - VA2;

    if (Vdif1 > 0.5 & Vdif1 < 0.9)
    {
      DACset(0, 0);
      DACset(1, 3000);
      DACset(2, 0);

      VA2 = mideTerminal(TERMINAL_A2);
      VC2 = mideTerminal(TERMINAL_C2);

      Vcomp1 = int(VA2 * 10);
      Vcomp2 = int(VC2 * 10);

      if (Vcomp1 == 0 & Vcomp2 == 0)
      {

        DACset(0, 0);
        DACset(1, 3000); ///aplico voltaje al terminal B para comprobar que no se trata de un NMOS
        DACset(2, 3000); /// si se trata de un NMOS la tensión de la fuente drenador bajaría y lo
                         /// habría detectado sería el diodo parasito del sustrato

        VA2 = mideTerminal(TERMINAL_A2);
        VC2 = mideTerminal(TERMINAL_C2);
        Vdif1 = VC2 - VA2;

        if (Vdif1 > 0.5 & Vdif1 < 0.9)
        { // si es un diodo la tensión ANODO CATODO seguira siendo la misma

          DACset(0, 3500);
          DACset(1, 0); ///miro a ver si conduce en inversa para ver si es un diodo zenner
          DACset(2, 0);

          VC2 = mideTerminal(TERMINAL_C2);
          Vcomp1 = int(VC2 * 10);

          if (Vcomp1 == 0)
          {

            Serial.println("Diodo: terminal 3 ANODO y terminal 1 CATODO");
            deteccion = "Diodo: terminal 3 ANODO y terminal 1 CATODO";

            DIODE = true;
            anodo = 3;
            catodo = 1;
            ++ndetecciones;
          }
          else
          {
            Serial.println("Diodo Zener: terminal 3 ANODO y terminal 1 CATODO");
            deteccion = "Diodo Zener: terminal 3 ANODO y terminal 1 CATODO";

            DIODE_ZENER = true;
            anodo = 3;
            catodo = 1;
            ++ndetecciones;
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     Detectar  MOSFET
/////////////////////////////////////////////////////////////////////////////////////////

void detectar_MOSFET()
{

  ///////////////////////detectar si es NMOS o PMOS ////////////////// detecto los ceros y luego cual es la guente

  i = 0;
  // voy a contar cuantas veces hay dos ceros cuando aplico una tension de 5 voltios, en caso de que haya 2 veces dos ceros sera NPN pq el transistor no conduce

  /////////////////////////////ceros canal A
  POTset(0, 127);
  POTset(1, 127);
  POTset(2, 127);

  DACset(0, 2000);
  DACset(1, 0);
  DACset(2, 0);

  // VA2 = mideTerminal(TERMINAL_A2); //no me hace falta
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vcomp1 = int(VB2 * 10);
  Vcomp2 = int(VC2 * 10);

  if (Vcomp1 == 0 & Vcomp2 == 0)
    i += 1;

  ///////////////////////////////ceros canal B
  POTset(0, 127);
  POTset(1, 127);
  POTset(2, 127);

  DACset(0, 0);
  DACset(1, 2000);
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vcomp1 = int(VA2 * 10);
  Vcomp2 = int(VC2 * 10);

  if (Vcomp1 == 0 & Vcomp2 == 0)
    i += 1;

  /////////////////////////////ceros canal C
  POTset(0, 127);
  POTset(1, 127);
  POTset(2, 127);

  DACset(0, 0);
  DACset(1, 0);
  DACset(2, 2000);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);

  Vcomp1 = int(VA2 * 10);
  Vcomp2 = int(VB2 * 10);

  if (Vcomp1 == 0 & Vcomp2 == 0)
    i += 1;

  ////////////////////////////////////////////// deteccion de NMOS ////////////////////////////
  ////////////////////////////////////////////// deteccion de NMOS ////////////////////////////

  if (i == 2)
  { /// en los BJT NPN tambien detecta 2 ceros

    //////////////////////// terminal A fuente (SOURCE)
    //////////////////////// terminal A fuente (SOURCE)

    POTset(0, 127);
    POTset(1, 127);
    POTset(2, 127);

    DACset(0, Vb);
    DACset(1, 0);
    DACset(2, 0);

    VA2 = mideTerminal(TERMINAL_A2);
    VB2 = mideTerminal(TERMINAL_B2);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VB2 * 10);
    Vcomp2 = int(VC2 * 10);

    //probamos terminal B GATE

    if (Vcomp1 == 0 & Vcomp2 != 0)
    { //si se cumple posible terminal B(2) GATE

      Vdif1 = VA2 - VC2;

      if (Vdif1 >= 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 SOURCE, terminal 2 GATE, terminal 3  DRAIN");
        deteccion = "NMOS: terminal 1 SOURCE, terminal 2 GATE, terminal 3  DRAIN";

        NMOS = true;
        source = 1;
        gate = 2;
        drain = 3;
        ++ndetecciones;
      }
    }

    // probamos que el terminal C es la GATE

    if (Vcomp2 == 0 & Vcomp1 != 0)
    { //si se cumple posible terminal C(3) GATE

      Vdif1 = VA2 - VB2;

      if (Vdif1 >= 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE");
        deteccion = "NMOS: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE";

        NMOS = true;
        source = 1;
        gate = 3;
        drain = 2;
        ++ndetecciones;
      }
    }

    ////////////////////////terminal B fuente (SOURCE)

    POTset(0, 127);
    POTset(1, 127);
    POTset(2, 127);

    DACset(0, 0);
    DACset(1, Vb);
    DACset(2, 0);

    VA2 = mideTerminal(TERMINAL_A2);
    VB2 = mideTerminal(TERMINAL_B2);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VA2 * 10);
    Vcomp2 = int(VC2 * 10);

    // probamos si el terminal A es la GATE

    if (Vcomp1 == 0 & Vcomp2 != 0)
    { //si se cumple posible terminal A(1) GATE

      Vdif1 = VB2 - VC2;

      if (Vdif1 > 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 GATE, terminal 2 SOURCE, terminal 3  DRAIN");
        deteccion = "NMOS: terminal 1 GATE, terminal 2 SOURCE, terminal 3  DRAIN";
        NMOS = true;
        source = 2;
        gate = 1;
        drain = 3;
        ++ndetecciones;
      }
    }

    // probamos si el terminal C es la GATE

    if (Vcomp2 == 0 & Vcomp1 != 0)
    { //si se cumple posible terminal C(3) GATE

      Vdif1 = VB2 - VA2;

      if (Vdif1 >= 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE");
        deteccion = "NMOS: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE";
        NMOS = true;
        source = 2;
        gate = 3;
        drain = 1;
        ++ndetecciones;
      }
    }

    ////////////////////////terminal C fuente (SOURCE)

    POTset(0, 127);
    POTset(1, 127);
    POTset(2, 127);

    DACset(0, 0);
    DACset(1, 0);
    DACset(2, Vb);

    VA2 = mideTerminal(TERMINAL_A2);
    VB2 = mideTerminal(TERMINAL_B2);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VA2 * 10);
    Vcomp2 = int(VB2 * 10);

    // probamos si el terminal A es la GATE

    if (Vcomp1 == 0 & Vcomp2 != 0)
    { //si se cumple posible terminal A(1) GATE

      Vdif1 = VC2 - VB2;

      if (Vdif1 >= 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE");
        deteccion = "NMOS: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE";

        NMOS = true;
        source = 3;
        gate = 1;
        drain = 2;
        ++ndetecciones;
      }
    }

    //probamos si el terminal B es la GATE

    if (Vcomp2 == 0 & Vcomp1 != 0)
    { //si se cumple posible terminal A(1) GATE

      Vdif1 = VC2 - VA2;

      if (Vdif1 >= 0.5)
      { // si se cumple tenemos las posiciones

        Serial.println("NMOS: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE");
        deteccion = "NMOS: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE";

        NMOS = true;
        source = 3;
        gate = 2;
        drain = 1;
        ++ndetecciones;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////// deteccion PMOS ////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// deteccion PMOS ////////////////////////////////////////////////

  if (i == 1)
  { //en los bjt PNP tambien detecta 1

    POTset(0, 127);
    POTset(1, 127);
    POTset(2, 127);

    // terminal 1 y 2  ;candidato a fuente C

    DACset(0, 1950);
    DACset(1, 1950);
    DACset(2, 0);

    VA1 = mideTerminal(TERMINAL_A1);
    VA2 = mideTerminal(TERMINAL_A2);
    VB1 = mideTerminal(TERMINAL_B1);
    VB2 = mideTerminal(TERMINAL_B2);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VC2 * 10); //si Vcomp1 no es 0 es que Vc2 es la fuente SOURCE

    if (Vcomp1 != 0)
    {

      Vdif1 = VA2 - VC2; //la diferencia mayor tiene que estar entre la puerta y la fuente
      Vdif2 = VB2 - VC2; //la diferencia menor tiene que estar el drenador y la fuente tiene que ser del orden de 0,6

      if (Vdif1 > Vdif2)
      {

        // mediante los siguentes calculos impedimos que se detecte como un BJT PNP
        //si el terminal A es la puerta VA1 tiene que ser practicamente igual a VA2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VA1 * 10);
        Vcomp2 = int(VA2 * 10);
        if (Vcomp1 == Vcomp2)
        {
          DACset(0, 0);
          DACset(1, 2000);
          DACset(2, 0);

          VA2 = mideTerminal(TERMINAL_A2);
          Vcomp3 = int(VA2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 0);
            DACset(1, 0);
            DACset(2, 3000);

            // mido tension drenador
            VB2 = mideTerminal(TERMINAL_B2);
            Vcomp1 = int(VB2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 3000);
            DACset(1, 0);
            DACset(2, 3000);

            //vuelvo a medir el drenador
            VB2 = mideTerminal(TERMINAL_B2);
            Vcomp2 = int(VB2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate
              Serial.println("PMOS: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE");
              deteccion = "PMOS: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE";

              PMOS = true;
              source = 3;
              gate = 1;
              drain = 2;
              ++ndetecciones;
            }
          }
        }
      }
      else
      {

        // mediante los siguentes calculos impedimos que se detecte como un BJT PNP
        //si el terminal B es la puerta VB1 tiene que ser practicamente igual a VB2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VB1 * 10);
        Vcomp2 = int(VB2 * 10);
        if (Vcomp1 == Vcomp2)
        {
          DACset(0, 2000);
          DACset(1, 0);
          DACset(2, 0);

          VB2 = mideTerminal(TERMINAL_B2);
          Vcomp3 = int(VB2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 0);
            DACset(1, 0);
            DACset(2, 3000);

            // mido tension drenador
            VA2 = mideTerminal(TERMINAL_A2);
            Vcomp1 = int(VA2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 0);
            DACset(1, 3000);
            DACset(2, 3000);

            //vuelvo a medir el drenador
            VA2 = mideTerminal(TERMINAL_A2);
            Vcomp2 = int(VA2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate

              Serial.println("PMOS: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE");
              deteccion = "PMOS: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE";

              PMOS = true;
              source = 3;
              gate = 2;
              drain = 1;
              ++ndetecciones;
            }
          }
        }
      }
    }

    // terminal 1 y 3  ; cadidate a fuente B
    DACset(0, 1950);
    DACset(1, 0);
    DACset(2, 1950);

    VA1 = mideTerminal(TERMINAL_A1);
    VA2 = mideTerminal(TERMINAL_A2);
    VB2 = mideTerminal(TERMINAL_B2);
    VC1 = mideTerminal(TERMINAL_C1);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VB2 * 10); //si Vcomp1 no es 0 es que VB2 es la fuente SOURCE

    if (Vcomp1 != 0)
    {

      Vdif1 = VA2 - VB2; //la diferencia mayor tiene que estar entre la puerta y la fuente
      Vdif2 = VC2 - VB2; //la diferencia menor tiene que estar el drenador y la fuente tiene que ser del orden de 0,6

      if (Vdif1 > Vdif2)
      {

        // mediante los siguentes calculos impedimos que se detecte como un BJT PNP
        //si el terminal A es la puerta VA1 tiene que ser practicamente igual a VA2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VA1 * 10);
        Vcomp2 = int(VA2 * 10);
        if (Vcomp1 == Vcomp2)
        {
          DACset(0, 0);
          DACset(1, 0);
          DACset(2, 2000);

          VA2 = mideTerminal(TERMINAL_A2);
          Vcomp3 = int(VA2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 0);
            DACset(1, 3000);
            DACset(2, 0);

            // mido tension drenador
            VC2 = mideTerminal(TERMINAL_C2);
            Vcomp1 = int(VC2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 3000);
            DACset(1, 3000);
            DACset(2, 0);

            //vuelvo a medir el drenador
            VC2 = mideTerminal(TERMINAL_C2);
            Vcomp2 = int(VC2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate

              Serial.println("PMOS: terminal 1 GATE, terminal 2 SOURCE, terminal 3 DRAIN");
              deteccion = "PMOS: terminal 1 GATE, terminal 2 SOURCE, terminal 3 DRAIN";

              PMOS = true;
              source = 2;
              gate = 1;
              drain = 3;
              ++ndetecciones;
            }
          }
        }
      }
      else
      {
        //si el terminal C es la puerta VC1 tiene que ser practicamente igual a VC2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VC1 * 10);
        Vcomp2 = int(VC2 * 10);
        if (Vcomp1 == Vcomp2)
        {
          DACset(0, 2000);
          DACset(1, 0);
          DACset(2, 0);

          VC2 = mideTerminal(TERMINAL_C2);
          Vcomp3 = int(VC2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 0);
            DACset(1, 3000);
            DACset(2, 0);

            // mido tension drenador
            VA2 = mideTerminal(TERMINAL_A2);
            Vcomp1 = int(VA2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 0);
            DACset(1, 3000);
            DACset(2, 3000);

            //vuelvo a medir el drenador
            VA2 = mideTerminal(TERMINAL_A2);
            Vcomp2 = int(VA2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate
              Serial.println("PMOS: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE");
              deteccion = "PMOS: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE";

              PMOS = true;
              source = 2;
              gate = 3;
              drain = 1;
              ++ndetecciones;
            }
          }
        }
      }
    }

    // terminal 2 y 3   ; cadidato  a fuente A

    DACset(0, 0);
    DACset(1, 1950);
    DACset(2, 1950);

    VA2 = mideTerminal(TERMINAL_A2);
    VB1 = mideTerminal(TERMINAL_B1);
    VB2 = mideTerminal(TERMINAL_B2);
    VC1 = mideTerminal(TERMINAL_C1);
    VC2 = mideTerminal(TERMINAL_C2);

    Vcomp1 = int(VA2 * 10); //si Vcomp1 no es 0 es que Vc2 es la fuente SOURCE

    if (Vcomp1 != 0)
    {

      Vdif1 = VB2 - VA2; //la diferencia mayor tiene que estar entre la puerta y la fuente
      Vdif2 = VC2 - VA2; //la diferencia menor tiene que estar el drenador y la fuente tiene que ser del orden de 0,6

      if (Vdif1 > Vdif2)
      {

        // mediante los siguentes calculos impedimos que se detecte como un BJT PNP
        //si el terminal B es la puerta VB1 tiene que ser practicamente igual a VB2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VB1 * 10);
        Vcomp2 = int(VB2 * 10);
        if (Vcomp1 == Vcomp2)
        {
          DACset(0, 0);
          DACset(1, 0);
          DACset(2, 2000);

          VB2 = mideTerminal(TERMINAL_B2);
          Vcomp3 = int(VB2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 3000);
            DACset(1, 0);
            DACset(2, 0);

            // mido tension drenador
            VC2 = mideTerminal(TERMINAL_C2);
            Vcomp1 = int(VC2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 3000);
            DACset(1, 3000);
            DACset(2, 0);

            //vuelvo a medir el drenador
            VC2 = mideTerminal(TERMINAL_C2);
            Vcomp2 = int(VC2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate

              Serial.println("PMOS: terminal 1 SOURCE, terminal 2 GATE, terminal 3 DRAIN");
              deteccion = "PMOS: terminal 1 SOURCE, terminal 2 GATE, terminal 3 DRAIN";

              PMOS = true;
              source = 1;
              gate = 2;
              drain = 3;
              ++ndetecciones;
            }
          }
        }
      }
      else
      {

        // mediante los siguentes calculos impedimos que se detecte como un BJT PNP
        //si el terminal C es la puerta VC1 tiene que ser practicamente igual a VC2 puesto que no circula casi corriente por la puerta
        Vcomp1 = int(VC1 * 10);
        Vcomp2 = int(VC2 * 10);
        if (Vcomp1 == Vcomp2)
        {

          DACset(0, 0);
          DACset(1, 2000);
          DACset(2, 0);

          VC2 = mideTerminal(TERMINAL_C2);
          Vcomp3 = int(VC2 * 10);
          if (Vcomp3 == 0)
          { //compruebo que no es un jfetP

            // aplico tension solo a la fuente
            DACset(0, 3000);
            DACset(1, 0);
            DACset(2, 0);

            // mido tension drenador
            VB2 = mideTerminal(TERMINAL_B2);
            Vcomp1 = int(VB2 * 10);

            // aplico tension tambien a la puerta
            DACset(0, 3000);
            DACset(1, 0);
            DACset(2, 3000);

            //vuelvo a medir el drenador
            VB2 = mideTerminal(TERMINAL_B2);
            Vcomp2 = int(VB2 * 10);

            if (Vcomp1 != Vcomp2)
            { // si no es un diodo zener la tension habra cambiado al aplicar tension en la gate

              Serial.println("PMOS: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE");
              deteccion = "PMOS: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE";

              PMOS = true;
              source = 1;
              gate = 3;
              drain = 2;
              ++ndetecciones;
            }
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     Detectar  Bjt
/////////////////////////////////////////////////////////////////////////////////////////
void detectar_NPNbjt()
{

  ////////////////////////////////////// detecto si es NPN //////////////////////////////////////

  ////////////////////////////////////////canal A base NPN

  POTset(0, 255); //minima R
  POTset(1, 242);
  POTset(2, 242);

  DACset(0, 2000);
  DACset(1, 0);
  DACset(2, 0);

  VA1 = mideTerminal(TERMINAL_A1);
  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC1 = mideTerminal(TERMINAL_C1);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VA2 - VB2;
  Vdif2 = VA2 - VC2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  {

    if (VB2 > 0.5 & VC2 > 0.5) //sabemos que es NPN y que la base es la A
    {
      DACset(0, 0);
      DACset(1, 2000);
      DACset(2, 0);

      VC2 = mideTerminal(TERMINAL_C2);
      Vcomp1 = int(VC2 * 10);

      if (Vcomp1 == 0)
      { ///hago esto para asegurarme de que no es un JFET tipo N y no conduce entre colector y drenador

        POTset(0, 127); // 100k en la base
        POTset(1, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k
        POTset(2, 255);

        DACset(0, Vb);
        DACset(1, 4000);
        DACset(2, 0);

        Ib = measureI(TERMINAL_B1, TERMINAL_B2, 10);

        POTset(0, 127);
        POTset(1, 255);
        POTset(2, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k

        DACset(0, Vb);
        DACset(1, 0);
        DACset(2, 4000);

        Ic = measureI(TERMINAL_C1, TERMINAL_C2, 10);

        // comparamos las corrientes la corriente de colector en un NPN tiene que ser mucho mayor ya que la ganancia de colector es mucho mayor
        if (Ib > Ic)
        {
          Serial.println("BJT NPN: terminal 1 BASE, terminal 2 COLECTOR, terminal 3 EMISOR");
          deteccion = "BJT NPN: terminal 1 BASE, terminal 2 COLECTOR, terminal 3 EMISOR";

          NBJT = true;
          colector = 2;
          base = 1;
          emisor = 3;
          ++ndetecciones;
        }
        else
        {
          Serial.println("BJT NPN: terminal 1 BASE, terminal 2 EMISOR, terminal 3 COLECTOR");
          deteccion = "BJT NPN: terminal 1 BASE, terminal 2 EMISOR, terminal 3 COLECTOR";

          NBJT = true;
          colector = 3;
          base = 1;
          emisor = 2;
          ++ndetecciones;
        }
      }
    }
  }

  //////////////////////////canal B base NPN
  //////////////////////////canal B base NPN

  POTset(0, 242);
  POTset(1, 255);
  POTset(2, 242);

  DACset(0, 0);
  DACset(1, Vb);
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB1 = mideTerminal(TERMINAL_B1);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VB2 - VA2;
  Vdif2 = VB2 - VC2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  {

    if (VA2 > 0.5 & VC2 > 0.5) //sabemos que es NPN y que la base es la B
    {

      DACset(0, 2000);
      DACset(1, 0);
      DACset(2, 0);

      VC2 = mideTerminal(TERMINAL_C2);
      Vcomp1 = int(VC2 * 10);

      if (Vcomp1 == 0)
      { ///hago esto para asegurarme de que no es un JFET tipo N y no conduce entre colector y drenador

        POTset(0, 242);
        POTset(1, 127);
        POTset(2, 255);
        DACset(0, 4000);
        DACset(1, Vb);
        DACset(2, 0);

        Ia = measureI(TERMINAL_A1, TERMINAL_A2, 10);

        POTset(0, 255);
        POTset(1, 127);
        POTset(2, 242);
        DACset(0, 0);
        DACset(1, Vb);
        DACset(2, 4000);

        Ic = measureI(TERMINAL_C1, TERMINAL_C2, 10);

        // comparamos las corrientes la corriente de colector en un NPN tiene que ser mucho mayor ya que la ganancia de colector es mucho mayor
        if (Ia > Ic)
        {
          Serial.println("BJT NPN: terminal 1 COLECTOR, terminal 2 BASE, terminal 3 EMISOR");
          deteccion = "BJT NPN: terminal 1 COLECTOR, terminal 2 BASE, terminal 3 EMISOR";

          NBJT = true;
          colector = 1;
          base = 2;
          emisor = 3;
          ++ndetecciones;
        }
        else
        {
          Serial.println("BJT NPN: terminal 1 EMISOR, terminal 2 BASE, terminal 3 COELCTOR");
          deteccion = "BJT NPN: terminal 1 EMISOR, terminal 2 BASE, terminal 3 COELCTOR";

          NBJT = true;
          colector = 3;
          base = 2;
          emisor = 1;
          ++ndetecciones;
        }
      }
    }
  }

  //////////////////////////////////////canal C base NPN
  //////////////////////////////////////canal C base NPN

  POTset(0, 242); //minima R
  POTset(1, 242);
  POTset(2, 255);

  DACset(0, 0);
  DACset(1, 0);
  DACset(2, Vb);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC1 = mideTerminal(TERMINAL_C1);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VC2 - VA2;
  Vdif2 = VC2 - VB2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  {

    if (VA2 > 0.5 & VB2 > 0.5) //sabemos que es NPN y que la base es la A
    {

      DACset(0, 2000);
      DACset(1, 0);
      DACset(2, 0);

      VB2 = mideTerminal(TERMINAL_B2);
      Vcomp1 = int(VB2 * 10);

      if (Vcomp1 == 0)
      { ///hago esto para asegurarme de que no es un JFET tipo N y no conduce entre colector y drenador

        POTset(0, 242);
        POTset(1, 255);
        POTset(2, 127);

        DACset(0, 4000);
        DACset(1, 0);
        DACset(2, Vb);

        Ia = measureI(TERMINAL_A1, TERMINAL_A2, 10);

        POTset(0, 127);
        POTset(1, 255);
        POTset(2, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k

        DACset(0, 0);
        DACset(1, 4000);
        DACset(2, Vb);

        Ib = measureI(TERMINAL_B1, TERMINAL_B2, 10);

        // comparamos las corrientes la corriente de colector en un NPN tiene que ser mucho mayor ya que la ganancia de colector es mucho mayor
        if (Ia > Ib)
        {
          Serial.println("BJT NPN: terminal 1 COLECTOR, terminal 2 EMISOR, terminal 3 BASE");
          deteccion = "BJT NPN: terminal 1 COLECTOR, terminal 2 EMISOR, terminal 3 BASE";

          NBJT = true;
          colector = 1;
          base = 3;
          emisor = 2;
          ++ndetecciones;
        }
        else
        {
          Serial.println("BJT NPN: terminal 1 EMISOR, terminal 2 COLECTOR, terminal 3 BASE");
          deteccion = "BJT NPN: terminal 1 EMISOR, terminal 2 COLECTOR, terminal 3 BASE";

          NBJT = true;
          colector = 2;
          base = 3;
          emisor = 1;
          ++ndetecciones;
        }
      }
    }
  }
}
void detectar_PNPbjt()
{

  ////////////////////////////////////////////////////////////////////////// detecto si es  PNP /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////// detecto si es  PNP /////////////////////////////////////

  ////////////////////////////////canal A base PNP
  ////////////////////////////////canal A base PNP

  POTset(0, 255); //minima R
  POTset(1, 247);
  POTset(2, 247);

  DACset(0, 0);
  DACset(1, 1300);
  DACset(2, 1300);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VA2 - VB2;
  Vdif2 = VA2 - VC2;

  if (Vdif1 < -0.5 & Vdif2 < -0.5)
  {

    if (VB2 < 1.5 & VC2 < 1.5) //sabemos que es PNP y que la base es la A
    {
      POTset(0, 127); // 100k en la base
      POTset(1, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k
      POTset(2, 255);

      DACset(0, Vb);
      DACset(1, 0);    //supuesto colector
      DACset(2, 4000); //supuesto emisor

      Ib = measureI(TERMINAL_B1, TERMINAL_B2, 10);

      POTset(0, 127);
      POTset(1, 255);
      POTset(2, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k

      DACset(0, Vb);
      DACset(1, 4000); //supuesto emisor
      DACset(2, 0);    //supuesto colector

      Ic = measureI(TERMINAL_C1, TERMINAL_C2, 10);

      // comparamos las corrientes, la corriente de colector en un PNP tiene que ser mucho mayor(en valor absoluto) ya que la ganancia de colector es mucho mayor
      if (Ib < Ic)
      {
        POTset(0, 255);
        POTset(1, 255);
        POTset(2, 255);

        DACset(0, 2000);
        DACset(1, 0);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_A2) - mideTerminal(TERMINAL_A1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP

          Serial.println("PNP: terminal 1 BASE, terminal 2 COLECTOR, terminal 3 EMISOR");
          deteccion = "PNP: terminal 1 BASE, terminal 2 COLECTOR, terminal 3 EMISOR";

          PBJT = true;
          colector = 2;
          base = 1;
          emisor = 3;
          ++ndetecciones;
        }
      }
      else
      {

        DACset(0, 2000);
        DACset(1, 0);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_A2) - mideTerminal(TERMINAL_A1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP cunado hay caida te tension en la resistencia de la base ya que circula una corriente
          Serial.println("PNP: terminal 1 BASE, terminal 2 EMISOR, terminal 3 COLECTOR");
          deteccion = "PNP: terminal 1 BASE, terminal 2 EMISOR, terminal 3 COLECTOR";

          PBJT = true;
          colector = 3;
          base = 1;
          emisor = 2;
          ++ndetecciones;
        }
      }
    }
  }

  ////////////////////////canal B base PNP
  ////////////////////////canal B base PNP

  POTset(0, 247);
  POTset(1, 255);
  POTset(2, 247);

  DACset(0, 1300);
  DACset(1, 0);
  DACset(2, 1300);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VB2 - VA2;
  Vdif2 = VB2 - VC2;

  if (Vdif1 < -0.5 & Vdif2 < -0.5)
  {

    if (VA2 < 1.5 & VC2 < 1.5) //sabemos que es PNP y que la base es la A
    {
      POTset(0, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k
      POTset(1, 127); // 100k en la base
      POTset(2, 255);

      DACset(0, 0); //supuesto colector
      DACset(1, Vb);
      DACset(2, 4000); //supuesto emisor

      Ia = measureI(TERMINAL_A1, TERMINAL_A2, 10);

      POTset(0, 255);
      POTset(1, 127);
      POTset(2, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k

      DACset(0, 4000); //supuesto emisor
      DACset(1, Vb);
      DACset(2, 0); //supuesto colector

      Ic = measureI(TERMINAL_C1, TERMINAL_C2, 10);

      // comparamos las corrientes, la corriente de colector en un PNP tiene que ser mucho mayor(en valor absoluto) ya que la ganancia de colector es mucho mayor
      if (Ia < Ic)
      {
        POTset(0, 255);
        POTset(1, 255);
        POTset(2, 255);

        DACset(0, 0);
        DACset(1, 2000);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_B2) - mideTerminal(TERMINAL_B1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP

          Serial.println("PNP: terminal 1 COLECTOR, terminal 2 BASE , terminal 3 EMISOR");
          deteccion = "PNP: terminal 1 COLECTOR, terminal 2 BASE , terminal 3 EMISOR";

          PBJT = true;
          colector = 1;
          base = 2;
          emisor = 3;
          ++ndetecciones;
        }
      }
      else
      {

        DACset(0, 0);
        DACset(1, 2000);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_B2) - mideTerminal(TERMINAL_B1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP

          Serial.println("PNP: terminal 1 EMISOR, terminal 2 BASE, terminal 3 COLECTOR");
          deteccion = "PNP: terminal 1 EMISOR, terminal 2 BASE, terminal 3 COLECTOR";

          PBJT = true;
          colector = 3;
          base = 2;
          emisor = 1;
          ++ndetecciones;
        }
      }
    }
  }

  ////////////////////////canal C base PNP
  ////////////////////////canal C base PNP

  POTset(0, 242); //minima R
  POTset(1, 242);
  POTset(2, 255);

  DACset(0, 1300);
  DACset(1, 1300);
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VC2 - VA2;
  Vdif2 = VC2 - VB2;

  if (Vdif1 < -0.5 & Vdif2 < -0.5)
  {

    if (VA2 < 1.5 & VB2 < 1.5) //sabemos que es PNP y que la base es la A
    {
      POTset(0, 242); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k
      POTset(1, 255);
      POTset(2, 127); // 100k en la base

      DACset(0, 0); //supuesto colector
      DACset(1, 4000);
      DACset(2, Vb); //supuesto emisor

      Ia = measureI(TERMINAL_A1, TERMINAL_A2, 10);

      POTset(0, 242);
      POTset(1, 255);
      POTset(2, 127); //medimos la corriente de este terminal por eso le fijamos una R de aprox 10k

      DACset(0, 4000); //supuesto emisor
      DACset(1, 0);
      DACset(2, Vb); //supuesto colector

      Ib = measureI(TERMINAL_B1, TERMINAL_B2, 10);

      // comparamos las corrientes, la corriente de colector en un PNP tiene que ser mucho mayor(en valor absoluto) ya que la ganancia de colector es mucho mayor
      if (Ia < Ib)
      {
        POTset(0, 255);
        POTset(1, 255);
        POTset(2, 255);

        DACset(0, 0);
        DACset(1, 3300);
        DACset(2, 2000);

        Vdif1 = mideTerminal(TERMINAL_C2) - mideTerminal(TERMINAL_C1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP

          Serial.println("PNP: terminal 1 COLECTOR, terminal 2 EMISOR , terminal 3 BASE");
          deteccion = "PNP: terminal 1 COLECTOR, terminal 2 EMISOR , terminal 3 BASE";

          PBJT = true;
          colector = 1;
          base = 3;
          emisor = 2;
          ++ndetecciones;
        }
      }
      else
      {

        DACset(0, 0);
        DACset(1, 3300);
        DACset(2, 2000);

        Vdif1 = mideTerminal(TERMINAL_C2) - mideTerminal(TERMINAL_C1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 != 0)
        { // compruebo que no es un jfetP
          Serial.println("PNP: terminal 1 EMISOR, terminal 2 COLECTOR , terminal 3 BASE");
          deteccion = "PNP: terminal 1 EMISOR, terminal 2 COLECTOR , terminal 3 BASE";

          PBJT = true;
          colector = 2;
          base = 3;
          emisor = 1;
          ++ndetecciones;
        }
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////
///////                     Detectar  JFET
/////////////////////////////////////////////////////////////////////////////////////////

void detectar_PJFET()
{
  POTset(0, 255);
  POTset(1, 255);
  POTset(2, 255);

  DACset(0, 2000);
  DACset(1, 0); // si aplico esta tension y no es la pue
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  if (VB2 > 0.5 || VC2 > 0.5)
  {

    Vcomp1 = int(VA2 * 10);
    Vcomp2 = int(VB2 * 10);

    if ((Vcomp1 - 1) <= Vcomp2) //
    {
      Vdif1 = VA2 - VC2;
      if (Vdif1 < 1.0 && Vdif1 > 0.4) //compruebo la diferencia entre el drenador y la gate debida a la unión PN
      {

        DACset(0, 0);
        DACset(1, 3300);
        DACset(2, 2000);

        Vdif1 = mideTerminal(TERMINAL_C2) - mideTerminal(TERMINAL_C1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 == 0)
        {

          Serial.println("JFET tipo P: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE");
          deteccion = "JFET tipo P: terminal 1 DRAIN, terminal 2 SOURCE, terminal 3 GATE";

          PJFET = true;
          source = 2;
          gate = 3;
          drain = 1;
          ++ndetecciones;
        }
      }
    }

    Vcomp2 = int(VC2 * 10);

    if ((Vcomp1 - 1) <= Vcomp2) //
    {

      Vdif1 = VA2 - VB2;
      if (Vdif1 < 1.0 && Vdif1 > 0.4) //compruebo la diferencia entre el drenador y la gate debida a la unión PN
      {

        DACset(0, 0);
        DACset(1, 2000);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_B2) - mideTerminal(TERMINAL_B1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 == 0)
        {
          Serial.println("JFET tipo P: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE");
          deteccion = "JFET tipo P: terminal 1 DRAIN, terminal 2 GATE, terminal 3 SOURCE";

          PJFET = true;
          source = 3;
          gate = 2;
          drain = 1;
          ++ndetecciones;
        }
      }
    }
  }

  DACset(0, 0);
  DACset(1, 2000); // si aplico esta tension y no es la pue
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  if (VA2 > 0.5 || VC2 > 0.5)
  {

    Vcomp1 = int(VB2 * 10);
    Vcomp2 = int(VC2 * 10);

    if ((Vcomp1 - 1) <= Vcomp2) //
    {

      Vdif1 = VB2 - VA2;
      if (Vdif1<1.0 & Vdif1> 0.4) //compruebo la diferencia entre el drenador y la gate debida a la unión PN
      {

        DACset(0, 2000);
        DACset(1, 0);
        DACset(2, 3300);

        Vdif1 = mideTerminal(TERMINAL_A2) - mideTerminal(TERMINAL_A1);
        Vcomp2 = int(Vdif1 * 100);
        if (Vcomp2 == 0)
        {
          Serial.println("JFET tipo P: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE");
          deteccion = "JFET tipo P: terminal 1 GATE, terminal 2 DRAIN, terminal 3 SOURCE";

          PJFET = true;
          source = 3;
          gate = 1;
          drain = 2;
          ++ndetecciones;
        }
      }
    }
  }
}

void detectar_NJFET()
{

  POTset(0, 127);
  POTset(1, 127);
  POTset(2, 127);

  DACset(0, 0);
  DACset(1, 0);
  DACset(2, 0);

  /// JFET tipo N  en el canal se comporta como una resistencia, esta R depende de Vgs canal de deplexion
  /// no distingo drenador de fuente el cal conduce pero hay una R, condue al no ser que VGS<<0
  /// entre la gate y el canal unión PN
  /// quieza podría detectarlo como al mosfet N, pero que los terminales los tecte como hice con el MOS tipo P

  // estategia:
  // voy a comprobar los terminales a la vez que determino si es un jfet
  // 1 la misma que en BJT NPN se aplico una tensión de 2v a la base D y S tendrán aprox 1,4 dif de 0,7 aprox

  //////////////////////////////////////////////////////////////////////////////////
  ////////////// en caso de JFET el terminal A es la GATE  /////////////////////////
  //////////////////////////////////////////////////////////////////////////////////

  DACset(0, 2000); // si aplico una tensión de 4.9 v al canal A tengo 2 v en el terminal de la gate
  DACset(1, 0);    // D y S a 0 v
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VA2 - VB2;
  Vdif2 = VA2 - VC2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  { // aqui doy por hecho que si se cumple las diferencia de Vdif1 respecto a Vdif2 son iguales

    if (VB2 > 0.5 & VC2 > 0.5) // PUEDE SER UN BJT NPN O UN JFET TIPO N
    {
      DACset(0, 0);
      DACset(1, 2000);
      DACset(2, 0);

      VC2 = mideTerminal(TERMINAL_C2);

      if (VC2 > 0.2)
      { // si tenemos tensión en ese canas el que conduce por lo tanto se trata de un jfet tipo y no de un bjt tipo N

        Serial.println("JFET tipo N: terminal 1 GATE, terminal 2 SOURCE, terminal 3 DRAIN");
        deteccion = "JFET tipo N: terminal 1 GATE, terminal 2 SOURCE, terminal 3 DRAIN";

        NJFET = true;
        source = 2;
        gate = 1;
        drain = 3;
        ++ndetecciones;
      }
    }
  }
  //////////////////////////////////////////////////////////////////////////////////
  ////////////// en caso de JFET el terminal B es la GATE  /////////////////////////
  //////////////////////////////////////////////////////////////////////////////////

  DACset(0, 0);    // si aplico una tensión de 4.9 v al canal A tengo 2 v en el terminal de la gate
  DACset(1, 2000); // D y S a 0 v
  DACset(2, 0);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VB2 - VA2;
  Vdif2 = VB2 - VC2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  { // aqui doy por hecho que si se cumple las diferencia de Vdif1 respecto a Vdif2 son iguales

    if (VA2 > 0.5 & VC2 > 0.5) // PUEDE SER UN BJT NPN O UN JFET TIPO N
    {
      DACset(0, 2000);
      DACset(1, 0);
      DACset(2, 0);

      VC2 = mideTerminal(TERMINAL_C2);

      if (VC2 > 0.2)
      { // si tenemos tensión en ese canas el que conduce por lo tanto se trata de un jfet tipo y no de un bjt tipo N

        Serial.println("JFET tipo N: terminal 1 SOURCE, terminal 2 GATE, terminal 3 DRAIN");
        deteccion = "JFET tipo N: terminal 1 SOURCE, terminal 2 GATE, terminal 3 DRAIN";

        NJFET = true;
        source = 1;
        gate = 2;
        drain = 3;
        ++ndetecciones;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////
  ////////////// en caso de JFET el terminal C es la GATE  /////////////////////////
  //////////////////////////////////////////////////////////////////////////////////

  DACset(0, 0); // si aplico una tensión de 4.9 v al canal A tengo 2 v en el terminal de la gate
  DACset(1, 0); // D y S a 0 v
  DACset(2, 2000);

  VA2 = mideTerminal(TERMINAL_A2);
  VB2 = mideTerminal(TERMINAL_B2);
  VC2 = mideTerminal(TERMINAL_C2);

  Vdif1 = VC2 - VA2;
  Vdif2 = VC2 - VB2;

  if (Vdif1 > 0.5 & Vdif2 > 0.5)
  { // aqui doy por hecho que si se cumple las diferencia de Vdif1 respecto a Vdif2 son iguales

    if (VA2 > 0.5 & VB2 > 0.5) // PUEDE SER UN BJT NPN O UN JFET TIPO N
    {
      DACset(0, 2000);
      DACset(1, 0);
      DACset(2, 0);

      VB2 = mideTerminal(TERMINAL_B2);

      if (VC2 > 0.2)
      { // si tenemos tensión en ese canas el que conduce por lo tanto se trata de un jfet tipo y no de un bjt tipo N

        Serial.println("JFET tipo N: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE");
        deteccion = "JFET tipo N: terminal 1 SOURCE, terminal 2 DRAIN, terminal 3 GATE";

        NJFET = true;
        source = 1;
        gate = 3;
        drain = 2;
        ++ndetecciones;
      }
      ////////////// podría comprovar que la caida es igual en ambos sentidos
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     DAC set   saca un valor por el DAC
/////////////////////////////////////////////////////////////////////////////////////////

void DACset(byte iDAC, int valorDAC)
{

  Wire.beginTransmission(direcDAC); // transmito la direccion
  Wire.write(trama1DAC | ((dacSelect[iDAC]) << 1));
  Wire.write(trama2DAC | ((valorDAC >> 8) & 0b1111));
  Wire.write((valorDAC & 0b11111111));
  Wire.endTransmission(); // stop transmitting
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     POT set   activa el potenciometro digital
/////////////////////////////////////////////////////////////////////////////////////////
void POTset(byte iPOT, byte datPOT)
{

  Wire.beginTransmission(direcPOT); // transmito la direccion
  Wire.write((channelPOT[iPOT] << 5) & 0b01100000);
  Wire.write(datPOT);
  Wire.endTransmission(); // stop transmitting
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     mideterminal
/////////////////////////////////////////////////////////////////////////////////////////

float mideTerminal(byte canal)
{

  if (canal & 1)
    digitalWrite(5, HIGH);
  else
    digitalWrite(5, LOW);
  if (canal & 2)
    digitalWrite(4, HIGH);
  else
    digitalWrite(4, LOW);
  if (canal & 4)
    digitalWrite(12, HIGH);
  else
    digitalWrite(12, LOW);

  delayMicroseconds(5); //3us de slew rate en el peor caso + 1us de Testablecimiento 5us por seguridad
  average = 0;
  //    for (int i = 0; i < 194; i++) {
  //    average = average + ((float)analogRead(analogInPin) / 102.4);
  //  }

  for (int i = 0; i < 19; i++)
  {
    average = average + ((float)analogRead(analogInPin) / 102.4);
  }

  return (average / 19.0); //esta incluido el factor 10 del divisor de tension
  // habria que filtrat 194 muestras para que se realizara en un ciclo de reloj y de esta manera evitar el ruido de la alimentacion
  // voy a promediar con menos ya que son demasiados valores
  //promedio con 19
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     measureI mide la intensidad
/////////////////////////////////////////////////////////////////////////////////////////

float measureI(byte canal1, byte canal2, float POTresist)
{
  float v1 = 0;
  float v2 = 0;
  v1 = mideTerminal(canal1);
  v2 = mideTerminal(canal2);

  return ((v1 - v2) * 1000 / POTresist);
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     POT resist
/////////////////////////////////////////////////////////////////////////////////////////

float POTresist(int D)
{

  return (((256 - float(D)) / 256) * 200000 + 120);
}

//////////////////////////////////////////////////////////////////////////////////////////
///////                     POTvalor
/////////////////////////////////////////////////////////////////////////////////////////
//funcion para que me calcule el valor que le tengo que dar a POT para conseguir la R que deseo
float POTvalor(int POTres)
{

  return 256 - ((120 + POTres) * 256 / 200000);
}
