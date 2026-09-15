/*
 * ============================================================
 * CHOCADEIRA - CONTROLE V1.6
 * ESP32-WROOM-32E
 *
 * V1.6:
 * - SHT31 / DS3231 / OLED SSD1306
 * - Perfis: CODORNA, GALINHA, PAVAO, PATO, CUSTOM
 * - Controle de umidade por bomba (GPIO25)
 * - PersistÃªncia da incubaÃ§Ã£o em NVS
 * - PersistÃªncia do perfil CUSTOM em NVS
 * - Wi-Fi:
 *     1) Sem rede configurada -> AP CHOCADEIRA-XXXX
 *     2) Acesse http://192.168.4.1
 *     3) Escolha a rede e informe a senha
 *     4) ESP32 salva e reinicia
 *     5) Conecta como STA e disponibiliza http://chocadeira.local
 * - Dashboard / IncubaÃ§Ã£o / ConfiguraÃ§Ã£o / DiagnÃ³stico
 * - API HTTP
 * - GrÃ¡fico de temperatura e umidade em tempo real
 * - Controle local continua funcionando sem Wi-Fi
 *
 * Hardware:
 * SHT31  : SDA GPIO21 / SCL GPIO22 / 0x44
 * DS3231 : SDA GPIO21 / SCL GPIO22 / 0x68
 * OLED   : SDA GPIO21 / SCL GPIO22 / 0x3C
 * Bomba  : GPIO25
 *
 * IMPORTANTE:
 * GPIO25 Ã© tratado como saÃ­da para a bomba.
 * Aquecedor, ventilador e motor de viragem nÃ£o sÃ£o acionados
 * fisicamente nesta versÃ£o.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>

// ---------------- HARDWARE ----------------
#define I2C_SDA       21
#define I2C_SCL       22
#define PUMP_PIN      25

#define SHT31_ADDR    0x44
#define RTC_ADDR      0x68
#define OLED_ADDR     0x3C

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ---------------- LIMITES ----------------
#define MAX_PHASES       8
#define MIN_PULSE        100UL
#define MAX_PULSE        10000UL
#define MIN_LOCKOUT      5000UL
#define MAX_LOCKOUT      600000UL
#define SENSOR_TIMEOUT   5000UL
#define SENSOR_INTERVAL  1000UL
#define OLED_INTERVAL    500UL
#define OLED_PAGE_INTERVAL 5000UL

#define MIN_RH_TARGET    30.0f
#define MAX_RH_TARGET    90.0f
#define MIN_HYSTERESIS   0.5f
#define MAX_HYSTERESIS   10.0f
#define MIN_RH_MAX       60.0f
#define MAX_RH_MAX       95.0f
#define MIN_TEMP         30.0f
#define MAX_TEMP         40.0f

// ---------------- OBJETOS ----------------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
RTC_DS3231 rtc;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Preferences prefs;
Preferences wifiPrefs;
Preferences customPrefs;

WebServer server(80);
DNSServer dnsServer;

// ------------------------------------------------------------
// FLAGS DOS HARDWARES
// ------------------------------------------------------------

bool shtOk = false;
bool rtcOk = false;
bool oledOk = false;
bool mdnsStarted = false;

// ---------------- ESTADOS ----------------
enum Profile {
  PROFILE_CODORNA = 0,
  PROFILE_GALINHA,
  PROFILE_PAVAO,
  PROFILE_PATO,
  PROFILE_CUSTOM
};

enum ControlState {
  STATE_OK = 0,
  STATE_SENSOR_ERROR,
  STATE_RH_MAX,
  STATE_LOCKOUT,
  STATE_PUMPING
};

struct Phase {
  int startDay;
  int endDay;
  float temp;
  float rh;
  float hyst;
  unsigned long pulse;
  unsigned long lockout;
  float rhMax;
  bool turning;
};

struct ProfileData {
  const char* name;
  int totalDays;
  Phase phases[MAX_PHASES];
  int phaseCount;
};

ProfileData profiles[5];

Profile selectedProfile = PROFILE_GALINHA;
ProfileData customProfile;

uint32_t guardBeforeIncubation = 0x12345678;

bool incubationActive = false;
bool lastIncubationActive = false;

uint32_t guardAfterIncubation = 0x87654321;

uint32_t incubationStartEpoch = 0;
int currentPhaseIndex = -1;

bool autoControl = true;
bool pumpState = false;

float currentTemp = NAN;
float currentRH = NAN;

float targetTemp = 37.5;
float targetRH = 50.0;
float hysteresis = 2.0;
float rhMax = 65.0;

unsigned long pumpPulseMs = 2000;
unsigned long pumpLockoutMs = 60000;

unsigned long pumpStartedAt = 0;
unsigned long currentPumpDuration = 0;
unsigned long lastPumpAt = 0;
unsigned long lastSensorRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastOLEDPageChange = 0;
uint8_t oledPage = 0;

ControlState controlState = STATE_OK;

bool wifiConfigured = false;
bool apMode = false;
bool staConnecting = false;
unsigned long staStartedAt = 0;
String savedSSID;
String savedPassword;
String apSSID;

IPAddress staIP;

void displayText(const String& a, const String& b, const String& c, const String& d);

// ---------------- HTML PRINCIPAL ----------------
const char MAIN_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">

<title>Chocadeira ESP32</title>

<style>
*{box-sizing:border-box}

body{
  margin:0;
  font-family:Arial,sans-serif;
  background:#f3f4f6;
  color:#202124
}

header{
  background:#202124;
  color:white;
  padding:14px 18px;
  position:sticky;
  top:0;
  z-index:2
}

header h1{
  font-size:20px;
  margin:0
}

nav{
  display:flex;
  gap:6px;
  overflow:auto;
  padding:8px;
  background:white;
  border-bottom:1px solid #ddd
}

nav button{
  border:0;
  background:#eee;
  border-radius:8px;
  padding:10px 14px;
  white-space:nowrap
}

nav button.active{
  background:#222;
  color:white
}

main{
  max-width:1100px;
  margin:auto;
  padding:14px
}

section{
  display:none
}

section.active{
  display:block
}

.grid{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(180px,1fr));
  gap:12px
}

.card{
  background:white;
  border-radius:12px;
  padding:16px;
  box-shadow:0 1px 4px #0002
}

.value{
  font-size:30px;
  font-weight:bold
}

.small{
  font-size:13px;
  color:#666
}

button,.btn{
  cursor:pointer;
  border:0;
  border-radius:8px;
  padding:10px 14px;
  background:#333;
  color:white;
  margin:3px
}

button.secondary{
  background:#777
}

button.danger{
  background:#b3261e
}

input,select{
  width:100%;
  padding:10px;
  border:1px solid #bbb;
  border-radius:7px;
  margin:5px 0 10px
}

.row{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
  gap:10px
}

table{
  width:100%;
  border-collapse:collapse;
  background:white
}

td,th{
  padding:8px;
  border-bottom:1px solid #ddd;
  text-align:left;
  font-size:13px
}

canvas{
  width:100%;
  height:260px;
  border:1px solid #ddd;
  border-radius:8px;
  background:white
}

#toast{
  position:fixed;
  right:15px;
  bottom:15px;
  background:#222;
  color:white;
  padding:12px 16px;
  border-radius:8px;
  display:none
}
</style>
</head>

<body>

<header>
<h1>🐣 Chocadeira ESP32 V1.6</h1>
<div id="net" class="small">conectando...</div>
</header>

<nav>
<button data-tab="dash">Dashboard</button>
<button data-tab="inc">Incubação</button>
<button data-tab="cfg">Configuração</button>
<button data-tab="diag">Diagnóstico</button>
</nav>

<main>

<section id="dash">

<div class="grid">

<div class="card">
<div class="small">Temperatura</div>
<div id="temp" class="value">--</div>
</div>

<div class="card">
<div class="small">Umidade</div>
<div id="rh" class="value">--</div>
</div>

<div class="card">
<div class="small">Dia de incubação</div>
<div id="day" class="value">--</div>
</div>

<div class="card">
<div class="small">Fase</div>
<div id="phase" class="value">--</div>
</div>

<div class="card">
<div class="small">Perfil</div>
<div id="profile" class="value"
style="font-size:20px">--</div>
</div>

<div class="card">
<div class="small">Bomba</div>
<div id="pump" class="value"
style="font-size:20px">--</div>
</div>

</div>

<br>

<div class="card">

<h3>Estado</h3>

<div id="state">--</div>

<p class="small">
Alvo:
<span id="targets">--</span>
</p>

<p class="small">
RTC:
<span id="rtc">--</span>
</p>

</div>

<br>

<div class="card">

<h3>Gráfico em tempo real</h3>

<canvas id="chart"
width="1000"
height="260"></canvas>

</div>

</section>


<section id="inc">

<div class="card">

<h2>Incubação</h2>

<div class="row">

<div>
<label>Perfil</label>

<select id="selProfile">

<option value="0">CODORNA</option>
<option value="1">GALINHA</option>
<option value="2">PAVAO</option>
<option value="3">PATO</option>
<option value="4">CUSTOM</option>

</select>
</div>

<div>

<label>Data/hora de início</label>

<input id="startDate"
type="datetime-local">

</div>

</div>

<div class="row">

<button onclick="startInc()">
Iniciar agora
</button>

<button onclick="startIncDate()">
Iniciar pela data
</button>

<button class="secondary"
onclick="cmd('INCUBACAO PARAR')">
Parar
</button>

<button class="danger"
onclick="cmd('INCUBACAO RESET')">
Reset
</button>

</div>

<p class="small">
Ao iniciar, o dia de incubação é calculado pelo DS3231.
</p>

</div>

</section>


<section id="cfg">

<div class="card">

<h2>Configuração</h2>

<div class="row">

<div>
<label>Temperatura °C</label>
<input id="cTemp"
type="number"
step="0.1">
</div>

<div>
<label>Umidade %</label>
<input id="cRH"
type="number"
step="0.1">
</div>

<div>
<label>Histerese %</label>
<input id="cHyst"
type="number"
step="0.1">
</div>

<div>
<label>RH máximo %</label>
<input id="cRHMax"
type="number"
step="0.1">
</div>

<div>
<label>Pulso bomba ms</label>
<input id="cPulse"
type="number">
</div>

<div>
<label>Lockout ms</label>
<input id="cLockout"
type="number">
</div>

<div>
<label>Viragem</label>
<select id="cTurning"><option value="1">ON</option><option value="0">OFF</option></select>
</div>

</div>

<button onclick="saveCfg()">
Salvar parâmetros da fase atual
</button>

<button class="secondary"
onclick="cmd('AUTO ON')">
Controle AUTO ON
</button>

<button class="secondary"
onclick="cmd('AUTO OFF')">
Controle AUTO OFF
</button>

</div>


<br>


<div class="card">

<h2>Perfil CUSTOM</h2>

<div class="row">

<div>
<label>Total de dias</label>
<input id="customDays"
type="number"
min="1"
max="200">
</div>

<div>
<label>Quantidade de fases</label>
<input id="phaseCount"
type="number"
min="1"
max="8">
</div>

</div>

<div id="phaseEditor"></div>

<button onclick="loadCustom()">
Carregar CUSTOM
</button>

<button onclick="saveCustom()">
Salvar CUSTOM
</button>

<button class="danger"
onclick="cmd('CUSTOM CLEAR')">
Limpar CUSTOM
</button>

</div>


<br>


<div class="card">

<h2>Relógio</h2>

<input id="rtcDate"
type="datetime-local">

<button onclick="setRTC()">
Ajustar RTC
</button>

</div>

</section>


<section id="diag">

<div class="card">

<h2>Diagnóstico</h2>

<table>
<tbody id="diagTable"></tbody>
</table>

</div>

<br>

<div class="card">

<h2>Rede Wi-Fi</h2>

<button onclick="scanWifi()">
Escanear redes
</button>

<div id="wifiList"></div>

</div>

</section>

</main>

<div id="toast"></div>


<script>

let data=null;
let histT=[];
let histR=[];
let histMax=60;


document.querySelectorAll('nav button').forEach(b=>{

 b.onclick=()=>{

  document.querySelectorAll('section')
  .forEach(s=>s.classList.remove('active'));

  document.querySelectorAll('nav button')
  .forEach(x=>x.classList.remove('active'));

  document.getElementById(b.dataset.tab)
  .classList.add('active');

  b.classList.add('active');

  if(b.dataset.tab==='cfg')
    loadConfig();

  if(b.dataset.tab==='diag')
    loadDiag();

 }

});


document.querySelector('nav button').click();


function toast(s){

 let e=document.getElementById('toast');

 e.textContent=s;
 e.style.display='block';

 setTimeout(()=>{
   e.style.display='none'
 },2200);

}


async function api(url,opt={}){

 let r=await fetch(url,opt);

 return await r.json();

}


function post(url,obj){

 return api(url,{
   method:'POST',
   headers:{
     'Content-Type':'application/json'
   },
   body:JSON.stringify(obj)
 });

}


async function update(){

 try{

  data=await api('/api/status');

  document.getElementById('temp')
  .textContent=
  isFinite(data.temp)
  ?data.temp.toFixed(1)+' °C'
  :'--';

  document.getElementById('rh')
  .textContent=
  isFinite(data.rh)
  ?data.rh.toFixed(1)+' %'
  :'--';

  document.getElementById('day')
  .textContent=
  data.day>0
  ?data.day
  :'Parada';

  document.getElementById('phase')
  .textContent=data.phase;

  document.getElementById('profile')
  .textContent=data.profile;

  document.getElementById('pump')
  .textContent=
  data.pump
  ?'LIGADA'
  :'DESLIGADA';

  document.getElementById('state')
  .textContent=data.state;

  document.getElementById('targets')
  .textContent=
  data.targetTemp.toFixed(1)
  +' °C / '
  +data.targetRH.toFixed(1)
  +' % RH';

  document.getElementById('rtc')
  .textContent=data.rtc;

  document.getElementById('net')
  .textContent=
  data.wifiMode
  +' | '
  +data.ip;

  if(isFinite(data.temp)){

   histT.push(data.temp);

   if(histT.length>histMax)
     histT.shift();

  }

  if(isFinite(data.rh)){

   histR.push(data.rh);

   if(histR.length>histMax)
     histR.shift();

  }

  drawChart();

 }catch(e){

  document.getElementById('net')
  .textContent='Sem comunicação';

 }

}


function drawChart(){

 let c=document.getElementById('chart');

 let x=c.getContext('2d');

 let w=c.width;
 let h=c.height;

 x.clearRect(0,0,w,h);

 x.strokeStyle='#ccc';
 x.lineWidth=1;

 for(let i=0;i<=5;i++){

  let yy=20+i*(h-40)/5;

  x.beginPath();

  x.moveTo(35,yy);
  x.lineTo(w-10,yy);

  x.stroke();

 }


 function plot(a,min,max,off,scale){

  if(a.length<2)return;

  x.beginPath();

  a.forEach((v,i)=>{

   let xx=
   35+i*(w-45)/(histMax-1);

   let yy=
   off+(max-v)*(scale);

   if(i)
     x.lineTo(xx,yy);
   else
     x.moveTo(xx,yy);

  });

  x.stroke();

 }


 x.strokeStyle='#d33';

 plot(
   histT,
   30,
   40,
   20,
   (h-40)/10
 );


 x.strokeStyle='#2684d9';

 plot(
   histR,
   0,
   100,
   20,
   (h-40)/100
 );


 x.fillStyle='#222';

 x.fillText('Temp °C',45,15);
 x.fillText('RH %',100,15);

}


async function cmd(c){

 let r=
 await post(
   '/api/command',
   {cmd:c}
 );

 toast(r.message||'OK');

 update();

}


async function startInc(){

 let p=
 document.getElementById('selProfile')
 .value;

 let r=
 await post(
   '/api/incubation/start',
   {profile:Number(p)}
 );

 toast(r.message);

 update();

}


async function startIncDate(){

 let p=
 document.getElementById('selProfile')
 .value;

 let d=
 document.getElementById('startDate')
 .value;

 let r=
 await post(
   '/api/incubation/start',
   {
    profile:Number(p),
    datetime:d
   }
 );

 toast(r.message);

 update();

}


async function loadConfig(){

 let d=
 await api('/api/config');

 cTemp.value=d.temp;
 cRH.value=d.rh;
 cHyst.value=d.hyst;
 cRHMax.value=d.rhMax;
 cPulse.value=d.pulse;
 cLockout.value=d.lockout;
 cTurning.value=d.turning?1:0;

 selProfile.value=d.profile;

 loadCustom();

}


async function saveCfg(){

 let o={

  temp:+cTemp.value,
  rh:+cRH.value,
  hyst:+cHyst.value,
  rhMax:+cRHMax.value,
  pulse:+cPulse.value,
  lockout:+cLockout.value,
  turning:+cTurning.value

 };

 let r=
 await post('/api/config',o);

 toast(r.message);

 update();

}


async function loadCustom(){

 let d=
 await api('/api/custom');

 customDays.value=d.totalDays;

 phaseCount.value=d.phaseCount;

 renderPhases(d.phases);

}


phaseCount.onchange=
()=>{
 renderPhases([]);
};


function renderPhases(arr){

 let n=
 Math.max(
   1,
   Math.min(
     8,
     +phaseCount.value||1
   )
 );

 let html='';

 for(let i=0;i<n;i++){

  let p=
  arr[i]||
  {
   start:1,
   end:1,
   temp:37.5,
   rh:50,
   hyst:2,
   pulse:2000,
   lockout:60000,
   rhMax:65,
   turning:1
  };


  html+=`

  <div class="card">

  <b>Fase ${i+1}</b>

  <div class="row">

  <div>
  <label>Início</label>
  <input id="ps${i}"
  type="number"
  value="${p.start}">
  </div>

  <div>
  <label>Fim</label>
  <input id="pe${i}"
  type="number"
  value="${p.end}">
  </div>

  <div>
  <label>Temp</label>
  <input id="pt${i}"
  type="number"
  step=".1"
  value="${p.temp}">
  </div>

  <div>
  <label>RH</label>
  <input id="pr${i}"
  type="number"
  step=".1"
  value="${p.rh}">
  </div>

  <div>
  <label>Histerese</label>
  <input id="ph${i}"
  type="number"
  step=".1"
  value="${p.hyst}">
  </div>

  <div>
  <label>Pulso ms</label>
  <input id="pp${i}"
  type="number"
  value="${p.pulse}">
  </div>

  <div>
  <label>Lockout ms</label>
  <input id="pl${i}"
  type="number"
  value="${p.lockout}">
  </div>

  <div>
  <label>RH Max</label>
  <input id="pm${i}"
  type="number"
  step=".1"
  value="${p.rhMax}">
  </div>

  <div>
  <label>Viragem</label>

  <select id="pv${i}">

  <option value="1"
  ${p.turning?'selected':''}>
  ON
  </option>

  <option value="0"
  ${!p.turning?'selected':''}>
  OFF
  </option>

  </select>

  </div>

  </div>
  </div>

  `;

 }

 phaseEditor.innerHTML=html;

}


async function saveCustom(){

 let n=
 Math.max(
   1,
   Math.min(
     8,
     +phaseCount.value
   )
 );

 let ph=[];

 for(let i=0;i<n;i++)

  ph.push({

   start:+document.getElementById('ps'+i).value,

   end:+document.getElementById('pe'+i).value,

   temp:+document.getElementById('pt'+i).value,

   rh:+document.getElementById('pr'+i).value,

   hyst:+document.getElementById('ph'+i).value,

   pulse:+document.getElementById('pp'+i).value,

   lockout:+document.getElementById('pl'+i).value,

   rhMax:+document.getElementById('pm'+i).value,

   turning:+document.getElementById('pv'+i).value

  });


 let r=
 await post(
   '/api/custom',
   {
    totalDays:+customDays.value,
    phaseCount:n,
    phases:ph
   }
 );

 toast(r.message);

}


async function setRTC(){

 let d=
 document.getElementById('rtcDate').value;

 let r=
 await post(
   '/api/rtc',
   {datetime:d}
 );

 toast(r.message);

 update();

}


async function loadDiag(){

 let d=
 await api('/api/status');

 let rows=[

  ['Sensor SHT31',
   d.sht31?'OK':'ERRO'],

  ['RTC DS3231',
   d.rtcOk?'OK':'ERRO'],

  ['OLED',
   d.oled?'OK':'ERRO'],

  ['Wi-Fi',d.wifiMode],

  ['IP',d.ip],

  ['RSSI',d.rssi+' dBm'],

  ['SSID',d.ssid],

  ['Incubação',
   d.active?'ATIVA':'PARADA'],

  ['Controle',
   d.auto?'AUTO':'MANUAL'],

  ['Estado',d.state]

 ];

 diagTable.innerHTML=
 rows.map(
  r=>
  '<tr><th>'
  +r[0]
  +'</th><td>'
  +r[1]
  +'</td></tr>'
 ).join('');

}


async function scanWifi(){

 let d=
 await api('/api/wifi/scan');

 if(d.scanning){
  wifiList.innerHTML='Escaneando redes...';
  setTimeout(scanWifi,900);
  return;
 }

 wifiList.innerHTML=
 d.networks.map(
  n=>
  `<div class="card">
   <b>${n.ssid||'(oculta)'}</b>
  — ${n.rssi} dBm —
  ${n.encryption?'🔒':'aberta'}
  </div>`
 ).join('');

}


setInterval(update,2000);

update();

</script>

</body>
</html>
)HTML";


// ---------------- HTML CONFIGURAÃ‡ÃƒO WI-FI ----------------

const char WIFI_HTML[] PROGMEM = R"HTML(

<!DOCTYPE html>

<html lang="pt-BR">

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>Configuração Wi-Fi</title>

<style>

body{
font-family:Arial;
background:#f3f4f6;
margin:0;
padding:20px
}

.card{
max-width:600px;
margin:auto;
background:white;
padding:20px;
border-radius:12px;
box-shadow:0 2px 8px #0002
}

input,select,button{
width:100%;
padding:12px;
margin:7px 0;
border-radius:7px;
border:1px solid #bbb
}

button{
background:#222;
color:white;
border:0;
cursor:pointer
}

.net{
padding:10px;
border-bottom:1px solid #ddd;
cursor:pointer
}

</style>

</head>

<body>

<div class="card">

<h2>🐣 Configuração Wi-Fi</h2>

<p>
O ESP32 está em modo de configuração.
Escolha sua rede:
</p>

<button onclick="scan()">
Escanear redes
</button>

<div id="nets">
Aguardando...
</div>

<label>SSID</label>

<input id="ssid">

<label>Senha</label>

<input id="pass"
type="password">

<button onclick="save()">
Salvar e reiniciar
</button>

<p id="msg"></p>

</div>

<script>

async function scan(){

 let r=
 await fetch('/api/wifi/scan');

 let d=
 await r.json();

 if(d.scanning){
  nets.innerHTML='Escaneando redes...';
  setTimeout(scan,900);
  return;
 }

 nets.innerHTML=
 d.networks.map(
 n=>
 `<div class="net"
 onclick="ssid.value=${JSON.stringify(n.ssid)}">

 ${n.ssid||'(oculta)'}
 —
 ${n.rssi} dBm
 ${n.encryption?'🔒':''}

 </div>`
 ).join('');

}


async function save(){

 msg.textContent='Salvando...';

 let r=
 await fetch(
 '/api/wifi/save',
 {
  method:'POST',
  headers:{
   'Content-Type':
   'application/json'
  },
  body:JSON.stringify({
   ssid:ssid.value,
   password:pass.value
  })
 }
 );

 let d=
 await r.json();

 msg.textContent=d.message;

}


scan();

</script>

</body>
</html>

)HTML";


// ---------------- PERFIS ----------------

Phase makePhase(
  int s,
  int e,
  float t,
  float rh,
  float hy,
  unsigned long pulse,
  unsigned long lock,
  float maxrh,
  bool turn
) {

  Phase p{
    s,
    e,
    t,
    rh,
    hy,
    pulse,
    lock,
    maxrh,
    turn
  };

  return p;
}


void initProfiles() {

  profiles[PROFILE_CODORNA].name =
    "CODORNA";

  profiles[PROFILE_CODORNA].totalDays =
    17;

  profiles[PROFILE_CODORNA].phaseCount =
    2;

  profiles[PROFILE_CODORNA].phases[0] =
    makePhase(
      1,
      14,
      37.5,
      50,
      2,
      2000,
      60000,
      65,
      true
    );

  profiles[PROFILE_CODORNA].phases[1] =
    makePhase(
      15,
      17,
      37.2,
      68,
      2,
      2000,
      60000,
      80,
      false
    );


  profiles[PROFILE_GALINHA].name =
    "GALINHA";

  profiles[PROFILE_GALINHA].totalDays =
    21;

  profiles[PROFILE_GALINHA].phaseCount =
    2;

  profiles[PROFILE_GALINHA].phases[0] =
    makePhase(
      1,
      17,
      37.5,
      50,
      2,
      2000,
      60000,
      65,
      true
    );

  profiles[PROFILE_GALINHA].phases[1] =
    makePhase(
      18,
      21,
      37.2,
      68,
      2,
      2000,
      60000,
      80,
      false
    );


  profiles[PROFILE_PAVAO].name =
    "PAVAO";

  profiles[PROFILE_PAVAO].totalDays =
    28;

  profiles[PROFILE_PAVAO].phaseCount =
    2;

  profiles[PROFILE_PAVAO].phases[0] =
    makePhase(
      1,
      25,
      37.5,
      50,
      2,
      2000,
      60000,
      65,
      true
    );

  profiles[PROFILE_PAVAO].phases[1] =
    makePhase(
      26,
      28,
      37.2,
      68,
      2,
      2000,
      60000,
      80,
      false
    );


  profiles[PROFILE_PATO].name =
    "PATO";

  profiles[PROFILE_PATO].totalDays =
    28;

  profiles[PROFILE_PATO].phaseCount =
    2;

  profiles[PROFILE_PATO].phases[0] =
    makePhase(
      1,
      25,
      37.5,
      50,
      2,
      2000,
      60000,
      65,
      true
    );

  profiles[PROFILE_PATO].phases[1] =
    makePhase(
      26,
      28,
      37.2,
      68,
      2,
      2000,
      60000,
      80,
      false
    );


  customProfile.name =
    "CUSTOM";

  customProfile.totalDays =
    21;

  customProfile.phaseCount =
    2;

  customProfile.phases[0] =
    makePhase(
      1,
      17,
      37.5,
      50,
      2,
      2000,
      60000,
      65,
      true
    );

  customProfile.phases[1] =
    makePhase(
      18,
      21,
      37.2,
      68,
      2,
      2000,
      60000,
      80,
      false
    );

}


ProfileData& activeProfileData() {

  if(selectedProfile==PROFILE_CUSTOM)
    return customProfile;

  return profiles[selectedProfile];

}


// ---------------- NVS INCUBAÃ‡ÃƒO ----------------

void saveIncubation()
{
    Serial.println();
    Serial.println("[NVS] SALVANDO ESTADO INCUBACAO");
    Serial.print("[NVS] active = ");
    Serial.println(incubationActive ? "1" : "0");
    Serial.print("[NVS] start = ");
    Serial.println(incubationStartEpoch);
    Serial.print("[NVS] profile ID = ");
    Serial.println((int)selectedProfile);

    prefs.begin("chocadeira", false);

    prefs.putBool("active", incubationActive);
    prefs.putULong("start", incubationStartEpoch);
    prefs.putUChar("profile", (uint8_t)selectedProfile);

    prefs.end();
}


void loadIncubation() {

  prefs.begin(
    "chocadeira",
    true
  );

  incubationActive =
    prefs.getBool(
      "active",
      false
    );

  incubationStartEpoch =
    prefs.getULong(
      "start",
      0
    );

  selectedProfile =
    (Profile)prefs.getUChar(
      "profile",
      PROFILE_GALINHA
    );

  prefs.end();

  if(selectedProfile>PROFILE_CUSTOM)
    selectedProfile=PROFILE_GALINHA;

  if(incubationActive && incubationStartEpoch == 0) {
    Serial.println();
    Serial.println("!!! ORIGEM: loadIncubation() colocou incubationActive = false !!!");
    Serial.print("Start epoch: ");
    Serial.println(incubationStartEpoch);
    incubationActive=false;
    saveIncubation();
  }

}


void checkIncubationMemory()
{
  if(guardBeforeIncubation != 0x12345678 ||
     guardAfterIncubation != 0x87654321)
  {
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!!! CORRUPCAO DE MEMORIA DETECTADA !!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    Serial.print("guardBefore = 0x");
    Serial.println(guardBeforeIncubation, HEX);

    Serial.print("guardAfter  = 0x");
    Serial.println(guardAfterIncubation, HEX);

    Serial.print("incubationActive = ");
    Serial.println(incubationActive ? "1" : "0");

    Serial.print("Start epoch = ");
    Serial.println(incubationStartEpoch);

    Serial.println();
  }
}


void printResetDiagnostic()
{
  esp_reset_reason_t reason = esp_reset_reason();

  Serial.println();
  Serial.println("========== DIAGNOSTICO RESET ==========");

  Serial.print("Reset reason: ");
  Serial.println((int)reason);

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  Serial.print("Minimum free heap: ");
  Serial.println(ESP.getMinFreeHeap());

  Serial.print("Incubacao RAM: ");
  Serial.println(incubationActive ? "ATIVA" : "PARADA");

  Serial.print("Start epoch RAM: ");
  Serial.println(incubationStartEpoch);

  Serial.print("Perfil RAM: ");
  Serial.println((int)selectedProfile);

  Serial.println("=======================================");
}


// ---------------- NVS CUSTOM ----------------

void saveCustomProfile() {

  customPrefs.begin(
    "custom",
    false
  );

  customPrefs.putInt(
    "total",
    customProfile.totalDays
  );

  customPrefs.putInt(
    "count",
    customProfile.phaseCount
  );


  for(int i=0;i<MAX_PHASES;i++){

    String k;

    k="s"+String(i);
    customPrefs.putInt(
      k.c_str(),
      customProfile.phases[i].startDay
    );

    k="e"+String(i);
    customPrefs.putInt(
      k.c_str(),
      customProfile.phases[i].endDay
    );

    k="t"+String(i);
    customPrefs.putFloat(
      k.c_str(),
      customProfile.phases[i].temp
    );

    k="r"+String(i);
    customPrefs.putFloat(
      k.c_str(),
      customProfile.phases[i].rh
    );

    k="h"+String(i);
    customPrefs.putFloat(
      k.c_str(),
      customProfile.phases[i].hyst
    );

    k="p"+String(i);
    customPrefs.putULong(
      k.c_str(),
      customProfile.phases[i].pulse
    );

    k="l"+String(i);
    customPrefs.putULong(
      k.c_str(),
      customProfile.phases[i].lockout
    );

    k="m"+String(i);
    customPrefs.putFloat(
      k.c_str(),
      customProfile.phases[i].rhMax
    );

    k="v"+String(i);
    customPrefs.putBool(
      k.c_str(),
      customProfile.phases[i].turning
    );

  }

  customPrefs.end();

}


void loadCustomProfile() {

  customPrefs.begin(
    "custom",
    true
  );

  bool exists =
    customPrefs.isKey("count");

  if(exists){

    customProfile.totalDays =
      customPrefs.getInt(
        "total",
        21
      );

    customProfile.phaseCount =
      customPrefs.getInt(
        "count",
        2
      );

    if(
      customProfile.phaseCount<1 ||
      customProfile.phaseCount>MAX_PHASES
    )
      customProfile.phaseCount=2;


    for(int i=0;i<MAX_PHASES;i++){

      String k;

      k="s"+String(i);
      customProfile.phases[i].startDay =
        customPrefs.getInt(
          k.c_str(),
          1
        );

      k="e"+String(i);
      customProfile.phases[i].endDay =
        customPrefs.getInt(
          k.c_str(),
          1
        );

      k="t"+String(i);
      customProfile.phases[i].temp =
        customPrefs.getFloat(
          k.c_str(),
          37.5
        );

      k="r"+String(i);
      customProfile.phases[i].rh =
        customPrefs.getFloat(
          k.c_str(),
          50
        );

      k="h"+String(i);
      customProfile.phases[i].hyst =
        customPrefs.getFloat(
          k.c_str(),
          2
        );

      k="p"+String(i);
      customProfile.phases[i].pulse =
        customPrefs.getULong(
          k.c_str(),
          2000
        );

      k="l"+String(i);
      customProfile.phases[i].lockout =
        customPrefs.getULong(
          k.c_str(),
          60000
        );

      k="m"+String(i);
      customProfile.phases[i].rhMax =
        customPrefs.getFloat(
          k.c_str(),
          65
        );

      k="v"+String(i);
      customProfile.phases[i].turning =
        customPrefs.getBool(
          k.c_str(),
          true
        );

    }

  }

  customPrefs.end();

}


// ---------------- INCUBACAO ----------------

int incubationDay() {

  if(
    !incubationActive ||
    !rtcOk ||
    incubationStartEpoch==0
  )
    return 0;


  DateTime now=
    rtc.now();


  int64_t diff=
    (int64_t)now.unixtime() -
    (int64_t)incubationStartEpoch;


  if(diff<0)
    return 0;


  return
    (int)(diff/86400L)+1;

}


  void monitorIncubationState()
  {
    if(incubationActive != lastIncubationActive)
    {
      Serial.println();
      Serial.println("========== MUDANCA ESTADO INCUBACAO ==========");

      Serial.print("Anterior: ");
      Serial.println(lastIncubationActive ? "ATIVA" : "PARADA");

      Serial.print("Novo: ");
      Serial.println(incubationActive ? "ATIVA" : "PARADA");

      Serial.print("Dia: ");
      Serial.println(incubationDay());

      Serial.print("Start epoch: ");
      Serial.println(incubationStartEpoch);

      Serial.print("Perfil ID: ");
      Serial.println((int)selectedProfile);

      Serial.println("==============================================");

      lastIncubationActive = incubationActive;
    }
  }


void applyCurrentPhase() {

  if(!incubationActive)
    return;

  ProfileData &p=
    activeProfileData();

  int day = incubationDay();
  if(day <= 0)
    return;

  int idx = -1;
  for(int i=0; i<p.phaseCount; i++) {
    if(day >= p.phases[i].startDay && day <= p.phases[i].endDay) {
      idx = i;
      break;
    }
  }

  if(idx < 0 || idx == currentPhaseIndex)
    return;

  Phase &ph=
    p.phases[idx];

  currentPhaseIndex = idx;


  targetTemp=
    ph.temp;

  targetRH=
    ph.rh;

  hysteresis=
    ph.hyst;

  pumpPulseMs=
    constrain(
      ph.pulse,
      MIN_PULSE,
      MAX_PULSE
    );
  pumpLockoutMs=
    constrain(
      ph.lockout,
      (unsigned long)MIN_LOCKOUT,
      (unsigned long)MAX_LOCKOUT
    );

  rhMax=
    constrain(
      ph.rhMax,
      MIN_RH_MAX,
      MAX_RH_MAX
    );

}


void restorePhase1Parameters() {
  ProfileData &p = activeProfileData();
  if(p.phaseCount < 1)
    return;

  Phase &ph = p.phases[0];
  targetTemp = ph.temp;
  targetRH = ph.rh;
  hysteresis = ph.hyst;
  rhMax = ph.rhMax;
  pumpPulseMs = constrain(ph.pulse, MIN_PULSE, MAX_PULSE);
  pumpLockoutMs = constrain(ph.lockout, MIN_LOCKOUT, MAX_LOCKOUT);
}

void pumpOff() {

  Serial.println();
  Serial.println("[DIAG] PUMP GPIO -> LOW");
  Serial.print("[DIAG] incubationActive = ");
  Serial.println(incubationActive ? "1" : "0");
  Serial.print("[DIAG] Free heap = ");
  Serial.println(ESP.getFreeHeap());

  digitalWrite(
    PUMP_PIN,
    LOW
  );

  pumpState=false;

}


void pumpOn(unsigned long duration = 0) {

  if(pumpState)
    return;

  if(!isnan(currentRH) && currentRH >= rhMax) {

    controlState=
      STATE_RH_MAX;

    return;
  }

  if(duration == 0)
    duration = pumpPulseMs;
  currentPumpDuration = constrain(duration, MIN_PULSE, MAX_PULSE);


  Serial.println();
  Serial.println("[DIAG] PUMP GPIO -> HIGH");
  Serial.print("[DIAG] incubationActive = ");
  Serial.println(incubationActive ? "1" : "0");
  Serial.print("[DIAG] Free heap = ");
  Serial.println(ESP.getFreeHeap());

  digitalWrite(
    PUMP_PIN,
    HIGH
  );

  pumpState=true;

  pumpStartedAt=
    millis();

  controlState=
    STATE_PUMPING;

}

void startIncubation(
  uint32_t startEpoch,
  Profile p
) {

  if(incubationActive || !rtcOk)
    return;


  selectedProfile=p;

  incubationStartEpoch=
    startEpoch;

  if(startEpoch > rtc.now().unixtime())
    return;

  incubationActive=true;
  currentPhaseIndex=-1;

  saveIncubation();

  applyCurrentPhase();

}


void stopIncubation()
{
    Serial.println();
    Serial.println("!!!!!!!! STOP INCUBACAO CHAMADO !!!!!!!!");
    Serial.print("Dia atual: ");
    Serial.println(incubationDay());
    Serial.print("Start epoch: ");
    Serial.println(incubationStartEpoch);
    Serial.print("Perfil ID: ");
    Serial.println((int)selectedProfile);

    Serial.println();
    Serial.println("!!! ORIGEM: stopIncubation() colocou incubationActive = false !!!");
    Serial.print("Dia atual: ");
    Serial.println(incubationDay());
    Serial.print("Start epoch: ");
    Serial.println(incubationStartEpoch);
    Serial.print("Perfil ID: ");
    Serial.println((int)selectedProfile);

    incubationActive = false;

    pumpOff();
    currentPhaseIndex = -1;
    controlState = STATE_OK;
    restorePhase1Parameters();

    saveIncubation();
}


void resetIncubation() {

  Serial.println();
  Serial.println("!!! ORIGEM: resetIncubation() colocou incubationActive = false !!!");
  Serial.print("Start epoch antes do reset: ");
  Serial.println(incubationStartEpoch);
  Serial.print("Perfil ID: ");
  Serial.println((int)selectedProfile);

  incubationActive=false;

  incubationStartEpoch=0;

  pumpOff();
  currentPhaseIndex=-1;
  controlState=STATE_OK;
  restorePhase1Parameters();

  saveIncubation();

}


// ============================================================
// SERIAL
// ============================================================

String profileName(Profile p) {

  if(p == PROFILE_CUSTOM)
    return "CUSTOM";

  return profiles[p].name;
}


// ------------------------------------------------------------

void scanI2C() {

  uint8_t deviceCount = 0;

  Serial.println(F("========================================"));
  Serial.println(F("              I2C SCAN"));
  Serial.println(F("========================================"));
  Serial.println(F("SDA: GPIO21"));
  Serial.println(F("SCL: GPIO22"));
  Serial.println();

  for(uint8_t address = 0x01; address <= 0x7F; address++) {

    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if(error == 0) {

      deviceCount++;

      Serial.println(F("Dispositivo encontrado:"));
      Serial.print(F("  Endereco: 0x"));
      if(address < 0x10)
        Serial.print('0');
      Serial.print(address, HEX);
      Serial.print(F(" ("));
      Serial.print(address);
      Serial.print(F(") -> "));

      if(address == OLED_ADDR)
        Serial.println(F("OLED SSD1306"));
      else if(address == SHT31_ADDR)
        Serial.println(F("SHT31"));
      else if(address == RTC_ADDR)
        Serial.println(F("DS3231"));
      else
        Serial.println(F("DESCONHECIDO"));
    }
  }

  if(deviceCount == 0)
    Serial.println(F("Nenhum dispositivo I2C encontrado."));

  Serial.println();
  Serial.print(F("Total: "));
  Serial.print(deviceCount);
  Serial.println(F(" dispositivo(s)"));
  Serial.println(F("========================================"));
}


// ------------------------------------------------------------

void diagnoseNVS() {

  prefs.begin(
    "chocadeira",
    true
  );

  bool activeFound = prefs.isKey("active");
  bool startFound = prefs.isKey("start");
  bool profileFound = prefs.isKey("profile");

  bool active = false;
  unsigned long startEpoch = 0;
  uint8_t profileValue = PROFILE_GALINHA;

  if(activeFound)
    active = prefs.getBool("active");

  if(startFound)
    startEpoch = prefs.getULong("start");

  if(profileFound)
    profileValue = prefs.getUChar("profile");

  Serial.println(F("========================================"));
  Serial.println(F("              NVS DIAGNOSTICO"));
  Serial.println(F("========================================"));
  Serial.println();

  if(activeFound)
    Serial.printf("Active: %d\n", active ? 1 : 0);
  else
    Serial.println(F("Chave active: NAO ENCONTRADA"));

  if(startFound)
    Serial.printf("Start epoch: %lu\n", startEpoch);
  else
    Serial.println(F("Chave start: NAO ENCONTRADA"));

  if(profileFound) {
    Serial.print(F("Profile: "));
    if(profileValue <= PROFILE_CUSTOM)
      Serial.println(profileName((Profile)profileValue));
    else
      Serial.println(F("DESCONHECIDO"));
  }
  else
    Serial.println(F("Chave profile: NAO ENCONTRADA"));

  if(activeFound && active && startFound && startEpoch != 0) {
    DateTime savedStart(startEpoch);

    Serial.println();
    Serial.println(F("Inicio salvo:"));
    Serial.printf(
      "%02d/%02d/%04d %02d:%02d:%02d\n",
      savedStart.day(),
      savedStart.month(),
      savedStart.year(),
      savedStart.hour(),
      savedStart.minute(),
      savedStart.second()
    );
  }

  Serial.println();
  Serial.println(F("Estado NVS:"));
  Serial.println(
    activeFound && active
      ? F("ATIVA")
      : F("PARADA")
  );
  Serial.println(F("========================================"));

  prefs.end();
}


// ------------------------------------------------------------

void serialHelp() {

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("        CHOCADEIRA ESP32 - V1.6"));
  Serial.println(F("================================================"));

  Serial.println(F("STATUS"));
  Serial.println(F("HELP"));
  Serial.println(F("I2C SCAN    - Varre o barramento I2C"));
  Serial.println(F("NVS    - Diagnostico somente leitura da memoria NVS"));
  Serial.println(F("PROFILES"));
  Serial.println(F("PROFILE CODORNA"));
  Serial.println(F("PROFILE GALINHA"));
  Serial.println(F("PROFILE PAVAO"));
  Serial.println(F("PROFILE PATO"));
  Serial.println(F("PROFILE CUSTOM"));
  Serial.println(F("PHASES"));

  Serial.println(F("INCUBACAO INICIAR"));
  Serial.println(F("INCUBACAO INICIAR DD/MM/YYYY HH:MM:SS"));
  Serial.println(F("INCUBACAO PARAR"));
  Serial.println(F("INCUBACAO RESET"));

  Serial.println(F("RTC"));
  Serial.println(F("RTC SET DD/MM/YYYY HH:MM:SS"));

  Serial.println(F("PUMP ON"));
  Serial.println(F("PUMP OFF"));
  Serial.println(F("PUMP <milliseconds>"));

  Serial.println(F("AUTO ON"));
  Serial.println(F("AUTO OFF"));

  Serial.println(F("SET RH <value>"));
  Serial.println(F("SET HYST <value>"));
  Serial.println(F("SET PULSE <ms>"));
  Serial.println(F("SET LOCKOUT <ms>"));
  Serial.println(F("SET RHMAX <value>"));
  Serial.println(F("SET TEMP <value>"));

  Serial.println(F("CUSTOM TOTAL <days>"));
  Serial.println(F("CUSTOM CLEAR"));

  Serial.println(
    F("CUSTOM PHASE <fase> <inicio> <fim> "
      "<temp> <rh> <hyst> <pulse> "
      "<lockout> <rhmax> <turning>")
  );

  Serial.println();
}


// ------------------------------------------------------------

bool parseDateTime(
  String s,
  DateTime &dt
) {

  s.trim();

  if(s.length() < 19)
    return false;


  int day =
    s.substring(0,2).toInt();

  int month =
    s.substring(3,5).toInt();

  int year =
    s.substring(6,10).toInt();

  int hour =
    s.substring(11,13).toInt();

  int minute =
    s.substring(14,16).toInt();

  int second =
    s.substring(17,19).toInt();


  if(
    year < 2020 ||
    month < 1 ||
    month > 12 ||
    day < 1 ||
    day > 31 ||
    hour < 0 ||
    hour > 23 ||
    minute < 0 ||
    minute > 59 ||
    second < 0 ||
    second > 59
  ) {

    return false;

  }


  dt =
    DateTime(
      year,
      month,
      day,
      hour,
      minute,
      second
    );


  return true;
}


// ------------------------------------------------------------


String stateText() {

  switch(
    controlState
  ) {

    case STATE_SENSOR_ERROR:
      return "ERRO SENSOR";

    case STATE_RH_MAX:
      return "RH MAXIMO";

    case STATE_LOCKOUT:
      return "LOCKOUT";

    case STATE_PUMPING:
      return "BOMBEANDO";

    default:
      return "NORMAL";

  }

}


// ------------------------------------------------------------


void printStatus() {

  Serial.println();
  Serial.println(F("============== STATUS =============="));

  Serial.print(F("Perfil: "));
  Serial.println(
    profileName(selectedProfile)
  );

  Serial.print(F("Incubacao: "));
  Serial.println(
    incubationActive
    ? F("ATIVA")
    : F("PARADA")
  );

  Serial.print(F("Dia: "));
  Serial.println(
    incubationDay()
  );

  Serial.print(F("Fase: "));
  Serial.println(
    (currentPhaseIndex + 1)
  );

  Serial.print(F("Temperatura: "));
  Serial.println(
    currentTemp
  );

  Serial.print(F("Umidade: "));
  Serial.println(
    currentRH
  );

  Serial.print(F("Alvo temperatura: "));
  Serial.println(
    targetTemp
  );

  Serial.print(F("Alvo umidade: "));
  Serial.println(
    targetRH
  );

  Serial.print(F("Histerese: "));
  Serial.println(
    hysteresis
  );

  Serial.print(F("RH Max: "));
  Serial.println(
    rhMax
  );

  Serial.print(F("Pulso bomba: "));
  Serial.println(
    pumpPulseMs
  );

  Serial.print(F("Lockout: "));
  Serial.println(
    pumpLockoutMs
  );

  Serial.print(F("Bomba: "));
  Serial.println(
    pumpState
    ? F("ON")
    : F("OFF")
  );

  Serial.print(F("Controle automatico: "));
  Serial.println(
    autoControl
    ? F("ON")
    : F("OFF")
  );

  Serial.print(F("Estado: "));
  Serial.println(
    stateText()
  );

  Serial.print(F("Wi-Fi: "));

  if(apMode)
    Serial.println(F("AP"));

  else if(
    WiFi.status() ==
    WL_CONNECTED
  )
    Serial.println(F("STA"));

  else
    Serial.println(F("DESCONECTADO"));


  if(
    WiFi.status() ==
    WL_CONNECTED
  ) {

    Serial.print(F("SSID: "));
    Serial.println(
      WiFi.SSID()
    );

    Serial.print(F("IP: "));
    Serial.println(
      WiFi.localIP()
    );

    Serial.print(F("RSSI: "));
    Serial.println(
      WiFi.RSSI()
    );

  }


  Serial.println(
    F("====================================")
  );

}


// ------------------------------------------------------------

void printPhases() {

  ProfileData &p =
    activeProfileData();


  Serial.println();
  Serial.println(F("============== FASES =============="));

  Serial.print(F("Perfil: "));
  Serial.println(p.name);

  Serial.print(F("Total dias: "));
  Serial.println(p.totalDays);

  Serial.print(F("Quantidade fases: "));
  Serial.println(p.phaseCount);

  Serial.println();


  for(
    int i=0;
    i<p.phaseCount;
    i++
  ) {

    Phase &x =
      p.phases[i];


    Serial.printf(
      "Fase %d | Dias %d-%d | "
      "T %.1f C | RH %.1f %% | "
      "Hyst %.1f | Pulse %lu ms | "
      "Lockout %lu ms | RHmax %.1f %% | "
      "Viragem %s\n",

      i+1,

      x.startDay,
      x.endDay,

      x.temp,
      x.rh,

      x.hyst,

      x.pulse,
      x.lockout,

      x.rhMax,

      x.turning
      ? "ON"
      : "OFF"
    );

  }

  Serial.println(
    F("====================================")
  );

}


// ------------------------------------------------------------

void processSerial(
  String cmd
) {

  cmd.trim();

  if(cmd.length() == 0)
    return;


  String up = cmd;

  up.toUpperCase();


  // ----------------------------------------------------------
  // HELP
  // ----------------------------------------------------------

  if(up == "HELP") {

    serialHelp();

    return;
  }


  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------

  if(up == "I2C SCAN") {

    scanI2C();

    return;
  }


  // ----------------------------------------------------------
  // NVS
  // ----------------------------------------------------------

  if(up == "NVS") {

    diagnoseNVS();

    return;
  }


  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------

  if(up == "STATUS") {

    printStatus();

    return;
  }


  // ----------------------------------------------------------
  // PROFILES
  // ----------------------------------------------------------

  if(up == "PROFILES") {

    Serial.println();
    Serial.println(
      F("CODORNA")
    );

    Serial.println(
      F("GALINHA")
    );

    Serial.println(
      F("PAVAO")
    );

    Serial.println(
      F("PATO")
    );

    Serial.println(
      F("CUSTOM")
    );

    return;
  }


  // ----------------------------------------------------------
  // PHASES
  // ----------------------------------------------------------

  if(up == "PHASES") {

    printPhases();

    return;
  }


  // ----------------------------------------------------------
  // PROFILE
  // ----------------------------------------------------------

  if(up.startsWith("PROFILE ")) {

    if(incubationActive) {

      Serial.println(
        F("ERRO: pare a incubacao primeiro.")
      );

      return;
    }


    String p =
      up.substring(8);

    p.trim();


    if(p == "CODORNA")
      selectedProfile =
        PROFILE_CODORNA;

    else if(p == "GALINHA")
      selectedProfile =
        PROFILE_GALINHA;

    else if(p == "PAVAO")
      selectedProfile =
        PROFILE_PAVAO;

    else if(p == "PATO")
      selectedProfile =
        PROFILE_PATO;

    else if(p == "CUSTOM")
      selectedProfile =
        PROFILE_CUSTOM;

    else {

      Serial.println(
        F("Perfil invalido.")
      );

      return;
    }


    saveIncubation();

    applyCurrentPhase();


    Serial.print(
      F("Perfil selecionado: ")
    );

    Serial.println(
      profileName(selectedProfile)
    );

    return;
  }


  // ----------------------------------------------------------
  // INCUBACAO INICIAR
  // ----------------------------------------------------------

  if(up == "INCUBACAO INICIAR") {

    if(incubationActive) {

      Serial.println(
        F("ERRO: incubacao ja esta ativa.")
      );

      return;
    }


    startIncubation(
      rtc.now().unixtime(),
      selectedProfile
    );


    Serial.println(
      F("Incubacao iniciada.")
    );

    return;
  }


  // ----------------------------------------------------------
  // INCUBACAO INICIAR COM DATA
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "INCUBACAO INICIAR "
    )
  ) {

    if(incubationActive) {

      Serial.println(
        F("ERRO: incubacao ja esta ativa.")
      );

      return;
    }


    String dateText =
      cmd.substring(
        strlen("INCUBACAO INICIAR ")
      );

    dateText.trim();


    DateTime dt;


    if(
      !parseDateTime(
        dateText,
        dt
      )
    ) {

      Serial.println(
        F("Data/hora invalida.")
      );

      return;
    }


    startIncubation(
      dt.unixtime(),
      selectedProfile
    );


    Serial.println(
      F("Incubacao iniciada pela data informada.")
    );

    return;
  }


  // ----------------------------------------------------------
  // INCUBACAO PARAR
  // ----------------------------------------------------------

  if(
    up == "INCUBACAO PARAR"
  ) {

    stopIncubation();

    Serial.println(
      F("Incubacao parada.")
    );

    return;
  }


  // ----------------------------------------------------------
  // INCUBACAO RESET
  // ----------------------------------------------------------

  if(
    up == "INCUBACAO RESET"
  ) {

    resetIncubation();

    Serial.println(
      F("Incubacao resetada.")
    );

    return;
  }


  // ----------------------------------------------------------
  // RTC
  // ----------------------------------------------------------

  if(up == "RTC") {

    DateTime n =
      rtc.now();


    Serial.printf(
      "%02d/%02d/%04d %02d:%02d:%02d\n",

      n.day(),
      n.month(),
      n.year(),

      n.hour(),
      n.minute(),
      n.second()
    );

    return;
  }


  // ----------------------------------------------------------
  // RTC SET
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "RTC SET "
    )
  ) {

    String dateText =
      cmd.substring(8);

    dateText.trim();


    DateTime dt;


    if(
      !parseDateTime(
        dateText,
        dt
      )
    ) {

      Serial.println(
        F("Data/hora invalida.")
      );

      return;
    }


    rtc.adjust(dt);


    Serial.println(
      F("RTC ajustado.")
    );

    return;
  }


  // ----------------------------------------------------------
  // PUMP ON
  // ----------------------------------------------------------

  if(up == "PUMP ON") {
    autoControl=false;
    pumpOn(pumpPulseMs);


    Serial.println(
      F("Bomba ON.")
    );

    return;
  }


  // ----------------------------------------------------------
  // PUMP OFF
  // ----------------------------------------------------------

  if(up == "PUMP OFF") {

    pumpOff();

    Serial.println(
      F("Bomba OFF.")
    );

    return;
  }


  // ----------------------------------------------------------
  // PUMP <MS>
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "PUMP "
    )
  ) {

    unsigned long ms =
      up.substring(5).toInt();


    if(ms < MIN_PULSE || ms > MAX_PULSE) {
      Serial.println(F("Tempo da bomba invalido."));
      return;
    }
    autoControl=false;
    pumpOn(ms);


    Serial.print(
      F("Bomba acionada por ")
    );

    Serial.print(ms);

    Serial.println(
      F(" ms.")
    );

    return;
  }


  // ----------------------------------------------------------
  // AUTO ON
  // ----------------------------------------------------------

  if(up == "AUTO ON") {

    autoControl=true;

    Serial.println(
      F("Controle automatico ON.")
    );

    return;
  }


  // ----------------------------------------------------------
  // AUTO OFF
  // ----------------------------------------------------------

  if(up == "AUTO OFF") {

    autoControl=false;

    pumpOff();

    Serial.println(
      F("Controle automatico OFF.")
    );

    return;
  }


  // ----------------------------------------------------------
  // SET TEMP
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET TEMP "
    )
  ) {

    targetTemp =
      up.substring(9).toFloat();


    Serial.print(
      F("Temperatura alvo: ")
    );

    Serial.println(
      targetTemp
    );

    return;
  }


  // ----------------------------------------------------------
  // SET RH
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET RH "
    )
  ) {

    targetRH =
      up.substring(7).toFloat();


    Serial.print(
      F("RH alvo: ")
    );

    Serial.println(
      targetRH
    );

    return;
  }


  // ----------------------------------------------------------
  // SET HYST
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET HYST "
    )
  ) {

    hysteresis =
      up.substring(9).toFloat();


    if(hysteresis < 0)
      hysteresis=0;


    Serial.print(
      F("Histerese: ")
    );

    Serial.println(
      hysteresis
    );

    return;
  }


  // ----------------------------------------------------------
  // SET PULSE
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET PULSE "
    )
  ) {

    pumpPulseMs =
      up.substring(10).toInt();


    pumpPulseMs =
      constrain(
        pumpPulseMs,
        100UL,
        MAX_PULSE
      );


    Serial.print(
      F("Pulso: ")
    );

    Serial.println(
      pumpPulseMs
    );

    return;
  }


  // ----------------------------------------------------------
  // SET LOCKOUT
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET LOCKOUT "
    )
  ) {

    pumpLockoutMs =
      up.substring(12).toInt();


    pumpLockoutMs =
      max(
        pumpLockoutMs,
        (unsigned long)MIN_LOCKOUT
      );


    Serial.print(
      F("Lockout: ")
    );

    Serial.println(
      pumpLockoutMs
    );

    return;
  }


  // ----------------------------------------------------------
  // SET RHMAX
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "SET RHMAX "
    )
  ) {

    rhMax =
      up.substring(10).toFloat();


    rhMax =
      constrain(
        rhMax,
        1.0f,
        100.0f
      );


    Serial.print(
      F("RH Max: ")
    );

    Serial.println(
      rhMax
    );

    return;
  }


  // ----------------------------------------------------------
  // CUSTOM TOTAL
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "CUSTOM TOTAL "
    )
  ) {

    if(incubationActive) {

      Serial.println(
        F("ERRO: pare a incubacao primeiro.")
      );

      return;
    }


    customProfile.totalDays =
      constrain(
        up.substring(13).toInt(),
        1,
        200
      );


    saveCustomProfile();


    Serial.print(
      F("Total CUSTOM: ")
    );

    Serial.println(
      customProfile.totalDays
    );

    return;
  }


  // ----------------------------------------------------------
  // CUSTOM CLEAR
  // ----------------------------------------------------------

  if(
    up == "CUSTOM CLEAR"
  ) {

    if(incubationActive) {

      Serial.println(
        F("ERRO: pare a incubacao primeiro.")
      );

      return;
    }


    customProfile.totalDays=21;

    customProfile.phaseCount=2;


    customProfile.phases[0] =
      makePhase(
        1,
        17,
        37.5,
        50,
        2,
        2000,
        60000,
        65,
        true
      );


    customProfile.phases[1] =
      makePhase(
        18,
        21,
        37.2,
        68,
        2,
        2000,
        60000,
        80,
        false
      );


    for(
      int i=2;
      i<MAX_PHASES;
      i++
    ) {

      customProfile.phases[i] =
        makePhase(
          1,
          1,
          37.5,
          50,
          2,
          2000,
          60000,
          65,
          true
        );

    }


    saveCustomProfile();


    Serial.println(
      F("CUSTOM restaurado para os valores padrao.")
    );

    return;
  }


  // ----------------------------------------------------------
  // CUSTOM PHASE
  // ----------------------------------------------------------

  if(
    up.startsWith(
      "CUSTOM PHASE "
    )
  ) {

    if(incubationActive) {

      Serial.println(
        F("ERRO: pare a incubacao primeiro.")
      );

      return;
    }


    String rest =
      cmd.substring(
        strlen("CUSTOM PHASE ")
      );

    rest.trim();


    String values[12];

    int valueCount=0;

    int start=0;


    while(
      start < rest.length() &&
      valueCount < 20
    ) {

      while(
        start < rest.length() &&
        rest[start]==' '
      )
        start++;


      if(start >= rest.length())
        break;


      int end =
        rest.indexOf(
          ' ',
          start
        );


      if(end < 0)
        end=rest.length();


      String token =
        rest.substring(
          start,
          end
        );


      values[valueCount++] = token;


      start=end+1;

    }


    if(valueCount != 9) {

      Serial.println(
        F("Formato invalido.")
      );

      Serial.println(
        F("CUSTOM PHASE <inicio> <fim> <temp> <rh> <hyst> <pulse> <lockout> <rhmax> <turning>")
      );

      return;
    }


    int phaseIndex = customProfile.phaseCount;


    if(
      phaseIndex < 0 ||
      phaseIndex >= MAX_PHASES
    ) {

      Serial.println(
        F("Numero da fase invalido.")
      );

      return;
    }


    int startDay =
      values[0].toInt();

    int endDay =
      values[1].toInt();

    float temp =
      values[2].toFloat();

    float rh =
      values[3].toFloat();

    float hyst =
      values[4].toFloat();

    unsigned long pulse =
      values[5].toInt();

    unsigned long lockout =
      values[6].toInt();

    float maxRH =
      values[7].toFloat();

    bool turning =
      values[8].toInt() != 0;

    if(startDay < 1 || endDay < startDay || endDay > 365 ||
       temp < MIN_TEMP || temp > MAX_TEMP ||
       rh < MIN_RH_TARGET || rh > MAX_RH_TARGET ||
       hyst < MIN_HYSTERESIS || hyst > MAX_HYSTERESIS ||
       pulse < MIN_PULSE || pulse > MAX_PULSE ||
       lockout < MIN_LOCKOUT || lockout > MAX_LOCKOUT ||
       maxRH < MIN_RH_MAX || maxRH > MAX_RH_MAX ||
       rh + hyst >= maxRH ||
       (values[8].toInt() != 0 && values[8].toInt() != 1)) {
      Serial.println(F("Parametros CUSTOM invalidos."));
      return;
    }


    customProfile.phases[
      phaseIndex
    ] =
      makePhase(
        startDay,
        endDay,
        temp,
        rh,
        hyst,
        pulse,
        lockout,
        maxRH,
        turning
      );


    customProfile.phaseCount = phaseIndex + 1;
    if(endDay > customProfile.totalDays)
      customProfile.totalDays = endDay;


    saveCustomProfile();


    Serial.print(
      F("Fase CUSTOM ")
    );

    Serial.print(
      phaseIndex+1
    );

    Serial.println(
      F(" salva.")
    );

    return;
  }


  // ----------------------------------------------------------
  // COMANDO DESCONHECIDO
  // ----------------------------------------------------------

  Serial.println(
    F("Comando desconhecido. Digite HELP.")
  );

}


// ------------------------------------------------------------

void readSerial() {

  if(!Serial.available())
    return;


  String cmd =
    Serial.readStringUntil(
      '\n'
    );


  processSerial(cmd);

}


// ============================================================
// WIFI - CREDENCIAIS
// ============================================================

void loadWifiCredentials() {

  wifiPrefs.begin(
    "wifi",
    false
  );


  savedSSID =
    wifiPrefs.getString(
      "ssid",
      ""
    );


  savedPassword =
    wifiPrefs.getString(
      "pass",
      ""
    );


  wifiPrefs.end();


  wifiConfigured =
    savedSSID.length() > 0;

}


// ------------------------------------------------------------

void saveWifiCredentials(
  String ssid,
  String password
) {

  wifiPrefs.begin(
    "wifi",
    false
  );


  wifiPrefs.putString(
    "ssid",
    ssid
  );


  wifiPrefs.putString(
    "pass",
    password
  );


  wifiPrefs.end();


  savedSSID=ssid;
  savedPassword=password;

  wifiConfigured=
    ssid.length()>0;

}


// ------------------------------------------------------------

void clearWifiCredentials() {

  wifiPrefs.begin(
    "wifi",
    false
  );


  wifiPrefs.clear();


  wifiPrefs.end();


  savedSSID="";
  savedPassword="";

  wifiConfigured=false;

}


// ============================================================
// WIFI - ACCESS POINT
// ============================================================

void startAP() {

  apMode=true;
  staConnecting=false;


  WiFi.disconnect(
    true,
    true
  );


  delay(100);


  WiFi.mode(
    WIFI_AP
  );


  uint64_t mac =
    ESP.getEfuseMac();


  char suffix[7];


  snprintf(
    suffix,
    sizeof(suffix),
    "%04X",
    (uint16_t)(
      mac & 0xFFFF
    )
  );


  apSSID =
    "CHOCADEIRA-" +
    String(suffix);


  bool ok =
    WiFi.softAP(
      apSSID.c_str(),
      "12345678"
    );


  Serial.println();
  Serial.println(
    F("================================")
  );

  Serial.println(
    F("     MODO CONFIGURACAO WIFI")
  );

  Serial.println(
    F("================================")
  );


  Serial.print(
    F("SSID: ")
  );

  Serial.println(
    apSSID
  );


  Serial.println(
    F("Senha: 12345678")
  );


  Serial.print(
    F("IP: ")
  );

  Serial.println(
    WiFi.softAPIP()
  );


  Serial.print(
    F("AP iniciado: ")
  );

  Serial.println(
    ok
    ? F("SIM")
    : F("NAO")
  );


  Serial.println(
    F("Abra http://192.168.4.1")
  );


  dnsServer.start(
    53,
    "*",
    WiFi.softAPIP()
  );


  displayText(
    "MODO AP",
    apSSID,
    "Senha: 12345678",
    "192.168.4.1"
  );

}


// ============================================================
// WIFI - STA
// ============================================================

bool connectSTA() {

  if(
    !wifiConfigured ||
    savedSSID.length()==0
  )
    return false;


  apMode=false;
  staConnecting=true;
  staStartedAt=millis();


  WiFi.mode(
    WIFI_STA
  );


  WiFi.setAutoReconnect(
    true
  );


  WiFi.persistent(
    false
  );


  Serial.println();

  Serial.print(
    F("Conectando em: ")
  );

  Serial.println(
    savedSSID
  );


  WiFi.begin(
    savedSSID.c_str(),
    savedPassword.c_str()
  );
  Serial.println();
  Serial.println(F("Tentativa Wi-Fi iniciada."));
  return true;

}


// ============================================================
// JSON
// ============================================================

String jsonEscape(
  String s
) {

  s.replace(
    "\\",
    "\\\\"
  );

  s.replace(
    "\"",
    "\\\""
  );

  s.replace(
    "\n",
    "\\n"
  );

  s.replace(
    "\r",
    "\\r"
  );


  return s;

}


// ------------------------------------------------------------

String jsonFloat(
  float x,
  int decimals=1
) {

  if(isnan(x))
    return "null";


  return String(
    x,
    decimals
  );

}


// ------------------------------------------------------------

String rtcString() {

  DateTime n =
    rtc.now();


  char b[24];


  snprintf(
    b,
    sizeof(b),
    "%02d/%02d/%04d %02d:%02d:%02d",

    n.day(),
    n.month(),
    n.year(),

    n.hour(),
    n.minute(),
    n.second()
  );


  return String(b);

}


// ============================================================
// JSON STATUS
// ============================================================

String statusJson() {

  ProfileData &p =
    activeProfileData();


  int phase =
    (currentPhaseIndex + 1);


  String ip;


  if(apMode)
    ip =
      WiFi.softAPIP().toString();

  else
    ip =
      WiFi.localIP().toString();


  String mode;


  if(apMode)
    mode="AP";

  else if(
    WiFi.status() ==
    WL_CONNECTED
  )
    mode="STA";

  else
    mode="OFF";


  long rssi=0;


  if(
    WiFi.status() ==
    WL_CONNECTED
  )
    rssi=WiFi.RSSI();


  String j="{";


  j +=
    "\"temp\":"+
    jsonFloat(currentTemp)+
    ",";


  j +=
    "\"rh\":"+
    jsonFloat(currentRH)+
    ",";


  j +=
    "\"targetTemp\":"+
    jsonFloat(targetTemp)+
    ",";


  j +=
    "\"targetRH\":"+
    jsonFloat(targetRH)+
    ",";


  j +=
    "\"rhMax\":"+
    jsonFloat(rhMax)+
    ",";


  j +=
    "\"hyst\":"+
    jsonFloat(hysteresis)+
    ",";


  j +=
    "\"pulse\":"+
    String(pumpPulseMs)+
    ",";


  j +=
    "\"lockout\":"+
    String(pumpLockoutMs)+
    ",";


  j +=
    "\"day\":"+
    String(incubationDay())+
    ",";


  j +=
    "\"phase\":"+
    String(phase)+
    ",";


  j +=
    "\"profile\":\""+
    jsonEscape(p.name)+
    "\",";


  j +=
    "\"active\":"+
    String(
      incubationActive
      ? "true"
      : "false"
    )+
    ",";


  j +=
    "\"pump\":"+
    String(
      pumpState
      ? "true"
      : "false"
    )+
    ",";


  j +=
    "\"auto\":"+
    String(
      autoControl
      ? "true"
      : "false"
    )+
    ",";


  j +=
    "\"state\":\""+
    jsonEscape(
      stateText()
    )+
    "\",";


  j +=
    "\"rtc\":\""+
    jsonEscape(
      rtcString()
    )+
    "\",";


  j +=
    "\"wifiMode\":\""+
    mode+
    "\",";


  j +=
    "\"ip\":\""+
    ip+
    "\",";


  j +=
    "\"rssi\":"+
    String(rssi)+
    ",";


  j +=
    "\"ssid\":\""+
    jsonEscape(
      WiFi.status()==WL_CONNECTED
      ? WiFi.SSID()
      : savedSSID
    )+
    "\",";


  j +=
    "\"sht31\":"+
    String(
      !isnan(currentTemp)
      ? "true"
      : "false"
    )+
    ",";


  j +=
    "\"rtcOk\":"+
    String(
      rtcOk
      ? "true"
      : "false"
    )+
    ",";


  j +=
    "\"oled\":"+
    String(
      oledOk
      ? "true"
      : "false"
    );


  j +=
    "}";


  return j;

}


// ============================================================
// ENVIO JSON
// ============================================================

void sendJson(
  String json,
  int code=200
) {

  server.send(
    code,
    "application/json; charset=UTF-8",
    json
  );

}


// ============================================================
// LEITURA DO BODY
// ============================================================

String bodyText() {

  return server.arg(
    "plain"
  );

}


// ------------------------------------------------------------
// Campo STRING simples
// ------------------------------------------------------------

String jsonStringField(
  String body,
  String key,
  String fallback=""
) {

  String k =
    "\"" +
    key +
    "\"";


  int p =
    body.indexOf(k);


  if(p<0)
    return fallback;


  p =
    body.indexOf(
      ':',
      p
    );


  if(p<0)
    return fallback;


  p++;


  while(
    p<body.length() &&
    (
      body[p]==' ' ||
      body[p]=='\t' ||
      body[p]=='"'
    )
  )
    p++;


  int e=p;

  bool escaped=false;


  for(
    ;
    e<body.length();
    e++
  ) {

    char c=
      body[e];


    if(
      c=='"' &&
      !escaped
    )
      break;


    if(
      c=='\\' &&
      !escaped
    )
      escaped=true;

    else
      escaped=false;

  }


  String value =
    body.substring(
      p,
      e
    );


  value.replace(
    "\\\"",
    "\""
  );


  value.replace(
    "\\\\",
    "\\"
  );


  return value;

}


// ------------------------------------------------------------
// Campo NUMBER
// ------------------------------------------------------------

float jsonNumberField(
  String body,
  String key,
  float fallback
) {

  String k =
    "\"" +
    key +
    "\"";


  int p =
    body.indexOf(k);


  if(p<0)
    return fallback;


  p =
    body.indexOf(
      ':',
      p
    );


  if(p<0)
    return fallback;


  p++;


  while(
    p<body.length() &&
    (
      body[p]==' ' ||
      body[p]=='\t' ||
      body[p]=='"'
    )
  )
    p++;


  int e=p;


  while(
    e<body.length()
  ) {

    char c=
      body[e];


    if(
      !(
        (
          c>='0' &&
          c<='9'
        ) ||
        c=='-' ||
        c=='+' ||
        c=='.'
      )
    )
      break;


    e++;

  }


  if(e<=p)
    return fallback;


  return body.substring(
    p,
    e
  ).toFloat();

}


// ------------------------------------------------------------

int jsonIntField(
  String body,
  String key,
  int fallback
) {

  return (int)
    jsonNumberField(
      body,
      key,
      fallback
    );

}


// ============================================================
// API ROOT
// ============================================================

void handleRoot() {

  if(apMode) {

    server.send_P(
      200,
      "text/html; charset=UTF-8",
      WIFI_HTML
    );

  }

  else {

    server.send_P(
      200,
      "text/html; charset=UTF-8",
      MAIN_HTML
    );

  }

}


// ============================================================
// API WIFI SCAN
// ============================================================

void handleWifiScan() {

  WiFi.mode(
    apMode
    ? WIFI_AP_STA
    : WIFI_STA
  );


  int n = WiFi.scanComplete();
  if(n == WIFI_SCAN_FAILED) {
    WiFi.scanNetworks(true, true);
    sendJson("{\"networks\":[],\"scanning\":true}");
    return;
  }
  if(n == WIFI_SCAN_RUNNING) {
    sendJson("{\"networks\":[],\"scanning\":true}");
    return;
  }


  String j =
    "{\"networks\":[";


  for(
    int i=0;
    i<n;
    i++
  ) {

    if(i)
      j += ",";


    String ssid =
      WiFi.SSID(i);


    bool encrypted =
      WiFi.encryptionType(i)
      != WIFI_AUTH_OPEN;


    j +=
      "{\"ssid\":\""+
      jsonEscape(ssid)+
      "\",";


    j +=
      "\"rssi\":"+
      String(
        WiFi.RSSI(i)
      )+
      ",";


    j +=
      "\"encryption\":"+
      String(
        encrypted
        ? "true"
        : "false"
      );


    j += "}";

  }


  j += "]}";


  WiFi.scanDelete();


  sendJson(
    j
  );

}


// ============================================================
// API WIFI SAVE
// ============================================================

void handleWifiSave() {

  String body =
    bodyText();


  String ssid =
    jsonStringField(
      body,
      "ssid",
      ""
    );


  String password =
    jsonStringField(
      body,
      "password",
      ""
    );


  ssid.trim();


  if(ssid.length()==0) {

    sendJson(
      "{\"message\":\"SSID vazio.\"}",
      400
    );

    return;
  }


  saveWifiCredentials(
    ssid,
    password
  );


  sendJson(
    "{\"message\":\"Wi-Fi salvo. Reiniciando...\"}"
  );


  delay(800);


  ESP.restart();

}


// ============================================================
// API WIFI FORGET
// ============================================================

void handleWifiForget() {

  clearWifiCredentials();


  sendJson(
    "{\"message\":\"Credenciais apagadas. Reiniciando...\"}"
  );


  delay(800);


  ESP.restart();

}


// ============================================================
// API STATUS
// ============================================================

void handleStatus() {

  sendJson(
    statusJson()
  );

}


// ============================================================
// API CONFIG GET
// ============================================================

void handleConfigGet() {

  String j="{";


  j +=
    "\"temp\":"+
    String(
      targetTemp,
      1
    )+
    ",";


  j +=
    "\"rh\":"+
    String(
      targetRH,
      1
    )+
    ",";


  j +=
    "\"hyst\":"+
    String(
      hysteresis,
      1
    )+
    ",";


  j +=
    "\"rhMax\":"+
    String(
      rhMax,
      1
    )+
    ",";


  j +=
    "\"pulse\":"+
    String(
      pumpPulseMs
    )+
    ",";


  j +=
    "\"lockout\":"+
    String(
      pumpLockoutMs
    )+
    ",";


  j +=
    "\"profile\":"+
    String(
      (int)selectedProfile
    )+",";

  int configPhase = currentPhaseIndex < 0 ? 0 : currentPhaseIndex;
  j += "\"turning\":" + String(activeProfileData().phases[configPhase].turning ? "true" : "false");


  j += "}";


  sendJson(
    j
  );

}


// ============================================================
// API CONFIG POST
// ============================================================

void handleConfigPost() {

  String b =
    bodyText();


  targetTemp =
    jsonNumberField(
      b,
      "temp",
      targetTemp
    );


  targetRH =
    jsonNumberField(
      b,
      "rh",
      targetRH
    );


  hysteresis =
    jsonNumberField(
      b,
      "hyst",
      hysteresis
    );


  rhMax =
    jsonNumberField(
      b,
      "rhMax",
      rhMax
    );


  pumpPulseMs =
    (unsigned long)
    jsonNumberField(
      b,
      "pulse",
      pumpPulseMs
    );


  pumpLockoutMs =
    (unsigned long)
    jsonNumberField(
      b,
      "lockout",
      pumpLockoutMs
    );


  if(targetTemp < MIN_TEMP || targetTemp > MAX_TEMP ||
     targetRH < MIN_RH_TARGET || targetRH > MAX_RH_TARGET ||
     hysteresis < MIN_HYSTERESIS || hysteresis > MAX_HYSTERESIS ||
     rhMax < MIN_RH_MAX || rhMax > MAX_RH_MAX ||
     targetRH + hysteresis >= rhMax ||
     pumpPulseMs < MIN_PULSE || pumpPulseMs > MAX_PULSE ||
     pumpLockoutMs < MIN_LOCKOUT || pumpLockoutMs > MAX_LOCKOUT) {
    sendJson("{\"message\":\"Parametros invalidos.\"}", 400);
    return;
  }

  bool turning = jsonIntField(b, "turning", 1) != 0;
  if(selectedProfile == PROFILE_CUSTOM) {
    int configPhase = currentPhaseIndex < 0 ? 0 : currentPhaseIndex;
    Phase &ph = customProfile.phases[configPhase];
    ph.temp = targetTemp; ph.rh = targetRH; ph.hyst = hysteresis;
    ph.rhMax = rhMax; ph.pulse = pumpPulseMs; ph.lockout = pumpLockoutMs;
    ph.turning = turning;
    saveCustomProfile();
    sendJson("{\"message\":\"Parametros CUSTOM salvos em NVS.\"}");
  } else {
    sendJson("{\"message\":\"Parametros aplicados ate a proxima fase.\"}");
  }

}


// ============================================================
// API COMMAND
// ============================================================

void handleCommand() {

  String body =
    bodyText();


  String command =
    jsonStringField(
      body,
      "cmd",
      ""
    );


  command.trim();


  if(command.length()==0) {

    sendJson(
      "{\"message\":\"Comando vazio.\"}",
      400
    );

    return;
  }


  processSerial(
    command
  );


  sendJson(
    "{\"message\":\"Comando executado: "+
    jsonEscape(command)+
    "\"}"
  );

}


// ============================================================
// CONVERSÃƒO DATETIME-LOCAL
// ============================================================

String isoToDateTime(
  String iso
) {

  iso.trim();

  if(iso.length() != 16 || iso[4] != '-' || iso[7] != '-' ||
     iso[10] != 'T' || iso[13] != ':')
    return "";


  int year =
    iso.substring(
      0,
      4
    ).toInt();


  int month =
    iso.substring(
      5,
      7
    ).toInt();


  int day =
    iso.substring(
      8,
      10
    ).toInt();


  int hour =
    iso.substring(
      11,
      13
    ).toInt();


  int minute =
    iso.substring(
      14,
      16
    ).toInt();

  if(year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
     hour < 0 || hour > 23 || minute < 0 || minute > 59)
    return "";


  char buffer[24];


  snprintf(
    buffer,
    sizeof(buffer),
    "%02d/%02d/%04d %02d:%02d:00",

    day,
    month,
    year,

    hour,
    minute
  );


  return String(
    buffer
  );

}


// ============================================================
// API START INCUBATION
// ============================================================

void handleIncStart() {

  if(!rtcOk) {
    sendJson("{\"message\":\"RTC indisponivel.\"}", 503);
    return;
  }

  if(incubationActive) {

    sendJson(
      "{\"message\":\"Incubação já está ativa.\"}",
      409
    );

    return;
  }


  String body =
    bodyText();


  int profile =
    jsonIntField(
      body,
      "profile",
      (int)selectedProfile
    );


  if(
    profile<0 ||
    profile>4
  )
    profile =
      PROFILE_GALINHA;


  String iso =
    jsonStringField(
      body,
      "datetime",
      ""
    );


  uint32_t startEpoch;


  if(iso.length()==0) {

    startEpoch =
      rtc.now().unixtime();

  }

  else {

    String dtText =
      isoToDateTime(
        iso
      );


    DateTime dt;


    if(
      !parseDateTime(
        dtText,
        dt
      )
    ) {

      sendJson(
        "{\"message\":\"Data/hora inválida.\"}",
        400
      );

      return;
    }


    startEpoch =
      dt.unixtime();

  }


  if(startEpoch > rtc.now().unixtime()) {
    sendJson("{\"message\":\"Data de início não pode estar no futuro.\"}", 400);
    return;
  }

  selectedProfile =
    (Profile)profile;

  startIncubation(startEpoch, selectedProfile);

  if(!incubationActive || incubationStartEpoch != startEpoch) {
    sendJson("{\"message\":\"Falha ao iniciar a incubação.\"}", 500);
    return;
  }


  sendJson(
    "{\"message\":\"Incubação iniciada.\"}"
  );

}


// ============================================================
// API CUSTOM GET
// ============================================================

String customJson() {

  String j =
    "{\"totalDays\":"+
    String(
      customProfile.totalDays
    )+
    ",";


  j +=
    "\"phaseCount\":"+
    String(
      customProfile.phaseCount
    )+
    ",";


  j +=
    "\"phases\":[";


  for(
    int i=0;
    i<customProfile.phaseCount;
    i++
  ) {

    if(i)
      j += ",";


    Phase &p =
      customProfile.phases[i];


    j +=
      "{\"start\":"+
      String(p.startDay)+
      ",";


    j +=
      "\"end\":"+
      String(p.endDay)+
      ",";


    j +=
      "\"temp\":"+
      String(p.temp,1)+
      ",";


    j +=
      "\"rh\":"+
      String(p.rh,1)+
      ",";


    j +=
      "\"hyst\":"+
      String(p.hyst,1)+
      ",";


    j +=
      "\"pulse\":"+
      String(p.pulse)+
      ",";


    j +=
      "\"lockout\":"+
      String(p.lockout)+
      ",";


    j +=
      "\"rhMax\":"+
      String(p.rhMax,1)+
      ",";


    j +=
      "\"turning\":"+
      String(
        p.turning
        ? "1"
        : "0"
      );


    j +=
      "}";

  }


  j +=
    "]}";


  return j;

}


// ------------------------------------------------------------

void handleCustomGet() {

  sendJson(
    customJson()
  );

}


// ============================================================
// API CUSTOM POST
// ============================================================

void handleCustomPost() {

  if(incubationActive) {

    sendJson(
      "{\"message\":\"Pare a incubação antes de editar CUSTOM.\"}",
      409
    );

    return;
  }


  String body =
    bodyText();


  customProfile.totalDays =
    constrain(
      jsonIntField(
        body,
        "totalDays",
        customProfile.totalDays
      ),
      1,
      200
    );


  customProfile.phaseCount =
    constrain(
      jsonIntField(
        body,
        "phaseCount",
        customProfile.phaseCount
      ),
      1,
      MAX_PHASES
    );


  int phasePos =
    body.indexOf(
      "\"phases\""
    );


  if(phasePos<0) {

    sendJson(
      "{\"message\":\"Fases não encontradas.\"}",
      400
    );

    return;
  }


  phasePos =
    body.indexOf(
      '[',
      phasePos
    );


  if(phasePos<0) {

    sendJson(
      "{\"message\":\"Lista de fases inválida.\"}",
      400
    );

    return;
  }


  int position =
    phasePos+1;


  for(
    int i=0;
    i<customProfile.phaseCount;
    i++
  ) {

    int objectStart =
      body.indexOf(
        '{',
        position
      );


    if(objectStart<0)
      break;


    int objectEnd =
      body.indexOf(
        '}',
        objectStart
      );


    if(objectEnd<0)
      break;


    String object =
      body.substring(
        objectStart,
        objectEnd+1
      );


    Phase &p =
      customProfile.phases[i];


    p.startDay =
      jsonIntField(
        object,
        "start",
        1
      );


    p.endDay =
      jsonIntField(
        object,
        "end",
        1
      );


    p.temp =
      jsonNumberField(
        object,
        "temp",
        37.5
      );


    p.rh =
      jsonNumberField(
        object,
        "rh",
        50
      );


    p.hyst =
      jsonNumberField(
        object,
        "hyst",
        2
      );


    p.pulse =
      constrain(
        (unsigned long)
        jsonNumberField(
          object,
          "pulse",
          2000
        ),
        100UL,
        MAX_PULSE
      );


    p.lockout =
      max(
        (unsigned long)
        jsonNumberField(
          object,
          "lockout",
          60000
        ),
        (unsigned long)MIN_LOCKOUT
      );


    p.rhMax =
      constrain(
        jsonNumberField(
          object,
          "rhMax",
          65
        ),
        1.0f,
        100.0f
      );


    p.turning =
      jsonIntField(
        object,
        "turning",
        1
      ) != 0;


    position =
      objectEnd+1;

  }


  saveCustomProfile();


  if(
    selectedProfile ==
    PROFILE_CUSTOM
  )
    applyCurrentPhase();


  sendJson(
    "{\"message\":\"CUSTOM salvo em NVS.\"}"
  );

}


// ============================================================
// API RTC
// ============================================================

void handleRTCPost() {

  String iso =
    jsonStringField(
      bodyText(),
      "datetime",
      ""
    );


  String dtText =
    isoToDateTime(
      iso
    );


  DateTime dt;


  if(
    !parseDateTime(
      dtText,
      dt
    )
  ) {

    sendJson(
      "{\"message\":\"Data/hora inválida.\"}",
      400
    );

    return;
  }


  rtc.adjust(
    dt
  );


  sendJson(
    "{\"message\":\"RTC ajustado.\"}"
  );

}


// ============================================================
// ROTAS WEB
// ============================================================

void setupRoutes() {

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );


  server.on(
    "/api/status",
    HTTP_GET,
    handleStatus
  );


  server.on(
    "/api/config",
    HTTP_GET,
    handleConfigGet
  );


  server.on(
    "/api/config",
    HTTP_POST,
    handleConfigPost
  );


  server.on(
    "/api/command",
    HTTP_POST,
    handleCommand
  );


  server.on(
    "/api/incubation/start",
    HTTP_POST,
    handleIncStart
  );


  server.on(
    "/api/wifi/scan",
    HTTP_GET,
    handleWifiScan
  );


  server.on(
    "/api/wifi/save",
    HTTP_POST,
    handleWifiSave
  );


  server.on(
    "/api/wifi/forget",
    HTTP_POST,
    handleWifiForget
  );


  server.on(
    "/api/custom",
    HTTP_GET,
    handleCustomGet
  );


  server.on(
    "/api/custom",
    HTTP_POST,
    handleCustomPost
  );


  server.on(
    "/api/rtc",
    HTTP_POST,
    handleRTCPost
  );


  server.onNotFound(
    []() {

      if(apMode) {

        server.send_P(
          200,
          "text/html; charset=UTF-8",
          WIFI_HTML
        );

      }

      else {

        server.send(
          404,
          "text/plain",
          "404 - Not Found"
        );

      }

    }
  );


  server.begin();


  Serial.println(
    F("Servidor HTTP iniciado.")
  );

}
// ============================================================
// CHOCADEIRA V1.6
// PARTE 3/3
// SETUP + LOOP + FUNÃ‡Ã•ES FINAIS
// ============================================================



// ============================================================
// INICIALIZAÃ‡ÃƒO DOS HARDWARES
// ============================================================

void initHardware() {

  // ----------------------------------------------------------
  // GPIO DA BOMBA
  // ----------------------------------------------------------

  pinMode(
    PUMP_PIN,
    OUTPUT
  );

  digitalWrite(
    PUMP_PIN,
    LOW
  );

  pumpState=false;


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );


  Wire.setClock(
    100000
  );


  // ----------------------------------------------------------
  // SHT31
  // ----------------------------------------------------------

  shtOk =
    sht31.begin(
      SHT31_ADDR
    );


  if(shtOk) {

    Serial.println(
      F("SHT31: OK")
    );

  }

  else {

    Serial.println(
      F("SHT31: ERRO")
    );

  }


  // ----------------------------------------------------------
  // DS3231
  // ----------------------------------------------------------

  rtcOk =
    rtc.begin();


  if(rtcOk) {

    Serial.println(
      F("DS3231: OK")
    );

  }

  else {

    Serial.println(
      F("DS3231: ERRO")
    );

  }


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  oledOk =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    );


  if(oledOk) {

    Serial.println(
      F("OLED: OK")
    );


    display.clearDisplay();


    display.setTextSize(
      1
    );


    display.setTextColor(
      SSD1306_WHITE
    );


    display.setCursor(
      0,
      0
    );


    display.println(
      F("CHOCADEIRA V1.6")
    );


    display.setCursor(
      0,
      18
    );


    display.println(
      F("Inicializando...")
    );


    display.display();

  }

  else {

    Serial.println(
      F("OLED: ERRO")
    );

  }


  // ----------------------------------------------------------
  // RTC
  // ----------------------------------------------------------

  if(rtcOk) {

    if(rtc.lostPower()) {

      Serial.println(
        F("RTC perdeu energia.")
      );


      Serial.println(
        F("Ajustando pela data/hora da compilacao.")
      );


      rtc.adjust(
        DateTime(
          F(__DATE__),
          F(__TIME__)
        )
      );

    }

  }

}


// ============================================================
// VERIFICAÃ‡ÃƒO DO RTC
// ============================================================

bool isRtcValid() {

  if(!rtcOk)
    return false;


  DateTime now =
    rtc.now();


  if(
    now.year() < 2020 ||
    now.year() > 2099
  )
    return false;


  return true;

}


// ============================================================
// WIFI STATUS NO OLED
// ============================================================

void showNetworkInfo() {

  if(!oledOk)
    return;


  if(apMode) {

    displayText(
      "CONFIG WIFI",
      apSSID,
      "Senha: 12345678",
      "192.168.4.1"
    );

    return;

  }


  if(
    WiFi.status() ==
    WL_CONNECTED
  ) {

    String line1 =
      "WiFi conectado";

    String line2 =
      WiFi.localIP().toString();

    String line3 =
      "SSID: " +
      WiFi.SSID();

    String line4 =
      "chocadeira.local";


    displayText(
      line1,
      line2,
      line3,
      line4
    );

  }

}


// ============================================================
// TENTATIVA DE RECONEXÃƒO WIFI
// ============================================================

void maintainWiFi() {

  // Se o ESP32 estÃ¡ em modo AP, nÃ£o tenta reconectar como STA
  if (WiFi.getMode() == WIFI_AP) {
    return;
  }

  // Wi-Fi conectado
  if (WiFi.status() == WL_CONNECTED) {

    staConnecting=false;
    staIP=WiFi.localIP();

    if (!mdnsStarted) {

      if (MDNS.begin("chocadeira")) {
        mdnsStarted = true;

        Serial.println();
        Serial.println("=================================");
        Serial.println("mDNS iniciado");
        Serial.println("Acesso: http://chocadeira.local");
        Serial.println("IP: " + WiFi.localIP().toString());
        Serial.println("=================================");

        displayText(
          "WIFI OK",
          WiFi.SSID(),
          WiFi.localIP().toString(),
          "chocadeira.local"
        );
      }
    }

    return;
  }

  if(staConnecting) {
    if(millis() - staStartedAt >= 15000UL) {
      staConnecting=false;
      Serial.println(F("Falha ao conectar. Iniciando AP."));
      WiFi.disconnect();
      startAP();
    }
    return;
  }

  // Wi-Fi desconectado
  mdnsStarted = false;

  // SÃ³ tenta reconectar se existir SSID salvo
  if (savedSSID.length() == 0) {
    return;
  }

  static unsigned long lastReconnect = 0;

  if (millis() - lastReconnect >= 10000UL) {

    lastReconnect = millis();
    connectSTA();
  }
}

// ============================================================
// LEITURA DOS SENSORES
// ============================================================

void updateSensors() {

  static unsigned long
    lastRead=0;


  if(pumpState)
    return;


  if(
    millis()-lastRead <
    1000
  )
    return;


  lastRead =
    millis();


  if(!shtOk) {

    controlState =
      STATE_SENSOR_ERROR;

    pumpOff();

    return;

  }


  float temperature =
    sht31.readTemperature();


  float humidity =
    sht31.readHumidity();


  if(
    isnan(temperature) ||
    isnan(humidity)
  ) {

    controlState =
      STATE_SENSOR_ERROR;

    pumpOff();

    Serial.println(
      F("ERRO: leitura SHT31.")
    );

    return;

  }


  if(
    temperature < -40 ||
    temperature > 125 ||
    humidity < 0 ||
    humidity > 100
  ) {

    controlState =
      STATE_SENSOR_ERROR;

    pumpOff();

    Serial.println(
      F("ERRO: valores SHT31 fora da faixa.")
    );

    return;

  }


  currentTemp =
    temperature;


  currentRH =
    humidity;


  lastSensorRead =
    millis();


  if(controlState == STATE_SENSOR_ERROR && !pumpState)
    controlState = STATE_OK;

}


// ============================================================
// CONTROLE DA BOMBA
// ============================================================

void updatePump() {
  if(!pumpState)
    return;

  unsigned long elapsed = millis() - pumpStartedAt;
  if(elapsed >= MAX_PULSE || elapsed >= currentPumpDuration) {
    pumpOff();
    lastPumpAt = millis();
    controlState = STATE_LOCKOUT;
  }

}

void updateLockout() {
  if(controlState == STATE_LOCKOUT &&
     millis() - lastPumpAt >= pumpLockoutMs) {
    controlState = STATE_OK;
  }
}

void humidityControl() {
  if(!incubationActive) {
    if(!pumpState && controlState != STATE_SENSOR_ERROR)
      controlState = STATE_OK;
    return;
  }

  applyCurrentPhase();

  if(!shtOk || isnan(currentRH) ||
     millis() - lastSensorRead > SENSOR_TIMEOUT) {
    pumpOff();
    controlState = STATE_SENSOR_ERROR;
    return;
  }

  if(!autoControl || pumpState || controlState == STATE_LOCKOUT)
    return;

  if(currentRH >= rhMax) {
    controlState = STATE_RH_MAX;
    return;
  }

  if(currentRH <= targetRH - hysteresis)
    pumpOn(pumpPulseMs);
  else
    controlState = STATE_OK;
}


// ============================================================
// VERIFICA FINAL DA INCUBAÃ‡ÃƒO
// ============================================================

void checkIncubationEnd() {

  if(!incubationActive)
    return;


  int day =
    incubationDay();


  ProfileData &p =
    activeProfileData();


  if(
    day > p.totalDays
  ) {

    Serial.println();
    Serial.println("========== DIAGNOSTICO FIM INCUBACAO ==========");
    Serial.print("Dia atual: ");
    Serial.println(day);
    Serial.print("Perfil ID: ");
    Serial.println((int)selectedProfile);
    Serial.print("Perfil: ");
    Serial.println(p.name);
    Serial.print("totalDays efetivo: ");
    Serial.println(p.totalDays);
    Serial.print("Condicao day > totalDays: ");
    Serial.println(day > p.totalDays ? "TRUE" : "FALSE");
    Serial.println("==============================================");

    stopIncubation();


    Serial.println();
    Serial.println(
      F("====================================")
    );

    Serial.println(
      F("       INCUBACAO CONCLUIDA")
    );

    Serial.println(
      F("====================================")
    );


    if(oledOk) {

      displayText(
        "INCUBACAO",
        "CONCLUIDA",
        p.name,
        "Retire os pintinhos"
      );

    }

  }

}


// ============================================================
// OLED PRINCIPAL
// ============================================================




// ============================================================
// OLED - QUATRO PAGINAS COM HEADER FIXO
// ============================================================

void drawOLEDHeader() {
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("T:");
  display.print(isnan(currentTemp) ? String("--") : String(currentTemp, 1));
  display.print("C");

  String humidityText = "RH:";
  humidityText += isnan(currentRH) ? String("--") : String(currentRH, 1);
  humidityText += "%";
  int16_t textX1;
  int16_t textY1;
  uint16_t textWidth;
  uint16_t textHeight;
  display.getTextBounds(humidityText, 0, 0, &textX1, &textY1, &textWidth, &textHeight);
  display.setCursor(SCREEN_WIDTH - textWidth, 0);
  display.print(humidityText);
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
}

void drawOLEDPage1() {
  char dateText[18];
  char timeText[6];
  if(rtcOk) {
    DateTime now = rtc.now();
    snprintf(dateText, sizeof(dateText), "DATA: %02d/%02d/%04d", now.day(), now.month(), now.year());
    snprintf(timeText, sizeof(timeText), "%02d:%02d", now.hour(), now.minute());
  } else {
    snprintf(dateText, sizeof(dateText), "DATA: --/--/----");
    snprintf(timeText, sizeof(timeText), "--:--");
  }
  display.clearDisplay();
  drawOLEDHeader();
  display.setCursor(0, 16); display.print("CHOCADEIRA");
  display.setCursor(0, 26); display.print(dateText);
  display.setCursor(0, 36); display.print("HORA: "); display.print(timeText);
  display.setCursor(0, 46); display.print("ALVO RH: "); display.print(targetRH, 1); display.print("%");
  display.setCursor(0, 56); display.print("BOMBA:"); display.print(pumpState ? "ON" : "OFF"); display.print(" AUTO:"); display.print(autoControl ? "ON" : "OFF");
}

void drawOLEDPage2() {
  display.clearDisplay();
  drawOLEDHeader();
  display.setCursor(0, 16); display.print("CONTROLE RH");
  display.setCursor(0, 26); display.print("ATUAL: "); display.print(isnan(currentRH) ? String("ERRO") : String(currentRH, 1)); display.print("%");
  display.setCursor(0, 36); display.print("ALVO : "); display.print(targetRH, 1); display.print("%");
  display.setCursor(0, 46); display.print("MIN  : "); display.print(targetRH - hysteresis, 1); display.print("%");
  display.setCursor(0, 56); display.print("MAX:"); display.print(rhMax, 1); display.print("% AUTO:"); display.print(autoControl ? "ON" : "OFF");
}

void drawOLEDPage3() {
  ProfileData &p = activeProfileData();
  unsigned long lockElapsed = millis() - lastPumpAt;
  unsigned long lockRemaining = controlState == STATE_LOCKOUT && pumpLockoutMs > lockElapsed ? pumpLockoutMs - lockElapsed : 0;
  bool turning = p.phases[currentPhaseIndex < 0 ? 0 : currentPhaseIndex].turning;
  display.clearDisplay();
  drawOLEDHeader();
  display.setCursor(0, 16); display.print("CONTROLE BOMBA");
  display.setCursor(0, 26); display.print("ESTADO: "); display.print(pumpState ? "ON" : "OFF");
  display.setCursor(0, 36); display.print("PULSO: "); display.print(pumpPulseMs / 1000.0f, 1); display.print("s");
  display.setCursor(0, 46); display.print("LOCK: "); display.print(lockRemaining / 1000); display.print("s");
  display.setCursor(0, 56); display.print("VIRAGEM: "); display.print(turning ? "ON" : "OFF");
}

void drawOLEDPage4() {
  ProfileData &p = activeProfileData();
  display.clearDisplay();
  drawOLEDHeader();
  display.setCursor(0, 16); display.print("INCUBACAO");
  display.setCursor(0, 26); display.print("PERFIL: "); display.print(p.name);
  display.setCursor(0, 36); display.print("DIA: "); if(incubationActive) { display.print(incubationDay()); display.print(" / "); display.print(p.totalDays); } else display.print("--");
  display.setCursor(0, 46); display.print("FASE: "); if(incubationActive && currentPhaseIndex >= 0) { display.print(currentPhaseIndex + 1); display.print(" / "); display.print(p.phaseCount); } else display.print("--");
  display.setCursor(0, 56); display.print("TEMP:"); display.print(targetTemp, 1); display.print("C RH:"); display.print(targetRH, 1); display.print("%");
}

void updateOLED() {
  if(!oledOk || millis() - lastOLEDUpdate < OLED_INTERVAL)
    return;

  lastOLEDUpdate = millis();
  if(millis() - lastOLEDPageChange >= OLED_PAGE_INTERVAL) {
    lastOLEDPageChange = millis();
    oledPage = (oledPage + 1) % 4;
  }

  switch(oledPage) {
    case 0: drawOLEDPage1(); break;
    case 1: drawOLEDPage2(); break;
    case 2: drawOLEDPage3(); break;
    default: drawOLEDPage4(); break;
  }
  display.display();
}

// Mensagens transitórias de inicialização/rede; a rotação normal retoma no próximo ciclo.
void displayText(const String& a, const String& b, const String& c, const String& d) {
  if(!oledOk)
    return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); display.println(a);
  display.setCursor(0, 16); display.println(b);
  display.setCursor(0, 32); display.println(c);
  display.setCursor(0, 48); display.println(d);
  display.display();
}

// ============================================================
// PRINT INICIAL
// ============================================================

void printStartupInfo() {

  Serial.println();
  Serial.println(
    F("================================================")
  );

  Serial.println(
    F("          CHOCADEIRA ESP32 V1.6")
  );

  Serial.println(
    F("================================================")
  );


  Serial.print(
    F("SHT31: ")
  );

  Serial.println(
    shtOk
    ? F("OK")
    : F("ERRO")
  );


  Serial.print(
    F("DS3231: ")
  );

  Serial.println(
    rtcOk
    ? F("OK")
    : F("ERRO")
  );


  Serial.print(
    F("OLED: ")
  );

  Serial.println(
    oledOk
    ? F("OK")
    : F("ERRO")
  );


  Serial.print(
    F("Perfil: ")
  );

  Serial.println(
    profileName(
      selectedProfile
    )
  );


  Serial.print(
    F("Incubacao: ")
  );

  Serial.println(
    incubationActive
    ? F("ATIVA")
    : F("PARADA")
  );


  Serial.print(
    F("Dia: ")
  );

  Serial.println(
    incubationDay()
  );


  Serial.println(
    F("------------------------------------------------")
  );


  if(apMode) {

    Serial.println(
      F("MODO AP / CONFIGURACAO")
    );


    Serial.print(
      F("SSID: ")
    );

    Serial.println(
      apSSID
    );


    Serial.println(
      F("Senha: 12345678")
    );


    Serial.println(
      F("Endereco: http://192.168.4.1")
    );

  }

  else {

    Serial.println(
      F("MODO STA / CLIENTE")
    );


    Serial.print(
      F("SSID: ")
    );

    Serial.println(
      WiFi.SSID()
    );


    Serial.print(
      F("IP: ")
    );

    Serial.println(
      WiFi.localIP()
    );


    Serial.println(
      F("Endereco: http://chocadeira.local")
    );

  }


  Serial.println(
    F("================================================")
  );

}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    500
  );


  Serial.println();
  Serial.println();
  Serial.println(
    F("Inicializando Chocadeira V1.6...")
  );


  // ----------------------------------------------------------
  // HARDWARE
  // ----------------------------------------------------------

  initHardware();


  // ----------------------------------------------------------
  // PERFIS
  // ----------------------------------------------------------

  initProfiles();


  // ----------------------------------------------------------
  // CUSTOM PERSISTENTE
  // ----------------------------------------------------------

  loadCustomProfile();


  // ----------------------------------------------------------
  // ESTADO DA INCUBAÃ‡ÃƒO
  // ----------------------------------------------------------

  loadIncubation();
  printResetDiagnostic();
  lastIncubationActive = incubationActive;


  // ----------------------------------------------------------
  // APLICA FASE ATUAL
  // ----------------------------------------------------------

  applyCurrentPhase();


  // ----------------------------------------------------------
  // PRIMEIRA LEITURA
  // ----------------------------------------------------------

  updateSensors();


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  loadWifiCredentials();


  if(
    wifiConfigured &&
    savedSSID.length()>0
  ) {

    connectSTA();

  }

  else {

    startAP();

  }


  // ----------------------------------------------------------
  // ROTAS WEB
  // ----------------------------------------------------------

  setupRoutes();


  // ----------------------------------------------------------
  // INFO
  // ----------------------------------------------------------

  printStartupInfo();


  serialHelp();


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if(apMode) {

    showNetworkInfo();

  }

  else {

    if(
      WiFi.status() ==
      WL_CONNECTED
    ) {

      showNetworkInfo();

    }

  }

}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // SERVIDOR WEB
  // ----------------------------------------------------------

  server.handleClient();


  // ----------------------------------------------------------
  // DNS DO ACCESS POINT
  // ----------------------------------------------------------

  if(apMode) {

    dnsServer.processNextRequest();

  }


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  maintainWiFi();


  // ----------------------------------------------------------
  // SENSORES
  // ----------------------------------------------------------

  updateSensors();


  // ----------------------------------------------------------
  // CONTROLE DE UMIDADE
  // ----------------------------------------------------------

  updatePump();
  updateLockout();
  humidityControl();


  // ----------------------------------------------------------
  // SEGURANÃ‡A DA BOMBA
  // ----------------------------------------------------------

  // ----------------------------------------------------------
  // FINAL DA INCUBAÃ‡ÃƒO
  // ----------------------------------------------------------

  checkIncubationEnd();


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  updateOLED();


  // ----------------------------------------------------------
  // SERIAL
  // ----------------------------------------------------------

  readSerial();
  monitorIncubationState();
  checkIncubationMemory();


  // ----------------------------------------------------------
  // PEQUENO DELAY
  // ----------------------------------------------------------

  delay(2);

}
