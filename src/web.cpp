// web.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <WebServer.h>
#include "parameter.h"
#include "web.h"

static WebServer g_server(80);
static bool g_server_started = false;

static void handle_root()
{
    const char *page =
        "<!doctype html><html><head><title>VST ONE</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
        "<style>"
        "body{margin:0;background:#000;color:#fff;font-family:Arial,Helvetica,sans-serif;}"
        ".wrap{position:relative;width:100vw;max-width:100vw;}"
        "#frame{display:block;width:100vw;height:auto;}"
        "#overlay{position:absolute;left:0;top:0;width:100%;height:100%;}"
        "#label{position:absolute;left:0;top:0;width:100%;"
        "box-sizing:border-box;padding:6px 10px;"
        "background:rgba(0,0,0,0.6);color:#fff;font-size:14px;}"
        "</style></head>"
        "<body><div class='wrap'>"
        "<img id='frame'/>"
        "<canvas id='overlay'></canvas>"
        "<div id='label'></div>"
        "</div>"
        "<script>"
        "const img=document.getElementById('frame');"
        "const canvas=document.getElementById('overlay');"
        "const label=document.getElementById('label');"
        "const ctx=canvas.getContext('2d');"
        "let lastBoxes=[];"
        "let lastLabel='';"
        "const colors={0:'#00c0ff',1:'#ff4d4d',2:'#ffe14d',3:'#4dff88',4:'#b84dff'};"
        "const names={0:'Apis mellifera',1:'Vespa crabro',2:'Vespula sp.',3:'Vespa velutina',4:'Class 4'};"
        "const coordsCenter=true;"
        "function resizeCanvas(){canvas.width=img.clientWidth;canvas.height=img.clientHeight;}"
        "function drawBoxes(){"
        "if(!img.naturalWidth||!img.naturalHeight) return;"
        "resizeCanvas();"
        "ctx.clearRect(0,0,canvas.width,canvas.height);"
        "const sx=canvas.width/img.naturalWidth;"
        "const sy=canvas.height/img.naturalHeight;"
        "lastBoxes.forEach(b=>{"
        "const c=colors[b.target]||'#00ffff';"
        "ctx.strokeStyle=c;ctx.lineWidth=2;"
        "let x=b.x, y=b.y, w=b.w, h=b.h;"
        "if(coordsCenter){x=b.x-(b.w/2); y=b.y-(b.h/2);}"
        "ctx.strokeRect(x*sx,y*sy,w*sx,h*sy);"
        "});"
        "label.textContent=lastLabel;"
        "}"
        "async function refreshFrame(){"
        "const r=await fetch('/frame.b64?t='+Date.now());"
        "if(!r.ok) return;"
        "const b64=await r.text();"
        "if(b64 && b64.length>0){img.src='data:image/jpeg;base64,'+b64;}"
        "}"
        "async function refreshInf(){"
        "const r=await fetch('/inf.json?t='+Date.now());"
        "if(!r.ok) return;"
        "const data=await r.json();"
        "lastBoxes=(data.boxes||[]).map(b=>({"
        "target:parseInt(b.target||0),"
        "score:parseInt(b.score||0),"
        "x:parseFloat(b.x||0),"
        "y:parseFloat(b.y||0),"
        "w:parseFloat(b.w||0),"
        "h:parseFloat(b.h||0)}));"
        "if(lastBoxes.length>0){"
        "const best=lastBoxes.reduce((a,b)=>b.score>a.score?b:a,lastBoxes[0]);"
        "const name=names[best.target]||('Class '+best.target);"
        "lastLabel=name+' | '+best.score+'%';"
        "}else{lastLabel='No detection';}"
        "drawBoxes();"
        "}"
        "img.onload=drawBoxes;"
        "setInterval(()=>{refreshFrame();refreshInf();},1000);"
        "</script>"
        "</body></html>";
    g_server.send(200, "text/html", page);
}

static void handle_frame_b64()
{
    if (!web_frame_available())
    {
        g_server.send(503, "text/plain", "No frame");
        return;
    }

    const uint8_t *ptr = web_frame_ptr();
    size_t len = web_frame_len();
    if (ptr == nullptr || len == 0)
    {
        g_server.send(503, "text/plain", "No frame");
        return;
    }

    g_server.sendHeader("Cache-Control", "no-store");
    g_server.setContentLength(len);
    g_server.send(200, "text/plain", "");
    WiFiClient client = g_server.client();
    client.write(ptr, len);
}

static void handle_inf_json()
{
    if (!web_inf_available())
    {
        g_server.send(503, "text/plain", "No inference");
        return;
    }
    const char *ptr = web_inf_ptr();
    size_t len = web_inf_len();
    if (ptr == nullptr || len == 0)
    {
        g_server.send(503, "text/plain", "No inference");
        return;
    }
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.setContentLength(len);
    g_server.send(200, "application/json", "");
    WiFiClient client = g_server.client();
    client.write((const uint8_t *)ptr, len);
}

bool start_wifi_client()
{
    if (WEB_SSID[0] == '\0')
    {
        Serial.println("WiFi STA: SSID is empty (skip)");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WEB_SSID, WEB_PASSWORD);

    const uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if ((millis() - start_ms) > 15000)
        {
            Serial.println("WiFi STA: connect timeout");
            return false;
        }
        delay(250);
    }

    Serial.print("WiFi STA: connected, IP=");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi STA: SSID=");
    Serial.println(WEB_SSID);
    return true;
}

bool start_wifi_ap()
{
    WiFi.mode(WIFI_AP);

    String base = (WEB_SSID[0] != '\0') ? WEB_SSID : WEB_AP_BASE;
    String ssid = base;
    if (WEB_AP_APPEND_MAC)
    {
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) == ESP_OK)
        {
            char suffix[10] = {0};
            snprintf(suffix, sizeof(suffix), "-%02X%02X%02X", mac[3], mac[4], mac[5]);
            ssid += suffix;
        }
    }

    bool ok = false;
    if (WEB_PASSWORD[0] == '\0' || strlen(WEB_PASSWORD) < 8)
    {
        // Open AP if password is empty or too short for WPA2
        ok = WiFi.softAP(ssid.c_str());
    }
    else
    {
        ok = WiFi.softAP(ssid.c_str(), WEB_PASSWORD);
    }

    if (!ok)
    {
        Serial.println("WiFi AP: start failed");
        return false;
    }

    IPAddress ip = WiFi.softAPIP();
    Serial.print("WiFi AP: started, IP=");
    Serial.println(ip);
    Serial.print("WiFi AP: SSID=");
    Serial.println(ssid);

    g_server.on("/", handle_root);
    g_server.on("/frame.b64", handle_frame_b64);
    g_server.on("/inf.json", handle_inf_json);
    g_server.onNotFound([]() {
        g_server.send(404, "text/plain", "Not found");
    });
    g_server.begin();
    g_server_started = true;
    Serial.println("HTTPD: started on port 80");

    return true;
}

void web_init()
{
    g_server_started = false;

    if (WEB_MODE == 0)
    {
        Serial.println("WiFi: WEB_MODE=0 (disabled)");
        WiFi.mode(WIFI_OFF);
        return;
    }

    if (WEB_MODE == 1)
    {
        Serial.println("WiFi: WEB_MODE=1 (client)");
        start_wifi_client();
        return;
    }

    if (WEB_MODE == 2)
    {
        Serial.println("WiFi: WEB_MODE=2 (AP)");
        start_wifi_ap();
        return;
    }

    Serial.println("WiFi: unknown WEB_MODE");
}

void web_loop()
{
    if (g_server_started)
    {
        g_server.handleClient();
    }
}
