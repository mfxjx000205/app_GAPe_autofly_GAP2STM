/**
 * 
  ____   _____  _   _         _   _        _    ____  ___ 
 / ___| | ____|| | | |       | \ | |  ___ | |_ / ___||_ _|
 \___ \ |  _|  | | | | _____ |  \| | / _ \| __|\___ \ | | 
  ___) || |___ | |_| ||_____|| |\  ||  __/| |_  ___) || | 
 |____/ |_____| \___/        |_| \_| \___| \__||____/|___|
                                                          
 *
 * AI-deck GAP8
 * Copyright (C) 2022 SEU-NetSI
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Send Any log From GAP8 to PC via WiFi
 */

#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/buffer.h"
#include "stdio.h"
#include "cpx.h"
#include "WiFiConnect.h"

static CPX_Packet_t txp;        //public CPX control message queue
static CPXPacket_t CPXWiFiStream;  //WiFiStream CPX message queue

/**
 * @brief Setup WiFi AP.
 * @param None
 * @return None
 * @author Bitcraze
*/
void setupWiFi(void) {
    static char ssid[] = "WiFi streaming example";
    cpxPrintToConsole(LOG_TO_CRTP, "Setting up WiFi AP\n");
    // Set up the routing for the WiFi CTRL packets
    txp.route.destination = CPX_T_ESP32;
    rxp.route.source = CPX_T_GAP8;
    txp.route.function = CPX_F_WIFI_CTRL;
    txp.route.version = CPX_VERSION;
    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) txp.data;

    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    txp.dataLength = sizeof(ssid);
    cpxSendPacketBlocking(&txp);

    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01;
    txp.dataLength = 2;
    cpxSendPacketBlocking(&txp);
}

/**
 * @brief Send WiFi Stream to PC: Minimum unit.
 * @param WiFiStream: WiFi Stream to send.
 * @return None
 * @author Hanjie Yu
*/
void SendWiFiStream(const WiFi_Stream_Packet_t* WiFiStream){
    uint16_t bandwidth = 0;
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &WiFiStream.route);
    memcpy(CPXWiFiStream.data, WiFiStream.data, sizeof(WiFi_Stream_Packet_t));
    CPXWiFiStream.dataLength = sizeof(WiFi_Stream_Packet_t);
    cpxSendPacketBlocking(&CPXWiFiStream);
    cpxPrintToConsole(LOG_TO_CRTP, "Send WiFiStream Successfully! bandwidth=%d/s\n",bandwidth);
}

