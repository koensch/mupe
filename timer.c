#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "timer.h"
#include "app_state.h"
#include "logger.h"
#include "bgnet.h"

void *timer_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    time_t t;

    unsigned char RxBuf[BGNET_RXTXRAW_SIZE], TxBuf[BGNET_RXTXRAW_SIZE]; // local read buffer
    size_t RxBufLen = 0, TxBufLen = 0, Counter;

    Counter = 0;
    while (g_running) {

        // get time, update our system time 
        time(&t);
        pCtrl->BGNET.SERVICE.SystemTime.value.tTime = t;
        pCtrl->BGNET.SERVICE.SystemTime.source = BGN_SOURCE_ACP;
        CounterUp(&pCtrl->BGNET.SERVICE.SystemTime.Counter, t);

        // check if there is serial data ready
        pthread_mutex_lock(&g_data_mutex);
        if (g_fastnet_data_lenRx > 0) {
            // data is present, wait for frame completion
            if (g_fastnet_data_counter > SerialCounter_Reset) {
                // wait for more, decrement counter
                g_fastnet_data_counter--;
            } else {
                // timeout, end of frame, get data into local buffer
                memcpy(RxBuf, g_last_fastnet_dataRx, g_fastnet_data_lenRx);
                RxBufLen = g_fastnet_data_lenRx;
                g_fastnet_data_lenRx = 0; // reset receiving buffer (so serial port can handle the next frame)
            }
        } else {
            // no data present, count down to the bottom (prevent underflow)
            if (g_fastnet_data_counter > SerialCounter_Bottom) {
                g_fastnet_data_counter--;
            }
        }
        pthread_mutex_unlock(&g_data_mutex);

        // handle if we got something
        if (RxBufLen > 0) {
            // convert frame into printable text, byte spaced
            // at this time i'm note sure if this is a 'real' BGNET frame. I treat it like a raw byte stream
            //byte2ascii(asciibuf, sizeof(asciibuf), RxBuf, RxBufLen, BYTE2ASCII_SPACED);
            //log_debug("Timer %d msec, %d bytes: %s", RxCounter, RxBufLen, asciibuf);

            /* do here the incoming message handling
            - until no we have only processed streams of data bytes coming from the serial port. In this protocol
              messages are separated by 'silence', typically around 45msec.
            - every frame we receive may contain 1 or more 'messages'.
            - we pass the received frame on to our 'frame decoder' that will separate the 'sub-frames'.
            - the 'frame decoder' check the CR in the header and body (if present) of every message
            - the 'frame decoder' will recognise the TO, FROM and CMD and handles it according to the 'device type'
              behaviour.
            - data from the received message will be stored in the channel/device/command data structure. A 'new' counter will be set
              for the system, the device, and each value within the message. This allows us to detect 'new' data.
            - data is stored in a 'virtual device' that mirrors the sender. Most message have a 'everyone' TO address,
              saving received data at destination addresses causes multiplication of data.
            - the architecture we are dealing with allow multiple identical stations to be present, but only 1 can have
              the actual sensor and others must be placed in the 'repeater' mode. These are not 'senders', but 'listeners'
            - if appicable, the 'frame decoder' will create a output message. All output messages are concatenated
              into a multi-frame packet. Every device must create their own frame (because of the FROM address), but
              all frames can be packed together in the same write buffer and send in 1 go.
            - Because the Rx and Tx signals are connected, we get our own Tx messages back into our Rx buffer. The frame decoder
              will ignore any incoming message with a FROM address that is our own.
            - The device data structure can be accessed (read & write) via de web interface. We can see what data is there
              and do something that triggers an output message to be send.
            - the writing of output messages is done by the 'serial-thread'. When data is in the TxBuf, it will output
              and reset it
            - The 'timer-thread' will trigger the 'frame decoder' to make sure it also runs where there is no incoming
              messages. When the IO bus has only 'passive' units, no data is send and the heartbeat has to come from us.
            */
           if (Counter < 1) {
              // we throw out the first message; usually the first message we only have the last fragment
              // and we get checksum error. So we skip the first message
              Counter++;
           } else {
              switch (pCtrl->pCfg->fastnet.handler) {
                  case BGNET_HANDLER_NETIO:
                    bgnet_decode_frame(pCtrl, RxBuf, RxBufLen);
                  break;
               default:
                  log_error("Timer: This should never happen");
                  break;
              }
           }
           RxBufLen = 0; // dump contents of local readbuffer, assume all was processed
        }

        // we do NOT do this every msec, but after some count-down was expired.
        // note that any application can trigger an instant execution of the device processing, by setting this counter to 0
        if (pCtrl->iDeviceCounter > 0) {
          pCtrl->iDeviceCounter--;
        } else {

          // This is what we call "watchdog" checking. WE do the count-down. As long as the PILOT
          // sends it's "PILOT_HELLO" message, we know the boat has a working PILOT to monitor and command the autopilot
          switch (g_fastnet_mode) {
            case BGN_FASTNET_MODE_PASSIVE:
            case BGN_FASTNET_MODE_RW:
              if (pCtrl->BGNET.SERVICE.PILOTWD.value.nLong > 0) {
                pCtrl->BGNET.SERVICE.PILOTWD.value.nLong--;
                pCtrl->BGNET.SERVICE.PILOTWD.source = BGN_SOURCE_ACP;
                CounterUp(&pCtrl->BGNET.SERVICE.PILOTWD.Counter, t);
              } else {
                log_error("PILOTWD expired at %d, fallback to STBY/COMP", pCtrl->BGNET.SERVICE.PILOTWD.value.nLong);
                // force controller into STBY
                pCtrl->BGNET.ACP.STATE.value.nByte = (pCtrl->BGNET.ACP.STATE.value.nByte & ~BGN_AP_MODE1_MASK_STATE) | (BGN_AP_MODE1_STBY & BGN_AP_MODE1_MASK_STATE);
                pCtrl->BGNET.ACP.STATE.source = BGN_SOURCE_ACP;
                CounterUp(&pCtrl->BGNET.ACP.STATE.Counter, t);
                // force controller into COMP
                pCtrl->BGNET.ACP.MODE.value.nByte = (pCtrl->BGNET.ACP.MODE.value.nByte & ~BGN_AP_MODE2_MASK_MODE) | (BGN_AP_MODE2_COMP & BGN_AP_MODE2_MASK_MODE);
                pCtrl->BGNET.ACP.MODE.source = BGN_SOURCE_ACP;
                CounterUp(&pCtrl->BGNET.ACP.MODE.Counter, t);
              }
              break;
          }

          //device processing start
          //log_info("Device Processing %d", pCtrl->iDeviceRun);
          TxBufLen = bgnet_process(pCtrl, TxBuf, sizeof(TxBuf));
          // reset device processing counter (we do not want to run every MSEC1)
          pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;

          //timer(s)
          int a = TimerDone(&pCtrl->BGNET.ACP.AUTO, t);
          if (a < 0) {
            // timer running
          } else {
            // timer expired, reset MEM bit
            pCtrl->BGNET.ACP.STATE.value.nByte = (pCtrl->BGNET.ACP.STATE.value.nByte & (!BGN_AP_MODE1_MEM));
          }

          // increment a run counter, wrap around to avoid overruns
          pCtrl->iDeviceRun++;
          if (pCtrl->iDeviceRun > 999) {
            pCtrl->iDeviceRun = 0;
          }

          // copy any output frame (from device processor) into the Txbuffer. I do not 'append' to any existing
          // contents, but overwrite what there might be. The reason is that i do not want to risk a buffer overflow
          if (TxBufLen > 0) {
            if ((TxBufLen * 10) > (sizeof(g_last_fastnet_dataTx) * 9)) {
              log_error("Tx buffer dangerously full (>90%)");
            }
            pthread_mutex_lock(&g_data_mutex);
            memcpy(g_last_fastnet_dataTx, &TxBuf, TxBufLen);
            g_fastnet_data_lenTx = TxBufLen;
            pthread_mutex_unlock(&g_data_mutex);
          }
        }

        // no racing allowed, use the configured timer interval, but protect against someone entering 0        
        usleep(BGNET_MIN(pCtrl->pCfg->timer.timer_interval_ms * 1000, MSEC1));
    }
    log_info("Timer: thread exiting");
    return NULL;
}
