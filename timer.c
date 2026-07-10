#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "timer.h"
#include "app_state.h"
#include "logger.h"
#include "bgnprotocol.h"

void *timer_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    Config *cfg = pCtrl->pCfg;
    time_t t;

    unsigned char RxBuf[BGNET_RXTXRAW_SIZE], TxBuf[BGNET_RXTXRAW_SIZE]; // local read buffer
    size_t RxBufLen = 0, TxBufLen = 0;
    int RxCounter = 0;

    char asciibuf[2048]; // store ascii for logging

    while (g_running) {
        time(&t);
        pthread_mutex_lock(&g_data_mutex);
        
        if (g_serial_data_lenRx>0) {
            // data is present, wait for frame completion
            if (g_serial_data_counter>SerialCounter_Reset) {
                // wait for more, decrement counter
                g_serial_data_counter--;
            } else {
                // timeout, end of frame, get data into local buffer
                memcpy(RxBuf,g_last_serial_dataRx,g_serial_data_lenRx);
                RxBufLen = g_serial_data_lenRx;
                g_serial_data_lenRx = 0; // reset receiving buffer (so serial port can handle the next frame)
            }
        } else {
            // no data present, count down to the bottom (prevent underflow)
            if (g_serial_data_counter>SerialCounter_Bottom) {
                g_serial_data_counter--;
            }
            // retrieve counter so we have some clue what time elapsed between this frame and the previous.
            // it was empirically establised that this is around 45msec for this protocol
            RxCounter = g_serial_data_counter;
        }
        pthread_mutex_unlock(&g_data_mutex);

        // show what we got
        if (RxBufLen > 0) {
            // convert frame into prinatble text, byte spaced
            byte2ascii(asciibuf, sizeof(asciibuf), RxBuf, RxBufLen, BYTE2ASCII_SPACED);
            log_debug("Timer %d msec, %d bytes: %s", RxCounter, RxBufLen, asciibuf);

            CommandToAscii(asciibuf, sizeof(asciibuf), RxBuf);
            log_debug("timer: %s", asciibuf);      

            /* do here the incoming message handling
            - until no we have only processed streams of data bytes coming from the serial port. In this protocol
              messages are separated by 'silence', typically around 45msec.
            - every frame we receive may contain 1 or more 'messages'.
            - we pass the received frame on to our 'device processor' that will separate the 'sub-frames'.
            - the 'device processor' check the CR in the header and body (if present) of every message
            - the 'device processor' will recognise the TO, FROM and CMD and handles it according to the 'device type'
              behaviour.
            - data from the received message will be stored in the devices data structure. A 'new' counter will be set
              for the system, the device, and each value within the message. This allows us to skip processing parts
              where nothing has changed.
            - data is stored in a 'virtual device' that mirrors the sender. Most message have a 'everyone' TO address,
              saving received data at destination addresses causes multiplication of data.
            - the architecture we are dealing with allow multiple identical stations to be present, but only 1 can have
              the actual sensor and others must be placed in the 'repeater' mode. These are not 'senders', but 'listeners'
            - if appicable, the 'device processor' will create a output message. All output messages are concatenated
              into a multi-frame packet. Every device must create their own frame (because of the FROM address), but
              all frames can be packed together in the same write buffer and send in 1 go.
            - Because the Rx and Tx signals are connected, we get our own Tx messages back into our Rx buffer. The device
              processor will ignore any incoming message with e FROM address that is served by the device processor.
            - The device data structure can be accessed (read & write) via de web interface. We can see what data is there
              and do something that triggers an output message to be send.
            - the writing of output messages is done by the 'serial-thread'. When data is in the TxBuf, it will output
              and reset it
            - The 'timer-thread' will trigger the 'device-processor' to make sure it also runs where there is no incoming
              messages. When the IO bus has only 'passive' units, no data is send and the heartbeat has to come from us.
            */
            //handle_received_frame(pCtrl, RxBuf, RxBufLen);
            // warning: frame decoder does not lock other threads, because is works on a copy
            // of a completely received frame
            bgnet_decode_frame(pCtrl, RxBuf, RxBufLen);
            RxBufLen = 0; // dump contents of local readbuffer, all data is processed
        }

        // we do not wat to execute DEVICE processing every msec. transmisson will also take some count down device counter.
        // note that any application can trigger an instant executionm of the device processing, by setting this counter to 0
        if (pCtrl->iDeviceCounter > 0) {
          pCtrl->iDeviceCounter--;
        } else {
            log_info("Device Processing %d", pCtrl->iDeviceRun);

            // count-down PILOT flunk counter
            if (pCtrl->ACP.PILOT.value.nLong > 0) {
              pCtrl->ACP.PILOT.value.nLong--;
              pCtrl->ACP.PILOT.source = BGN_SOURCE_NONE;
              CounterUp(&pCtrl->ACP.PILOT.Counter, t);
            } else {
              // force controller into STBY/COMP
              pCtrl->ACP.STATE.value.nByte = BGN_AP_MODE1_STBY;
              pCtrl->ACP.STATE.source = BGN_SOURCE_NONE;
              CounterUp(&pCtrl->ACP.STATE.Counter, t);
              pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
              pCtrl->ACP.MODE.source = BGN_SOURCE_NONE;
              CounterUp(&pCtrl->ACP.MODE.Counter, t);
            }

            // device processing
          TxBufLen = bgnet_process(pCtrl, TxBuf, sizeof(TxBuf));
          pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;

          // increment a run counter, wrap around to avoid overruns
          pCtrl->iDeviceRun++;
          if (pCtrl->iDeviceRun > 999) {
            pCtrl->iDeviceRun = 0;
          }

          // copy any output frame (from device processor) into the Txbuffer. I do not 'append' to any existing
          // contents, but overwrite what there might be. The reason is that i do not want to risk a buffer overflow
          if (TxBufLen > 0) {
            if ((TxBufLen * 10) > (sizeof(g_last_serial_dataTx) * 9)) {
              log_error("Tx buffer dangerously full (>90%)");
            }
            pthread_mutex_lock(&g_data_mutex);
            memcpy(g_last_serial_dataTx, &TxBuf, TxBufLen);
            g_serial_data_lenTx = TxBufLen;
            pthread_mutex_unlock(&g_data_mutex);
          }
        }

        // no racing allowed
        usleep(cfg->timer_interval_ms * MSEC1);
    }
    log_info("Timer: thread exiting");
    return NULL;
}
