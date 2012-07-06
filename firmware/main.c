/* Name: main.c
 * Project: hid-data, example how to use HID for data transfer
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: main.c 777 2010-01-15 18:34:48Z cs $
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "ir_ctrl.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x09, 0x00,                    //   USAGE (Undefined)
    0x82, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

struct report {
  uint8_t fmt;		/* Frame format */
  uint8_t len;		/* Number of bits received */
  uint8_t buff[6];	/* Data buffer */
};

static struct report reportBuffer;
static struct report txBuffer;

/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

typedef unsigned short ushort;

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    //eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    while (len-- > 0) {
      *((uchar*)&txBuffer + currentAddress++) = *data++;
      bytesRemaining--;
    }

    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = sizeof reportBuffer;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = sizeof txBuffer;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}


/* ------------------------------------------------------------------------- */

#define IR_TX_ON()		GTCCR |= _BV(COM1B0) 	/* Tx: Start IR burst */
#define IR_TX_38K()		OCR1C = 216; TCNT1 = 0	/* Tx: Set IR burst frequency to 38kHz */
#define IR_TX_40K()		OCR1C = 204; TCNT1 = 0	/* Tx: Set IR burst frequency to 40kHz */

static void 
ir_recieved(void)
{
  reportBuffer.fmt = IrCtrl.fmt;
  reportBuffer.len = IrCtrl.len;
  uint8_t len = (IrCtrl.len + 7) / 8;
  uint8_t i;
  if (len >= sizeof IrCtrl.buff)
    len = sizeof IrCtrl.buff;
  for (i = 0; i < len; i++)
    reportBuffer.buff[i] = IrCtrl.buff[i];
  for (; i < sizeof IrCtrl.buff; i++)
    IrCtrl.buff[i] = 0;         
  //eeprom_write_block(buf, 0, 6);
  //PORTB ^= _BV(PB0);

  IrCtrl.state = IR_IDLE;		/* Ready to receive next frame */
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

#define EEPROM_OSCCAL	((uchar *)511)

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(EEPROM_OSCCAL, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */

#define WHITE_LED 3
#define YELLOW_LED 1

int main(void)
{
    uchar   i;

    /* calibration value from last time */
    uchar calibrationValue = eeprom_read_byte(EEPROM_OSCCAL); 
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */

    DDRB = _BV(PB4); //PB4を出力設定
    cbi(PORTB,PB4);  //PB4をLOW出力
    DDRB |= _BV(PB0); //PB0を出力設定
    cbi(PORTB,PB0);  //PB0をLOW出力

    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    IR_initialize();

    DDRB |= _BV(PB0); //デバッグのため出力に設定
    uchar repeat = 0;

    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;){                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
        usbPoll();

        if (txBuffer.fmt != 0 && bytesRemaining == 0) {
          if (repeat == 0)
            repeat = 10;

            PORTB |= _BV(PB0);
            IR_xmit(txBuffer.fmt, txBuffer.buff, txBuffer.len);
            if (--repeat == 0)
                txBuffer.fmt = 0;
        }

        // winでは160,320では動作しない(認識後消失する)が、80だと動作。 */
        //IR_event_loop(80 * 4); /* 10ms */
        //IR_event_loop(80 * 2); /* 5ms */
        IR_event_loop(80); /* 2.5ms */

        PORTB &= ~_BV(PB0);
        if (IrCtrl.state == IR_RECVED) {
            PORTB |= _BV(PB0);
            ir_recieved(); /* 受信フレームの処理 */
        }

        if (usbInterruptIsReady()) {
          /* called after every poll of the interrupt endpoint */
          usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
