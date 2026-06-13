#include "IRremote.h"
#include "IRremoteInt.h"
#include <avr/interrupt.h>

volatile irparams_t irparams;

int MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}

void IRsend::sendNEC(unsigned long data, int nbits)
{
  enableIROut(38);
  mark(NEC_HDR_MARK);
  space(NEC_HDR_SPACE);
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      mark(NEC_BIT_MARK);
      space(NEC_ONE_SPACE);
    }
    else {
      mark(NEC_BIT_MARK);
      space(NEC_ZERO_SPACE);
    }
    data <<= 1;
  }
  mark(NEC_BIT_MARK);
  space(0);
}

void IRsend::sendSony(unsigned long data, int nbits) {
  enableIROut(40);
  mark(SONY_HDR_MARK);
  space(SONY_HDR_SPACE);
  data = data << (32 - nbits);
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      mark(SONY_ONE_MARK);
      space(SONY_HDR_SPACE);
    }
    else {
      mark(SONY_ZERO_MARK);
      space(SONY_HDR_SPACE);
    }
    data <<= 1;
  }
}

void IRsend::sendRaw(unsigned int buf[], int len, int hz)
{
  enableIROut(hz);
  for (int i = 0; i < len; i++) {
    if (i & 1) {
      space(buf[i]);
    }
    else {
      mark(buf[i]);
    }
  }
  space(0);
}

void IRsend::sendRC5(unsigned long data, int nbits)
{
  enableIROut(36);
  data = data << (32 - nbits);
  mark(RC5_T1);
  space(RC5_T1);
  mark(RC5_T1);
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space(RC5_T1);
      mark(RC5_T1);
    }
    else {
      mark(RC5_T1);
      space(RC5_T1);
    }
    data <<= 1;
  }
  space(0);
}

void IRsend::sendRC6(unsigned long data, int nbits)
{
  enableIROut(36);
  data = data << (32 - nbits);
  mark(RC6_HDR_MARK);
  space(RC6_HDR_SPACE);
  mark(RC6_T1);
  space(RC6_T1);
  int t;
  for (int i = 0; i < nbits; i++) {
    if (i == 3) {
      t = 2 * RC6_T1;
    }
    else {
      t = RC6_T1;
    }
    if (data & TOPBIT) {
      mark(t);
      space(t);
    }
    else {
      space(t);
      mark(t);
    }
    data <<= 1;
  }
  space(0);
}

void IRsend::sendPanasonic(unsigned int address, unsigned long data) {
    enableIROut(35);
    mark(PANASONIC_HDR_MARK);
    space(PANASONIC_HDR_SPACE);

    for(int i=0;i<16;i++)
    {
        mark(PANASONIC_BIT_MARK);
        if (address & 0x8000) {
            space(PANASONIC_ONE_SPACE);
        } else {
            space(PANASONIC_ZERO_SPACE);
        }
        address <<= 1;
    }
    for (int i=0; i < 32; i++) {
        mark(PANASONIC_BIT_MARK);
        if (data & TOPBIT) {
            space(PANASONIC_ONE_SPACE);
        } else {
            space(PANASONIC_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(PANASONIC_BIT_MARK);
    space(0);
}

void IRsend::sendJVC(unsigned long data, int nbits, int repeat)
{
    enableIROut(38);
    data = data << (32 - nbits);
    if (!repeat){
        mark(JVC_HDR_MARK);
        space(JVC_HDR_SPACE);
    }
    for (int i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(JVC_BIT_MARK);
            space(JVC_ONE_SPACE);
        }
        else {
            mark(JVC_BIT_MARK);
            space(JVC_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(JVC_BIT_MARK);
    space(0);
}

void IRsend::sendSAMSUNG(unsigned long data, int nbits)
{
  enableIROut(38);
  mark(SAMSUNG_HDR_MARK);
  space(SAMSUNG_HDR_SPACE);
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      mark(SAMSUNG_BIT_MARK);
      space(SAMSUNG_ONE_SPACE);
    }
    else {
      mark(SAMSUNG_BIT_MARK);
      space(SAMSUNG_ZERO_SPACE);
    }
    data <<= 1;
  }
  mark(SAMSUNG_BIT_MARK);
  space(0);
}

void IRsend::mark(int time) {
  TIMER_ENABLE_PWM;
  if (time > 0) delayMicroseconds(time);
}

void IRsend::space(int time) {
  TIMER_DISABLE_PWM;
  if (time > 0) delayMicroseconds(time);
}

void IRsend::enableIROut(int khz) {
  TIMER_DISABLE_INTR;
  pinMode(TIMER_PWM_PIN, OUTPUT);
  digitalWrite(TIMER_PWM_PIN, LOW);
  TIMER_CONFIG_KHZ(khz);
}

IRrecv::IRrecv(int recvpin)
{
  irparams.recvpin = recvpin;
  irparams.blinkflag = 0;
}

void IRrecv::enableIRIn() {
  cli();
  TIMER_CONFIG_NORMAL();
  TIMER_ENABLE_INTR;
  TIMER_RESET;
  sei();
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
  pinMode(irparams.recvpin, INPUT);
}

void IRrecv::blink13(int blinkflag)
{
  irparams.blinkflag = blinkflag;
  if (blinkflag)
    pinMode(BLINKLED, OUTPUT);
}

ISR(TIMER_INTR_NAME)
{
  TIMER_RESET;
  uint8_t irdata = (uint8_t)digitalRead(irparams.recvpin);
  irparams.timer++;
  if (irparams.rawlen >= RAWBUF) {
    irparams.rcvstate = STATE_STOP;
  }
  switch(irparams.rcvstate) {
  case STATE_IDLE:
    if (irdata == MARK) {
      if (irparams.timer < GAP_TICKS) {
        irparams.timer = 0;
      }
      else {
        irparams.rawlen = 0;
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
    }
    break;
  case STATE_MARK:
    if (irdata == SPACE) {
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE:
    if (irdata == MARK) {
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_MARK;
    }
    else {
      if (irparams.timer > GAP_TICKS) {
        irparams.rcvstate = STATE_STOP;
      }
    }
    break;
  case STATE_STOP:
    if (irdata == MARK) {
      irparams.timer = 0;
    }
    break;
  }
  if (irparams.blinkflag) {
    if (irdata == MARK) {
      BLINKLED_ON();
    }
    else {
      BLINKLED_OFF();
    }
  }
}

void IRrecv::resume() {
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
}

int IRrecv::decode(decode_results *results) {
  results->rawbuf = irparams.rawbuf;
  results->rawlen = irparams.rawlen;
  if (irparams.rcvstate != STATE_STOP) {
    return ERR;
  }
  if (decodeNEC(results)) {
    return DECODED;
  }
  if (decodeSony(results)) {
    return DECODED;
  }
  if (decodeSanyo(results)) {
    return DECODED;
  }
  if (decodeMitsubishi(results)) {
    return DECODED;
  }
  if (decodeRC5(results)) {
    return DECODED;
  }
  if (decodeRC6(results)) {
    return DECODED;
  }
  if (decodePanasonic(results)) {
    return DECODED;
  }
  if (decodeLG(results)) {
    return DECODED;
  }
  if (decodeJVC(results)) {
    return DECODED;
  }
  if (decodeSAMSUNG(results)) {
    return DECODED;
  }
  if (decodeHash(results)) {
    return DECODED;
  }
  resume();
  return ERR;
}

long IRrecv::decodeNEC(decode_results *results) {
  long data = 0;
  int offset = 1;
  if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
    return ERR;
  }
  offset++;
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE) &&
    MATCH_MARK(results->rawbuf[offset+1], NEC_BIT_MARK)) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = NEC;
    return DECODED;
  }
  if (irparams.rawlen < 2 * NEC_BITS + 4) {
    return ERR;
  }
  if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  for (int i = 0; i < NEC_BITS; i++) {
    if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
  }
  results->bits = NEC_BITS;
  results->value = data;
  results->decode_type = NEC;
  return DECODED;
}

long IRrecv::decodeSony(decode_results *results) {
  long data = 0;
  if (irparams.rawlen < 2 * SONY_BITS + 2) {
    return ERR;
  }
  int offset = 0;
  if (results->rawbuf[offset] < SONY_DOUBLE_SPACE_USECS) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = SANYO;
    return DECODED;
  }
  offset++;
  if (!MATCH_MARK(results->rawbuf[offset], SONY_HDR_MARK)) {
    return ERR;
  }
  offset++;
  while (offset + 1 < irparams.rawlen) {
    if (!MATCH_SPACE(results->rawbuf[offset], SONY_HDR_SPACE)) {
      break;
    }
    offset++;
    if (MATCH_MARK(results->rawbuf[offset], SONY_ONE_MARK)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_MARK(results->rawbuf[offset], SONY_ZERO_MARK)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
  }
  results->bits = (offset - 1) / 2;
  if (results->bits < 12) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = SONY;
  return DECODED;
}

long IRrecv::decodeSanyo(decode_results *results) {
  long data = 0;
  if (irparams.rawlen < 2 * SANYO_BITS + 2) {
    return ERR;
  }
  int offset = 0;
  if (results->rawbuf[offset] < SANYO_DOUBLE_SPACE_USECS) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = SANYO;
    return DECODED;
  }
  offset++;
  if (!MATCH_MARK(results->rawbuf[offset], SANYO_HDR_MARK)) {
    return ERR;
  }
  offset++;
  if (!MATCH_MARK(results->rawbuf[offset], SANYO_HDR_MARK)) {
    return ERR;
  }
  offset++;
  while (offset + 1 < irparams.rawlen) {
    if (!MATCH_SPACE(results->rawbuf[offset], SANYO_HDR_SPACE)) {
      break;
    }
    offset++;
    if (MATCH_MARK(results->rawbuf[offset], SANYO_ONE_MARK)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_MARK(results->rawbuf[offset], SANYO_ZERO_MARK)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
  }
  results->bits = (offset - 1) / 2;
  if (results->bits < 12) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = SANYO;
  return DECODED;
}

long IRrecv::decodeMitsubishi(decode_results *results) {
  long data = 0;
  if (irparams.rawlen < 2 * MITSUBISHI_BITS + 2) {
    return ERR;
  }
  int offset = 0;
  offset++;
  if (!MATCH_MARK(results->rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  while (offset + 1 < irparams.rawlen) {
    if (MATCH_MARK(results->rawbuf[offset], MITSUBISHI_ONE_MARK)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_MARK(results->rawbuf[offset], MITSUBISHI_ZERO_MARK)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
    if (!MATCH_SPACE(results->rawbuf[offset], MITSUBISHI_HDR_SPACE)) {
      break;
    }
    offset++;
  }
  results->bits = (offset - 1) / 2;
  if (results->bits < MITSUBISHI_BITS) {
    results->bits = 0;
    return ERR;
  }
  results->value = data;
  results->decode_type = MITSUBISHI;
  return DECODED;
}

int IRrecv::getRClevel(decode_results *results, int *offset, int *used, int t1) {
  if (*offset >= results->rawlen) {
    return SPACE;
  }
  int width = results->rawbuf[*offset];
  int val = ((*offset) % 2) ? MARK : SPACE;
  int correction = (val == MARK) ? MARK_EXCESS : - MARK_EXCESS;
  int avail;
  if (MATCH(width, t1 + correction)) {
    avail = 1;
  }
  else if (MATCH(width, 2*t1 + correction)) {
    avail = 2;
  }
  else if (MATCH(width, 3*t1 + correction)) {
    avail = 3;
  }
  else {
    return -1;
  }
  (*used)++;
  if (*used >= avail) {
    *used = 0;
    (*offset)++;
  }
  return val;
}

long IRrecv::decodeRC5(decode_results *results) {
  if (irparams.rawlen < MIN_RC5_SAMPLES + 2) {
    return ERR;
  }
  int offset = 1;
  long data = 0;
  int used = 0;
  if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
  if (getRClevel(results, &offset, &used, RC5_T1) != SPACE) return ERR;
  if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
  int nbits;
  for (nbits = 0; offset < irparams.rawlen; nbits++) {
    int levelA = getRClevel(results, &offset, &used, RC5_T1);
    int levelB = getRClevel(results, &offset, &used, RC5_T1);
    if (levelA == SPACE && levelB == MARK) {
      data = (data << 1) | 1;
    }
    else if (levelA == MARK && levelB == SPACE) {
      data <<= 1;
    }
    else {
      return ERR;
    }
  }
  results->bits = nbits;
  results->value = data;
  results->decode_type = RC5;
  return DECODED;
}

long IRrecv::decodeRC6(decode_results *results) {
  if (results->rawlen < MIN_RC6_SAMPLES) {
    return ERR;
  }
  int offset = 1;
  if (!MATCH_MARK(results->rawbuf[offset], RC6_HDR_MARK)) {
    return ERR;
  }
  offset++;
  if (!MATCH_SPACE(results->rawbuf[offset], RC6_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  long data = 0;
  int used = 0;
  if (getRClevel(results, &offset, &used, RC6_T1) != MARK) return ERR;
  if (getRClevel(results, &offset, &used, RC6_T1) != SPACE) return ERR;
  int nbits;
  for (nbits = 0; offset < results->rawlen; nbits++) {
    int levelA, levelB;
    levelA = getRClevel(results, &offset, &used, RC6_T1);
    if (nbits == 3) {
      if (levelA != getRClevel(results, &offset, &used, RC6_T1)) return ERR;
    }
    levelB = getRClevel(results, &offset, &used, RC6_T1);
    if (nbits == 3) {
      if (levelB != getRClevel(results, &offset, &used, RC6_T1)) return ERR;
    }
    if (levelA == MARK && levelB == SPACE) {
      data = (data << 1) | 1;
    }
    else if (levelA == SPACE && levelB == MARK) {
      data <<= 1;
    }
    else {
      return ERR;
    }
  }
  results->bits = nbits;
  results->value = data;
  results->decode_type = RC6;
  return DECODED;
}

long IRrecv::decodePanasonic(decode_results *results) {
    unsigned long long data = 0;
    int offset = 1;
    if (!MATCH_MARK(results->rawbuf[offset], PANASONIC_HDR_MARK)) {
        return ERR;
    }
    offset++;
    if (!MATCH_MARK(results->rawbuf[offset], PANASONIC_HDR_SPACE)) {
        return ERR;
    }
    offset++;
    for (int i = 0; i < PANASONIC_BITS; i++) {
        if (!MATCH_MARK(results->rawbuf[offset++], PANASONIC_BIT_MARK)) {
            return ERR;
        }
        if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ONE_SPACE)) {
            data = (data << 1) | 1;
        } else if (MATCH_SPACE(results->rawbuf[offset],PANASONIC_ZERO_SPACE)) {
            data <<= 1;
        } else {
            return ERR;
        }
        offset++;
    }
    results->value = (unsigned long)data;
    results->panasonicAddress = (unsigned int)(data >> 32);
    results->decode_type = PANASONIC;
    results->bits = PANASONIC_BITS;
    return DECODED;
}

long IRrecv::decodeLG(decode_results *results) {
    long data = 0;
    int offset = 1;
    if (!MATCH_MARK(results->rawbuf[offset], LG_HDR_MARK)) {
        return ERR;
    }
    offset++;
    if (irparams.rawlen < 2 * LG_BITS + 1 ) {
        return ERR;
    }
    if (!MATCH_SPACE(results->rawbuf[offset], LG_HDR_SPACE)) {
        return ERR;
    }
    offset++;
    for (int i = 0; i < LG_BITS; i++) {
        if (!MATCH_MARK(results->rawbuf[offset], LG_BIT_MARK)) {
            return ERR;
        }
        offset++;
        if (MATCH_SPACE(results->rawbuf[offset], LG_ONE_SPACE)) {
            data = (data << 1) | 1;
        }
        else if (MATCH_SPACE(results->rawbuf[offset], LG_ZERO_SPACE)) {
            data <<= 1;
        }
        else {
            return ERR;
        }
        offset++;
    }
    if (!MATCH_MARK(results->rawbuf[offset], LG_BIT_MARK)){
        return ERR;
    }
    results->bits = LG_BITS;
    results->value = data;
    results->decode_type = LG;
    return DECODED;
}

long IRrecv::decodeJVC(decode_results *results) {
    long data = 0;
    int offset = 1;
    if (irparams.rawlen - 1 == 33 &&
        MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK) &&
        MATCH_MARK(results->rawbuf[irparams.rawlen-1], JVC_BIT_MARK)) {
        results->bits = 0;
        results->value = REPEAT;
        results->decode_type = JVC;
        return DECODED;
    }
    if (!MATCH_MARK(results->rawbuf[offset], JVC_HDR_MARK)) {
        return ERR;
    }
    offset++;
    if (irparams.rawlen < 2 * JVC_BITS + 1 ) {
        return ERR;
    }
    if (!MATCH_SPACE(results->rawbuf[offset], JVC_HDR_SPACE)) {
        return ERR;
    }
    offset++;
    for (int i = 0; i < JVC_BITS; i++) {
        if (!MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK)) {
            return ERR;
        }
        offset++;
        if (MATCH_SPACE(results->rawbuf[offset], JVC_ONE_SPACE)) {
            data = (data << 1) | 1;
        }
        else if (MATCH_SPACE(results->rawbuf[offset], JVC_ZERO_SPACE)) {
            data <<= 1;
        }
        else {
            return ERR;
        }
        offset++;
    }
    if (!MATCH_MARK(results->rawbuf[offset], JVC_BIT_MARK)){
        return ERR;
    }
    results->bits = JVC_BITS;
    results->value = data;
    results->decode_type = JVC;
    return DECODED;
}

long IRrecv::decodeSAMSUNG(decode_results *results) {
  long data = 0;
  int offset = 1;
  if (!MATCH_MARK(results->rawbuf[offset], SAMSUNG_HDR_MARK)) {
    return ERR;
  }
  offset++;
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(results->rawbuf[offset], SAMSUNG_RPT_SPACE) &&
    MATCH_MARK(results->rawbuf[offset+1], SAMSUNG_BIT_MARK)) {
    results->bits = 0;
    results->value = REPEAT;
    results->decode_type = SAMSUNG;
    return DECODED;
  }
  if (irparams.rawlen < 2 * SAMSUNG_BITS + 4) {
    return ERR;
  }
  if (!MATCH_SPACE(results->rawbuf[offset], SAMSUNG_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  for (int i = 0; i < SAMSUNG_BITS; i++) {
    if (!MATCH_MARK(results->rawbuf[offset], SAMSUNG_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(results->rawbuf[offset], SAMSUNG_ONE_SPACE)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_SPACE(results->rawbuf[offset], SAMSUNG_ZERO_SPACE)) {
      data <<= 1;
    }
    else {
      return ERR;
    }
    offset++;
  }
  results->bits = SAMSUNG_BITS;
  results->value = data;
  results->decode_type = SAMSUNG;
  return DECODED;
}

int IRrecv::compare(unsigned int oldval, unsigned int newval) {
  if (newval < oldval * .8) {
    return 0;
  }
  else if (oldval < newval * .8) {
    return 2;
  }
  else {
    return 1;
  }
}

#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

long IRrecv::decodeHash(decode_results *results) {
  if (results->rawlen < 6) {
    return ERR;
  }
  long hash = FNV_BASIS_32;
  for (int i = 1; i+2 < results->rawlen; i++) {
    int value =  compare(results->rawbuf[i], results->rawbuf[i+2]);
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  results->value = hash;
  results->bits = 32;
  results->decode_type = UNKNOWN;
  return DECODED;
}

void IRsend::sendSharpRaw(unsigned long data, int nbits) {
  enableIROut(38);
  for (int n = 0; n < 3; n++) {
    for (int i = 1 << (nbits-1); i > 0; i>>=1) {
      if (data & i) {
        mark(SHARP_BIT_MARK);
        space(SHARP_ONE_SPACE);
      }
      else {
        mark(SHARP_BIT_MARK);
        space(SHARP_ZERO_SPACE);
      }
    }
    mark(SHARP_BIT_MARK);
    space(SHARP_ZERO_SPACE);
    delay(40);
    data = data ^ SHARP_TOGGLE_MASK;
  }
}

void IRsend::sendSharp(unsigned int address, unsigned int command) {
  sendSharpRaw((address << 10) | (command << 2) | 2, 15);
}

void IRsend::sendDISH(unsigned long data, int nbits) {
  enableIROut(56);
  mark(DISH_HDR_MARK);
  space(DISH_HDR_SPACE);
  for (int i = 0; i < nbits; i++) {
    if (data & DISH_TOP_BIT) {
      mark(DISH_BIT_MARK);
      space(DISH_ONE_SPACE);
    }
    else {
      mark(DISH_BIT_MARK);
      space(DISH_ZERO_SPACE);
    }
    data <<= 1;
  }
}
