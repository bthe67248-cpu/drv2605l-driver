/* Host regression tests. No board or Arduino SDK needed. */
#include "drv2605l.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  uint8_t reg[256]; uint32_t tick;
  int fail_write, fail_read, fail_once_write;
  bool reset_accepted_error, go_accepted_error, ignored_mode;
  bool block_stop; int writes[256], cal_writes_running;
  int callbacks; drv_status_t callback_result;
  int recover_result; bool reset_stuck;
} bus_t;
static int failed;
#define CHECK(x) do { if (!(x)) { printf("FAIL %s:%d: %s\n", __func__, __LINE__, #x); failed++; return; } } while (0)
static void reset_chip(bus_t *b) {
  memset(b->reg,0,sizeof(b->reg)); b->reg[0]=0xe0;b->reg[1]=0x40;
  b->reg[3]=1;b->reg[0x16]=0x3e;b->reg[0x17]=0x8c;
  b->reg[0x18]=0x0c;b->reg[0x19]=0x6c;b->reg[0x1a]=0x36;
  b->reg[0x1b]=0x93;b->reg[0x1c]=0xf5;b->reg[0x1d]=0xa0;
}
static int wr(void *ctx,uint8_t addr,const uint8_t *p,uint16_t n) {
  bus_t *b=ctx; assert(addr==0x5a && n==2);b->writes[p[0]]++;
  if (b->block_stop && ((p[0]==0xc && p[1]==0) || p[0]==1)) return -1;
  if(p[0]==b->fail_write)return -1;
  if(p[0]==b->fail_once_write){b->fail_once_write=-1;return -1;}
  if(p[0]==1&&p[1]==0x80){reset_chip(b);if(b->reset_stuck)b->reg[1]=0x80;return b->reset_accepted_error?-1:0;}
  if((p[0]==0x18||p[0]==0x19||p[0]==0x1a) && b->reg[0xc] && (b->reg[1]&7)==7)b->cal_writes_running++;
  if(p[0]==1&&b->ignored_mode)return 0;
  b->reg[p[0]]=p[1];
  if(p[0]==0xc && p[1]==1 && b->go_accepted_error && (b->reg[1]&7)==7)b->reg[0x18]=0x77;
  return p[0]==0xc&&p[1]==1&&b->go_accepted_error?-1:0;
}
static int rd(void *ctx,uint8_t addr,uint8_t r,uint8_t *v,uint16_t n) {
  bus_t *b=ctx;assert(addr==0x5a&&n==1);if(r==b->fail_read)return -1;
  *v=b->reg[r]; if(r==0)b->reg[0]&=(uint8_t)~0x0a;return 0;
}
static uint32_t ms(void *ctx){return ((bus_t*)ctx)->tick;}
static void pause_ms(void *ctx,uint32_t n){((bus_t*)ctx)->tick+=n;}
static int recover_bus(void *ctx){return ((bus_t*)ctx)->recover_result;}
static void callback(void *ctx,drv_status_t s){bus_t*b=ctx;b->callbacks++;b->callback_result=s;}
static void setup(drv_t *d,bus_t *b) {
  memset(b,0,sizeof(*b));b->fail_read=b->fail_write=b->fail_once_write=-1;reset_chip(b);
  drv_port_t p={0};p.ctx=b;p.i2c_write=wr;p.i2c_read=rd;p.millis=ms;p.delay_ms=pause_ms;p.bus_recover=recover_bus;
  drv_config_t c;drv_default_config(&c);assert(drv_init(d,&p,&c)==0);drv_set_callback(d,callback,b);
}
static drv_cal_t old_cal={0x11,0x22,1,true}, new_cal={0x33,0x44,2,true};
static void reset_uncertain(void){drv_t d;bus_t b;setup(&d,&b);b.reset_accepted_error=true;CHECK(drv_recover(&d)==DRV_ERR_NACK);CHECK(!d.config_ok);CHECK(drv_stop(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(drv_play(&d,1)==DRV_ERR_NEEDS_RECOVER);}
static void rollback_requires_stop(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);CHECK(drv_start_calibration(&d)==0);b.reg[0x18]=0x77;b.block_stop=true;CHECK(drv_stop(&d)!=0);CHECK(b.cal_writes_running==0);CHECK(!d.config_ok);b.block_stop=false;CHECK(drv_stop(&d)!=0);CHECK(drv_play(&d,1)==DRV_ERR_NEEDS_RECOVER);}
static void rollback_after_go_error(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);b.go_accepted_error=true;CHECK(drv_start_calibration(&d)==DRV_ERR_NACK);CHECK(b.reg[0xc]==0);CHECK(d.state==DRV_STATE_IDLE);CHECK(b.reg[0x18]==old_cal.comp);CHECK(b.callbacks==0);}
static void loading_confirms_stop(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_set_strength(&d,100)==0);d.cfg.verify_writes=false;b.ignored_mode=true;CHECK(drv_load_calibration(&d,&new_cal)!=0);CHECK(d.state==DRV_STATE_FAULT);CHECK(b.reg[0x18]==0x0c);}
static void cal_timeout(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);CHECK(drv_start_calibration(&d)==0);b.reg[0x18]=0x77;b.tick+=3000;CHECK(drv_update(&d)==DRV_ERR_TIMEOUT);CHECK(b.reg[0x18]==old_cal.comp&&b.reg[0x19]==old_cal.bemf);CHECK(d.state==DRV_STATE_IDLE);CHECK(b.callbacks==1&&b.callback_result==DRV_ERR_TIMEOUT);}
static void cal_cancel(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);CHECK(drv_start_calibration(&d)==0);b.reg[0x18]=0x77;CHECK(drv_stop(&d)==0);CHECK(b.reg[0x18]==old_cal.comp);CHECK(b.callbacks==1&&b.callback_result==DRV_ERR_CANCELLED);CHECK(drv_update(&d)==0&&b.callbacks==1);}
static void cal_read_failure(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);CHECK(drv_start_calibration(&d)==0);b.reg[0xc]=0;b.reg[0x18]=0x77;b.fail_read=0x19;CHECK(drv_update(&d)==DRV_ERR_NACK);CHECK(d.cfg.cal.comp==old_cal.comp&&d.cfg.cal.bemf==old_cal.bemf);CHECK(d.state==DRV_STATE_FAULT);CHECK(drv_stop(&d)!=0);}
static void cal_success(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_start_calibration(&d)==0);b.reg[0xc]=0;b.reg[0x18]=0x33;b.reg[0x19]=0x44;b.reg[0x1a]=0x29;CHECK(drv_update(&d)==0);CHECK(d.cfg.cal.valid&&d.cfg.cal.comp==0x33&&d.cfg.cal.bemf==0x44&&d.cfg.cal.bemf_gain==1);CHECK(d.state==DRV_STATE_IDLE&&b.callbacks==1);}
static void cal_restore_default(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_start_calibration(&d)==0);b.reg[0xc]=0;b.reg[0]=0xe8;b.reg[0x18]=0xff;CHECK(drv_update(&d)==DRV_ERR_CAL_FAIL);CHECK(b.reg[0x18]==0x0c&&b.reg[0x19]==0x6c);CHECK(!d.cfg.cal.valid);}
static void cal_load_rollback(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_load_calibration(&d,&old_cal)==0);CHECK(drv_set_strength(&d,100)==0);b.fail_once_write=0x19;CHECK(drv_load_calibration(&d,&new_cal)==DRV_ERR_NACK);CHECK(b.reg[0x18]==old_cal.comp&&b.reg[0x19]==old_cal.bemf);CHECK(d.state==DRV_STATE_IDLE&&b.reg[2]==0);}
static void all_stop_writes(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_set_strength(&d,100)==0);int r=b.writes[2],m=b.writes[1];b.fail_write=0xc;CHECK(drv_stop(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(b.writes[2]>r&&b.writes[1]>m);CHECK(b.reg[2]==0&&b.reg[1]==0);CHECK(d.state==DRV_STATE_FAULT);}
static void callback_failed_stop(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_play(&d,1)==0);b.fail_write=0xc;CHECK(drv_stop(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(b.callbacks==1&&b.callback_result==DRV_ERR_NEEDS_RECOVER);CHECK(drv_update(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(drv_wait(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(b.callbacks==1);}
static void mode_verify_error(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_start_diagnostics(&d)==0);b.reg[0xc]=0;b.ignored_mode=true;CHECK(drv_update(&d)==DRV_ERR_VERIFY);CHECK(drv_last_error(&d)==DRV_ERR_VERIFY);}
static void half_configuration(void){drv_t d;bus_t b;setup(&d,&b);drv_config_t c=d.cfg;drv_port_t p=d.port;c.actuator=DRV_LRA;c.library=6;c.open_loop=false;c.od_clamp=0x84;CHECK(drv_init(&d,&p,&c)==0);b.fail_write=0x16;CHECK(drv_recover(&d)==DRV_ERR_NACK);b.fail_write=-1;CHECK(drv_stop(&d)==DRV_ERR_NEEDS_RECOVER);CHECK(drv_play(&d,1)==DRV_ERR_NEEDS_RECOVER);CHECK(drv_recover(&d)==0);CHECK(drv_play(&d,1)==0&&b.reg[3]==6&&b.reg[0x17]==0x84);}
static void busy_not_fault(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_play(&d,1)==0);CHECK(drv_host_write(&d,DRV_HOST_COMMAND,DRV_CMD_PLAY)==DRV_ERR_BUSY);CHECK(d.last_error==0&&d.host_last_result==DRV_ERR_BUSY);}
static void rtp_thermal(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_set_strength(&d,100)==0);b.tick+=51;b.reg[0]=0xe2;CHECK(drv_update(&d)==DRV_ERR_OVER_TEMP);CHECK(b.reg[2]==0&&d.state==DRV_STATE_IDLE);}
static void diag_overcurrent(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_start_diagnostics(&d)==0);b.reg[0xc]=0;b.reg[0]=0xe1;CHECK(drv_update(&d)==DRV_ERR_OVERCURRENT);CHECK(b.callbacks==1);}
static void recovered_callback(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_play(&d,1)==0);CHECK(drv_recover(&d)==0);CHECK(b.callbacks==1&&b.callback_result==DRV_ERR_CANCELLED);CHECK(d.on_done==callback);}
static void failed_recovery_callback(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_play(&d,1)==0);b.recover_result=-1;CHECK(drv_recover(&d)==DRV_ERR_NACK);CHECK(b.callbacks==1&&b.callback_result==DRV_ERR_NACK);}
static void reset_timeout(void){drv_t d;bus_t b;setup(&d,&b);b.reset_stuck=true;CHECK(drv_recover(&d)==DRV_ERR_TIMEOUT);CHECK(d.state==DRV_STATE_FAULT&&!d.config_ok);}
static void preinit(void){drv_t d={0};CHECK(drv_load_calibration(&d,&new_cal)==DRV_ERR_NOT_INIT);CHECK(drv_update(&d)==DRV_ERR_NOT_INIT);CHECK(drv_wait(&d)==DRV_ERR_NOT_INIT);}
static void validation(void){drv_t d;bus_t b;setup(&d,&b);drv_port_t p=d.port;drv_config_t c=d.cfg;c.actuator=(drv_actuator_t)99;CHECK(drv_init(&d,&p,&c)==DRV_ERR_BAD_ARG);setup(&d,&b);CHECK(drv_play(&d,0)==DRV_ERR_BAD_ARG);CHECK(drv_set_strength(&d,128)==DRV_ERR_BAD_ARG);CHECK(drv_select_library(&d,6)==DRV_ERR_BAD_ARG);}
static void rollover(void){drv_t d;bus_t b;setup(&d,&b);b.tick=0xfffffff0u;CHECK(drv_play(&d,1)==0);b.tick=20;CHECK(drv_update(&d)==DRV_ERR_BUSY);b.tick=6000;CHECK(drv_update(&d)==DRV_ERR_TIMEOUT);}
static void effect_and_shadow(void){drv_t d;bus_t b;setup(&d,&b);CHECK(drv_set_strength(&d,100)==0);CHECK(drv_play(&d,123)==0);CHECK(d.host_rtp==0&&b.reg[2]==0);b.reg[0xc]=0;CHECK(drv_update(&d)==0);CHECK(b.callbacks==1&&b.callback_result==0);}
int main(void){
#define RUN(t) do {int before=failed;t();if(failed==before)printf("PASS %s\n",#t);}while(0)
 RUN(reset_uncertain);RUN(rollback_requires_stop);RUN(rollback_after_go_error);RUN(loading_confirms_stop);
 RUN(cal_timeout);RUN(cal_cancel);RUN(cal_read_failure);RUN(cal_success);RUN(cal_restore_default);RUN(cal_load_rollback);
 RUN(all_stop_writes);RUN(callback_failed_stop);RUN(mode_verify_error);RUN(half_configuration);RUN(busy_not_fault);
 RUN(rtp_thermal);RUN(diag_overcurrent);RUN(recovered_callback);RUN(failed_recovery_callback);RUN(reset_timeout);
 RUN(preinit);RUN(validation);RUN(rollover);RUN(effect_and_shadow);
 printf("24 cases; %d failures\n",failed);return failed?1:0;
}
