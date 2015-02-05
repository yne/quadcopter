#ifndef _PWM_H
#define _PWM_H

#include <stdint.h>



#define GPIO_1_START 0x4804C000
#define GPIO_2_START 0x481AC000
#define PWM_1_START  0x48302000
#define PWM_2_START  0x48304000

typedef struct{
	volatile uint32_t rev,_0[3],sysconfig,_1[3],eoi,
	irqstatus_raw_0,irqstatus_raw_1,irqstatus_0,irqstatus_1,
	irqstatus_set_0,irqstatus_set_1,irqstatus_clr_0,irqstatus_clr_1,
	irqwaken_0,irqwaken_1,_2[50],sysstatus,_3[6],ctrl,oe,datain,dataout,
	leveldetect0,leveldetect1,risingdetecte,fallingdetecte,
	debouncenable,debouncingtime,_4[14],cleardataout,setdataout;
}GPIO_map;

typedef struct{
	//PWM
	volatile uint32_t idver,sysconfig,clkconfig,_[125];
	//EPWM 0x200
	volatile uint16_t tbctl,tbsts,tbphshr,tbphs,tbcnt,tbprd,tbprd_,cmpctl,cmpahr,cmpa,
	cmpb,aqctla,aqctlb,aqsfrc,aqcsfrc,dbctl,dbred,dbfed,tzsel,tzsel_,tzctl,
	tzeint,tzflg,tzclr,tzfrc,etsel,etps,etflg,etclr,etfrc,pcctl,pcctl_,hrctl;
}PWM_map;


void pwmInit();
void pwmStop();
#endif
