/* KBD */
void*kbd_thread(void* arg){
	int pwm_fd=100;
	struct input_event ie;
	while(alive && read(keyboard, &ie, sizeof(struct input_event))){
		if((ie.type==EV_MSC) || !ie.code || (ie.value==2))continue;//skip misc/empty/repeating event
		if(ie.value==1){//keypress
			switch(ie.code){
				case KEY_ESC     :alive=0;break;
				case KEY_PAGEUP  :pwm_set(PWM0|PWM1|PWM2|PWM3,++pwm_fd);break;
				case KEY_PAGEDOWN:pwm_set(PWM0|PWM1|PWM2|PWM3,--pwm_fd);break;
				//lower a PWM using directionals arrows
				case KEY_UP      :pwm_set(PWM0,pwm_fd-5);break;
				case KEY_RIGHT   :pwm_set(PWM1,pwm_fd-5);break;
				case KEY_DOWN    :pwm_set(PWM3,pwm_fd-5);break;
				case KEY_LEFT    :pwm_set(PWM2,pwm_fd-5);break;
				default:fprintf(stderr,"unhandled code:%i\n",ie.code);
			}
			pwm_print();
		}
		if(ie.value==0){//keyrelease = set back to original value
			switch(ie.code){
				case KEY_UP      :pwm_set(PWM0,pwm_fd);break;
				case KEY_RIGHT   :pwm_set(PWM1,pwm_fd);break;
				case KEY_DOWN    :pwm_set(PWM3,pwm_fd);break;
				case KEY_LEFT    :pwm_set(PWM2,pwm_fd);break;
				default:;
			}
			pwm_print();
		}
	}
	return NULL;
}
void kbd_start(){
	printf("%s\n",__func__);
	
	if((keyboard = open(KBD_PATH, O_RDONLY))<0)
		printf("can't open %s\n",KBD_PATH);
	
	if(pthread_create(&kbd_th, NULL, kbd_thread, NULL)<0)
		printf("kbd_thread start error\n");
}
void kbd_stop(){
	printf("%s\n",__func__);
	close(keyboard);
}
