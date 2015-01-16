/* PWM */
void pwm_print(){
	printf("%i	%i	%i	%i\n",duties[0],duties[1],duties[2],duties[3]);
}
void pwm_set(int mask,int duty){
	int i;
	char buf[64];//temporary itoa buffer
	sprintf(buf,"%i0000",duty);
	for(i=0;i<4;i++){
		if(!((1<<i) & mask))continue;
		duties[i]=duty;
		write(pwm_fd[i],buf,strlen(buf)+1);
	}
}
int  pwm_get(int n){
	return duties[n];
}
void pwm_stop(int mask){
	printf("%s\n",__func__);
	int i;
	for(i=0;i<4;i++){
		if(!((1<<i) & mask))continue;
		//pwm_set((1<<i),0);//go back to speed 0 ?
		if(close(pwm_fd[i])<0)
			fprintf(stderr,"can't stop <pwm_fd:%i>\n",i);
	}
}
void pwm_start(int mask){
	printf("%s\n",__func__);
	
	int i;
	char path[128];
	
	for(i=0;i<4;i++){
		if(!((1<<i) && mask))continue;
		sprintf(path,PWM_PATH,1+(i>>1),i&1);
		if((pwm_fd[i]=open(path,O_WRONLY))<0)
			fprintf(stderr,"can't init <%s>\n",path);
	}
}