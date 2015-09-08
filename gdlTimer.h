
struct timer_data{
	unsigned long ReAlarmWaitTimer;
	unsigned long ReAlarmWaitDelay;
	
	unsigned long TriggerCountWithinTimerStart;
	int reqTriggersWithinTicks;
	int TriggerCount;
	int reqNumTriggers;
	unsigned long CurrentTick;
};
unsigned long tick();
void TimerInit(struct timer_data * t, unsigned long reAlarmWaitTicks, int reqTriggers, int reqTriggersWithinTicks);
void PrintTimer(struct timer_data * t);
int	TriggerTimer(struct timer_data * t); //returns 0 for no alarm.  1 means alarm.

