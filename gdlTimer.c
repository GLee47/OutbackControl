//timer.c
#define FirstTick	(60*60*24*365)
#include <stdio.h>
#include "gdlTimer.h"
/*
struct timer_data{
	unsigned long ReAlarmWaitTimer;
	unsigned long ReAlarmWaitDelay;
	
	unsigned long TriggerCountWithinTimerStart;
	int reqTriggersWithinTicks;
	int TriggerCount;
	int reqNumTriggers;
};
*/
static unsigned long Ticks = FirstTick;


unsigned long
tick ()
{
  Ticks++;
  return Ticks;
}

void
TimerInit (struct timer_data *t, unsigned long reAlarmWaitTicks,
	   int reqTriggers, int reqTriggersWithinTicks)
{
  t->ReAlarmWaitTimer = 0ul;
  t->ReAlarmWaitDelay = reAlarmWaitTicks;
  t->TriggerCountWithinTimerStart = 0ul;
  t->reqTriggersWithinTicks = reqTriggersWithinTicks;
  t->reqNumTriggers = reqTriggers;
  t->TriggerCount = 0;
}

void
PrintTimer (struct timer_data *t)
{
  printf ("%lu\t%lu\t%lu\t%d\t%d\t%d\n", t->ReAlarmWaitTimer,
	  t->ReAlarmWaitDelay, t->TriggerCountWithinTimerStart,
	  t->reqTriggersWithinTicks, t->TriggerCount, t->reqNumTriggers);
}

int
TriggerTimer (struct timer_data *t)	//returns 0 for no alarm.  1 means alarm.
{

  if (Ticks == t->CurrentTick)
    return 0;			//if ticks hasn't incremented then ignore this call
  t->CurrentTick = Ticks;	//rember the time to compare next on next call
  if (t->TriggerCount == 0)
    t->TriggerCountWithinTimerStart = Ticks;
  t->TriggerCount++;
  int remainingTriggers = ((t->reqNumTriggers) - (t->TriggerCount));
  if ((t->ReAlarmWaitTimer + t->ReAlarmWaitDelay) >= Ticks)
    {				//time since last alarm has not yet expired
      t->TriggerCount = 0;	//reset trigger count because this one doesn't count
    }
  //passed test continue testing
  else if ((t->TriggerCountWithinTimerStart + t->reqTriggersWithinTicks) <
	   (Ticks + remainingTriggers))
    {				//time to get required number of triggers has already expired or too may remaining triggers must 
      //occur to fit in alloted time.  Reset timer.
      t->TriggerCount = 1;	// reset count but count this one
      t->TriggerCountWithinTimerStart = Ticks;
    }
  else if (t->TriggerCount < t->reqNumTriggers)	// still within time limit.  Is count reached yet?
    {
      // not enough triggers to cause alarm yet
      // do nothing -- I think
    }
  else
    {				// triggering alarm and resetting
      t->ReAlarmWaitTimer = Ticks;
      t->TriggerCount = 0;
      return 1;
    }
  return 0;
}

/*
int main(void)
{
	struct timer_data testTimer;
	
	TimerInit(&testTimer,5ul,3,4);
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());
	printf("%d %d\n",__LINE__,TriggerTimer(&testTimer));
	PrintTimer(&testTimer);
	printf("%d %lu\n",__LINE__,tick());

	return 0;
}
*/
