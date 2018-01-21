matemonitor: matemonitor17.c load_control.o gdlTimer.o outback.h usb_arduino.h SMS.o wh.o usb_com.o usb_arduino.o
	gcc -Wall -o matemonitor -lwiringPi -lpanel -lncurses -ldl -lsqlite3 matemonitor17.c wh.o gdlTimer.o SMS.o usb_com.o usb_arduino.o load_control.o

load_control.o:load_control.c load_control.h outback.h usb_arduino.h
	gcc -c -Wall load_control.c

gdlTimer.o:gdlTimer.c gdlTimer.h
	gcc -Wall -c -o gdlTimer.o gdlTimer.c

wh.o:wh.c usb_arduino.h
	gcc -Wall -c wh.c

SMS.o:SMS.c 		
	gcc -Wall -c SMS.c

usb_com.o:usb_com.c usb_com.h
	gcc -Wall -c usb_com.c

usb_arduino.o:usb_arduino.c outback.h usb_arduino.h
	gcc -Wall -c usb_arduino.c
