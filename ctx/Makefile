all: ctx-idrw-203 rfid_reader

ctx-idrw-203: ctx-idrw-203.c
	gcc ctx-idrw-203.c -O0 -g3 -o ctx-idrw-203 -I/usr/local/include -L. -lm -lc -L/usr/local/lib -lusb-1.0
	
rfid_reader:
	gcc rfid_reader.c -O0 -g3 -o rfid_reader -I/usr/local/include -L. -lm -lc -L/usr/local/lib -lusb-1.0

clean:
	rm -f *.o ctx-idrw-203 rfid_reader

install:
	cp 20-rwrfid.rules /etc/udev/rules.d/

