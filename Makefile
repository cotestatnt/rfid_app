all: rfid_reader

	
rfid_reader:
	gcc rfid_reader.c -O0 -g3 -o rfid_reader -I/usr/local/include -L. -lm -lc -L/usr/local/lib -lusb-1.0

clean:
	rm -f *.o rfid_reader

install:
	cp 20-rwrfid.rules /etc/udev/rules.d/

