noinst_LIBRARIES = libpedro.a

libpedro_a_SOURCES = channel_watch.c channel.c client.c  clientConnection.c 

# for building a dll under windows
# run make libpedro.dll



#PEDROOBJ = $(libpedro_a_SOURCES:.c=.o)

#libpedro.dll: $(PEDROOBJ)
#	$(COMPILE) -shared $(PEDROOBJ) -o libpedro.dll $(LIBS)


#DLLWRAP = i686-w64-mingw32-dllwrap

.DLLWRAP_FLAGS = --as=$(AS) --export-all --driver-name $(CC) -s

PEDROOBJ = $(libpedro_a_SOURCES:.c=.o)

libpedro.dll: $(PEDROOBJ)
	ld $(DLLWRAP_FLAGS) \
	--output-def libpedro.def --implib libpedro.a \
	-o $@ $^ $(LDFLAGS)
