
ACLOCAL_AMFLAGS = -I m4

JANSSON_CPPFLAGS= -I$(top_builddir)/compat/jansson-2.9/src -I$(top_srcdir)/compat/jansson-2.9/src

if WANT_USBUTILS
USBUTILS_CPPFLAGS = `pkg-config libusb-1.0 --libs --cflags`
else
USBUTILS_CPPFLAGS =
endif

if STATIC_CURL
LIBCURL_CPPFLAGS = -DCURL_STATICLIB
else
LIBCURL_CPPFLAGS =
endif

EXTRA_DIST	= example.conf linux-usb-cgminer \
		  api-example.php miner.php	\
		  API.class API.java api-example.c windows-build.txt \
		  bitstreams/README API-README FPGA-README \
		  bitforce-firmware-flash.c hexdump.c ASIC-README \
		  01-cgminer.rules

SUBDIRS		= lib compat ccan

cgminer_CPPFLAGS = $(PTHREAD_FLAGS) -fno-strict-aliasing $(JANSSON_CPPFLAGS) $(USBUTILS_CPPFLAGS) $(LIBCURL_CPPFLAGS)

bin_PROGRAMS	= cgminer

cgminer_LDFLAGS	= $(PTHREAD_FLAGS)
cgminer_LDADD	= $(DLOPEN_FLAGS) @LIBCURL_LIBS@ @JANSSON_LIBS@ @LIBZ_LIBS@ @PTHREAD_LIBS@ \
		  @NCURSES_LIBS@ @PDCURSES_LIBS@ @WS2_LIBS@ \
		  @LIBUSB_LIBS@ @MM_LIBS@ @RT_LIBS@ @LIBSYSTEMD_LIBS@ \
		  @MATH_LIBS@ lib/libgnu.a ccan/libccan.a

cgminer_CPPFLAGS += -I$(top_builddir)/lib -I$(top_srcdir)/lib

if !HAVE_WINDOWS
cgminer_CPPFLAGS += @LIBCURL_CFLAGS@
endif

# common sources
cgminer_SOURCES := cgminer.c

cgminer_SOURCES	+= elist.h miner.h compat.h bench_block.h	\
		   util.c util.h uthash.h logging.h		\
		   sha2.c sha2.h api.c

cgminer_SOURCES	+= logging.c

cgminer_SOURCES	+= klist.h klist.c

cgminer_SOURCES	+= noncedup.c

if WANT_USBUTILS
cgminer_SOURCES += usbutils.c usbutils.h
endif

if WANT_CRC16
cgminer_SOURCES += crc16.c crc.h
endif

# Device drivers

if HAS_GEKKO
cgminer_SOURCES += driver-gekko.c driver-gekko.h
endif

if HAS_BM1397
cgminer_SOURCES += driver-bm1397.c driver-bm1397.h
endif