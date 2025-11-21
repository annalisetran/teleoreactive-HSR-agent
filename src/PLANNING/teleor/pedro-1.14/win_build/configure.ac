AC_INIT([pedro],[1.6])
AC_CONFIG_SRCDIR([src/server/pedro.c])
AM_INIT_AUTOMAKE
AC_PROG_CC
AC_PROG_LEX
AC_PROG_INSTALL
AC_PROG_RANLIB
#AC_PROG_LIBTOOL
#PKG_CHECK_MODULES([GLIB],[glib-2.0 >= 2.12])
AM_CONFIG_HEADER(config.h)
AC_CONFIG_MACRO_DIR([m4])
CPPFLAGS="-DNDEBUG -O2 -g  `pkg-config --cflags glib-2.0`"
case "${host}" in
        *mingw32*)
                WIN32="yes"
		LDFLAGS="-static"
		#LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
                LIBS="-L../lib -lpedro -lglib-2.0 -lm -lwsock32 -lws2_32 -lwinmm"
                ;;
        *)
                WIN32="no"
                #LDFLAGS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
		LDFLAGS="-static"
                LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -liconv"
		#LIBS="-L../lib -lpedro -lglib-2.0 -lm"
                ;;
esac
AC_OUTPUT(Makefile src/Makefile src/server/Makefile src/lib/Makefile src/examples/Makefile doc/Makefile)
