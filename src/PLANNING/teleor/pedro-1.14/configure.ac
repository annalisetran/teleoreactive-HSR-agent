AC_INIT([pedro],[1.14])
AC_CONFIG_SRCDIR([src/server/pedro.c])
AM_INIT_AUTOMAKE
AC_PROG_CC
AC_PROG_LEX([noyywrap])
AC_PROG_INSTALL
AC_PROG_RANLIB
#AC_PROG_LIBTOOL
PKG_CHECK_MODULES([GLIB],[glib-2.0 >= 2.12])
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_MACRO_DIR([m4])
CPPFLAGS="-DNDEBUG -O2 -g `pkg-config --cflags glib-2.0`"
case "${host}" in
        *i686*)
                WIN32="yes"
		LDFLAGS=""
		#LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
                LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm -lwsock32 -lws2_32 -lwinmm"
                ;;
        *x86*)
                WIN32="no"
		LDFLAGS=""
		#LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
                LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm -lwsock32 -lws2_32 -lwinmm"
                ;;
        *)
                WIN32="no"
                #LDFLAGS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
		LDFLAGS=""
		LIBS="-L../lib -lpedro `pkg-config --libs glib-2.0` -lm"
                ;;
esac
AC_CONFIG_FILES([Makefile src/Makefile src/server/Makefile src/lib/Makefile src/examples/Makefile doc/Makefile])
AC_OUTPUT
