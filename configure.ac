##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_define([v_maj], [4])
m4_define([v_min], [12])
m4_define([v_mic], [2])
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_define([v_ver], [v_maj.v_min.v_mic])
m4_define([lt_rev], m4_eval(v_maj + v_min))
m4_define([lt_cur], v_mic)
m4_define([lt_age], v_min)
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##

AC_INIT([cgminer], [v_ver], [hestiia])

AC_PREREQ(2.59)
AC_CANONICAL_SYSTEM
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([cgminer.c])
AC_CONFIG_HEADERS([config.h])

AM_INIT_AUTOMAKE([foreign subdir-objects])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])
AC_USE_SYSTEM_EXTENSIONS

##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_ifdef([v_rev], , [m4_define([v_rev], [0])])
m4_ifdef([v_rel], , [m4_define([v_rel], [])])
AC_DEFINE_UNQUOTED(CGMINER_MAJOR_VERSION, [v_maj], [Major version])
AC_DEFINE_UNQUOTED(CGMINER_MINOR_VERSION, [v_min], [Minor version])
AC_DEFINE_UNQUOTED(CGMINER_MINOR_SUBVERSION, [v_mic], [Micro version])
version_info="lt_rev:lt_cur:lt_age"
release_info="v_rel"
AC_SUBST(version_info)
AC_SUBST(release_info)
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
VMAJ=v_maj
AC_SUBST(VMAJ)

AC_CANONICAL_BUILD
AC_CANONICAL_HOST

dnl Make sure anyone changing configure.ac/Makefile.am has a clue
AM_MAINTAINER_MODE

dnl Checks for programs
AC_PROG_CC
gl_EARLY
AC_PROG_GCC_TRADITIONAL
AM_PROG_CC_C_O

# This breaks autoreconf so disable it
# LT_INIT([disable-shared])

gl_INIT

dnl Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS(syslog.h)

AC_FUNC_ALLOCA

have_win32=false
PTHREAD_FLAGS="-lpthread"
LIBZ_LIBS="-lz"
DLOPEN_FLAGS="-ldl"
WS2_LIBS=""
MM_LIBS=""
MATH_LIBS="-lm"
RT_LIBS="-lrt"

case $target in
  amd64-*)
    have_x86_64=true
    ;;
  x86_64-*)
    have_x86_64=true
    ;;
  *)
    have_x86_64=false
    ;;
esac

#CFLAGS="$CFLAGS -Wimplicit-fallthrough=0"
#^ causes problem on gcc 4.9 - pass in from CFLAG if needed

case $target in
  *-*-linux-gnu*)
    have_linux=true
    ;;
  *-*-mingw*)
    have_win32=true
    PTHREAD_FLAGS=""
    DLOPEN_FLAGS=""
    WS2_LIBS="-lws2_32"
    MM_LIBS="-lwinmm"
    RT_LIBS=""
    #AC_DEFINE([_WIN32_WINNT], [0x0501], "WinNT version for XP+ support")
    ;;
  powerpc-*-darwin*)
    have_darwin=true
    CFLAGS="$CFLAGS -faltivec"
    PTHREAD_FLAGS=""
    RT_LIBS=""
    ;;
  *-*-darwin*)
    have_darwin=true
    PTHREAD_FLAGS=""
    RT_LIBS=""
	;;
  *-*-freebsd*)
    PTHREAD_FLAGS=""
    DLOPEN_FLAGS=""
    RT_LIBS=""
    ;;
esac

has_winpthread=false
if test "x$have_win32" = xtrue; then
        has_winpthread=true
        AC_CHECK_LIB(winpthread, nanosleep, , has_winpthread=false)
        PTHREAD_LIBS=-lwinpthread
fi

if test "x$has_winpthread" != xtrue; then
        AC_CHECK_LIB(pthread, pthread_create, ,
                AC_MSG_ERROR([Could not find pthread library - please install libpthread]))
        PTHREAD_LIBS=-lpthread
fi

# Drivers that are designed to be run on dedicated hardware should set standalone to yes
# All drivers should prepend an x to the drivercount

standalone="no"
drivercount=""

gekko="no"

AC_ARG_ENABLE([gekko],
	[AC_HELP_STRING([--enable-gekko],[Compile support for GekkoScience (default disabled)])],
	[gekko=$enableval]
	)
if test "x$gekko" = xyes; then
	AC_DEFINE([USE_GEKKO], [1], [Defined to 1 if GekkoScience support is wanted])
	AC_DEFINE([USE_VMASK], [1], [Defined to 1 if version mask rolling is wanted])
	drivercount=x$drivercount
fi
AM_CONDITIONAL([HAS_GEKKO], [test x$gekko = xyes])


bm1397="no"

AC_ARG_ENABLE([bm1397],
	[AC_HELP_STRING([--enable-bm1397],[Compile support for bm1397 (default disabled)])],
	[bm1397=$enableval]
	)
if test "x$bm1397" = xyes; then
	AC_DEFINE([USE_BM1397], [1], [Defined to 1 if bm1397 support is wanted])
	AC_DEFINE([USE_VMASK], [1], [Defined to 1 if version mask rolling is wanted])
	drivercount=x$drivercount
fi
AM_CONDITIONAL([HAS_BM1397], [test x$bm1397 = xyes])

curses="auto"

AC_ARG_WITH([curses],
	[AC_HELP_STRING([--without-curses],[Compile support for curses TUI (default enabled)])],
	[curses=$withval]
	)
if test "x$curses" = "xno"; then
	cursesmsg='User specified --without-curses. TUI support DISABLED'
else
	AC_SEARCH_LIBS(setupterm, pdcurses ncurses pdcurses, [
		curses=yes
		cursesmsg="FOUND: ${ac_cv_search_addstr}"
		AC_DEFINE([HAVE_CURSES], [1], [Defined to 1 if curses TUI support is wanted])
	], [
		if test "x$curses" = "xyes"; then
			AC_MSG_ERROR([Could not find curses library - please install libncurses-dev or pdcurses-dev (or configure --without-curses)])
		else
			AC_MSG_WARN([Could not find curses library - if you want a TUI, install libncurses-dev or pdcurses-dev])
			curses=no
			cursesmsg='NOT FOUND. TUI support DISABLED'
		fi
	])
fi


#Add a new device to this list if it needs libusb, along with a no on the end.
if test x$gekko != xno; then
	want_usbutils=true
else
	want_usbutils=false
fi

if test x$gekko$bm1397 != xnono; then
	want_crc16=true
else
	want_crc16=false
fi

if test x$bm1397 != xno; then
	want_gpiod=true
else
	want_gpiod=false
fi

AM_CONDITIONAL([WANT_USBUTILS], [test x$want_usbutils != xfalse])
AM_CONDITIONAL([HAVE_CURSES], [test x$curses = xyes])
AM_CONDITIONAL([HAVE_WINDOWS], [test x$have_win32 = xtrue])
AM_CONDITIONAL([HAVE_x86_64], [test x$have_x86_64 = xtrue])
AM_CONDITIONAL([WANT_CRC16], [test x$want_crc16 != xfalse])
AM_CONDITIONAL([WANT_GPIOD], [test x$want_gpiod != xfalse])

if test "x$want_usbutils" != xfalse; then
	AC_DEFINE([USE_USBUTILS], [1], [Defined to 1 if usbutils support required])
	case $target in
		*-*-freebsd*)
			LIBUSB_LIBS="-lusb"
			LIBUSB_CFLAGS=""
			AC_DEFINE(HAVE_LIBUSB, 1, [Define if you have libusb-1.0])
			;;
		*)
			PKG_CHECK_MODULES(LIBUSB, libusb-1.0, [AC_DEFINE(HAVE_LIBUSB, 1, [Define if you have libusb-1.0])], [AC_MSG_ERROR([Could not find usb library - please install libusb-1.0])])
			;;
	esac
else
	LIBUSB_LIBS=""
fi

djansson="yes"
case $target in
	*-*-freebsd*)
		JANSSON_LIBS="-ljansson"
		JANSSON_CFLAGS=""
		AC_DEFINE(HAVE_JANSSON, 1, [Define if you have jansson >= 2.6])
		;;
	*)
		PKG_CHECK_MODULES(JANSSON, jansson >= 2.6, [AC_DEFINE(HAVE_JANSSON, 1, [Define if you have jansson >= 2.6])], [djansson="no"])
		;;
esac

if test "x$djansson" = xno; then
	AC_CONFIG_SUBDIRS([compat/jansson-2.9])
	JANSSON_LIBS="compat/jansson-2.9/src/.libs/libjansson.a"
fi

AM_CONDITIONAL([WANT_STATIC_JANSSON], [test x$djansson = xno])

AC_CHECK_HEADERS([uthash.h])

PKG_PROG_PKG_CONFIG()

if test "x$have_cgminer_sdk" = "xtrue"; then
	if test "x$have_x86_64" = xtrue; then
		ARCH_DIR=x86_64
	else
		ARCH_DIR=x86
	fi
	PKG_CONFIG="${PKG_CONFIG:-pkg-config} --define-variable=arch=$ARCH_DIR --define-variable=target=$target --define-variable=cgminersdkdir=$CGMINER_SDK"
	PKG_CONFIG_PATH="$CGMINER_SDK/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
fi

AC_SUBST(LIBUSB_LIBS)
AC_SUBST(LIBUSB_CFLAGS)

AC_ARG_ENABLE([libcurl],
	[AC_HELP_STRING([--disable-libcurl],[Disable building with libcurl for GBT support])],
	[libcurl=$enableval]
	)

static_curl="no"
if test "x$libcurl" != xno; then
	if test "x$have_win32" != xtrue; then
		PKG_CHECK_MODULES([LIBCURL], [libcurl >= 7.25.0], [AC_DEFINE([CURL_HAS_KEEPALIVE], [1], [Defined if version of curl supports keepalive.])],
			[PKG_CHECK_MODULES([LIBCURL], [libcurl >= 7.18.2], ,[AC_MSG_ERROR([Missing required libcurl dev >= 7.18.2])])])
	else
		PKG_CHECK_MODULES([LIBCURL], [libcurl >= 7.25.0], ,[AC_MSG_ERROR([Missing required libcurl dev >= 7.25.0])])
		AC_DEFINE([CURL_HAS_KEEPALIVE], [1])
		static_curl="yes"
	fi
	AC_DEFINE([HAVE_LIBCURL], [1], [Defined to 1 if libcurl support built in])
else
	LIBCURL_LIBS=""
fi
AC_SUBST(LIBCURL_LIBS)
AM_CONDITIONAL([STATIC_CURL], [test x$static_curl = xyes])

libsystemd="no"

AC_ARG_ENABLE([libsystemd],
	[AC_HELP_STRING([--enable-libsystemd],[Enable building with libsystemd for watchdog and status notification support])],
	[libsystemd=$enableval]
)

if test "x$libsystemd" != xno; then
	if test "x$have_linux" != xtrue; then
		AC_MSG_ERROR([libsystemd is only supported on Linux platforms])
	fi

	PKG_CHECK_MODULES(LIBSYSTEMD, libsystemd, , AC_MSG_ERROR(Could not find libsystemd dev))
	AC_DEFINE([USE_LIBSYSTEMD], [1], [Defined to 1 if libsystemd support is wanted])
else
	LIBSYSTEMD_LIBS=""
fi

#check execv signature
AC_COMPILE_IFELSE([AC_LANG_SOURCE([
		   #include <process.h>
		   int execv(const char*, const char*const*);
		   ])],
		   AC_DEFINE([EXECV_2ND_ARG_TYPE], [const char* const*], [int execv(const char*, const char*const*);]),
		   AC_DEFINE([EXECV_2ND_ARG_TYPE], [char* const*], [int execv(const char*, char*const*);]))

dnl CCAN wants to know a lot of vars.
# All the configuration checks.  Regrettably, the __attribute__ checks will
# give false positives on old GCCs, since they just cause warnings.  But that's
# fairly harmless.
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((cold)) cleanup(void) { }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_COLD], [1],
                                    [Define if __attribute__((cold))]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((const)) cleanup(void) { }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_CONST], [1],
                                    [Define if __attribute__((const))]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((noreturn)) cleanup(void) { exit(1); }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_NORETURN], [1],
                                    [Define if __attribute__((noreturn))]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((format(__printf__, 1, 2))) cleanup(const char *fmt, ...) { }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_PRINTF], [1],
                                    [Define if __attribute__((format(__printf__)))]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((unused)) cleanup(void) { }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_UNUSED], [1],
                                    [Define if __attribute__((unused))]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static void __attribute__((used)) cleanup(void) { }])],
			  AC_DEFINE([HAVE_ATTRIBUTE_USED], [1],
                                    [Define if __attribute__((used))]))
AC_LINK_IFELSE([AC_LANG_SOURCE([int main(void) { return __builtin_constant_p(1) ? 0 : 1; }])],
			  AC_DEFINE([HAVE_BUILTIN_CONSTANT_P], [1],
                                    [Define if have __builtin_constant_p]))
AC_LINK_IFELSE([AC_LANG_SOURCE([int main(void) { return __builtin_types_compatible_p(char *, int) ? 1 : 0; }])],
			  AC_DEFINE([HAVE_BUILTIN_TYPES_COMPATIBLE_P], [1],
                                    [Define if have __builtin_types_compatible_p]))
AC_COMPILE_IFELSE([AC_LANG_SOURCE([static int __attribute__((warn_unused_result)) func(int x) { return x; }])],
			  AC_DEFINE([HAVE_WARN_UNUSED_RESULT], [1],
                                    [Define if __attribute__((warn_unused_result))]))

if test "x$prefix" = xNONE; then
	prefix=/usr/local
fi

AC_DEFINE_UNQUOTED([CGMINER_PREFIX], ["$prefix/bin"], [Path to cgminer install])

AC_SUBST(JANSSON_LIBS)
AC_SUBST(LIBZ_LIBS)
AC_SUBST(PTHREAD_FLAGS)
AC_SUBST(DLOPEN_FLAGS)
AC_SUBST(PTHREAD_LIBS)
AC_SUBST(NCURSES_LIBS)
AC_SUBST(PDCURSES_LIBS)
AC_SUBST(WS2_LIBS)
AC_SUBST(MM_LIBS)
AC_SUBST(MATH_LIBS)
AC_SUBST(RT_LIBS)

AC_CONFIG_FILES([
	Makefile
	compat/Makefile
	ccan/Makefile
	lib/Makefile
	])
AC_OUTPUT


echo
echo
echo
echo "------------------------------------------------------------------------"
echo "$PACKAGE $VERSION"
echo "------------------------------------------------------------------------"
echo
echo
echo "Configuration Options Summary:"
echo

if test "x$libcurl" != xno; then
	echo "  libcurl(GBT).: Enabled: $LIBCURL_LIBS"
else
	echo "  libcurl(GBT).: Disabled"
fi

if test "x$libsystemd" != xno; then
	echo "  libsystemd...........: Enabled: $LIBSYSTEMD_LIBS"
else
	echo "  libsystemd...........: Disabled"
fi

echo "  curses.TUI...........: $cursesmsg"

if test "x$gekko" = xyes; then
	echo "  GekkoScience.ASICs...: Enabled"
else
	echo "  GekkoScience.ASICs...: Disabled"
fi

if test "x$bm1397" = xyes; then
	echo "  bm1397.ASICs...: Enabled"
else
	echo "  bm1397.ASICs...: Disabled"
fi

#Add any new device to this, along with a no on the end of the test
if test "x$$gekko$bm1397" = xnono; then
	echo
	AC_MSG_ERROR([No mining devices configured in])
	echo
fi

if test "x$standalone" = xyes; then
	if test $drivercount != x; then
		echo
		AC_MSG_ERROR([You have configured more than one driver in with a driver that is designed to be standalone only (see ./configure --help)])
		echo
	fi
fi

echo
echo "Compilation............: make (or gmake)"
echo "  CPPFLAGS.............: $CPPFLAGS"
echo "  CFLAGS...............: $CFLAGS"
echo "  LDFLAGS..............: $LDFLAGS $PTHREAD_FLAGS"
echo "  LDADD................: $DLOPEN_FLAGS $LIBCURL_LIBS $LIBSYSTEMD_LIBS $JANSSON_LIBS $LIBZ_LIBS $PTHREAD_LIBS $NCURSES_LIBS $PDCURSES_LIBS $WS2_LIBS $MATH_LIBS $LIBUSB_LIBS $RT_LIBS"
echo
echo "Installation...........: make install (as root if needed, with 'su' or 'sudo')"
echo "  prefix...............: $prefix"
echo

